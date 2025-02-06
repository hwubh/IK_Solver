using System;
using Unity.Mathematics;
using UnityEngine.PathTracing.Integration;
using UnityEngine.Rendering;
using UnityEngine.Rendering.UnifiedRayTracing;

namespace UnityEngine.PathTracing.Lightmapping
{
    internal static class BakeLightmapDriver
    {
        public class LightmapBakeState
        {
            public uint SampleIndex;
            public UInt64 TexelIndex;

            public void Init()
            {
                SampleIndex = 0;
                TexelIndex = 0;
            }

            public void Tick(uint passSampleCount, uint totalSampleCount, UInt64 chunkTexelCount, UInt64 totalTexelCount, out bool instanceIsDone, out bool chunkIsDone)
            {
                instanceIsDone = false;
                chunkIsDone = false;

                SampleIndex += passSampleCount;
                Debug.Assert(SampleIndex <= totalSampleCount);
                if (SampleIndex == totalSampleCount)
                {
                    // a chunk is done since we have reached `totalSampleCount`
                    chunkIsDone = true;
                    TexelIndex += chunkTexelCount;
                    SampleIndex = 0;
                }
                Debug.Assert(TexelIndex <= totalTexelCount);
                if (TexelIndex == totalTexelCount)
                {
                    // an instance is done since we have reached `totalTexelCount`
                    instanceIsDone = true;
                }
            }
        }

        public struct IntegrationSettings
        {
            public RayTracingBackend Backend;
            public uint MaxDispatchesPerFlush;      // how many dispatches to do before flushing the GPU
            public bool DebugDispatches;

            public static readonly IntegrationSettings Default = new IntegrationSettings
            {
                Backend = RayTracingBackend.Compute,
                MaxDispatchesPerFlush = 1,
                DebugDispatches = false
            };
        }

        public class LightmapBakeSettings
        {
            public uint AOSampleCount = 32;
            public uint DirectSampleCount = 32;
            public uint IndirectSampleCount = 512;
            public uint ValiditySampleCount = 512;

            public AntiAliasingType AOAntiAliasingType = AntiAliasingType.Stochastic;
            public AntiAliasingType DirectAntiAliasingType = AntiAliasingType.SuperSampling;
            public AntiAliasingType IndirectAntiAliasingType = AntiAliasingType.Stochastic;
            public AntiAliasingType ValidityAntiAliasingType = AntiAliasingType.Stochastic;

            public uint BounceCount = 4;
            public float AOMaxDistance = 1.0f;
            public float PushOff = 0.00001f;
            public uint AntiAliasingSampleCount = 4;
            public UInt64 ExpandedBufferSize = 262144;
            public uint GetSampleCount(IntegratedOutputType integratedOutputType)
            {
                switch (integratedOutputType)
                {
                    case IntegratedOutputType.AO: return AOSampleCount;
                    case IntegratedOutputType.Direct: return DirectSampleCount;
                    case IntegratedOutputType.DirectionalityDirect: return DirectSampleCount;
                    case IntegratedOutputType.Indirect: return IndirectSampleCount;
                    case IntegratedOutputType.DirectionalityIndirect: return IndirectSampleCount;
                    case IntegratedOutputType.Validity: return ValiditySampleCount;
                    case IntegratedOutputType.ShadowMask: return DirectSampleCount;
                    default:
                        Debug.Assert(false, "Unexpected case.");
                        return 0;
                }
            }

            public AntiAliasingType GetAntiAliasingType(IntegratedOutputType integratedOutputType)
            {
                switch (integratedOutputType)
                {
                    case IntegratedOutputType.AO: return AOAntiAliasingType;
                    case IntegratedOutputType.Direct: return DirectAntiAliasingType;
                    case IntegratedOutputType.DirectionalityDirect: return DirectAntiAliasingType;
                    case IntegratedOutputType.Indirect: return IndirectAntiAliasingType;
                    case IntegratedOutputType.DirectionalityIndirect: return IndirectAntiAliasingType;
                    case IntegratedOutputType.Validity: return ValidityAntiAliasingType;
                    case IntegratedOutputType.ShadowMask: return DirectAntiAliasingType;
                    default:
                        Debug.Assert(false, "Unexpected case.");
                        return 0;
                }
            }
        }

        internal static uint AccumulateLightmapInstance(
            LightmapBakeState bakeState,
            Instance instance,
            LightmapBakeSettings lightmapBakeSettings,
            IntegratedOutputType integratedOutputType,
            LightmappingContext lightmappingContext,
            UVAccelerationStructure uvAS,
            UVFallbackBuffer uvFallbackBuffer,
            out uint chunkSize,
            out bool instanceIsDone)
        {
            CommandBuffer cmd = lightmappingContext.GetCommandBuffer();

            GraphicsBuffer traceScratchBuffer = lightmappingContext.TraceScratchBuffer;
            {
                // Retrieve the resources for the current instance
                Vector2Int instanceTexelOffset = instance.TexelOffset;
                int instanceWidth = instance.TexelSize.x;
                int instanceHeight = instance.TexelSize.y;
                UInt64 instanceTexelCount = (UInt64)instanceWidth * (UInt64)instanceHeight;

                // Calculate chunking
                uint maxChunkSize = (uint)lightmappingContext.ExpandedOutput.count;
                UInt64 chunkTexelOffset = bakeState.TexelIndex;
                uint2 chunkOffset = new uint2((uint)(chunkTexelOffset % (UInt64)instanceWidth), (uint)(chunkTexelOffset / (UInt64)instanceWidth));
                uint remainingTexels = (uint)instanceWidth - chunkOffset.x + ((uint)(instanceHeight - 1) - chunkOffset.y) * (uint)instanceWidth;
                Debug.Assert(remainingTexels > 0);
                chunkSize = math.min(maxChunkSize, remainingTexels);
                uint maxSamplesPerChunk = maxChunkSize / chunkSize;
                uint expandedSampleWidth = math.ceilpow2(maxSamplesPerChunk);
                if (expandedSampleWidth > maxSamplesPerChunk) expandedSampleWidth /= 2;
                Debug.Assert(expandedSampleWidth >= 1);
                Debug.Assert(expandedSampleWidth * chunkSize <= maxChunkSize);
                bool newChunkStarted = bakeState.SampleIndex == 0;

                // Calculate next AA sample
                uint maxSampleCountPerTexel = lightmapBakeSettings.GetSampleCount(integratedOutputType); // Retrieve the setting for total samples per texel for a given lightmap type
                uint maxAASampleCountPerTexel = math.min(maxSampleCountPerTexel, lightmapBakeSettings.AntiAliasingSampleCount); // We can at a maximum take as many AA samples as there are total samples
                uint samplesBetweenAASamples = math.max(1, maxSampleCountPerTexel / maxAASampleCountPerTexel);
                uint samplesUntilNextAA = samplesBetweenAASamples - bakeState.SampleIndex % samplesBetweenAASamples;
                bool generateAntiAliasingSamples = samplesUntilNextAA == samplesBetweenAASamples;
                uint currentAAIndex = bakeState.SampleIndex / samplesBetweenAASamples;

                // Work out the super sampling resolution. Its the width of the kernel.
                Debug.Assert(Math.Sqrt(lightmapBakeSettings.AntiAliasingSampleCount) % 1 == 0); // check that AA sample count is a perfect square
                uint superSampleMultiplier = (uint)Math.Sqrt(lightmapBakeSettings.AntiAliasingSampleCount);
                if (superSampleMultiplier * superSampleMultiplier > maxSampleCountPerTexel)
                    superSampleMultiplier = 1; // if the AA sample count is larger than the total sample count, we can't use super sampling

                // Rebuild the GBuffer if necessary
                if (generateAntiAliasingSamples)
                {
                    ExpansionHelpers.GenerateGBuffer(
                        cmd,
                        lightmappingContext.IntegratorContext.GBufferShader,
                        lightmappingContext.GBuffer,
                        traceScratchBuffer,
                        lightmappingContext.IntegratorContext.SamplingResources,
                        uvAS,
                        uvFallbackBuffer,
                        instanceTexelOffset,
                        chunkOffset,
                        chunkSize,
                        lightmapBakeSettings.GetAntiAliasingType(integratedOutputType),
                        currentAAIndex,
                        superSampleMultiplier
                        );
                }
                else
                    Debug.Assert(!newChunkStarted, "When a new chunk starts the Gbuffer needs to be computed for the chunk.");

                // Calculate the maximum number of samples that can be taken in a single pass
                uint remainingSampleCount = math.max(0, maxSampleCountPerTexel - bakeState.SampleIndex);
                UInt64 sampleBudget = expandedSampleWidth;
                UInt64 passSampleCount = math.min(samplesUntilNextAA, math.min(remainingSampleCount, sampleBudget)); // account for the sample budget and when the next AA sample should be taken
                Debug.Assert(passSampleCount > 0);

                // Now take 'passSampleCount' samples
                {
                    uint passSamplesToTake = (uint)passSampleCount;
                    Debug.Assert(bakeState.SampleIndex + passSamplesToTake <= maxSampleCountPerTexel);
                    bool doDirectional = lightmappingContext.AccumulatedDirectionalOutput != null;
                    var ctx = lightmappingContext.IntegratorContext;
                    var expansionShaders = ctx.ExpansionShaders;

                    if (newChunkStarted)
                    {
                        // Compact the GBuffer.
                        ExpansionHelpers.CompactGBuffer(cmd, expansionShaders, ctx.CompactGBufferKernel, lightmappingContext.GBuffer, chunkSize, ctx.CompactedGBufferLength, lightmappingContext.CompactedTexelIndices);
                        // Populate the expanded clear indirect dispatch buffer - using the compacted size.
                        expansionShaders.GetKernelThreadGroupSizes(ctx.ClearBufferKernel, out uint clearThreadGroupSizeX, out uint clearThreadGroupSizeY, out uint clearThreadGroupSizeZ);
                        Debug.Assert(clearThreadGroupSizeY == 1 && clearThreadGroupSizeZ == 1);
                        ExpansionHelpers.PopulateClearExpandedOutputIndirectDispatch(cmd, expansionShaders, ctx.PopulateClearDispatchKernel, clearThreadGroupSizeX, expandedSampleWidth, ctx.CompactedGBufferLength, ctx.ClearDispatchBuffer);
                        // Clear the output buffers.
                        ExpansionHelpers.ClearExpandedOutput(cmd, expansionShaders, ctx.ClearBufferKernel, lightmappingContext.ExpandedOutput, ctx.ClearDispatchBuffer);
                        if (doDirectional)
                            ExpansionHelpers.ClearExpandedOutput(cmd, expansionShaders, ctx.ClearBufferKernel, lightmappingContext.ExpandedDirectional, ctx.ClearDispatchBuffer);
                    }

                    cmd.BeginSample("AccumulateLightmapInstance");
                    GraphicsBuffer expandedDirectional = doDirectional ? lightmappingContext.ExpandedDirectional : null;
                    var instanceGeometryIndex = lightmappingContext.World.PathTracingWorld.GetAccelerationStructure().GeometryPool.GetInstanceGeometryIndex(instance.Mesh);

                    switch (integratedOutputType)
                    {
                        case IntegratedOutputType.AO:
                        {
                            lightmappingContext.IntegratorContext.LightmapAOIntegrator.Accumulate(
                                cmd,
                                passSamplesToTake,
                                bakeState.SampleIndex,
                                instance.LocalToWorldMatrix,
                                instance.LocalToWorldMatrixNormals,
                                instanceGeometryIndex,
                                instance.TexelSize,
                                chunkOffset,
                                lightmappingContext.World.PathTracingWorld,
                                traceScratchBuffer,
                                lightmappingContext.GBuffer,
                                expandedSampleWidth,
                                lightmappingContext.ExpandedOutput,
                                lightmappingContext.CompactedTexelIndices,
                                lightmappingContext.IntegratorContext.CompactedGBufferLength,
                                lightmapBakeSettings.PushOff,
                                lightmapBakeSettings.AOMaxDistance,
                                newChunkStarted
                            );
                            break;
                        }
                        case IntegratedOutputType.Validity:
                        {
                            lightmappingContext.IntegratorContext.LightmapValidityIntegrator.Accumulate(
                                cmd,
                                passSamplesToTake,
                                bakeState.SampleIndex,
                                instance.LocalToWorldMatrix,
                                instance.LocalToWorldMatrixNormals,
                                instanceGeometryIndex,
                                instance.TexelSize,
                                chunkOffset,
                                lightmappingContext.World.PathTracingWorld,
                                traceScratchBuffer,
                                lightmappingContext.GBuffer,
                                expandedSampleWidth,
                                lightmappingContext.ExpandedOutput,
                                lightmappingContext.CompactedTexelIndices,
                                lightmappingContext.IntegratorContext.CompactedGBufferLength,
                                lightmapBakeSettings.PushOff,
                                newChunkStarted
                            );
                            break;
                        }
                        case IntegratedOutputType.Direct:
                        case IntegratedOutputType.DirectionalityDirect:
                        {
                            lightmappingContext.IntegratorContext.LightmapDirectIntegrator.Accumulate(
                                cmd,
                                passSamplesToTake,
                                bakeState.SampleIndex,
                                instance.LocalToWorldMatrix,
                                instance.LocalToWorldMatrixNormals,
                                instanceGeometryIndex,
                                instance.TexelSize,
                                chunkOffset,
                                lightmappingContext.World.PathTracingWorld,
                                traceScratchBuffer,
                                lightmappingContext.GBuffer,
                                expandedSampleWidth,
                                lightmappingContext.ExpandedOutput,
                                expandedDirectional,
                                lightmappingContext.CompactedTexelIndices,
                                lightmappingContext.IntegratorContext.CompactedGBufferLength,
                                instance.ReceiveShadows,
                                lightmapBakeSettings.PushOff,
                                newChunkStarted
                            );
                            break;
                        }
                        case IntegratedOutputType.Indirect:
                        case IntegratedOutputType.DirectionalityIndirect:
                        {
                            lightmappingContext.IntegratorContext.LightmapIndirectIntegrator.Accumulate(
                                cmd,
                                passSamplesToTake,
                                bakeState.SampleIndex,
                                lightmapBakeSettings.BounceCount,
                                instance.LocalToWorldMatrix,
                                instance.LocalToWorldMatrixNormals,
                                instanceGeometryIndex,
                                instance.TexelSize,
                                chunkOffset,
                                lightmappingContext.World.PathTracingWorld,
                                traceScratchBuffer,
                                lightmappingContext.GBuffer,
                                expandedSampleWidth,
                                lightmappingContext.ExpandedOutput,
                                expandedDirectional,
                                lightmappingContext.CompactedTexelIndices,
                                lightmappingContext.IntegratorContext.CompactedGBufferLength,
                                lightmapBakeSettings.PushOff,
                                newChunkStarted
                            );
                            break;
                        }
                        case IntegratedOutputType.ShadowMask:
                        {
                            lightmappingContext.IntegratorContext.LightmapShadowMaskIntegrator.Accumulate(
                                cmd,
                                passSamplesToTake,
                                bakeState.SampleIndex,
                                instance.LocalToWorldMatrix,
                                instance.LocalToWorldMatrixNormals,
                                instanceGeometryIndex,
                                instance.TexelSize,
                                chunkOffset,
                                lightmappingContext.World.PathTracingWorld,
                                traceScratchBuffer,
                                lightmappingContext.GBuffer,
                                expandedSampleWidth,
                                lightmappingContext.ExpandedOutput,
                                expandedDirectional,
                                lightmappingContext.CompactedTexelIndices,
                                lightmappingContext.IntegratorContext.CompactedGBufferLength,
                                instance.ReceiveShadows,
                                lightmapBakeSettings.PushOff,
                                newChunkStarted
                            );
                            break;
                        }
                    }
                    cmd.EndSample("AccumulateLightmapInstance");

                    // Update the baking state
                    bakeState.Tick(
                        passSamplesToTake,
                        maxSampleCountPerTexel,
                        chunkSize,
                        instanceTexelCount,
                        out instanceIsDone,
                        out bool chunkIsDone);

                    if (chunkIsDone)
                    {
                        int maxExpandedDispatchSize = instanceWidth * instanceHeight * (int)expandedSampleWidth;
                        // Gather to lightmap -> first reduce to output resolution
                        // Populate the reduce indirect dispatch buffer - using the compacted size.
                        expansionShaders.GetKernelThreadGroupSizes(ctx.ReductionKernel, out uint reduceThreadGroupSizeX, out uint reduceThreadGroupSizeY, out uint reduceThreadGroupSizeZ);
                        Debug.Assert(reduceThreadGroupSizeY == 1 && reduceThreadGroupSizeZ == 1);
                        ExpansionHelpers.PopulateReduceExpandedOutputIndirectDispatch(cmd, expansionShaders, ctx.PopulateReduceDispatchKernel, reduceThreadGroupSizeX, expandedSampleWidth, ctx.CompactedGBufferLength, ctx.ReduceDispatchBuffer);
                        ExpansionHelpers.ReduceExpandedOutput(cmd, expansionShaders, ctx.ReductionKernel, lightmappingContext.ExpandedOutput, maxExpandedDispatchSize, expandedSampleWidth, ctx.ReduceDispatchBuffer);
                        if (doDirectional)
                            ExpansionHelpers.ReduceExpandedOutput(cmd, expansionShaders, ctx.ReductionKernel, lightmappingContext.ExpandedDirectional, maxExpandedDispatchSize, expandedSampleWidth, ctx.ReduceDispatchBuffer);

                        // Populate the copy indirect dispatch buffer - using the compacted size.
                        expansionShaders.GetKernelThreadGroupSizes(ctx.CopyToLightmapKernel, out uint copyThreadGroupSizeX, out uint copyThreadGroupSizeY, out uint copyThreadGroupSizeZ);
                        Debug.Assert(copyThreadGroupSizeY == 1 && copyThreadGroupSizeZ == 1);
                        ExpansionHelpers.PopulateCopyToLightmapIndirectDispatch(cmd, expansionShaders, ctx.PopulateCopyDispatchKernel, copyThreadGroupSizeX, ctx.CompactedGBufferLength, ctx.CopyDispatchBuffer);
                        ExpansionHelpers.CopyToLightmap(cmd, expansionShaders, ctx.CopyToLightmapKernel, expandedSampleWidth, instanceWidth, instanceTexelOffset, chunkOffset, ctx.CompactedGBufferLength, lightmappingContext.CompactedTexelIndices, lightmappingContext.ExpandedOutput, ctx.CopyDispatchBuffer, lightmappingContext.AccumulatedOutput);
                        if (doDirectional)
                            ExpansionHelpers.CopyToLightmap(cmd, expansionShaders, ctx.CopyToLightmapKernel, expandedSampleWidth, instanceWidth, instanceTexelOffset, chunkOffset, ctx.CompactedGBufferLength, lightmappingContext.CompactedTexelIndices, lightmappingContext.ExpandedDirectional, ctx.CopyDispatchBuffer, lightmappingContext.AccumulatedDirectionalOutput);
                    }
                }

                return (uint)passSampleCount;
            }
        }
    }
}
