using Unity.Mathematics;
using UnityEngine.PathTracing.Core;
using UnityEngine.PathTracing.Integration;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Sampling;
using UnityEngine.Rendering.UnifiedRayTracing;
using static UnityEngine.PathTracing.Lightmapping.LightmapIntegrationHelpers;

namespace UnityEngine.PathTracing.Lightmapping
{
    internal static class ExpansionShaderIDs
    {
        public static readonly int GBuffer = Shader.PropertyToID("g_GBuffer");
        public static readonly int DestinationTexture = Shader.PropertyToID("g_DestinationTexture");
        public static readonly int DestinationX = Shader.PropertyToID("g_DestinationX");
        public static readonly int DestinationY = Shader.PropertyToID("g_DestinationY");
        public static readonly int ExpandedTexelSampleWidth = Shader.PropertyToID("g_ExpandedTexelSampleWidth");
        public static readonly int Float4Buffer = Shader.PropertyToID("g_Float4Buffer");
        public static readonly int Float4BufferLength = Shader.PropertyToID("g_Float4BufferLength");
        public static readonly int BinaryBufferSize = Shader.PropertyToID("g_BinaryBufferSize");
        public static readonly int BinaryGroupSize = Shader.PropertyToID("g_BinaryGroupSize");
        public static readonly int SourceBuffer = Shader.PropertyToID("g_SourceBuffer");
        public static readonly int SourceStride = Shader.PropertyToID("g_SourceStride");
        public static readonly int GBufferLength = Shader.PropertyToID("g_GBufferLength");
        public static readonly int CompactedGBuffer = Shader.PropertyToID("g_CompactedGBuffer");
        public static readonly int CompactedGBufferLength = Shader.PropertyToID("g_CompactedGBufferLength");
        public static readonly int InstanceWidth = Shader.PropertyToID("g_InstanceWidth");
        public static readonly int ThreadGroupSizeX = Shader.PropertyToID("g_ThreadGroupSizeX");
        public static readonly int AccumulationDispatchBuffer = Shader.PropertyToID("g_AccumulationDispatchBuffer");
        public static readonly int ClearDispatchBuffer = Shader.PropertyToID("g_ClearDispatchBuffer");
        public static readonly int CopyDispatchBuffer = Shader.PropertyToID("g_CopyDispatchBuffer");
        public static readonly int ReduceDispatchBuffer = Shader.PropertyToID("g_ReduceDispatchBuffer");
        public static readonly int ChunkOffsetX = Shader.PropertyToID("g_ChunkOffsetX");
        public static readonly int ChunkOffsetY = Shader.PropertyToID("g_ChunkOffsetY");
    }

    internal static class ExpansionHelpers
    {
        static internal int PopulateAccumulationIndirectDispatch(CommandBuffer cmd, IRayTracingShader accumulationShader, ComputeShader populateShader, int populateKernel, uint expandedSampleWidth, GraphicsBuffer compactedGbufferLength, GraphicsBuffer accumulationDispatchBuffer)
        {
            cmd.SetComputeIntParam(populateShader, ExpansionShaderIDs.ExpandedTexelSampleWidth, (int)expandedSampleWidth);
            cmd.SetComputeBufferParam(populateShader, populateKernel, ExpansionShaderIDs.CompactedGBufferLength, compactedGbufferLength);
            cmd.SetComputeBufferParam(populateShader, populateKernel, ExpansionShaderIDs.AccumulationDispatchBuffer, accumulationDispatchBuffer);
            cmd.DispatchCompute(populateShader, populateKernel, 1, 1, 1);

            //LightmapIntegrationHelpers.LogGraphicsBuffer(cmd, accumulationDispatchBuffer, "accumulationDispatchBuffer");
            return 1;
        }

        static internal int PopulateClearExpandedOutputIndirectDispatch(CommandBuffer cmd, ComputeShader populateClearDispatch, int populateClearDispatchKernel, uint clearThreadGroupSizeX, uint expandedSampleWidth, GraphicsBuffer compactedGBufferLength, GraphicsBuffer clearDispatchBuffer)
        {
            // Populate the expanded clear indirect dispatch buffer - using the compacted size.
            cmd.SetComputeIntParam(populateClearDispatch, ExpansionShaderIDs.ExpandedTexelSampleWidth, (int)expandedSampleWidth);
            cmd.SetComputeIntParam(populateClearDispatch, ExpansionShaderIDs.ThreadGroupSizeX, (int)clearThreadGroupSizeX);
            cmd.SetComputeBufferParam(populateClearDispatch, populateClearDispatchKernel, ExpansionShaderIDs.CompactedGBufferLength, compactedGBufferLength);
            cmd.SetComputeBufferParam(populateClearDispatch, populateClearDispatchKernel, ExpansionShaderIDs.ClearDispatchBuffer, clearDispatchBuffer);
            cmd.DispatchCompute(populateClearDispatch, populateClearDispatchKernel, 1, 1, 1);
            //LightmapIntegrationHelpers.LogGraphicsBuffer(cmd, clearDispatchBuffer, "_clearDispatchBuffer");
            return 1;
        }

        static internal int ClearExpandedOutput(CommandBuffer cmd, ComputeShader clearExpandedOutput, int clearExpandedOutputKernel, GraphicsBuffer expandedOutput, GraphicsBuffer clearDispatchBuffer)
        {
            Debug.Assert(expandedOutput.stride == 16);
            // Clear the output buffers.
            cmd.SetComputeIntParam(clearExpandedOutput, ExpansionShaderIDs.Float4BufferLength, expandedOutput.count);
            cmd.SetComputeBufferParam(clearExpandedOutput, clearExpandedOutputKernel, ExpansionShaderIDs.Float4Buffer, expandedOutput);
            cmd.BeginSample("Clear (Expanded)");
            cmd.DispatchCompute(clearExpandedOutput, clearExpandedOutputKernel, clearDispatchBuffer, 0);
            cmd.EndSample("Clear (Expanded)");
            return 1;
        }

        static internal void GenerateGBuffer(
            CommandBuffer cmd,
            IRayTracingShader gBufferShader,
            GraphicsBuffer gBuffer,
            GraphicsBuffer traceScratchBuffer,
            SamplingResources samplingResources,
            UVAccelerationStructure uvAS,
            UVFallbackBuffer uvFallbackBuffer,
            Vector2Int instanceTexelOffset,
            uint2 chunkOffset,
            uint chunkSize,
            AntiAliasingType aaType,
            uint currentAAIndex,
            uint superSampleMultiplier)
        {
            var stochasticAntiAliasing = aaType == AntiAliasingType.Stochastic;

            // bind buffers
            gBufferShader.SetAccelerationStructure(cmd, "g_UVAccelStruct", uvAS._uvAS);
            gBufferShader.SetBufferParam(cmd, LightmapIntegratorShaderIDs.GBuffer, gBuffer);
            SamplingResources.BindSobolBlueNoiseTextures(cmd, samplingResources);
            uvFallbackBuffer.BindChunked(cmd, gBufferShader, instanceTexelOffset, chunkOffset, chunkSize);

            // set antialiasing parameters
            var ssParams = stochasticAntiAliasing ? new float2(0.5f, 0.5f) : SuperSamplingHelpers.GetSuperSamplingOffset(currentAAIndex, superSampleMultiplier);
            SuperSamplingHelpers.BindSuperSamplingParameters(cmd, gBufferShader, ssParams, stochasticAntiAliasing);
            gBufferShader.SetIntParam(cmd, LightmapIntegratorShaderIDs.AASampleOffset, (int)currentAAIndex);

            // dispatch the shader
            Debug.Assert(gBufferShader is HardwareRayTracingShader || GraphicsHelpers.DivUp((int)chunkSize, gBufferShader.GetThreadGroupSizes().x) < 65536, "Chunk size is too large for the shader to handle.");
            cmd.BeginSample("UV Sampling");
            gBufferShader.Dispatch(cmd, traceScratchBuffer, chunkSize, 1, 1);
            cmd.EndSample("UV Sampling");
        }

        static internal int CompactGBuffer(CommandBuffer cmd, ComputeShader compactGBuffer, int compactGBufferKernel, GraphicsBuffer gBuffer, uint chunkSize, GraphicsBuffer compactedGBufferLength, GraphicsBuffer compactedTexelIndices)
        {
            compactGBuffer.GetKernelThreadGroupSizes(compactGBufferKernel, out uint gbuf_x, out _, out _);
            cmd.SetBufferData(compactedGBufferLength, new uint[] { 0 });
            cmd.SetComputeIntParam(compactGBuffer, ExpansionShaderIDs.GBufferLength, (int)chunkSize);
            cmd.SetComputeBufferParam(compactGBuffer, compactGBufferKernel, ExpansionShaderIDs.GBuffer, gBuffer);
            cmd.SetComputeBufferParam(compactGBuffer, compactGBufferKernel, ExpansionShaderIDs.CompactedGBuffer, compactedTexelIndices);
            cmd.SetComputeBufferParam(compactGBuffer, compactGBufferKernel, ExpansionShaderIDs.CompactedGBufferLength, compactedGBufferLength);
            cmd.BeginSample("Compact GBuffer");
            cmd.DispatchCompute(compactGBuffer, compactGBufferKernel, GraphicsHelpers.DivUp((int)chunkSize, gbuf_x), 1, 1);
            cmd.EndSample("Compact GBuffer");
            //LightmapIntegrationHelpers.LogGraphicsBuffer(cmd, compactedGBufferLength, "compactedGBufferLength");
            //LightmapIntegrationHelpers.LogGraphicsBuffer(cmd, compactedTexelIndices, "compactedTexelIndices");
            return 1;
        }

        static internal int PopulateReduceExpandedOutputIndirectDispatch(CommandBuffer cmd, ComputeShader populateReduceExpandedOutput, int populateReduceExpandedOutputKernel, uint reduceThreadGroupSizeX, uint expandedSampleWidth, GraphicsBuffer compactedGBufferLength, GraphicsBuffer reduceDispatchBuffer)
        {
            // Populate the reduce copy indirect dispatch buffer - using the compacted size.
            cmd.SetComputeIntParam(populateReduceExpandedOutput, ExpansionShaderIDs.ExpandedTexelSampleWidth, (int)expandedSampleWidth);
            cmd.SetComputeIntParam(populateReduceExpandedOutput, ExpansionShaderIDs.ThreadGroupSizeX, (int)reduceThreadGroupSizeX);
            cmd.SetComputeBufferParam(populateReduceExpandedOutput, populateReduceExpandedOutputKernel, ExpansionShaderIDs.CompactedGBufferLength, compactedGBufferLength);
            cmd.SetComputeBufferParam(populateReduceExpandedOutput, populateReduceExpandedOutputKernel, ExpansionShaderIDs.ReduceDispatchBuffer, reduceDispatchBuffer);
            cmd.DispatchCompute(populateReduceExpandedOutput, populateReduceExpandedOutputKernel, 1, 1, 1);
            //LightmapIntegrationHelpers.LogGraphicsBuffer(cmd, reduceDispatchBuffer, "reduceDispatchBuffer");
            return 1;
        }

        static internal int ReduceExpandedOutput(CommandBuffer cmd, ComputeShader binaryGroupSumLeftShader, int binaryGroupSumLeftKernel, GraphicsBuffer expandedOutput, int expandedDispatchSize, uint expandedSampleWidth, GraphicsBuffer reduceDispatch)
        {
            Debug.Assert(math.ispow2(expandedSampleWidth));

            // the expandedOutput buffer contains groups of expandedSampleWidth samples
            // do binary summation within the sample groups ending up with summed samples in the leftmost sample for each texel
            cmd.SetComputeBufferParam(binaryGroupSumLeftShader, binaryGroupSumLeftKernel, ExpansionShaderIDs.Float4Buffer, expandedOutput);
            cmd.SetComputeIntParam(binaryGroupSumLeftShader, ExpansionShaderIDs.BinaryBufferSize, expandedDispatchSize);
            int groupSize = 1;
            int dispatches = 0;
            while (groupSize < expandedSampleWidth)
            {
                cmd.SetComputeIntParam(binaryGroupSumLeftShader, ExpansionShaderIDs.BinaryGroupSize, groupSize);
                cmd.BeginSample("Binary Sum");
                cmd.DispatchCompute(binaryGroupSumLeftShader, binaryGroupSumLeftKernel, reduceDispatch, 0);
                cmd.EndSample("Binary Sum");
                groupSize *= 2;
                dispatches++;
            }
            return dispatches;
        }


        static internal int PopulateCopyToLightmapIndirectDispatch(CommandBuffer cmd, ComputeShader populateCopyToLightmap, int populateCopyToLightmapKernel, uint copyThreadGroupSizeX, GraphicsBuffer compactedGBufferLength, GraphicsBuffer copyDispatch)
        {
            // Populate the expanded copy indirect dispatch buffer - using the compacted size.
            cmd.SetComputeIntParam(populateCopyToLightmap, ExpansionShaderIDs.ThreadGroupSizeX, (int)copyThreadGroupSizeX);
            cmd.SetComputeBufferParam(populateCopyToLightmap, populateCopyToLightmapKernel, ExpansionShaderIDs.CompactedGBufferLength, compactedGBufferLength);
            cmd.SetComputeBufferParam(populateCopyToLightmap, populateCopyToLightmapKernel, ExpansionShaderIDs.CopyDispatchBuffer, copyDispatch);
            cmd.DispatchCompute(populateCopyToLightmap, populateCopyToLightmapKernel, 1, 1, 1);
            //LightmapIntegrationHelpers.LogGraphicsBuffer(cmd, copyDispatch, "copyDispatch");
            return 1;
        }

        static internal int CopyToLightmap(CommandBuffer cmd, ComputeShader copyToLightmap, int copyToLightmapKernel, uint expandedSampleWidth, int instanceWidth, Vector2Int instanceTexelOffset, uint2 chunkOffset, GraphicsBuffer compactedGBufferLength, GraphicsBuffer compactedTexelIndices, GraphicsBuffer expandedOutput, GraphicsBuffer copyDispatch, RenderTexture output)
        {
            cmd.SetComputeBufferParam(copyToLightmap, copyToLightmapKernel, ExpansionShaderIDs.SourceBuffer, expandedOutput);
            cmd.SetComputeIntParam(copyToLightmap, ExpansionShaderIDs.SourceStride, (int)expandedSampleWidth);
            cmd.SetComputeTextureParam(copyToLightmap, copyToLightmapKernel, ExpansionShaderIDs.DestinationTexture, output);
            cmd.SetComputeBufferParam(copyToLightmap, copyToLightmapKernel, ExpansionShaderIDs.CompactedGBuffer, compactedTexelIndices);
            cmd.SetComputeBufferParam(copyToLightmap, copyToLightmapKernel, ExpansionShaderIDs.CompactedGBufferLength, compactedGBufferLength);
            cmd.SetComputeIntParam(copyToLightmap, ExpansionShaderIDs.InstanceWidth, instanceWidth);
            cmd.SetComputeIntParam(copyToLightmap, ExpansionShaderIDs.DestinationX, instanceTexelOffset.x);
            cmd.SetComputeIntParam(copyToLightmap, ExpansionShaderIDs.DestinationY, instanceTexelOffset.y);
            cmd.SetComputeIntParam(copyToLightmap, ExpansionShaderIDs.ChunkOffsetX, (int)chunkOffset.x);
            cmd.SetComputeIntParam(copyToLightmap, ExpansionShaderIDs.ChunkOffsetY, (int)chunkOffset.y);

            copyToLightmap.GetKernelThreadGroupSizes(copyToLightmapKernel, out uint copyx, out uint _, out _);
            cmd.BeginSample("Copy to Lightmap");
            cmd.DispatchCompute(copyToLightmap, copyToLightmapKernel, copyDispatch, 0);
            cmd.EndSample("Copy to Lightmap");
            return 1;
        }
    }
}
