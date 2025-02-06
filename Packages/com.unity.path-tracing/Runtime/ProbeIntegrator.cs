using System;
using UnityEngine.LightTransport;
using UnityEngine.PathTracing.Core;
using UnityEngine.Rendering;
using UnityEngine.Rendering.UnifiedRayTracing;
using UnityEngine.Rendering.Sampling;
using UnityEngine.PathTracing.Lightmapping;

namespace UnityEngine.PathTracing.Integration
{
    internal class ProbeIntegrator : IDisposable
    {
        GraphicsBuffer _positionsBuffer;
        SamplingResources _samplingResources;
        ResourceLibrary _resourceLibrary;
        GraphicsBuffer _traceScratchBuffer;
        RTHandle _emptyExposureTexture;
        bool _excludeMeshAndEnvironmentFromDirect;
        bool _countNEERayAsPathSegment;
        private BakeProgressState _progressState;

        // This is a magic number, chosen to be a decent balance between performance and memory usage.
        // This will use at maximum (1024 * 1024) * 27 * 4 bytes = 108 MB of VRAM for the expansion buffer when using SH.
        const uint maxTotalSamplesPerDispatch = 1024 * 1024;

        private static class ShaderProperties
        {
            public static readonly int Positions = Shader.PropertyToID("g_Positions");
            public static readonly int PositionsOffset = Shader.PropertyToID("g_PositionsOffset");
            public static readonly int RadianceShl2 = Shader.PropertyToID("g_RadianceShl2");
            public static readonly int RadianceShl2Offset = Shader.PropertyToID("g_RadianceShl2Offset");
            public static readonly int Validity = Shader.PropertyToID("g_Validity");
            public static readonly int ValidityOffset = Shader.PropertyToID("g_ValidityOffset");
            public static readonly int SampleOffset = Shader.PropertyToID("g_SampleOffset");
            public static readonly int SampleCount = Shader.PropertyToID("g_SampleCount");
            public static readonly int Occlusion = Shader.PropertyToID("g_Occlusion");
            public static readonly int OcclusionOffset = Shader.PropertyToID("g_OcclusionOffset");
            public static readonly int PerProbeLightIndices = Shader.PropertyToID("g_PerProbeLightIndices");
            public static readonly int PerProbeLightIndicesOffset = Shader.PropertyToID("g_PerProbeLightIndicesOffset");
            public static readonly int MaxLightsPerProbe = Shader.PropertyToID("g_MaxLightsPerProbe");
        }

        public class ResourceLibrary
        {
            public IRayTracingShader IndirectShader;
            public IRayTracingShader DirectShader;
            public IRayTracingShader ValidityShader;
            public IRayTracingShader OcclusionShader;
            public SegmentedReduction GatherKernel;

#if UNITY_EDITOR
            public void Load(RayTracingContext context)
            {
                const string packageFolder = "Packages/com.unity.path-tracing/";

                IndirectShader = context.LoadRayTracingShader(packageFolder + "Runtime/Shaders/ProbeIntegrationIndirect.urtshader");
                DirectShader = context.LoadRayTracingShader(packageFolder + "Runtime/Shaders/ProbeIntegrationDirect.urtshader");
                ValidityShader = context.LoadRayTracingShader(packageFolder + "Runtime/Shaders/ProbeIntegrationValidity.urtshader");
                OcclusionShader = context.LoadRayTracingShader(packageFolder + "Runtime/Shaders/ProbeIntegrationOcclusion.urtshader");
                GatherKernel = new(SegmentedReduction.LoadShader());
            }
#endif
        }

        public ProbeIntegrator(bool excludeMeshAndEnvironmentFromDirect, bool countNEERayAsPathSegment)
        {
            _excludeMeshAndEnvironmentFromDirect = excludeMeshAndEnvironmentFromDirect;
            _countNEERayAsPathSegment = countNEERayAsPathSegment;
        }

        internal void Prepare(GraphicsBuffer positionsBuffer, ResourceLibrary resources)
        {
            // First release any previously allocated resources, prepare may be called multiple times
            ReleaseExistingAllocations();

            _positionsBuffer = positionsBuffer;

            _resourceLibrary = resources;

            _samplingResources = new SamplingResources();
            _samplingResources.Load((uint)SamplingResources.ResourceType.All);

            _emptyExposureTexture = RTHandles.Alloc(1, 1, enableRandomWrite: true, name: "Empty EV100 Exposure");
        }

        // We need 2 buffers for expansion and gathering: 1 buffer to store expanded samples, and 1 buffer to use as scratch space for the segmented reduction.
        public static void GetScratchBufferSizesInDwords(uint outputStride, uint positionCount, uint sampleCount, out uint expansionBufferSize, out uint reductionBufferSize)
        {
            uint maxProbesPerDispatch = maxTotalSamplesPerDispatch / sampleCount;
            expansionBufferSize = Math.Min(maxTotalSamplesPerDispatch, positionCount * sampleCount) * outputStride;
            reductionBufferSize = SegmentedReduction.GetScratchBufferSizeInDwords(sampleCount, outputStride, Math.Min(maxProbesPerDispatch, positionCount));
        }

        public static void GetRadianceScratchBufferSizesInDwords(uint positionCount, uint sampleCount, out uint expansionBufferSize, out uint reductionBufferSize)
            => GetScratchBufferSizesInDwords(27, positionCount, sampleCount, out expansionBufferSize, out reductionBufferSize);

        public static void GetValidityScratchBufferSizesInDwords(uint positionCount, uint sampleCount, out uint expansionBufferSize, out uint reductionBufferSize)
            => GetScratchBufferSizesInDwords(1, positionCount, sampleCount, out expansionBufferSize, out reductionBufferSize);

        public static void GetOcclusionScratchBufferSizesInDwords(uint maxLightsPerProbe, uint positionCount, uint sampleCount, out uint expansionBufferSize, out uint reductionBufferSize)
            => GetScratchBufferSizesInDwords(maxLightsPerProbe, positionCount, sampleCount, out expansionBufferSize, out reductionBufferSize);

        private void DispatchRadianceEstimationKernel(
            CommandBuffer cmd,
            IRayTracingShader shader,
            World world,
            uint positionOffset,
            uint positionCount,
            uint bounceCount,
            uint sampleOffset,
            uint sampleCount,
            float environmentIntensityMultiplier,
            GraphicsBuffer radianceShl2,
            uint radianceOffset,
            GraphicsBuffer expansionBuffer,
            GraphicsBuffer reductionBuffer)
        {
            Debug.Assert(world.GetAccelerationStructure() != null);

            // General path tracing parameters
            uint lightEvaluations = 1;
            bool preExpose = false;
            Util.BindPathTracingInputs(cmd, shader, _countNEERayAsPathSegment, lightEvaluations, preExpose, (int)bounceCount, environmentIntensityMultiplier, RenderedGameObjectsFilter.OnlyStatic, _samplingResources, _emptyExposureTexture);
            Util.BindWorld(cmd, shader, world, 32);

            // Zero initialize the output buffer
            const uint floatsPerSH = 27;
            cmd.SetBufferData(radianceShl2, new float[positionCount * floatsPerSH]);

            DispatchProbeKernel(cmd, shader, positionOffset, positionCount, sampleOffset, sampleCount, floatsPerSH, ShaderProperties.RadianceShl2, radianceShl2, radianceOffset, expansionBuffer, reductionBuffer,
                bounceCount);
        }

        private void DispatchProbeKernel(
            CommandBuffer cmd,
            IRayTracingShader shader,
            uint positionOffset,
            uint positionCount,
            uint sampleOffset,
            uint sampleCount,
            uint outputStride,
            int outputBufferPropertyID,
            GraphicsBuffer outputBuffer,
            uint outputOffset,
            GraphicsBuffer expansionBuffer,
            GraphicsBuffer reductionBuffer,
            uint bounceCount)
        {
            // Set constant kernel parameters
            shader.SetBufferParam(cmd, ShaderProperties.Positions, _positionsBuffer);
            shader.SetIntParam(cmd, ShaderProperties.SampleCount, (int)sampleCount);
            shader.SetBufferParam(cmd, outputBufferPropertyID, expansionBuffer);

            uint totalSamples = positionCount * sampleCount;
            RayTracingHelper.ResizeScratchBufferForTrace(shader, Math.Min(maxTotalSamplesPerDispatch, totalSamples), 1, 1, ref _traceScratchBuffer);

            uint maxProbesPerDispatch = Math.Max(maxTotalSamplesPerDispatch / sampleCount, 1);

            // This outer loop is only here to handle the case where sampleCount > maxTotalSamplesPerDispatch,
            // which is possible, but extremely unlikely. In this case, we just run the entire integation loop multiple times.
            for (uint sampleWindow = 0; sampleWindow < sampleCount; sampleWindow += maxTotalSamplesPerDispatch)
            {
                shader.SetIntParam(cmd, ShaderProperties.SampleOffset, (int)sampleOffset + (int)sampleWindow);

                // Loop over chunks of probes, calculate all samples for each chunk.
                for (uint probeOffset = 0; probeOffset < positionCount; probeOffset += maxProbesPerDispatch)
                {
                    shader.SetIntParam(cmd, ShaderProperties.PositionsOffset, (int)positionOffset + (int)probeOffset);

                    // Calculate as many samples as possible given the budget
                    uint probesToDispatch = Math.Min(maxProbesPerDispatch, positionCount - probeOffset);
                    uint samplesToDispatch = probesToDispatch * sampleCount;
                    shader.Dispatch(cmd, _traceScratchBuffer, samplesToDispatch, 1, 1);

                    // Perform reduction of each probes samples
                    _resourceLibrary.GatherKernel.TwoPassSegmentedReduction(
                        cmd,
                        sampleCount,
                        outputStride,
                        probesToDispatch,
                        0,
                        outputOffset + probeOffset,
                        expansionBuffer,
                        reductionBuffer,
                        outputBuffer,
                        false);

                    // Chip-off work steps based on the current request
                    ulong workStepsForThisRequest = CalculateWorkSteps(probesToDispatch, sampleCount, bounceCount);
                    // We need some resource in order to be able to get an async readback, outputBuffer is as good a candidate as any other
                    cmd.RequestAsyncReadback(outputBuffer, 1, 0, _ => { _progressState.IncrementCompletedWorkSteps(workStepsForThisRequest); });

                    GraphicsHelpers.Flush(cmd);
                }
            }
        }
        internal static ulong CalculateWorkSteps(uint probesCount, uint sampleCount, uint bounceCount) => probesCount*sampleCount*(0 == bounceCount ? 1 : bounceCount);

        internal void EstimateIndirectRadianceShl2(
            CommandBuffer cmd,
            World world,
            uint positionOffset,
            uint positionCount,
            uint bounceCount,
            uint sampleOffset,
            uint sampleCount,
            bool ignoreEnvironment,
            GraphicsBuffer radianceShl2,
            uint radianceOffset,
            GraphicsBuffer expansionBuffer,
            GraphicsBuffer reductionBuffer)
        {
            float environmentIntensityMultiplier = ignoreEnvironment ? 0.0f : 1.0f;
            DispatchRadianceEstimationKernel(cmd, _resourceLibrary.IndirectShader, world, positionOffset, positionCount, bounceCount, sampleOffset, sampleCount, environmentIntensityMultiplier, radianceShl2, radianceOffset, expansionBuffer, reductionBuffer);
        }

        internal void EstimateDirectRadianceShl2(
            CommandBuffer cmd,
            World world,
            uint positionOffset,
            uint positionCount,
            uint sampleOffset,
            uint sampleCount,
            GraphicsBuffer radianceShl2,
            uint radianceOffset,
            GraphicsBuffer expansionBuffer,
            GraphicsBuffer reductionBuffer)
        {
            _resourceLibrary.DirectShader.SetIntParam(cmd, Shader.PropertyToID("g_ExcludeMeshAndEnvironment"), _excludeMeshAndEnvironmentFromDirect ? 1 : 0);
            float environmentIntensityMultiplier = 1.0f;
            DispatchRadianceEstimationKernel(cmd, _resourceLibrary.DirectShader, world, positionOffset, positionCount, 0, sampleOffset, sampleCount, environmentIntensityMultiplier, radianceShl2, radianceOffset, expansionBuffer, reductionBuffer);
        }

        internal void EstimateValidity(
            CommandBuffer cmd,
            World world,
            uint positionOffset,
            uint positionCount,
            uint sampleOffset,
            uint sampleCount,
            GraphicsBuffer validity,
            uint validityOffset,
            GraphicsBuffer expansionBuffer,
            GraphicsBuffer reductionBuffer)
        {
            var validityShader = _resourceLibrary.ValidityShader;

            // General path tracing parameters
            Util.BindAccelerationStructure(cmd, validityShader, world.GetAccelerationStructure());
            Util.BindMaterialsAndTextures(cmd, validityShader, world);
            SamplingResources.BindSobolBlueNoiseTextures(cmd, _samplingResources);

            DispatchProbeKernel(cmd, validityShader, positionOffset, positionCount, sampleOffset, sampleCount, 1, ShaderProperties.Validity, validity, validityOffset, expansionBuffer, reductionBuffer,
                0);
        }

        internal void EstimateLightOcclusion(
            CommandBuffer cmd,
            World world,
            uint positionOffset,
            uint positionCount,
            uint sampleOffset,
            uint sampleCount,
            uint maxLightsPerProbe,
            GraphicsBuffer perProbeLightIndices,
            uint perProbeLightIndicesOffset,
            GraphicsBuffer occlusion,
            uint occlusionOffset,
            GraphicsBuffer expansionBuffer,
            GraphicsBuffer reductionBuffer)
        {
            var occlusionShader = _resourceLibrary.OcclusionShader;

            // General path tracing parameters
            Util.BindWorld(cmd, occlusionShader, world, 32);
            SamplingResources.BindSobolBlueNoiseTextures(cmd, _samplingResources);

            occlusionShader.SetBufferParam(cmd, ShaderProperties.PerProbeLightIndices, perProbeLightIndices);
            occlusionShader.SetIntParam(cmd, ShaderProperties.PerProbeLightIndicesOffset, (int)perProbeLightIndicesOffset);
            occlusionShader.SetIntParam(cmd, ShaderProperties.MaxLightsPerProbe, (int)maxLightsPerProbe);

            DispatchProbeKernel(cmd, occlusionShader, positionOffset, positionCount, sampleOffset, sampleCount, maxLightsPerProbe, ShaderProperties.Occlusion, occlusion, occlusionOffset, expansionBuffer, reductionBuffer,
                0);
        }

        private void ReleaseExistingAllocations()
        {
            _emptyExposureTexture?.Release();
            _traceScratchBuffer?.Dispose();
            _traceScratchBuffer = null;
            _samplingResources?.Dispose();
            _samplingResources = null;
        }

        public void Dispose()
        {
            ReleaseExistingAllocations();
        }

        public void SetProgressReporter(BakeProgressState progressState) => _progressState = progressState;
    }
}
