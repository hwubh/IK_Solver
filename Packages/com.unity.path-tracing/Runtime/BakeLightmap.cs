using System;
using Unity.Mathematics;
using UnityEngine.Experimental.Rendering;
using UnityEngine.PathTracing.Core;
using UnityEngine.PathTracing.Integration;
using UnityEngine.Rendering;
using UnityEngine.Rendering.UnifiedRayTracing;
using UnityEngine.Rendering.Sampling;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace UnityEngine.PathTracing.Lightmapping
{
    internal class Instance
    {
        public Mesh Mesh;
        public Vector4 NormalizedOccupiedST; // Transforms coordinates in [0; 1] range into the occupied rectangle in the lightmap atlas, also in [0; 1] range.
        public Vector2Int TexelSize; // Instance size in the lightmap atlas, in pixels.
        public Vector2Int TexelOffset; // Instance offset in the lightmap atlas, in pixels.
        public Matrix4x4 LocalToWorldMatrix;
        public Matrix4x4 LocalToWorldMatrixNormals;
        public bool ReceiveShadows;
        public LodsIdentifier LodIdentifier;

        private static float4x4 NormalMatrix(float4x4 m)
        {
            float3x3 t = new float3x3(m);
            return new float4x4(math.inverse(math.transpose(t)), new float3(0.0));
        }

        public void Build(Mesh mesh, Vector4 normalizedOccupiedST, Vector2Int texelSize, Vector2Int texelOffset, Matrix4x4 localToWorldMatrix, bool receiveShadows, LodsIdentifier lodIdentifier)
        {
            Mesh = mesh;
            NormalizedOccupiedST = normalizedOccupiedST;
            TexelSize = texelSize;
            TexelOffset = texelOffset;
            LocalToWorldMatrix = localToWorldMatrix;
            ReceiveShadows = receiveShadows;
            LocalToWorldMatrixNormals = NormalMatrix(this.LocalToWorldMatrix);
            LodIdentifier = lodIdentifier;
        }
    }

    internal class LightmapDesc
    {
        public uint Resolution;
        public uint AntiAliasingSampleCount;
        public float PushOff;
        public Instance[] Instances;
    }

    internal enum IntegratedOutputType
    {
        Direct,
        Indirect,
        AO,
        Validity,
        DirectionalityDirect,
        DirectionalityIndirect,
        ShadowMask
    }

    internal class LightmapIntegratorContext : IDisposable
    {
        internal UVFallbackBufferBuilder UVFallbackBufferBuilder;
        internal LightmapDirectIntegrator LightmapDirectIntegrator;
        internal LightmapIndirectIntegrator LightmapIndirectIntegrator;
        internal LightmapAOIntegrator LightmapAOIntegrator;
        internal LightmapValidityIntegrator LightmapValidityIntegrator;
        internal LightmapOccupancyIntegrator LightmapOccupancyIntegrator;
        internal LightmapShadowMaskIntegrator LightmapShadowMaskIntegrator;
        internal IRayTracingShader GBufferShader;
        internal ComputeShader ExpansionShaders;
        internal SamplingResources SamplingResources;
        private RTHandle _emptyExposureTexture;
        internal GraphicsBuffer ClearDispatchBuffer;
        internal GraphicsBuffer CopyDispatchBuffer;
        internal GraphicsBuffer ReduceDispatchBuffer;
        internal GraphicsBuffer CompactedGBufferLength;
        internal int CompactGBufferKernel;
        internal int PopulateAccumulationDispatchKernel;
        internal int PopulateClearDispatchKernel;
        internal int PopulateCopyDispatchKernel;
        internal int PopulateReduceDispatchKernel;
        internal int ClearBufferKernel;
        internal int ReductionKernel;
        internal int CopyToLightmapKernel;

        public void Dispose()
        {
            UVFallbackBufferBuilder?.Dispose();
            UVFallbackBufferBuilder = null;
            LightmapDirectIntegrator?.Dispose();
            LightmapDirectIntegrator = null;
            LightmapIndirectIntegrator?.Dispose();
            LightmapIndirectIntegrator = null;
            LightmapAOIntegrator?.Dispose();
            LightmapAOIntegrator = null;
            LightmapValidityIntegrator?.Dispose();
            LightmapValidityIntegrator = null;
            LightmapOccupancyIntegrator = null;
            LightmapShadowMaskIntegrator?.Dispose();
            LightmapShadowMaskIntegrator = null;
            _emptyExposureTexture?.Release();
            _emptyExposureTexture = null;

            SamplingResources?.Dispose();
            SamplingResources = null;

            ClearDispatchBuffer?.Dispose();
            CopyDispatchBuffer?.Dispose();
            ReduceDispatchBuffer?.Dispose();
            CompactedGBufferLength?.Dispose();
        }

        internal void Initialize(LightmapResourceLibrary resources, bool excludeMeshAndEnvironmentFromDirect, bool countNEERayAsPathSegment)
        {
            SamplingResources = new SamplingResources();
            SamplingResources.Load((uint)SamplingResources.ResourceType.All);
            _emptyExposureTexture = RTHandles.Alloc(1, 1, enableRandomWrite: true, name: "Empty EV100 Exposure", colorFormat: GraphicsFormat.R8G8B8A8_UNorm);

            UVFallbackBufferBuilder = new UVFallbackBufferBuilder();
            UVFallbackBufferBuilder.Prepare(resources.UVFallbackBufferGenerationMaterial);
            LightmapDirectIntegrator = new LightmapDirectIntegrator(excludeMeshAndEnvironmentFromDirect);
            LightmapDirectIntegrator.Prepare(resources.DirectAccumulationShader, resources.NormalizationShader, resources.ExpansionHelpers, SamplingResources, _emptyExposureTexture);
            LightmapIndirectIntegrator = new LightmapIndirectIntegrator(countNEERayAsPathSegment);
            LightmapIndirectIntegrator.Prepare(resources.IndirectAccumulationShader, resources.NormalizationShader, resources.ExpansionHelpers, SamplingResources, _emptyExposureTexture);
            LightmapAOIntegrator = new LightmapAOIntegrator();
            LightmapAOIntegrator.Prepare(resources.AOAccumulationShader, resources.NormalizationShader, resources.ExpansionHelpers, SamplingResources, _emptyExposureTexture);
            LightmapValidityIntegrator = new LightmapValidityIntegrator();
            LightmapValidityIntegrator.Prepare(resources.ValidityAccumulationShader, resources.NormalizationShader, resources.ExpansionHelpers, SamplingResources, _emptyExposureTexture);
            LightmapOccupancyIntegrator = new LightmapOccupancyIntegrator();
            LightmapOccupancyIntegrator.Prepare(resources.OccupancyShader);
			LightmapShadowMaskIntegrator = new LightmapShadowMaskIntegrator(excludeMeshAndEnvironmentFromDirect);
            LightmapShadowMaskIntegrator.Prepare(resources.ShadowMaskAccumulationShader, resources.NormalizationShader, resources.ExpansionHelpers, SamplingResources, _emptyExposureTexture);
            GBufferShader = resources.GBufferShader;
            ExpansionShaders = resources.ExpansionHelpers;

            CompactGBufferKernel = ExpansionShaders.FindKernel("CompactGBuffer");
            PopulateAccumulationDispatchKernel = ExpansionShaders.FindKernel("PopulateAccumulationDispatch");
            PopulateClearDispatchKernel = ExpansionShaders.FindKernel("PopulateClearDispatch");
            PopulateCopyDispatchKernel = ExpansionShaders.FindKernel("PopulateCopyDispatch");
            PopulateReduceDispatchKernel = ExpansionShaders.FindKernel("PopulateReduceDispatch");
            ClearBufferKernel = ExpansionShaders.FindKernel("ClearFloat4Buffer");
            ReductionKernel = ExpansionShaders.FindKernel("BinaryGroupSumLeft");
            CopyToLightmapKernel = ExpansionShaders.FindKernel("AdditivelyCopyCompactedTo2D");

            ClearDispatchBuffer = new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments | GraphicsBuffer.Target.Structured | GraphicsBuffer.Target.CopySource | GraphicsBuffer.Target.CopyDestination, 3, sizeof(uint));
            CopyDispatchBuffer = new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments | GraphicsBuffer.Target.Structured | GraphicsBuffer.Target.CopySource | GraphicsBuffer.Target.CopyDestination, 3, sizeof(uint));
            ReduceDispatchBuffer = new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments | GraphicsBuffer.Target.Structured | GraphicsBuffer.Target.CopySource | GraphicsBuffer.Target.CopyDestination, 3, sizeof(uint));
            CompactedGBufferLength = new GraphicsBuffer(GraphicsBuffer.Target.Structured | GraphicsBuffer.Target.CopySource, 1, sizeof(uint));
        }
    }

    internal class LightmapResourceLibrary
    {
        internal IRayTracingShader GBufferShader;
        internal ComputeShader NormalizationShader;
        internal IRayTracingShader DirectAccumulationShader;
        internal IRayTracingShader AOAccumulationShader;
        internal IRayTracingShader ValidityAccumulationShader;
        internal IRayTracingShader IndirectAccumulationShader;
        internal IRayTracingShader ShadowMaskAccumulationShader;
        internal IRayTracingShader NormalAccumulationShader;
        internal Material UVFallbackBufferGenerationMaterial;
        internal ComputeShader OccupancyShader;
        internal LightmapIntegrationHelpers.ComputeHelpers ComputeHelpers;
        internal ComputeShader BoxFilterShader;
        internal ComputeShader SelectGraphicsBufferShader;
        internal ComputeShader CopyTextureAdditiveShader;
        internal ComputeShader ExpansionHelpers;
        internal Shader SoftwareChartRasterizationShader;
        internal Shader HardwareChartRasterizationShader;

#if UNITY_EDITOR
        public void Load(RayTracingContext context)
        {
            const string packageFolder = "Packages/com.unity.path-tracing/";

            GBufferShader = context.LoadRayTracingShader(packageFolder + "Runtime/Shaders/LightmapGBufferIntegration.urtshader");

            NormalizationShader = AssetDatabase.LoadAssetAtPath<ComputeShader>(packageFolder + "Runtime/Shaders/ResolveAccumulation.compute");
            DirectAccumulationShader = context.LoadRayTracingShader(packageFolder + "Runtime/Shaders/LightmapDirectIntegration.urtshader");
            AOAccumulationShader = context.LoadRayTracingShader(packageFolder + "Runtime/Shaders/LightmapAOIntegration.urtshader");
            ValidityAccumulationShader = context.LoadRayTracingShader(packageFolder + "Runtime/Shaders/LightmapValidityIntegration.urtshader");
            IndirectAccumulationShader = context.LoadRayTracingShader(packageFolder + "Runtime/Shaders/LightmapIndirectIntegration.urtshader");
            ShadowMaskAccumulationShader = context.LoadRayTracingShader(packageFolder + "Runtime/Shaders/LightmapShadowMaskIntegration.urtshader");

            NormalAccumulationShader = context.LoadRayTracingShader(packageFolder + "Runtime/Shaders/LightmapNormalIntegration.urtshader");

            UVFallbackBufferGenerationMaterial = new Material(AssetDatabase.LoadAssetAtPath<Shader>(packageFolder + "Runtime/Shaders/Lightmapping/UVFallbackBufferGeneration.shader"));

            OccupancyShader = AssetDatabase.LoadAssetAtPath<ComputeShader>(packageFolder + "Runtime/Shaders/LightmapOccupancy.compute");

            ExpansionHelpers = AssetDatabase.LoadAssetAtPath<ComputeShader>(packageFolder + "Runtime/Shaders/ExpansionHelpers.compute");

            ComputeHelpers = new LightmapIntegrationHelpers.ComputeHelpers();
            ComputeHelpers.Load();

            ChartRasterizer.LoadShaders(out SoftwareChartRasterizationShader, out HardwareChartRasterizationShader);

            BoxFilterShader = ComputeHelpers.ComputeHelperShader;
            SelectGraphicsBufferShader = ComputeHelpers.ComputeHelperShader;
            CopyTextureAdditiveShader = ComputeHelpers.ComputeHelperShader;
        }
#endif
    }
}
