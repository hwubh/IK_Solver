using System;
using UnityEditor;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Sampling;
using UnityEngine.Rendering.UnifiedRayTracing;
using static UnityEngine.PathTracing.Core.World;

namespace UnityEngine.PathTracing.Core
{
    internal class Util
    {
        internal static class ShaderProperties
        {
            public static readonly int ScramblingTileXSPP = Shader.PropertyToID("_ScramblingTileXSPP");
            public static readonly int RankingTileXSPP = Shader.PropertyToID("_RankingTileXSPP");
            public static readonly int ScramblingTexture = Shader.PropertyToID("_ScramblingTexture");
            public static readonly int OwenScrambledTexture = Shader.PropertyToID("_OwenScrambledTexture");
            public static readonly int NumLights = Shader.PropertyToID("g_NumLights");
            public static readonly int NumEmissiveMeshes = Shader.PropertyToID("g_NumEmissiveMeshes");
            public static readonly int LightList = Shader.PropertyToID("g_LightList");
            public static readonly int LightCdf = Shader.PropertyToID("g_LightCdf");
            public static readonly int LightFalloff = Shader.PropertyToID("g_LightFalloff");
            public static readonly int LightFalloffLUTRange = Shader.PropertyToID("g_LightFalloffLUTRange");
            public static readonly int LightFalloffLUTLength = Shader.PropertyToID("g_LightFalloffLUTLength");
            public static readonly int MaterialList = Shader.PropertyToID("g_MaterialList");
            public static readonly int AlbedoTextures = Shader.PropertyToID("g_AlbedoTextures");
            public static readonly int EmissionTextures = Shader.PropertyToID("g_EmissionTextures");
            public static readonly int TransmissionTextures = Shader.PropertyToID("g_TransmissionTextures");
            public static readonly int AtlasTexelSize = Shader.PropertyToID("g_AtlasTexelSize");
            public static readonly int PathTracingSkyConditionalResolution = Shader.PropertyToID("_PathTracingSkyConditionalResolution");
            public static readonly int PathTracingSkyMarginalResolution = Shader.PropertyToID("_PathTracingSkyMarginalResolution");
            public static readonly int PathTracingSkyConditionalBuffer = Shader.PropertyToID("_PathTracingSkyConditionalBuffer");
            public static readonly int PathTracingSkyMarginalBuffer = Shader.PropertyToID("_PathTracingSkyMarginalBuffer");
            public static readonly int SceneAccelStruct = Shader.PropertyToID("g_SceneAccelStruct");
            public static readonly int EnvTex = Shader.PropertyToID("g_EnvTex");
            public static readonly int LightEvaluations = Shader.PropertyToID("g_LightEvaluations");
            public static readonly int PathtracerAsGiPreviewMode = Shader.PropertyToID("g_PathtracerAsGiPreviewMode");
            public static readonly int CountNEERayAsPathSegment = Shader.PropertyToID("g_CountNEERayAsPathSegment");
            public static readonly int RenderedInstances = Shader.PropertyToID("g_RenderedInstances");
            public static readonly int PreExpose = Shader.PropertyToID("g_PreExpose");
            public static readonly int BounceCount = Shader.PropertyToID("g_BounceCount");
            public static readonly int MaxIntensity = Shader.PropertyToID("g_MaxIntensity");
            public static readonly int ExposureScale = Shader.PropertyToID("g_ExposureScale");
            public static readonly int LightPickingMethod  = Shader.PropertyToID("g_LightPickingMethod");
            public static readonly int IndirectScale = Shader.PropertyToID("g_IndirectScale");
            public static readonly int FrameIndex = Shader.PropertyToID("g_FrameIndex");
            public static readonly int EnableSubPixelJittering = Shader.PropertyToID("g_EnableSubPixelJittering");
            public static readonly int AlbedoBoost = Shader.PropertyToID("g_AlbedoBoost");
            public static readonly int EnvIntensityMultiplier = Shader.PropertyToID("g_EnvIntensityMultiplier");
            public static readonly int ExposureTexture = Shader.PropertyToID("_ExposureTexture");
            public static readonly int CookieAtlas = Shader.PropertyToID("g_CookieAtlas");
            public static readonly int CubemapAtlas = Shader.PropertyToID("g_CubemapAtlas");
        }

        public static void BindLightBuffers(CommandBuffer cmd, IRayTracingShader shader, World world)
        {
            shader.SetIntParam(cmd, ShaderProperties.NumLights, world.LightCount);
            shader.SetIntParam(cmd, ShaderProperties.NumEmissiveMeshes, world.MeshLightCount);
            shader.SetBufferParam(cmd, ShaderProperties.LightList, world.LightListBuffer);
            shader.SetBufferParam(cmd, ShaderProperties.LightCdf, world.LightCdfBuffer);
            shader.SetBufferParam(cmd, ShaderProperties.LightFalloff, world.LightFalloffBuffer);
            shader.SetBufferParam(cmd, ShaderProperties.LightFalloffLUTRange, world.LightFalloffLUTRangeBuffer);
            shader.SetIntParam(cmd, ShaderProperties.LightFalloffLUTLength, (int)world.LightFalloffLUTLength);
            shader.SetTextureParam(cmd, ShaderProperties.CookieAtlas, world.GetLightCookieTextures());
            shader.SetTextureParam(cmd, ShaderProperties.CubemapAtlas, world.GetLightCubemapTextures());
        }

        internal static void BindMaterials(CommandBuffer cmd, IRayTracingShader shader, World world)
        {
            shader.SetBufferParam(cmd, ShaderProperties.MaterialList, world.GetMaterialListBuffer());
        }

        internal static void BindTextures(CommandBuffer cmd, IRayTracingShader shader, World world)
        {
            var albedoAtlas = world.GetMaterialAlbedoTextures();
            var emissionAtlas = world.GetMaterialEmissionTextures();
            var transmissionAtlas = world.GetMaterialTransmissionTextures();

            shader.SetTextureParam(cmd, ShaderProperties.AlbedoTextures, albedoAtlas);
            shader.SetTextureParam(cmd, ShaderProperties.EmissionTextures, emissionAtlas);
            shader.SetTextureParam(cmd, ShaderProperties.TransmissionTextures, transmissionAtlas);

            if (albedoAtlas != null)
                Debug.Assert(albedoAtlas.width == albedoAtlas.height, "Atlas expected to be square");
            if (emissionAtlas != null)
                Debug.Assert(emissionAtlas.width == albedoAtlas.width && emissionAtlas.height == albedoAtlas.height, "Atlases expected to have same size");
            if (transmissionAtlas != null)
                Debug.Assert(transmissionAtlas.width == albedoAtlas.width && transmissionAtlas.height == albedoAtlas.height, "Atlases expected to have same size");

            int atlasSize = albedoAtlas?.width ?? 1;
            float atlasTexelSize = 1.0f / atlasSize;
            shader.SetFloatParam(cmd, ShaderProperties.AtlasTexelSize, atlasTexelSize);
        }

        internal static void BindMaterialsAndTextures(CommandBuffer cmd, IRayTracingShader shader, World world)
        {
            BindMaterials(cmd, shader, world);
            BindTextures(cmd, shader, world);
        }

        // Helper function to set the skybox CDF resources
        internal static void SetEnvSamplingShaderParams(CommandBuffer cmd, IRayTracingShader shader, EnvironmentCDF envCDF)
        {
            shader.SetIntParam(cmd, ShaderProperties.PathTracingSkyConditionalResolution, envCDF.ConditionalResolution);
            shader.SetIntParam(cmd, ShaderProperties.PathTracingSkyMarginalResolution, envCDF.MarginalResolution);
            shader.SetBufferParam(cmd, ShaderProperties.PathTracingSkyConditionalBuffer, envCDF.ConditionalBuffer);
            shader.SetBufferParam(cmd, ShaderProperties.PathTracingSkyMarginalBuffer, envCDF.MarginalBuffer);
        }

        internal static void BindAccelerationStructure(CommandBuffer cmd, IRayTracingShader shader, AccelStructAdapter accel)
        {
            accel.Bind(cmd, "g_SceneAccelStruct", shader);
        }

        internal static void BindWorld(CommandBuffer cmd, IRayTracingShader shader, World world, int skyBoxTextureResolution)
        {
            BindAccelerationStructure(cmd, shader, world.GetAccelerationStructure());

            BindLightBuffers(cmd, shader, world);
            BindMaterialsAndTextures(cmd, shader, world);

            var envTex = world.GetEnvironmentTexture(cmd, skyBoxTextureResolution, out EnvironmentCDF envCDF);
            shader.SetTextureParam(cmd, Shader.PropertyToID("g_EnvTex"), envTex);
            SetEnvSamplingShaderParams(cmd, shader, envCDF);
        }

        static internal void BindPathTracingInputs(
            CommandBuffer cmd,
            IRayTracingShader shader,
            bool countNEERayAsPathSegment,
            uint lightEvaluations,
            bool preExpose,
            int bounces,
            float environmentIntensityMultiplier,
            RenderedGameObjectsFilter renderedGameObjectsFilter,
            SamplingResources samplingResources,
            RTHandle emptyTexture)
        {
            shader.SetIntParam(cmd, ShaderProperties.LightEvaluations, (int)lightEvaluations);
            shader.SetIntParam(cmd, ShaderProperties.PathtracerAsGiPreviewMode, 0);
            shader.SetIntParam(cmd, ShaderProperties.CountNEERayAsPathSegment, countNEERayAsPathSegment ? 1 : 0);
            shader.SetIntParam(cmd, ShaderProperties.RenderedInstances, (int)renderedGameObjectsFilter);
            shader.SetIntParam(cmd, ShaderProperties.PreExpose, preExpose ? 1 : 0);
            shader.SetIntParam(cmd, ShaderProperties.BounceCount, bounces);
            shader.SetIntParam(cmd, ShaderProperties.MaxIntensity, int.MaxValue);
            shader.SetFloatParam(cmd, ShaderProperties.ExposureScale, 1.0f);
            shader.SetIntParam(cmd, ShaderProperties.LightPickingMethod, 0);
            shader.SetFloatParam(cmd, ShaderProperties.IndirectScale, 1.0f);
            shader.SetIntParam(cmd, ShaderProperties.FrameIndex, 0);
            shader.SetIntParam(cmd, ShaderProperties.EnableSubPixelJittering, 0);
            shader.SetIntParam(cmd, ShaderProperties.LightPickingMethod, 0);
            shader.SetFloatParam(cmd, ShaderProperties.AlbedoBoost, 1.0f);
            shader.SetFloatParam(cmd, ShaderProperties.EnvIntensityMultiplier, environmentIntensityMultiplier);
            SamplingResources.BindSobolBlueNoiseTextures(cmd, samplingResources);
            // To avoid shader permutations, we always need to set an exposure texture, even if we don't read it
            if (!preExpose)
            {
                shader.SetTextureParam(cmd, ShaderProperties.ExposureTexture, emptyTexture);
            }
        }

        internal static RayTracingResources LoadOrCreateRayTracingResources()
        {
            RayTracingResources resources = new RayTracingResources();
            resources.Load();
            return resources;
        }

        internal static bool IsStatic(GameObject obj)
        {
#if UNITY_EDITOR
            // If we are in the editor, access the StaticEditorFlags
            int flags = (int)GameObjectUtility.GetStaticEditorFlags(obj);
            return (flags & (int)StaticEditorFlags.ContributeGI) != 0;
#else
            return obj.isStatic;
#endif
        }

        internal static bool IsCookieValid(uint cookieTextureIndex)
        {
            return cookieTextureIndex != UInt32.MaxValue;
        }

        internal static bool IsPunctualLightType(LightType lightType)
        {
            return lightType == LightType.Directional || lightType == LightType.Spot || lightType == LightType.Point || lightType == LightType.Pyramid || lightType == LightType.Box;
        }

        // Our old baker, LightBaker, multiplied intensities of punctual lights by PI. This isn't quite correct, but it was never changed as it would be breaking.
        // The 'multiplyPunctualLightIntensityByPI' can be used to mimic that old behavior.
        internal static LightDescriptor[] ConvertUnityLightsToLightDescriptors(Light[] lights, bool multiplyPunctualLightIntensityByPI)
        {
            LightDescriptor[] lightDescriptors = new LightDescriptor[lights.Length];
            for (int i = 0; i < lights.Length; i++)
            {
                Light light = lights[i];
                ref LightDescriptor ld = ref lightDescriptors[i];
                ld.Type = light.type;
                ld.LinearLightColor = GetLinearLightColor(light);
                if (multiplyPunctualLightIntensityByPI && IsPunctualLightType(light.type))
                    ld.LinearLightColor *= Mathf.PI;
                ld.Shadows = light.shadows;
                ld.Transform = light.transform.localToWorldMatrix;
                ld.ColorTemperature = light.colorTemperature;
                ld.LightmapBakeType = light.lightmapBakeType;
                ld.AreaSize = light.areaSize;
                ld.SpotAngle = light.spotAngle;
                ld.InnerSpotAngle = light.innerSpotAngle;
                ld.CullingMask = (uint)light.cullingMask;
                ld.BounceIntensity = light.bounceIntensity;
                ld.Range = light.range;
                ld.ShadowMaskChannel = -1;
                ld.UseColorTemperature = light.useColorTemperature;
                ld.FalloffType = Experimental.GlobalIllumination.FalloffType.InverseSquared; // When we extract lights for the path tracer, we assume inverse squared falloff.
                ld.ShadowRadius = Util.IsPunctualLightType(light.type) ? light.shadowRadius : 0.0f;
                ld.CookieTexture = light.cookie;
                ld.CookieSize = light.cookieSize;
            }
            return lightDescriptors;
        }

        // Copy of internal API from Color.cs
        private static Color RGBMultiplied(Color color, float multiplier)
        {
            return new Color(color.r * multiplier, color.g * multiplier, color.b * multiplier, color.a);
        }

        private static Vector3 GetLinearLightColor(Light light)
        {
            Color lightColor = (GraphicsSettings.lightsUseLinearIntensity) ? RGBMultiplied(light.color.linear, light.intensity) : RGBMultiplied(light.color, light.intensity).linear;
            lightColor *= light.useColorTemperature ? Mathf.CorrelatedColorTemperatureToRGB(light.colorTemperature) : Color.white;
            return new Vector3(lightColor.r, lightColor.g, lightColor.b) * lightColor.a;
        }

        internal static Material[] GetMaterials(MeshRenderer renderer)
        {
            int submeshCount = 1;
            var meshFilter = renderer.GetComponent<MeshFilter>();
            if (meshFilter)
                submeshCount = renderer.GetComponent<MeshFilter>().sharedMesh.subMeshCount;

            Material[] mats = new Material[submeshCount];
            for (int i = 0; i < mats.Length; ++i)
            {
                if (i < renderer.sharedMaterials.Length && renderer.sharedMaterials[i] != null)
                    mats[i] = renderer.sharedMaterials[i];
                else
                    mats[i] = null;
            }

            return mats;
        }
    }
}
