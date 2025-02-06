using System;
using System.Collections.Generic;
using Unity.Mathematics;
using System.Runtime.InteropServices;
using UnityEngine.Rendering;
using UnityEngine.Rendering.UnifiedRayTracing;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace UnityEngine.PathTracing.Core
{
    using InstanceHandle = Handle<World.InstanceKey>;
    using InstanceHandleSet = HandleSet<World.InstanceKey>;

    using LightHandle = Handle<World.LightDescriptor>;
    using LightHandleSet = HandleSet<World.LightDescriptor>;

    using MaterialHandle = Handle<World.MaterialDescriptor>;
    using MaterialHandleSet = HandleSet<World.MaterialDescriptor>;

    internal enum RenderedGameObjectsFilter
    {
        All = 0,
        OnlyStatic = 1,
        AllInCameraRaysThenOnlyStatic = 2
    }
    internal enum LightPickingMethod
    {
        Uniform = 0,
        Power = 1
    }
    internal enum InstanceFlags
    {
        DIRECT_RAY_VIS_MASK = 1,
        INDIRECT_RAY_VIS_MASK = 2,
        SHADOW_RAY_VIS_MASK = 4,
        CURRENT_LOD_FOR_LIGHTMAP_INSTANCE = 8,
        LOD_ZERO_FOR_LIGHTMAP_INSTANCE = 16,
        CURRENT_LOD_FOR_LIGHTMAP_INSTANCE_SHADOW = 32,
        LOD_ZERO_FOR_LIGHTMAP_INSTANCE_SHADOW = 64
    }

    internal class World : IDisposable
    {
        private const int EMISSIVE_MESH = 8; // Must match the EMISSIVE_MESH define in Common.hlsl
        private const int ENVIRONMENT_LIGHT = 9; // Must match the ENVIRONMENT_LIGHT define in Common.hlsl

        internal class ResourceLibrary
        {
            public ComputeShader BlitCubemap;
            public ComputeShader BlitGrayScaleCookie;
            public ComputeShader SetAlphaChannelShader;
            public ComputeShader PathTracingSkySamplingDataShader;
            public Mesh SkyBoxMesh;
            public Mesh SixFaceSkyBoxMesh;

#if UNITY_EDITOR
            public void Load()
            {
                const string packageFolder = "Packages/com.unity.path-tracing/";

                BlitCubemap = AssetDatabase.LoadAssetAtPath<ComputeShader>(packageFolder + "Runtime/Shaders/BlitCubemap.compute");
                BlitGrayScaleCookie = AssetDatabase.LoadAssetAtPath<ComputeShader>(packageFolder + "Runtime/Shaders/BlitCookie.compute");
                SetAlphaChannelShader = AssetDatabase.LoadAssetAtPath<ComputeShader>(packageFolder + "Runtime/Shaders/SetAlphaChannel.compute");
                PathTracingSkySamplingDataShader = AssetDatabase.LoadAssetAtPath<ComputeShader>(packageFolder + "Runtime/Shaders/PathTracingSkySamplingData.compute");
                SkyBoxMesh = AssetDatabase.LoadAssetAtPath<Mesh>(packageFolder + "Runtime/Meshes/SkyboxMesh.mesh");
                SixFaceSkyBoxMesh = AssetDatabase.LoadAssetAtPath<Mesh>(packageFolder + "Runtime/Meshes/6FaceSkyboxMesh.mesh");
            }
#endif
        }

        // This trivial type only exists so that handles can be type-safe.
        // If we make an InstanceDescriptor type, we can use that instead.
        internal readonly struct InstanceKey { }

        internal struct MaterialDescriptor
        {
            public Texture Albedo;
            public Vector2 AlbedoScale;
            public Vector2 AlbedoOffset;

            public Texture Emission;
            public Vector2 EmissionScale;
            public Vector2 EmissionOffset;
            public Vector3 EmissionColor;
            public MaterialPropertyType EmissionType;

            public Texture Transmission;
            public Vector2 TransmissionScale;
            public Vector2 TransmissionOffset;
            public TransmissionChannels TransmissionChannels;

            public float Alpha;
            public float AlphaCutoff;
            public bool UseAlphaCutoff;

            public bool DoubleSidedGI;

            public bool PointSampleTransmission;
        }

        internal struct LightDescriptor
        {
            public LightType Type;
            public Vector3 LinearLightColor;
            public LightShadows Shadows;
            public Matrix4x4 Transform;
            public float ColorTemperature;
            public LightmapBakeType LightmapBakeType;
            public Experimental.GlobalIllumination.FalloffType FalloffType;
            public Vector2 AreaSize;
            public float SpotAngle;
            public float InnerSpotAngle;
            public uint CullingMask;
            public float BounceIntensity;
            public float Range;
            public int ShadowMaskChannel;
            public bool UseColorTemperature;
            public float ShadowRadius;
            public Texture CookieTexture;
            public float CookieSize;
        }

        [StructLayout(LayoutKind.Sequential)]
        internal struct PTLight // Must match the PTLight definition in Common.hlsl
        {
            public Vector3 position;
            public int type;
            public Vector3 intensity;
            public int castShadows;
            public Vector3 forward;
            public int contributesToDirectLighting;
            public Vector4 attenuation;
            public Vector3 up;
            public float width;
            public Vector3 right;
            public float height;
            public uint layerMask;
            public float indirectScale;
            public float spotAngle;
            public float innerSpotAngle;
            public float range;
            public int shadowMaskChannel;
            public int falloffIndex;
            public float shadowRadius;
            public int cookieIndex;

            public override readonly int GetHashCode()
            {
                return HashCode.Combine(position, type, intensity, castShadows, forward, contributesToDirectLighting, range)
                       ^ HashCode.Combine(attenuation, up, width, right, height, layerMask, indirectScale, falloffIndex);
            }
        }

        private readonly InstanceHandleSet _instanceHandleSet = new();
        private readonly MaterialHandleSet _materialHandleSet = new();

        #region Lighting State
        private class LightState
        {
            public bool CdfLightPicking;

            // Holds a list of light sources in the scene
            public readonly List<PTLight> LightList = new(64);
            public ComputeBuffer LightListBuffer;

            // CDF-based light sampling
            public List<float> LightCdf = new();
            public ComputeBuffer LightCdfBuffer;

            // Holds a list of light falloff LUTs
            public const uint LightFalloffLUTLength = 1024;
            public List<LightFalloffDesc> LightFalloffDescs = new();
            public float[] LightFalloff = null; // The light falloff LUT tables
            public float[] LightFalloffLUTRanges = null; // The range that each LUT applies to
            public ComputeBuffer LightFalloffBuffer;
            public ComputeBuffer LightFalloffLUTRangeBuffer;

            // Light dictionary (Handle, Light entry)
            public Dictionary<LightHandle, PTLight> LightHandleToLightListEntry = new();

            // Lookup table from LightHandle to the corresponding index in LightList. Only used for baking.
            public Dictionary<LightHandle, int> LightHandleToLightListIndex = new();

            // Mesh light dictionary (SubMeshHash, emissive mesh entry)
            public Dictionary<int, PTLight> MeshLights = new();

            // Light handle set
            public LightHandleSet LightHandleSet = new();

            private static PTLight CreateEnvironmentLight()
            {
                PTLight envLight;
                envLight.type = ENVIRONMENT_LIGHT;
                envLight.position = Vector3.zero;
                envLight.intensity = Vector3.zero;
                envLight.castShadows = 1;
                envLight.contributesToDirectLighting = 1;
                envLight.forward = Vector3.zero;
                envLight.attenuation = Vector4.one;
                envLight.up = Vector3.zero;
                envLight.right = Vector3.zero;
                envLight.width = 0;
                envLight.height = 0;
                envLight.spotAngle = 0;
                envLight.innerSpotAngle = 0;
                envLight.layerMask = uint.MaxValue;
                envLight.indirectScale = 1.0f;
                envLight.attenuation = Vector4.zero;
                envLight.range = float.MaxValue;
                envLight.shadowMaskChannel = -1;
                envLight.falloffIndex = -1;
                envLight.shadowRadius = 0.0f;
                envLight.cookieIndex = -1;
                return envLight;
            }

            public void Build(Bounds sceneBounds, CommandBuffer cmdBuf, bool enableEmissiveSampling)
            {
                // Make sure the light list is empty before we build it.
                LightList.Clear();
                LightHandleToLightListIndex.Clear();

                // Combine the mesh lights and the rest of the lights.
                foreach (var meshLight in MeshLights)
                    LightList.Add(meshLight.Value);
                foreach (var light in LightHandleToLightListEntry)
                {
                    LightHandleToLightListIndex.Add(light.Key, LightList.Count);
                    LightList.Add(light.Value);
                }

                // If we explicitly sample emitters, include the environment light in the light list
                if (enableEmissiveSampling)
                    LightList.Add(CreateEnvironmentLight());

                BuildCdf(this, sceneBounds);
                BuildLightFalloffLUTs(this);
                SetLightDataOnCommandBuffer(this, cmdBuf);
            }

            private static void BuildLightFalloffLUTs(LightState lightState)
            {
                // Build the LUT data
                lightState.LightFalloff = LightFalloffLUT.BuildLightFalloffLUTs(lightState.LightFalloffDescs.ToArray(), LightFalloffLUTLength);
                Debug.Assert(lightState.LightFalloff.Length == LightFalloffLUTLength * lightState.LightFalloffDescs.Count);

                // Store the range of each LUT
                lightState.LightFalloffLUTRanges = new float[lightState.LightFalloffDescs.Count];
                int i = 0;
                foreach (var desc in lightState.LightFalloffDescs)
                {
                    Debug.Assert(desc.LUTRange >= 0.0f);
                    lightState.LightFalloffLUTRanges[i] = desc.LUTRange;
                    i++;
                }

                // We have built the LUTs, now clear the descriptors so we don't rebuild them every frame
                lightState.LightFalloffDescs.Clear();
            }

            private static void BuildCdf(LightState lightState, Bounds sceneBounds)
            {
                lightState.LightCdf.Clear();
                if (lightState.CdfLightPicking)
                {
                    float sceneBoundingSphereRadius = Mathf.Max(Mathf.Max(sceneBounds.extents.x, sceneBounds.extents.y), sceneBounds.extents.z);
                    Debug.Assert(lightState.LightCdf.Count == 0);

                    // Calculate CDF.
                    foreach (var light in lightState.LightList)
                    {
                        int count = lightState.LightCdf.Count;
                        if (light.type == EMISSIVE_MESH)
                        {
                            float intensityScalar = light.intensity.x + light.intensity.y + light.intensity.z;
                            const float visibleFraction = 0.5f; // just an approximation: on average half of the mesh will be back-facing.
                            float lightWeight = intensityScalar * visibleFraction * light.width; //Note: light.width store approximate area for mesh lights
                            lightState.LightCdf.Add((count == 0 ? 0.0f : lightState.LightCdf[count - 1]) + lightWeight);
                        }
                        else if (light.type == ENVIRONMENT_LIGHT)
						{
							//TODO: LIGHT-1684, move the CDF computation on the GPU and read the skybox LODs to estimate the emissive power, taking also into account the scene size (if we go for a PBR formulation).
                            float lightWeight = 1.0f;
                            lightState.LightCdf.Add((count == 0 ? 0.0f : lightState.LightCdf[count - 1]) + lightWeight);
						}
						else
                        {
                            float lightWeight = ApproximateTotalPower(light, sceneBoundingSphereRadius);
                            lightState.LightCdf.Add((count == 0 ? 0.0f : lightState.LightCdf[count - 1]) + lightWeight);
                        }
                    }

                    // Normalize CDF.
                    if (lightState.LightCdf.Count != 0)
                    {
                        float cdfNormalizationConstant = 1.0f / lightState.LightCdf[lightState.LightCdf.Count - 1];
                        for (int i = 0; i < lightState.LightCdf.Count; ++i)
                        {
                            lightState.LightCdf[i] *= cdfNormalizationConstant;
                        }
                    }
                }
            }

            private static void SetLightDataOnCommandBuffer(LightState lightState, CommandBuffer cmdBuf)
            {
                if (lightState.LightListBuffer == null || lightState.LightListBuffer.count < lightState.LightList.Count)
                {
                    lightState.LightListBuffer?.Release();
                    lightState.LightListBuffer = new ComputeBuffer(math.max(64, lightState.LightList.Count), Marshal.SizeOf<PTLight>());
                }

                if (lightState.LightList.Count > 0)
                {
                    cmdBuf.SetBufferData(lightState.LightListBuffer, lightState.LightList, 0, 0, lightState.LightList.Count);
                }

                if (lightState.LightCdfBuffer == null || lightState.LightCdfBuffer.count < lightState.LightCdf.Count)
                {
                    lightState.LightCdfBuffer?.Release();
                    int count = math.max(1, lightState.LightCdf.Count);
                    int stride = sizeof(float);
                    lightState.LightCdfBuffer = new ComputeBuffer(count, stride);
                }

                if (lightState.LightCdf.Count > 0)
                {
                    cmdBuf.SetBufferData(lightState.LightCdfBuffer, lightState.LightCdf, 0, 0, lightState.LightCdf.Count);
                }

                if (lightState.LightFalloffBuffer == null || lightState.LightFalloffBuffer.count < lightState.LightFalloff.Length)
                {
                    lightState.LightFalloffBuffer?.Release();
                    int count = math.max(1, lightState.LightFalloff.Length);
                    int stride = sizeof(float);
                    lightState.LightFalloffBuffer = new ComputeBuffer(count, stride);
                }

                if (lightState.LightFalloff.Length > 0)
                {
                    cmdBuf.SetBufferData(lightState.LightFalloffBuffer, lightState.LightFalloff, 0, 0, lightState.LightFalloff.Length);
                }

                if (lightState.LightFalloffLUTRangeBuffer == null || lightState.LightFalloffLUTRangeBuffer.count < lightState.LightFalloffLUTRanges.Length)
                {
                    lightState.LightFalloffLUTRangeBuffer?.Release();
                    int count = math.max(1, lightState.LightFalloffLUTRanges.Length);
                    int stride = sizeof(float);
                    lightState.LightFalloffLUTRangeBuffer = new ComputeBuffer(count, stride);
                }

                if (lightState.LightFalloffLUTRanges.Length > 0)
                {
                    cmdBuf.SetBufferData(lightState.LightFalloffLUTRangeBuffer, lightState.LightFalloffLUTRanges, 0, 0, lightState.LightFalloffLUTRanges.Length);
                }
            }
        }
        private LightState _lightState;
        public bool cdfLightPicking
        {
            set => _lightState.CdfLightPicking = value;
        }

        public bool EnableEmissiveSampling;

        public int MaterialCount => _materialPool.MaterialCount;

        public int LightCount => NonMeshLightCount + MeshLightCount + EnvLightCount;
        public int NonMeshLightCount => _lightState.LightHandleToLightListEntry.Count;
        public int MeshLightCount => _lightState.MeshLights.Count;

        public int EnvLightCount => EnableEmissiveSampling ? 1 : 0;

        public List<PTLight> LightList => _lightState.LightList;
        public Dictionary<LightHandle, int> LightHandleToLightListIndex => _lightState.LightHandleToLightListIndex;
        public ComputeBuffer LightListBuffer => _lightState.LightListBuffer;
        public ComputeBuffer LightCdfBuffer => _lightState.LightCdfBuffer;
        public ComputeBuffer LightFalloffBuffer => _lightState.LightFalloffBuffer;
        public ComputeBuffer LightFalloffLUTRangeBuffer => _lightState.LightFalloffLUTRangeBuffer;
        public uint LightFalloffLUTLength => LightState.LightFalloffLUTLength;
        public int LightListHashCode
        {
            get
            {
                int lightHashCode = 0;
                foreach (var light in _lightState.LightList)
                    lightHashCode = HashCode.Combine(lightHashCode, light.GetHashCode());
                return lightHashCode;
            }
        }

        // SubMesh dictionary (instance handle, emissive submesh list)
        private readonly Dictionary<InstanceHandle, List<int>> _subMeshIndices = new();

        #endregion

        private MaterialPool _materialPool;
        private AccelStructAdapter _rayTracingAccelerationStructure;

        // Skybox sampling
        private CubemapRender _cubemapRender;
        private EnvironmentImportanceSampling _environmentSampling;
        private int _currentSkyboxHash;

        public void Init(RayTracingContext ctx, ResourceLibrary worldResources)
        {
            _materialPool = new MaterialPool(worldResources.SetAlphaChannelShader, worldResources.BlitCubemap, worldResources.BlitGrayScaleCookie);

            var options = new AccelerationStructureOptions()
            {
                buildFlags = BuildFlags.None, // TODO: Consider whether to use BuildFlags.MinimizeMemory once https://jira.unity3d.com/browse/UUM-54575 is fixed.
                enableCompaction = false
            };
            _rayTracingAccelerationStructure = new AccelStructAdapter(ctx.CreateAccelerationStructure(options), new GeometryPool(GeometryPoolDesc.NewDefault(), ctx.Resources.geometryPoolKernels, ctx.Resources.copyBuffer));

            _lightState = new LightState();

            _cubemapRender = new CubemapRender(worldResources.SkyBoxMesh, worldResources.SixFaceSkyBoxMesh);
            _environmentSampling = new EnvironmentImportanceSampling(worldResources.PathTracingSkySamplingDataShader);
        }

        public void SetEnvironmentMaterial(Material mat)
        {
            _cubemapRender.SetMaterial(mat);
        }

        public ComputeBuffer GetMaterialListBuffer()
        {
            return _materialPool.MaterialBuffer;
        }

        public RenderTexture GetMaterialAlbedoTextures()
        {
            return _materialPool.AlbedoTextures;
        }

        public RenderTexture GetMaterialEmissionTextures()
        {
            return _materialPool.EmissionTextures;
        }

        public RenderTexture GetMaterialTransmissionTextures()
        {
            return _materialPool.TransmissionTextures;
        }

        public RenderTexture GetLightCookieTextures()
        {
            return _materialPool.LightCookieTextures;
        }

        public RenderTexture GetLightCubemapTextures()
        {
            return _materialPool.LightCubemapTextures;
        }

        public Texture GetEnvironmentTexture(CommandBuffer cmd, int resolution, out EnvironmentCDF environmentCDF)
        {
            Debug.Assert(_environmentSampling != null, "You should call World::Init() first");

            var envTex = _cubemapRender.GetCubemap(resolution, out int skyHash);
            if (_currentSkyboxHash != skyHash)
            {
                _environmentSampling.ComputeCDFBuffers(cmd, envTex);
                _currentSkyboxHash = skyHash;
            }
            environmentCDF = _environmentSampling.GetSkyboxCDF();
            return envTex;
        }

        public void Dispose()
        {
            _rayTracingAccelerationStructure?.Dispose();
            _materialPool?.Dispose();
            _subMeshIndices.Clear();
            _lightState.LightCdfBuffer?.Dispose();
            _lightState.LightListBuffer?.Dispose();
            _lightState.LightFalloffBuffer?.Dispose();
            _lightState.LightFalloffLUTRangeBuffer?.Dispose();
            _lightState.LightCdf.Clear();
            _lightState.LightFalloffDescs.Clear();
            _lightState.LightFalloff = null;
            _lightState.LightList.Clear();
            _lightState.LightHandleToLightListEntry.Clear();
            _lightState.MeshLights.Clear();
            _cubemapRender?.Dispose();
            _environmentSampling?.Dispose();
        }

        public AccelStructAdapter GetAccelerationStructure()
        {
            return _rayTracingAccelerationStructure;
        }

        public void NextFrame()
        {
            _rayTracingAccelerationStructure.NextFrame();
        }

        // Provide a unique hash for the renderer sub meshes
        private static int GetSubMeshHash(InstanceHandle instance, int subMeshIndex)
        {
            return HashCode.Combine(instance.Value, subMeshIndex);
        }

        public void RemoveInstance(InstanceHandle instance)
        {
            try
            {
                _rayTracingAccelerationStructure.RemoveInstance(instance.Value);
                _instanceHandleSet.Remove(instance);
                RemoveEmissiveMeshes(instance);
            }
            catch (Exception e)
            {
                LogException("Failed to remove instance", e, instance.Value);
            }
        }

        private void RemoveEmissiveMeshes(InstanceHandle instance)
        {
            if (_subMeshIndices.ContainsKey(instance))
            {
                var emissiveSubMeshes = _subMeshIndices[instance];
                foreach (var subMeshIndex in emissiveSubMeshes)
                {
                    var subMeshHash = GetSubMeshHash(instance, subMeshIndex);
                    if (_lightState.MeshLights.ContainsKey(subMeshHash))
                        _lightState.MeshLights.Remove(subMeshHash);
                }
                _subMeshIndices.Remove(instance);
            }
        }

        public void RemoveMaterial(MaterialHandle materialHandle)
        {
            try
            {
                _materialHandleSet.Remove(materialHandle);
                _materialPool.RemoveMaterial(materialHandle.Value);
            }
            catch (Exception e)
            {
                LogException("failed to remove material", e, materialHandle.Value);
            }
        }

        public MaterialHandle AddMaterial(in MaterialDescriptor material, UVChannel albedoAndEmissionUVChannel)
        {
            MaterialHandle handle = _materialHandleSet.Add();
            try
            {
                _materialPool.AddMaterial(handle.Value, in material, albedoAndEmissionUVChannel);
            }
            catch (Exception e)
            {
                LogException("failed to add material", e, handle.Value);
            }
            return handle;
        }

        public void UpdateMaterial(MaterialHandle materialHandle, in MaterialDescriptor material, UVChannel albedoAndEmissionUVChannel)
        {
            try
            {
                _materialPool.UpdateMaterial(materialHandle.Value, in material, albedoAndEmissionUVChannel);
            }
            catch (Exception e)
            {
                LogException("failed to modify material", e, materialHandle.Value);
            }
        }

        private void LogException(string message, Exception e, Object obj)
        {
            var objName = obj != null ? obj.name : "null";
            Debug.LogError($"PathTracing: {message} <{objName}> \n{e.Message}", obj);
        }

        private void LogException(string message, Exception e, int instanceID)
        {
            Debug.LogError($"PathTracing: {message} <{instanceID}> \n{e.Message}");
        }

        private void LogError(string message)
        {
            Debug.LogError($"PathTracing: {message} \n");
        }

        public InstanceHandle AddInstance(
            Mesh mesh,
            Span<MaterialHandle> materials,
            Span<uint> masks,
            uint renderingLayerMask,
            in Matrix4x4 localToWorldMatrix,
            Bounds bounds,
            bool isStatic,
            RenderedGameObjectsFilter filter,
            bool enableEmissiveSampling)
        {
            Debug.Assert(mesh.subMeshCount == materials.Length);
            Debug.Assert(mesh.subMeshCount == masks.Length);

            Span<uint> materialIndices = stackalloc uint[mesh.subMeshCount];
            for (int i = 0; i < materials.Length; ++i)
            {
                if (materials[i] != MaterialHandle.Invalid)
                    materialIndices[i] = _materialPool.GetMaterialIndex(materials[i].Value);
            }

            InstanceHandle instance = _instanceHandleSet.Add();
            _rayTracingAccelerationStructure.AddInstance(instance.Value, mesh, localToWorldMatrix, masks, materialIndices, renderingLayerMask);

            if (enableEmissiveSampling && !ProcessEmissiveMeshes(instance, mesh, bounds, materials, isStatic, _rayTracingAccelerationStructure, _materialPool, filter, _lightState.MeshLights, _subMeshIndices))
                LogError($"Failed to process emissive triangles in mesh {mesh.name}.");

            return instance;
        }

        public void UpdateInstanceTransform(InstanceHandle instance, Matrix4x4 localToWorldMatrix)
        {
            _rayTracingAccelerationStructure.UpdateInstanceTransform(instance.Value, localToWorldMatrix);
        }

        public void UpdateInstanceMask(InstanceHandle instance, Span<uint> perSubMeshMask)
        {
            _rayTracingAccelerationStructure.UpdateInstanceMask(instance.Value, perSubMeshMask);
        }
        public void UpdateInstanceMask(InstanceHandle instance, uint mask)
        {
            _rayTracingAccelerationStructure.UpdateInstanceMask(instance.Value, mask);
        }

        public void UpdateInstanceMaterials(InstanceHandle instance, Span<MaterialHandle> materials)
        {
            Span<uint> materialIndices = stackalloc uint[materials.Length];
            for (int i = 0; i < materials.Length; ++i)
                materialIndices[i] = _materialPool.GetMaterialIndex(materials[i].Value);

            _rayTracingAccelerationStructure.UpdateInstanceMaterialIDs(instance.Value, materialIndices);
        }

        public void UpdateInstanceEmission(
            InstanceHandle instance,
            Mesh mesh,
            Bounds bounds,
            Span<MaterialHandle> materials,
            bool isStatic,
            RenderedGameObjectsFilter filter)
        {
            if (!ProcessEmissiveMeshes(instance, mesh, bounds, materials, isStatic, _rayTracingAccelerationStructure, _materialPool, filter, _lightState.MeshLights, _subMeshIndices))
            {
                LogError($"failed to process emissive triangles in mesh with handle {instance}");
            }
        }

        // InstanceMask bit encoding format:
        internal static uint GetInstanceMask(ShadowCastingMode shadowMode, bool isStatic, RenderedGameObjectsFilter filter, bool hasLightmaps = true)
        {
            uint instanceMask = 0u;

            if (shadowMode != ShadowCastingMode.Off)
            {
                if (filter == RenderedGameObjectsFilter.All)
                {
                    instanceMask |= (uint)InstanceFlags.SHADOW_RAY_VIS_MASK;
                }
                else
                {
                    if (isStatic)
                    {
                        instanceMask |= (uint)InstanceFlags.SHADOW_RAY_VIS_MASK;
                    }
                }
            }

            if (shadowMode != ShadowCastingMode.ShadowsOnly)
            {
                if (filter == RenderedGameObjectsFilter.All)
                {
                    instanceMask |= (uint)InstanceFlags.DIRECT_RAY_VIS_MASK;
                    instanceMask |= (uint)InstanceFlags.INDIRECT_RAY_VIS_MASK;
                }
                else if (filter == RenderedGameObjectsFilter.OnlyStatic)
                {
                    if (isStatic)
                    {
                        if (hasLightmaps)
                        {
                            instanceMask |= (uint)InstanceFlags.DIRECT_RAY_VIS_MASK;
                        }
                        instanceMask |= (uint)InstanceFlags.INDIRECT_RAY_VIS_MASK;
                    }
                }
                else if (filter == RenderedGameObjectsFilter.AllInCameraRaysThenOnlyStatic)
                {
                    if (isStatic)
                    {
                        instanceMask |= (uint)InstanceFlags.DIRECT_RAY_VIS_MASK;
                        instanceMask |= (uint)InstanceFlags.INDIRECT_RAY_VIS_MASK;
                    }
                    else
                    {
                        instanceMask |= (uint)InstanceFlags.DIRECT_RAY_VIS_MASK;
                    }
                }
            }

            return instanceMask;
        }

        private static bool ProcessEmissiveMeshes(
            InstanceHandle instance,
            Mesh mesh,
            Bounds bounds,
            Span<MaterialHandle> materials,
            bool isStatic,
            AccelStructAdapter rtAccelStruct,
            MaterialPool sceneMaterials,
            RenderedGameObjectsFilter filter,
            Dictionary<int, PTLight> meshLights,
            Dictionary<InstanceHandle, List<int>> subMeshIndexMap)
        {
            if (filter != RenderedGameObjectsFilter.All && !isStatic)
                return true;

            int subMeshCount = mesh.subMeshCount;

            if (!rtAccelStruct.GetInstanceIDs(instance.Value, out int[] instanceIDs))
            {
                // This should never happen as long as the renderer was already added to the acceleration structure
                return false;
            }

            // Approximate area of the emissive mesh using the bounding box
            float boundingBoxArea = 2 * (bounds.size.x * bounds.size.y) +
                 2 * (bounds.size.y * bounds.size.z) +
                 2 * (bounds.size.x * bounds.size.z);

            // List to keep track of the emissive subMeshes
            List<int> subMeshIndices = new List<int>(subMeshCount);
            for (int i = 0; i < subMeshCount; ++i)
            {
                MaterialHandle mat = materials[i];

                // If it's an emissive material, create emissive triangles for this submesh
                if (sceneMaterials.IsEmissive(mat.Value, out float3 emission))
                {
                    var triangleIndices = mesh.GetTriangles(i);
                    var triangleCount = triangleIndices.Length / 3;

                    // create a new MeshLight entry
                    PTLight newLight;
                    {
                        newLight.type = EMISSIVE_MESH;
                        newLight.height = instanceIDs[i];
                        newLight.attenuation = Vector4.one;
                        newLight.attenuation.x = triangleCount; // number of emissive triangles
                        newLight.intensity = emission;
                        newLight.position = Vector3.zero;
                        newLight.up = Vector3.zero;
                        newLight.right = Vector3.zero;
                        newLight.forward = Vector3.zero;
                        newLight.width = boundingBoxArea;
                        newLight.castShadows = 1;
                        newLight.contributesToDirectLighting = 1;
                        newLight.indirectScale = 1.0f;
                        newLight.spotAngle = 0;
                        newLight.innerSpotAngle = 0;
                        newLight.layerMask = uint.MaxValue;
                        newLight.range = float.MaxValue;
                        newLight.shadowMaskChannel = -1;
                        newLight.falloffIndex = -1;
                        newLight.shadowRadius = 0.0f;
                        newLight.cookieIndex = -1; // mesh lights sample directly the emission texture, they don't have a light cookie
                    }

                    var subMeshHash = GetSubMeshHash(instance, i);
                    meshLights[subMeshHash] = newLight;

                    // keep track of which subMeshes are emissive (need this for when removing meshes)
                    subMeshIndices.Add(i);
                }
            }

            if (subMeshIndices.Count > 0)
            {
                subMeshIndexMap[instance] = subMeshIndices;
            }
            return true;
        }

        // Approximates the total power transferred onto the scene due to a given light.
        private static float ApproximateTotalPower(PTLight light, float sceneBoundingSphereRadius)
        {
            Vector3 intensity = light.intensity;
            float intensityScalar = intensity.x + intensity.y + intensity.z;

            LightType lightType = (LightType)light.type;
            switch (lightType)
            {
                case LightType.Point:
                {
                    // Intensity is expected to be in lumen per solid angle.
                    // We multiply be solid angle of the entire sphere (4π)
                    // to get total power output.
                    return Mathf.PI * 4.0f * intensityScalar;
                }

                case LightType.Spot:
                {
                    // The solid angle of a cone is given by 2π (1 - cos(theta). We approximate the power output as the average
                    // of this expression evaluated for the inner and the outer angle, all multiplied by the intensity.
                    // Intensity is expected to be in Candela.
                    return intensityScalar * Mathf.PI * (2.0f - Mathf.Cos(light.spotAngle) - Mathf.Cos(light.innerSpotAngle));
                }

                case LightType.Rectangle:
                {
                    // Intensity assumed to be in nits.
                    float area = light.width * light.height;
                    return intensityScalar * area * Mathf.PI;
                }

                case LightType.Disc:
                {
                    // Intensity assumed to be in nits.
                    float area = light.width * light.height * Mathf.PI;
                    return intensityScalar * area * Mathf.PI;
                }

                case LightType.Directional:
                {
                    // Intensity assumed by to in lux. We approximate the area visible to the directional light, as a fraction
                    // of the area of a cross section disc inside the scene's bounding sphere.
                    float sceneSurfaceAreaApproximation = sceneBoundingSphereRadius * sceneBoundingSphereRadius * Mathf.PI;
                    const float visibleFraction = 0.5f; // This is just a guess.
                    return intensityScalar * visibleFraction * sceneSurfaceAreaApproximation;
                }

                default:
                {
                    Debug.Assert(false, "Unexpected light type when approximating total power output.");
                    return intensityScalar; // Unexpected.
                }
            }
        }

        public LightHandle[] AddLights(Span<LightDescriptor> lights,
            bool respectLightLayers,
            bool autoEstimateLUTRange,
            MixedLightingMode mixedLightingMode)
        {
            // Generate handles
            LightHandle[] handles = new LightHandle[lights.Length];
            for (int i = 0; i < lights.Length; i++)
            {
                LightHandle handle = _lightState.LightHandleSet.Add();
                handles[i] = handle;
            }

            // Use the handles to update the lights for the first time
            UpdateLights(handles, lights, respectLightLayers, autoEstimateLUTRange, mixedLightingMode);

            return handles;
        }

        private static float EstimateLUTRange(float range, float luminance, Experimental.GlobalIllumination.FalloffType falloffType, float threshold = 0.01f)
        {
            Debug.Assert(threshold > 0.0f);
            Debug.Assert(range > 0.0f);
            if (luminance <= 0.0f)
                return 1.0f;

            switch (falloffType)
            {
                case Experimental.GlobalIllumination.FalloffType.InverseSquaredNoRangeAttenuation:
                case Experimental.GlobalIllumination.FalloffType.InverseSquared:
                    {
                        // compute the range at which the attenuated luminance is below the threshold
                        float estimatedRange = math.max(1.0f, math.ceil(math.sqrt(luminance / threshold)));
                        Debug.Assert(luminance * LightFalloffLUT.InverseSquaredFalloff(estimatedRange * estimatedRange) <= threshold);
                        return math.min(estimatedRange, range);
                    }
                case Experimental.GlobalIllumination.FalloffType.Linear:
                case Experimental.GlobalIllumination.FalloffType.Legacy:
                    return range;
            }

            return range;
        }

        public void UpdateLights(LightHandle[] lights, Span<LightDescriptor> lightDescriptors,
            bool respectLightLayers,
            bool autoEstimateLUTRange,
            MixedLightingMode mixedLightingMode)
        {
            Debug.Assert(lights.Length == lightDescriptors.Length);

            Dictionary<int, int> falloffHashToFalloffIndex = new();
            int falloffIndex = 0;

            // Convert the lights.
            for (int i = 0; i < lights.Length; i++)
            {
                ref readonly LightDescriptor light = ref lightDescriptors[i];

                PTLight newLight;
                newLight.position = light.Transform.GetPosition();
                newLight.intensity = light.LinearLightColor;
                newLight.type = (int)light.Type;
                newLight.castShadows = light.Shadows != LightShadows.None ? 1 : 0;
                newLight.forward = (light.Transform.rotation * Vector3.forward).normalized;
                newLight.attenuation = Vector4.one;
                newLight.up = (light.Transform.rotation * Vector3.up).normalized;
                newLight.right = (light.Transform.rotation * Vector3.right).normalized;
                newLight.width = light.AreaSize.x;
                newLight.height = light.AreaSize.y;
                newLight.spotAngle = light.SpotAngle;
                newLight.innerSpotAngle = light.InnerSpotAngle;
                newLight.layerMask = respectLightLayers ? light.CullingMask : uint.MaxValue;
                newLight.indirectScale = light.BounceIntensity;
                newLight.range = light.Range;
                newLight.shadowRadius = light.ShadowRadius;
                newLight.shadowMaskChannel = light.ShadowMaskChannel;

                switch (light.LightmapBakeType)
                {
                    case LightmapBakeType.Baked:
                    case LightmapBakeType.Mixed when mixedLightingMode == MixedLightingMode.Subtractive:
                        newLight.contributesToDirectLighting = 1;
                        break;
                    case LightmapBakeType.Mixed when mixedLightingMode == MixedLightingMode.Shadowmask:
                        // Fallback to baked behavior if we don't have a valid shadowmask channel
                        newLight.contributesToDirectLighting = newLight.shadowMaskChannel != -1 ? 0 : 1;
                        break;
                    case LightmapBakeType.Mixed when mixedLightingMode == MixedLightingMode.IndirectOnly:
                    case LightmapBakeType.Realtime:
                    default:
                        newLight.contributesToDirectLighting = 0;
                        break;
                }

                if (light.Type == LightType.Pyramid || light.Type == LightType.Spot)
                {
                    // aspect ratio is serialized in areaSize.x
                    float aspect = light.AreaSize.x;
                    float frustumHeight = 2.0f * Mathf.Tan(light.SpotAngle * 0.5f * Mathf.Deg2Rad);
                    float frustumWidth = frustumHeight * aspect;
                    newLight.width = frustumWidth;
                    newLight.height = frustumHeight;
                }

                if (light.Type == LightType.Pyramid || light.Type == LightType.Spot || light.Type == LightType.Point)
                {
                    Debug.Assert(light.FalloffType != Experimental.GlobalIllumination.FalloffType.Undefined);

                    float estimatedLUTRange = light.Range;
                    if (autoEstimateLUTRange)
                    {
                        // Guesstimate a good LUT range, such that the LUT covers the falloff up to a distance where it is nearly 0.
                        // The range can be any number and we don't want to stretch the LUT to some arbitrary range (most of which is practically 0).
                        estimatedLUTRange = EstimateLUTRange(light.Range, Lightmapping.LightmapIntegrationHelpers.Luminance(new Color(light.LinearLightColor.x, light.LinearLightColor.y, light.LinearLightColor.z, 1.0f)), light.FalloffType, 0.01f);
                    }

                    LightFalloffDesc falloffDesc = new LightFalloffDesc
                    {
                        LUTRange = estimatedLUTRange,
                        FalloffType = light.FalloffType
                    };
                    var falloffHash = falloffDesc.GetHashCode();
                    if (!falloffHashToFalloffIndex.TryGetValue(falloffHash, out newLight.falloffIndex))
                    {
                        // Add new falloff entry
                        newLight.falloffIndex = falloffIndex++;
                        falloffHashToFalloffIndex.Add(falloffHash, newLight.falloffIndex);
                        _lightState.LightFalloffDescs.Add(falloffDesc);
                    }
                }
                else
                    newLight.falloffIndex = -1;

#pragma warning disable 162 // Disable unreachable code warning

                // Light attenuation parameters and math from HDRP
                const bool applyRangeAttenuation = false;
                if (applyRangeAttenuation)
                {
                    newLight.attenuation.x = 1.0f / (light.Range * light.Range);
                    newLight.attenuation.y = 1.0f;
                }
                else
                {
                    const float hugeValue = 16777216.0f;
                    const float sqrtHuge = 4096.0f;
                    newLight.attenuation.x = sqrtHuge / (light.Range * light.Range);
                    newLight.attenuation.y = hugeValue;
                }
#pragma warning restore 162

                newLight.attenuation.z = 0.0f;
                if (light.Type == LightType.Spot)
                {
                    var spotAngle = light.SpotAngle;

                    var cosSpotOuterHalfAngle = Mathf.Clamp(Mathf.Cos(spotAngle * 0.5f * Mathf.Deg2Rad), 0.0f, 1.0f);
                    var cosSpotInnerHalfAngle = Mathf.Clamp(Mathf.Cos(0.5f * light.InnerSpotAngle * Mathf.Deg2Rad), 0.0f, 1.0f); // inner cone

                    var val = Mathf.Max(0.0001f, (cosSpotInnerHalfAngle - cosSpotOuterHalfAngle));
                    newLight.attenuation.z = 1.0f / val;
                    newLight.attenuation.w = -cosSpotOuterHalfAngle * newLight.attenuation.z;
                }

                newLight.cookieIndex = _materialPool.AddCookieTexture(light.CookieTexture);

                _lightState.LightHandleToLightListEntry[lights[i]] = newLight;
            }
        }

        public void RemoveLights(Span<LightHandle> lights)
        {
            foreach (var light in lights)
            {
                if (_lightState.LightHandleToLightListEntry.TryGetValue(light, out var ptLight))
                {
                    var cookieFaces = (ptLight.type == (int)LightType.Point) ? 6 : 1;
                    _materialPool.RemoveCookieTexture(cookieFaces, ptLight.cookieIndex);
                }

                _lightState.LightHandleSet.Remove(light);
                _lightState.LightHandleToLightListEntry.Remove(light);
            }
        }

        public void Build(Bounds sceneBounds, CommandBuffer cmdBuf, ref GraphicsBuffer scratchBuffer)
        {
            Debug.Assert(_rayTracingAccelerationStructure != null);
            _lightState.Build(sceneBounds, cmdBuf, EnableEmissiveSampling);
            _materialPool.Build(cmdBuf);
            _rayTracingAccelerationStructure.Build(cmdBuf, ref scratchBuffer);
        }

        public int GetInstanceID(InstanceHandle handle)
        {
            int[] ids;
            _rayTracingAccelerationStructure.GetInstanceIDs(handle.Value, out ids);
            return ids[0];
        }

    }
}
