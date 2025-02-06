using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine.PathTracing.Core;
using UnityEngine.LightTransport;
using UnityEngine.PathTracing.LightBakerBridge;
using UnityEngine.Rendering;
using UnityEngine.Rendering.UnifiedRayTracing;

namespace UnityEngine.PathTracing.Integration
{
    using MaterialHandle = Handle<World.MaterialDescriptor>;
    using LightHandle = Handle<World.LightDescriptor>;
    using InstanceHandle = Handle<World.InstanceKey>;

    internal struct LodsIdentifier
    {
        public LodsIdentifier(Int32 lodGroup, byte lodMask)
        {
            this.LodGroup = lodGroup;
            this.LodMask = lodMask;
        }

        public Int32 LodGroup;
        public byte LodMask;

        public override int GetHashCode() => HashCode.Combine(LodGroup, LodMask);
        public override bool Equals(object obj) => obj is LodsIdentifier other && other.LodGroup == LodGroup && other.LodMask == LodMask;
        public static readonly LodsIdentifier Invalid = new LodsIdentifier(-1, 0);
        public bool IsValid() => LodGroup != -1;
        public bool IsLodZero() { return (LodMask & 1) != 0; }
        public byte MinLodLevelMask() { return (byte)(LodMask & -LodMask); }
    }


    internal class UnityComputeWorld : IWorld
    {
        private World _pathTracingWorld;
        private GraphicsBuffer _scratchBuffer;
        private Mesh[] _meshes;
        private int _terrainMeshOffset;

        internal Mesh[] Meshes => _meshes;
        internal int TerrainMeshOffset => _terrainMeshOffset;
        private LightHandle[] _lightHandles;
        internal LightHandle[] LightHandles => _lightHandles;
        internal World PathTracingWorld => _pathTracingWorld;
        internal RayTracingContext RayTracingContext { get; private set; }
        private readonly List<Object> _temporaryObjects = new();
        Dictionary<int, List<LodInstanceBuildData>> _lodInstances = new();
        Dictionary<Int32, List<LodZeroInfo>> _lodgroupToLevelZeroInstances = new();

        struct LodInstanceBuildData
        {
            public int LodMask;
            public Mesh Mesh;
            public MaterialHandle[] Materials;
            public uint[] Masks;
            public Matrix4x4 LocalToWorldMatrix;
            public Bounds Bounds;
            public bool IsStatic;
            public RenderedGameObjectsFilter Filter;
        }

        struct LodZeroInfo
        {
            public InstanceHandle InstanceHandle;
            public uint[] Masks;
            public int LodMask;
        }

        private static Mesh MeshDataToMesh(in MeshData meshData)
        {
            ref readonly VertexData vertexData = ref meshData.vertexData;

            var outMesh = new Mesh();
            var outRawMeshArray = Mesh.AllocateWritableMeshData(1);
            var outRawMesh = outRawMeshArray[0];

            int vertexCount = (int)vertexData.vertexCount;
            List<VertexAttributeDescriptor> vertexLayout = new();
            if (vertexData.meshShaderChannelMask.HasFlag(MeshShaderChannelMask.Vertex))
                vertexLayout.Add(new VertexAttributeDescriptor(VertexAttribute.Position, VertexAttributeFormat.Float32, 3));
            if (vertexData.meshShaderChannelMask.HasFlag(MeshShaderChannelMask.Normal))
                vertexLayout.Add(new VertexAttributeDescriptor(VertexAttribute.Normal, VertexAttributeFormat.Float32, 3));
            if (vertexData.meshShaderChannelMask.HasFlag(MeshShaderChannelMask.TexCoord0))
                vertexLayout.Add(new VertexAttributeDescriptor(VertexAttribute.TexCoord0, VertexAttributeFormat.Float32, 2));
            if (vertexData.meshShaderChannelMask.HasFlag(MeshShaderChannelMask.TexCoord1))
                vertexLayout.Add(new VertexAttributeDescriptor(VertexAttribute.TexCoord1, VertexAttributeFormat.Float32, 2));
            outRawMesh.SetVertexBufferParams(vertexCount, vertexLayout.ToArray());
            outRawMesh.GetVertexData<byte>().CopyFrom(vertexData.data);

            outRawMesh.SetIndexBufferParams(meshData.indexBuffer.Length, IndexFormat.UInt32);
            outRawMesh.GetIndexData<uint>().CopyFrom(meshData.indexBuffer);

            int subMeshCount = meshData.subMeshAABB.Length;
            outRawMesh.subMeshCount = subMeshCount;
            for (int sm = 0; sm < outRawMesh.subMeshCount; sm++)
            {
                var smd = new SubMeshDescriptor((int)meshData.subMeshIndexOffset[sm], (int)meshData.subMeshIndexCount[sm]);
                outRawMesh.SetSubMesh(sm, smd);
            }

            Mesh.ApplyAndDisposeWritableMeshData(outRawMeshArray, outMesh);
            outMesh.RecalculateBounds();

            // MeshData from LightBaker contains UVs that are scaled to perfectly fit in the [0, 1] range.
            // The scale and offset used to achieve that are stored in the uvScaleOffset field.
            // Here we undo the scaling to get the original UVs of the input mesh.
            Vector2[] uv2 = outMesh.uv2;
            float4 uvScaleOffset = meshData.uvScaleOffset;
            Vector2 uvScale = new Vector2(uvScaleOffset.x, uvScaleOffset.y);
            Vector2 uvOffset = new Vector2(uvScaleOffset.z, uvScaleOffset.w);
            for (int i = 0; i < uv2.Length; i++)
            {
                uv2[i] = (uv2[i] - uvOffset) / uvScale;
            }
            outMesh.uv2 = uv2;

            return outMesh;
        }

        private static Texture2D CreateTexture2DFromTextureData(in TextureData textureData, string name = "CreateTexture2DFromTextureData")
        {
            Texture2D texture = new Texture2D((int)textureData.width, (int)textureData.height, TextureFormat.RGBAFloat, false, linear: true) { name = name };
            texture.SetPixelData(textureData.data, 0);
            texture.Apply(false, false);
            texture.wrapMode = TextureWrapMode.Clamp;
            return texture;
        }

        private static Texture CreateTextureFromCookieData(in CookieData textureData)
        {
            if (textureData.slices == 1)
            {
                Texture2D texture = new Texture2D((int)textureData.width, (int)textureData.height, TextureFormat.RGBA32, false, linear: true);
                texture.SetPixelData(textureData.textureData, 0);
                texture.Apply(false, false);
                return texture;

            }
            else
            {
                Cubemap texture = new Cubemap((int)textureData.width, TextureFormat.RGBA32, false);
                uint faceStride = textureData.width * textureData.width * textureData.pixelStride;

                for (int faceIndex = 0; faceIndex < textureData.slices; faceIndex++)
                    texture.SetPixelData(textureData.textureData, 0, (CubemapFace)faceIndex, faceIndex * (int)faceStride);

                texture.Apply(false, false);
                return texture;
            }
        }

        private static LightType LightBakerLightTypeToUnityLightType(LightBakerBridge.LightType type)
        {
            switch (type)
            {
                case LightBakerBridge.LightType.Directional: return LightType.Directional;
                case LightBakerBridge.LightType.Point: return LightType.Point;
                case LightBakerBridge.LightType.Spot: return LightType.Spot;
                case LightBakerBridge.LightType.Rectangle: return LightType.Rectangle;
                case LightBakerBridge.LightType.Disc: return LightType.Disc;
                case LightBakerBridge.LightType.SpotPyramidShape: return LightType.Pyramid;
                case LightBakerBridge.LightType.SpotBoxShape: return LightType.Box;
                default: throw new ArgumentException("Unknown light type");
            }
        }

        private static Experimental.GlobalIllumination.FalloffType LightBakerFalloffTypeToUnityFalloffType(FalloffType falloff)
        {
            switch (falloff)
            {
                case FalloffType.InverseSquared:
                    return Experimental.GlobalIllumination.FalloffType.InverseSquared;
                case FalloffType.InverseSquaredNoRangeAttenuation:
                    return Experimental.GlobalIllumination.FalloffType.InverseSquaredNoRangeAttenuation;
                case FalloffType.Linear:
                    return Experimental.GlobalIllumination.FalloffType.Linear;
                case FalloffType.Legacy:
                    return Experimental.GlobalIllumination.FalloffType.Legacy;
                case FalloffType.None:
                    return Experimental.GlobalIllumination.FalloffType.Undefined;
                default:
                    Debug.Assert(false, $"Unknown falloff type: {falloff}");
                    return Experimental.GlobalIllumination.FalloffType.Undefined;
            }
        }

        static void MultiValueDictAdd<TKey, TValue>(Dictionary<TKey, List<TValue>> dict, TKey key, TValue value)
        {
            List<TValue> values = null;
            if (dict.TryGetValue(key, out values))
            {
                values.Add(value);
            }
            else
            {
                values = new List<TValue>{ value };
                dict.Add(key, values);
            }
        }

        const uint renderingObjectLayer = 1 << 0;

        private static void InjectBakeInputData(World world, bool multiplyPunctualLightIntensityByPI, bool autoEstimateLUTRange, in BakeInput bakeInput,
            out Bounds sceneBounds, out Mesh[] meshes, out int terrainMeshOffset, out LightHandle[] lightHandles,
            ref Dictionary<int, List<LodInstanceBuildData>> lodInstances, ref Dictionary<Int32, List<LodZeroInfo>> lodgroupToLevelZeroInstances,
            List<Object> allocatedObjects)
        {
            sceneBounds = new Bounds();

            // Extract meshes
            meshes = new Mesh[bakeInput.meshData.Length + bakeInput.terrainData.Length];
            int meshIndex = 0;
            for (int i = 0; i < bakeInput.meshData.Length; i++)
            {
                meshes[meshIndex] = MeshDataToMesh(in bakeInput.meshData[meshIndex]);
                meshIndex++;
            }

            // Extract terrains
            terrainMeshOffset = meshIndex; // remember where the terrains start
            for (int i = 0; i < bakeInput.terrainData.Length; i++)
            {
                var heightMap = bakeInput.heightMapData[bakeInput.terrainData[i].heightMapIndex];
                var holeMap = bakeInput.terrainData[i].terrainHoleIndex >= 0 ? bakeInput.terrainHoleData[bakeInput.terrainData[i].terrainHoleIndex] : new TerrainHoleData();
                meshes[meshIndex] = Lightmapping.LightmapIntegrationHelpers.TerrainDataToMesh(in bakeInput.terrainData[i], in heightMap, in holeMap);
                meshIndex++;
            }
            allocatedObjects.AddRange(meshes);

            // Extract materials
            var perTexturePairMaterials = new World.MaterialDescriptor[bakeInput.albedoData.Length];
            Debug.Assert(bakeInput.albedoData.Length == bakeInput.emissiveData.Length);
            for (int i = 0; i < bakeInput.albedoData.Length; i++)
            {
                ref var material = ref perTexturePairMaterials[i];
                var baseTexture = CreateTexture2DFromTextureData(in bakeInput.albedoData[i], "World (albedo)");
                allocatedObjects.Add(baseTexture);
                var emissiveTexture = CreateTexture2DFromTextureData(in bakeInput.emissiveData[i], "World (emissive)");
                allocatedObjects.Add(emissiveTexture);
                material.Albedo = baseTexture;
                material.Emission = emissiveTexture;

                // Only mark emissive if it isn't the default black texture
                bool isEmissiveSinglePixel = bakeInput.emissiveData[i].data.Length == 1;
                bool isEmissiveBlack = math.all(bakeInput.emissiveData[i].data[0].xyz == float3.zero);
                if (isEmissiveSinglePixel && isEmissiveBlack)
                {
                    material.EmissionType = Core.MaterialPropertyType.None;
                    material.EmissionColor = Vector3.zero;
                }
                else
                {
                    material.EmissionType = Core.MaterialPropertyType.Texture;
                    material.EmissionColor = Vector3.one;
                }

                perTexturePairMaterials[i] = material;
            }

            // Certain material properties we can only determine by looking at individual submeshes of each instance.
            // Therefore, we must make a copy of the base material for each submesh. We create these materials here.
            var perInstanceSubMeshMaterials = new MaterialHandle[bakeInput.instanceData.Length][];
            var perInstanceSubMeshVisibility = new bool[bakeInput.instanceData.Length][];
            // To avoid needlessly creating duplicate materials, we also cache the materials we've already created:
            // Hashing texturePairIdx handles deduplicating by source mesh, scale and the set of source materials. We hash materialIdx
            // to identify the specific source material in the set associated with the texture pair (in case there are submeshes).
            Dictionary<(uint texturePairIdx, int materialIdx), MaterialHandle> materialCache = new();
            for (int instanceIdx = 0; instanceIdx < bakeInput.instanceData.Length; instanceIdx++)
            {
                // Get base (per-instance) material
                ref readonly InstanceData instanceData = ref bakeInput.instanceData[instanceIdx];
                uint texturePairIdx = bakeInput.instanceToTextureDataIndex[instanceIdx];
                ref readonly World.MaterialDescriptor baseMaterial = ref perTexturePairMaterials[texturePairIdx];

                // Make space for per-submesh materials and visibility
                perInstanceSubMeshMaterials[instanceIdx] = new MaterialHandle[instanceData.subMeshMaterialIndices.Length];
                perInstanceSubMeshVisibility[instanceIdx] = new bool[instanceData.subMeshMaterialIndices.Length];

                // Extract per-subMesh materials
                for (int subMeshIdx = 0; subMeshIdx < instanceData.subMeshMaterialIndices.Length; subMeshIdx++)
                {
                    int subMeshMaterialIdx = instanceData.subMeshMaterialIndices[subMeshIdx];

                    // If we've already created this material, use it
                    if (materialCache.TryGetValue((texturePairIdx, subMeshMaterialIdx), out MaterialHandle existingHandle))
                    {
                        perInstanceSubMeshMaterials[instanceIdx][subMeshIdx] = existingHandle;
                        perInstanceSubMeshVisibility[instanceIdx][subMeshIdx] = true;
                        continue;
                    }

                    // Copy the base material
                    World.MaterialDescriptor subMeshMaterial = baseMaterial;

                    // Get per-subMesh material properties, set them on the copy
                    if (-1 != subMeshMaterialIdx)
                    {
                        ref readonly MaterialData materialData = ref bakeInput.materialData[subMeshMaterialIdx];
                        subMeshMaterial.DoubleSidedGI = materialData.doubleSidedGI;

                        // Set transmission texture, if any
                        int transmissionDataIndex = bakeInput.materialToTransmissionDataIndex[subMeshMaterialIdx];
                        if (-1 != transmissionDataIndex)
                        {
                            ref readonly TextureData transmissionData = ref bakeInput.transmissionData[transmissionDataIndex];
                            ref readonly TextureProperties transmissionDataProperties = ref bakeInput.transmissionDataProperties[transmissionDataIndex];
                            Texture2D transmissiveTexture = CreateTexture2DFromTextureData(in transmissionData, "World (transmission)");
                            transmissiveTexture.wrapModeU = transmissionDataProperties.wrapModeU;
                            transmissiveTexture.wrapModeV = transmissionDataProperties.wrapModeV;
                            transmissiveTexture.filterMode = transmissionDataProperties.filterMode;
                            allocatedObjects.Add(transmissiveTexture);
                            subMeshMaterial.Transmission = transmissiveTexture;
                            subMeshMaterial.TransmissionScale = transmissionDataProperties.transmissionTextureST.scale;
                            subMeshMaterial.TransmissionOffset = transmissionDataProperties.transmissionTextureST.offset;
                            subMeshMaterial.TransmissionChannels = Core.TransmissionChannels.RGB;
                            subMeshMaterial.PointSampleTransmission = transmissionDataProperties.filterMode == FilterMode.Point;
                        }

                        // Apply the stretching operation that LightBaker applies - ensures that the UV layout fills the entire UV space
                        if (instanceData.meshIndex >= 0)
                        {
                            Vector4 uvScaleOffset = bakeInput.meshData[instanceData.meshIndex].uvScaleOffset;
                            Vector2 uvScale = new Vector2(uvScaleOffset.x, uvScaleOffset.y);
                            Vector2 uvOffset = new Vector2(uvScaleOffset.z, uvScaleOffset.w);
                            subMeshMaterial.AlbedoScale = uvScale;
                            subMeshMaterial.AlbedoOffset = uvOffset;
                            subMeshMaterial.EmissionScale = uvScale;
                            subMeshMaterial.EmissionOffset = uvOffset;
                        }
                        else
                        {
                            subMeshMaterial.AlbedoScale = Vector2.one;
                            subMeshMaterial.AlbedoOffset = Vector2.zero;
                            subMeshMaterial.EmissionScale = Vector2.one;
                            subMeshMaterial.EmissionOffset = Vector2.zero;
                        }
                    }
                    MaterialHandle addedHandle = world.AddMaterial(in subMeshMaterial, UVChannel.UV1);
                    materialCache.Add((texturePairIdx, subMeshMaterialIdx), addedHandle);
                    perInstanceSubMeshMaterials[instanceIdx][subMeshIdx] = addedHandle;
                    perInstanceSubMeshVisibility[instanceIdx][subMeshIdx] = subMeshMaterialIdx != -1;
                }
            }

            // Baking specific settings
            RenderedGameObjectsFilter filter = RenderedGameObjectsFilter.OnlyStatic;
            const bool isStatic = true;

            // Extract instances
            for (int i = 0; i < bakeInput.instanceData.Length; i++)
            {
                // Get materials
                ref readonly InstanceData instanceData = ref bakeInput.instanceData[i];
                var materials = perInstanceSubMeshMaterials[i];
                var visibility = perInstanceSubMeshVisibility[i];

                // Get other instance data
                float4x4 localToWorldFloat4x4 = instanceData.transform;
                Matrix4x4 localToWorldMatrix4x4 = new Matrix4x4(localToWorldFloat4x4.c0, localToWorldFloat4x4.c1, localToWorldFloat4x4.c2, localToWorldFloat4x4.c3);
                ShadowCastingMode shadowCastingMode = instanceData.castShadows ? ShadowCastingMode.On : ShadowCastingMode.Off;
                Mesh mesh;
                int globalMeshIndex = instanceData.meshIndex >= 0 ? instanceData.meshIndex : terrainMeshOffset + instanceData.terrainIndex; // the mesh array is a concatenation of the meshes and terrain meshes - figure out the right index
                Debug.Assert(globalMeshIndex >= 0 && globalMeshIndex < meshes.Length);
                mesh = meshes[globalMeshIndex];

                // Calculate bounds
                var bounds = new Bounds();
                foreach (Vector3 vert in mesh.vertices)
                {
                    bounds.Encapsulate(localToWorldMatrix4x4.MultiplyPoint(vert)); // TODO: transform the bounding box instead of looping verts (https://jira.unity3d.com/browse/GFXFEAT-667)
                }

                // Keep track of scene bounds as we go
                if (i == 0)
                    sceneBounds = bounds;
                else
                    sceneBounds.Encapsulate(bounds);

                // Get masks
                uint[] subMeshMasks = new uint[mesh.subMeshCount];
                for (int s = 0; s < mesh.subMeshCount; ++s)
                {
                    subMeshMasks[s] = visibility[s] ? World.GetInstanceMask(shadowCastingMode, isStatic, filter) : 0u;
                }

                if (instanceData.lodGroup != -1 && (instanceData.lodMask & 1) == 0)
                {
                    MultiValueDictAdd(lodInstances, instanceData.lodGroup, new LodInstanceBuildData
                    {
                        LodMask = instanceData.lodMask,
                        Mesh = mesh,
                        Materials = materials,
                        Masks = subMeshMasks,
                        LocalToWorldMatrix = localToWorldMatrix4x4,
                        Bounds = bounds,
                        IsStatic = isStatic,
                        Filter = filter
                    });
                    continue;
                }

                // Add instance to world
                const bool enableEmissiveSampling = true;
                var instanceHandle = world.AddInstance(
                    mesh,
                    materials,
                    subMeshMasks,
                    renderingObjectLayer,
                    in localToWorldMatrix4x4,
                    bounds,
                    isStatic,
                    filter,
                    enableEmissiveSampling);

                if (instanceData.lodGroup != -1 & ((instanceData.lodMask & 1) != 0))
                    MultiValueDictAdd(lodgroupToLevelZeroInstances, instanceData.lodGroup, new LodZeroInfo { LodMask = instanceData.lodMask, InstanceHandle = instanceHandle, Masks = subMeshMasks });
            }

            // Extract lights
            // Not yet supported:
            // - cookies
            // - soft shadows
            // TODO(pema.malling): https://jira.unity3d.com/browse/LIGHT-1759
            // TODO(pema.malling): https://jira.unity3d.com/browse/LIGHT-1764
            var lights = new World.LightDescriptor[bakeInput.lightData.Length];
            for (int i = 0; i < bakeInput.lightData.Length; i++)
            {
                ref readonly LightData lightData = ref bakeInput.lightData[i];

                // TODO(pema.malling): The following transform is only correct for linear color space :( https://jira.unity3d.com/browse/LIGHT-1763
                float maxColor = Mathf.Max(lightData.color.x, Mathf.Max(lightData.color.y, lightData.color.z));
                float maxIndirectColor = Mathf.Max(lightData.indirectColor.x, Mathf.Max(lightData.indirectColor.y, lightData.indirectColor.z));
                float bounceIntensity = maxColor <= 0 ? 0 : maxIndirectColor / maxColor;

                World.LightDescriptor lightDescriptor;
                lightDescriptor.Type = LightBakerLightTypeToUnityLightType(lightData.type);
                // We multiply intensity by PI, since LightBaker produces radiance estimates that are too bright by a factor of PI,
                // for light coming from punctual light sources. This isn't correct, but we need to match LightBaker's output.
                // Instead of adding incorrect code to the baker itself, we do the multiplication on the outside.
                float3 linearColor = (multiplyPunctualLightIntensityByPI && Util.IsPunctualLightType(lightDescriptor.Type)) ? lightData.color * Mathf.PI : lightData.color;
                lightDescriptor.LinearLightColor = linearColor;
                lightDescriptor.Shadows = lightData.castsShadows ? LightShadows.Hard : LightShadows.None;
                lightDescriptor.Transform = Matrix4x4.TRS(lightData.position, lightData.orientation, Vector3.one);
                lightDescriptor.ColorTemperature = 0;
                lightDescriptor.LightmapBakeType = lightData.mode == LightMode.Mixed ? LightmapBakeType.Mixed : LightmapBakeType.Baked;
                lightDescriptor.AreaSize = Vector2.one;
                lightDescriptor.SpotAngle = 0;
                lightDescriptor.InnerSpotAngle = 0;
                lightDescriptor.CullingMask = uint.MaxValue;
                lightDescriptor.BounceIntensity = bounceIntensity;
                lightDescriptor.Range = lightData.range;
                lightDescriptor.ShadowMaskChannel = (lightData.shadowMaskChannel < 4) ? (int)lightData.shadowMaskChannel : -1;
                lightDescriptor.UseColorTemperature = false;
                lightDescriptor.FalloffType = LightBakerFalloffTypeToUnityFalloffType(lightData.falloff);
                lightDescriptor.ShadowRadius = Util.IsPunctualLightType(lightDescriptor.Type) ? lightData.shape0 : 0.0f;
                lightDescriptor.CookieSize = lightData.cookieScale;
                lightDescriptor.CookieTexture = Util.IsCookieValid(lightData.cookieTextureIndex) ? CreateTextureFromCookieData(in bakeInput.cookieData[lightData.cookieTextureIndex]) : null;

                switch (lightDescriptor.Type)
                {
                    case LightType.Box:
                    case LightType.Rectangle:
                        lightDescriptor.AreaSize = new Vector2(lightData.shape0, lightData.shape1);
                        break;

                    case LightType.Disc:
                        lightDescriptor.AreaSize = new Vector2(lightData.shape0, lightData.shape0);
                        break;

                    case LightType.Spot:
                        lightDescriptor.SpotAngle = Mathf.Rad2Deg * lightData.coneAngle;
                        // TODO(pema.malling): This isn't quite correct, but very close. I couldn't figure out the math. See ExtractInnerCone(). https://jira.unity3d.com/browse/LIGHT-1727
                        lightDescriptor.InnerSpotAngle = Mathf.Rad2Deg * lightData.innerConeAngle;
                        break;

                    case LightType.Pyramid:
                        lightDescriptor.AreaSize = new Vector2(lightData.shape0, lightData.shape1);
                        lightDescriptor.SpotAngle = Mathf.Rad2Deg * lightData.coneAngle;
                        break;

                    case LightType.Directional:
                        lightDescriptor.AreaSize = new Vector2(lightData.coneAngle, lightData.innerConeAngle);
                        break;
                }

                lights[i] = lightDescriptor;
            }
            world.cdfLightPicking = false;
            lightHandles = world.AddLights(lights, false, autoEstimateLUTRange, bakeInput.lightingSettings.mixedLightingMode);

            // Setup environment light
            int envCubemapResolution = (int)bakeInput.environmentData.cubeResolution;
            var envCubemap = new Cubemap(envCubemapResolution, TextureFormat.RGBAFloat, false);
            for (int i = 0; i < 6; i++)
                envCubemap.SetPixelData(bakeInput.environmentData.cubeData, 0, (CubemapFace)i, envCubemapResolution * envCubemapResolution * i);
            envCubemap.Apply();
            var envCubemapMaterial = new Material(Shader.Find("Skybox/Cubemap"));
            envCubemapMaterial.SetTexture("_Tex", envCubemap);
            world.SetEnvironmentMaterial(envCubemapMaterial);
            allocatedObjects.Add(envCubemap);
            allocatedObjects.Add(envCubemapMaterial);
        }

        internal void PopulateUsingBakeInput(UnityComputeDeviceContext deviceContext, Rendering.UnifiedRayTracing.RayTracingBackend backend, bool multiplyPunctualLightIntensityByPI, bool autoEstimateLUTRange, in BakeInput bakeInput,
            World.ResourceLibrary worldResources)
        {
            // Create and init world
            _pathTracingWorld = new World();
            Debug.Assert(RayTracingContext.IsBackendSupported(backend), $"Backend {backend} is not supported!");
            RayTracingContext = new RayTracingContext(backend, Util.LoadOrCreateRayTracingResources());
            _pathTracingWorld.Init(RayTracingContext, worldResources);

            // Deserialize BakeInput, inject data into world
            InjectBakeInputData(_pathTracingWorld, multiplyPunctualLightIntensityByPI, autoEstimateLUTRange, in bakeInput, out Bounds sceneBounds, out _meshes, out _terrainMeshOffset, out _lightHandles, ref _lodInstances, ref _lodgroupToLevelZeroInstances, _temporaryObjects);

            // Build world with extracted data
            _pathTracingWorld.Build(sceneBounds, deviceContext.GetCommandBuffer(), ref _scratchBuffer);
        }

        public InstanceHandle[] AddLODInstances(CommandBuffer cmd, LodsIdentifier lodIdentifier, bool pathtracingShader)
        {
            if (!lodIdentifier.IsValid() || lodIdentifier.IsLodZero())
                return null;

            var currentLodInstancesBuildData = _lodInstances[lodIdentifier.LodGroup];
            var instanceHandles = new InstanceHandle[currentLodInstancesBuildData.Count];

            // add current lod instances
            int i = 0;
            uint currentLodLevel = lodIdentifier.MinLodLevelMask();
            foreach (LodInstanceBuildData lodBuildData in currentLodInstancesBuildData)
            {
                if ((lodBuildData.LodMask & currentLodLevel) == 0)
                    continue;

                Span<uint> currentLodMasks = stackalloc uint[lodBuildData.Masks.Length];
                GetCurrentLodInstanceMasks(pathtracingShader, lodBuildData.Masks, currentLodMasks);

                instanceHandles[i++] = _pathTracingWorld.AddInstance(lodBuildData.Mesh, lodBuildData.Materials, currentLodMasks, renderingObjectLayer, lodBuildData.LocalToWorldMatrix, lodBuildData.Bounds, lodBuildData.IsStatic, lodBuildData.Filter, true);
            }
            Array.Resize(ref instanceHandles, i);

            // update lod0 instance mask
            List<LodZeroInfo> lodZeroInstances;
            if (_lodgroupToLevelZeroInstances.TryGetValue(lodIdentifier.LodGroup, out lodZeroInstances))
            {
                foreach (var lodZeroInstance in lodZeroInstances)
                {
                    if ((lodZeroInstance.LodMask & currentLodLevel) != 0)
                        continue;

                    Span<uint> lodZeroMasks = stackalloc uint[lodZeroInstance.Masks.Length];
                    GetLodZeroInstanceMasks(pathtracingShader, lodZeroInstance.Masks, lodZeroMasks);

                    _pathTracingWorld.UpdateInstanceMask(lodZeroInstance.InstanceHandle, lodZeroMasks);
                }
            }

            return instanceHandles;
        }

        void GetLodZeroInstanceMasks(bool pathtracingShader, uint[] originalMasks, Span<uint> lodZeroMasks)
        {
            for (int j = 0; j < originalMasks.Length; j++)
            {
                // if we don't need bounce rays, we just make the lod0 invisible (instanceMask=0), so that only the current lod can be traced against
                // otherwise we set bits so that we can select one or the other using the raymask in the shader
                if (pathtracingShader)
                {
                    lodZeroMasks[j] = (uint)InstanceFlags.LOD_ZERO_FOR_LIGHTMAP_INSTANCE;
                    if ((originalMasks[j] & (uint)InstanceFlags.SHADOW_RAY_VIS_MASK) != 0)
                        lodZeroMasks[j] |= (uint)InstanceFlags.LOD_ZERO_FOR_LIGHTMAP_INSTANCE_SHADOW;
                }
                else
                {
                    lodZeroMasks[j] = 0;
                }
            }
        }

        void GetCurrentLodInstanceMasks(bool pathtracingShader, uint[] originalMasks, Span<uint> currentLodMasks)
        {
            for (int j = 0; j < originalMasks.Length; j++)
            {
                if (pathtracingShader)
                {
                    currentLodMasks[j] = (uint)InstanceFlags.CURRENT_LOD_FOR_LIGHTMAP_INSTANCE;
                    if ((originalMasks[j] & (uint)InstanceFlags.SHADOW_RAY_VIS_MASK) != 0)
                        currentLodMasks[j] |= (uint)InstanceFlags.CURRENT_LOD_FOR_LIGHTMAP_INSTANCE_SHADOW;
                }
                else
                {
                    currentLodMasks[j] = originalMasks[j];
                }
            }
        }

        public void RemoveLODInstances(CommandBuffer cmd, LodsIdentifier lodIdentifier, Span<InstanceHandle> currentLodInstanceHandles)
        {
            if (!lodIdentifier.IsValid() || lodIdentifier.IsLodZero())
                return;

            // Remove current lod instances
            foreach (var instanceHandle in currentLodInstanceHandles )
                _pathTracingWorld.RemoveInstance(instanceHandle);

            // Restore lod0 instances
            List<LodZeroInfo> lodZeroInstances;
            if (_lodgroupToLevelZeroInstances.TryGetValue(lodIdentifier.LodGroup, out lodZeroInstances))
            {
                foreach (var lodZeroInstance in lodZeroInstances)
                    _pathTracingWorld.UpdateInstanceMask(lodZeroInstance.InstanceHandle, lodZeroInstance.Masks);
            }
        }

        public void BuildAccelerationStructure(CommandBuffer cmd)
        {
            _pathTracingWorld.GetAccelerationStructure().Build(cmd, ref _scratchBuffer);
        }

        public void Dispose()
        {
            _pathTracingWorld?.Dispose();
            RayTracingContext?.Dispose();
            _scratchBuffer?.Dispose();

            foreach (var obj in _temporaryObjects)
                CoreUtils.Destroy(obj);
        }
    }

}
