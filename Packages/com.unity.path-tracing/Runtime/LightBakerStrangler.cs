using System.Collections.Generic;
using System.IO;
using System;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine.LightTransport;
using UnityEngine.PathTracing.Integration;
using UnityEngine.Rendering;
using UnityEngine.Assertions;
using UnityEngine.PathTracing.Core;
using UnityEngine.PathTracing.Lightmapping;
using UnityEngine.Rendering.UnifiedRayTracing;
using Unity.Collections.LowLevel.Unsafe;
using System.Runtime.InteropServices;
using System.Diagnostics;
using UnityEngine.Experimental.Rendering;

namespace UnityEngine.PathTracing.LightBakerBridge
{
    using static BakeLightmapDriver;

    internal class LightBakerStrangler
    {
        internal enum Result
        {
            Success,
            InitializeFailure,
            CreateDirectoryFailure,
            CreateLightmapFailure,
            AddResourcesToCacheFailure,
            InitializeExpandedBufferFailure,
            WriteToDiskFailure,
        };

        // Mirrors native side LightProbeOcclusion struct
        [StructLayout(LayoutKind.Sequential)]
        public unsafe struct LightProbeOcclusion
        {
            public const int MaxLightsPerProbe = 4;

            public fixed int ProbeOcclusionLightIndex[MaxLightsPerProbe];
            public fixed float Occlusion[MaxLightsPerProbe];
            public fixed sbyte OcclusionMaskChannel[MaxLightsPerProbe];

            public int GetProbeOcclusionLightIndex(int index) => ProbeOcclusionLightIndex[index];
            public void SetProbeOcclusionLightIndex(int index, int value) => ProbeOcclusionLightIndex[index] = value;
            public float GetOcclusion(int index) => Occlusion[index];
            public void SetOcclusion(int index, float value) => Occlusion[index] = value;
            public sbyte GetOcclusionMaskChannel(int index) => OcclusionMaskChannel[index];
            public void SetOcclusionMaskChannel(int index, sbyte value) => OcclusionMaskChannel[index] = value;

            public void SetDefaultValues()
            {
                for (int i = 0; i < MaxLightsPerProbe; ++i)
                {
                    ProbeOcclusionLightIndex[i] = -1;
                    Occlusion[i] = 0.0f;
                    OcclusionMaskChannel[i] = -1;
                }
            }
        }

        // File layout matches WriteInterleavedSHArrayToFile.
        private static bool SaveProbesToFileInterleaved(string filename, NativeArray<SphericalHarmonicsL2> shArray)
        {
            Debug.Assert(filename is not null, "Filename is null");

            string path = Path.GetDirectoryName(filename);
            Debug.Assert(path is not null, "path is null");
            var info = Directory.CreateDirectory(path);
            if (info.Exists == false)
                return false;

            // Write the number of probes to file as Int64.
            Int64 arraySize = shArray.Length;
            byte[] probeCountBytes = BitConverter.GetBytes(arraySize);
            List<byte> byteList = new();
            byteList.AddRange(probeCountBytes);

            // Write all probe coefficients, ordered by coefficient.
            const int sphericalHarmonicsL2CoeffCount = 9;
            byte[] floatPaddingBytes = BitConverter.GetBytes(0.0f);
            for (int coefficient = 0; coefficient < sphericalHarmonicsL2CoeffCount; ++coefficient)
            {
                for (int i = 0; i < shArray.Length; i++)
                {
                    SphericalHarmonicsL2 sh = shArray[i];
                    for (int rgb = 0; rgb < 3; rgb++)
                    {
                        float coefficientValue = sh[rgb, coefficient];
                        byte[] floatBytes = BitConverter.GetBytes(coefficientValue);
                        byteList.AddRange(floatBytes);
                    }
                    byteList.AddRange(floatPaddingBytes); // pad to match Vector4 size.
                }
            }
            File.WriteAllBytes(filename, byteList.ToArray());
            return true;
        }

        private static bool SaveArrayToFile<T>(string filename, int count, T[] array)
            where T : unmanaged
        {
            // Create output directory if it doesn't exist already.
            Debug.Assert(filename is not null, "Filename is null");
            string path = Path.GetDirectoryName(filename);
            Debug.Assert(path is not null, "path is null");
            var info = Directory.CreateDirectory(path);
            if (info.Exists == false)
                return false;

            // Prepare output buffer.
            int numElementBytes = array.Length * UnsafeUtility.SizeOf<T>();
            byte[] bytes = new byte[sizeof(Int64) + numElementBytes]; // + sizeof(Int64) for the count.

            // Write the number of elements to file as Int64.
            Int64 arraySize = count;
            byte[] countBytes = BitConverter.GetBytes(arraySize);
            Buffer.BlockCopy(countBytes, 0, bytes, 0, countBytes.Length);

            // Write contents of array to file. This is safe because of the unmanaged constraint on T.
            unsafe
            {
                fixed (T* arrayPtr = array)
                fixed (byte* bytesPtr = bytes)
                {
                    UnsafeUtility.MemCpy(bytesPtr + sizeof(Int64), arrayPtr, numElementBytes);
                }
            }
            File.WriteAllBytes(filename, bytes);
            return true;
        }

        private static HashSet<LightmapRequestOutputType> BitfieldToList(int bitfield)
        {
            var outputTypes = new HashSet<LightmapRequestOutputType>();

            // Loop through the enum values
            foreach (LightmapRequestOutputType channel in Enum.GetValues(typeof(LightmapRequestOutputType)))
            {
                // Check if the bitfield has the current value set
                if ((bitfield & (int) channel) == 0)
                    continue;
                // Add the value to the list
                if (channel != LightmapRequestOutputType.All)
                    outputTypes.Add(channel);
            }
            return outputTypes;
        }

        private static bool LightmapRequestOutputTypeToIntegratedOutputType(LightmapRequestOutputType type, out IntegratedOutputType integratedOutputType)
        {
            integratedOutputType = IntegratedOutputType.AO;
            switch (type)
            {
                case LightmapRequestOutputType.IrradianceIndirect:
                    integratedOutputType = IntegratedOutputType.Indirect;
                    return true;
                case LightmapRequestOutputType.IrradianceDirect:
                    integratedOutputType = IntegratedOutputType.Direct;
                    return true;
                case LightmapRequestOutputType.IrradianceEnvironment:
                    // Environment is part of indirect irradiance
                    return false;
                case LightmapRequestOutputType.Occupancy: // Occupancy do not need accumulation
                    return false;
                case LightmapRequestOutputType.Validity:
                    integratedOutputType = IntegratedOutputType.Validity;
                    return true;
                case LightmapRequestOutputType.DirectionalityIndirect:
                    integratedOutputType = IntegratedOutputType.DirectionalityIndirect;
                    return true;
                case LightmapRequestOutputType.DirectionalityDirect:
                    integratedOutputType = IntegratedOutputType.DirectionalityDirect;
                    return true;
                case LightmapRequestOutputType.AmbientOcclusion:
                    integratedOutputType = IntegratedOutputType.AO;
                    return true;
                case LightmapRequestOutputType.Shadowmask:
                    integratedOutputType = IntegratedOutputType.ShadowMask;
                    return true;
                case LightmapRequestOutputType.Normal: // Normals do not need accumulation
                    return false;
                case LightmapRequestOutputType.ChartIndex:
                    // LIGHT-1766: add chart index buffer support
                    return false;
                case LightmapRequestOutputType.OverlapPixelIndex:
                    // LIGHT-1767: add overlap buffer for visualization
                    return false;
            }
            Debug.Assert(false, $"Error unknown LightmapRequestOutputType {type}.");
            return false;
        }

        // We can ignore the G-Buffer data part of atlassing, we are using stochastic sampling instead.
        internal static LightmapDesc[] PopulateLightmapDescsFromAtlassing(in PVRAtlassingData atlassing, in InstanceData[] instanceData, in Mesh[] meshes, int terrainMeshOffset)
        {
            // Compute the tight UV scale and offset for each mesh.
            Vector2[] uvBoundsSizes = new Vector2[meshes.Length];
            Vector2[] uvBoundsOffsets = new Vector2[meshes.Length];
            for (int i = 0; i < meshes.Length; ++i)
            {
                if (meshes[i].uv2.Length == 0)
                    LightmapIntegrationHelpers.ComputeUVBounds(meshes[i].uv, out uvBoundsSizes[i], out uvBoundsOffsets[i]);
                else
                    LightmapIntegrationHelpers.ComputeUVBounds(meshes[i].uv2, out uvBoundsSizes[i], out uvBoundsOffsets[i]);
            }

            int atlasCount = atlassing.m_AtlasHashToGBufferHash.Count;
            var lightmapDescs = new LightmapDesc[atlasCount];
            foreach (var atlasIdToAtlasHash in atlassing.m_AtlasIdToAtlasHash)
            {
                int atlasId = atlasIdToAtlasHash.m_AtlasId;

                // Get the array of object ID hashes for the given atlas.
                bool found = atlassing.m_AtlasHashToObjectIDHashes.TryGetValue(
                        atlasIdToAtlasHash.m_AtlasHash, out IndexHash128[] objectIDHashes);
                Debug.Assert(found, $"Couldn't find object ID hashes for atlas {atlasIdToAtlasHash.m_AtlasHash}.");

                // Get the GBuffer data for the given atlas, so we can use it to find instance data like IDs and transforms.
                found = atlassing.m_AtlasHashToGBufferHash.TryGetValue(
                    atlasIdToAtlasHash.m_AtlasHash, out _);
                if (found == false)
                    continue; // Skip atlasses without GBuffers, they are used for SSD objects.

                uint lightmapResolution = (uint)atlassing.m_AtlasSizes[atlasId].width;
                Debug.Assert(lightmapResolution == atlassing.m_AtlasSizes[atlasId].height,
                    "The following code assumes that we always have square lightmaps.");
                uint antiAliasingSampleCount = (uint)((int)atlasIdToAtlasHash.m_BakeParameters.supersamplingMultiplier * (int)atlasIdToAtlasHash.m_BakeParameters.supersamplingMultiplier);
                var lightmapDesc = new LightmapDesc
                {
                    AntiAliasingSampleCount = antiAliasingSampleCount,
                    Resolution = lightmapResolution,
                    PushOff = atlasIdToAtlasHash.m_BakeParameters.pushOff
                };
                int instanceCount = objectIDHashes.Length;
                var instances = new Instance[instanceCount];
                int instanceCounter = 0;
                foreach (var instanceHash in objectIDHashes)
                {
                    // Get the AtlassedInstanceData for the instance.
                    found = atlassing.m_InstanceAtlassingData.TryGetValue(instanceHash,
                        out AtlassedInstanceData atlassedInstanceData);
                    Debug.Assert(found, $"Didn't find AtlassedInstanceData for instance {instanceHash}.");
                    Debug.Assert(atlassedInstanceData.m_AtlasId == atlasId, "Atlas ID mismatch.");

                    // Get the instance from bakeInput and create a Mesh.
                    int bakeInputInstanceIndex = (int)instanceHash.Index;
                    ref readonly InstanceData instance = ref instanceData[bakeInputInstanceIndex];
                    int meshIndex = instance.meshIndex >= 0 ? instance.meshIndex : terrainMeshOffset + instance.terrainIndex;
                    Debug.Assert(meshIndex >= 0);
                    var mesh = meshes[meshIndex];
                    var uvBoundsSize = uvBoundsSizes[meshIndex];
                    var uvBoundsOffset = uvBoundsOffsets[meshIndex];
                    LightmapIntegrationHelpers.ComputeOccupiedTexelRegionForInstance(
                        lightmapResolution, lightmapResolution, atlassedInstanceData.m_LightmapST, uvBoundsSize, uvBoundsOffset,
                        out Vector4 normalizedOccupiedST, out Vector2Int occupiedTexelSize, out Vector2Int occupiedTexelOffset);
                    instances[instanceCounter] = new Instance();
                    instances[instanceCounter].Build(mesh, normalizedOccupiedST, occupiedTexelSize, occupiedTexelOffset, instance.transform, instance.receiveShadows, new LodsIdentifier(instance.lodGroup, instance.lodMask));
                    ++instanceCounter;
                }

                lightmapDesc.Instances = instances;
                lightmapDescs[atlasId] = lightmapDesc;
            }

            return lightmapDescs;
        }

        private static void MakeOutputFolderPathsFullyQualified(ProbeRequest[] probeRequestsToFixUp, string bakeOutputFolderPath)
        {
            for (int i = 0; i < probeRequestsToFixUp.Length; i++)
            {
                ref ProbeRequest request = ref probeRequestsToFixUp[i];
                request.outputFolderPath = Path.GetFullPath(request.outputFolderPath, bakeOutputFolderPath);
            }
        }

        private static void MakeOutputFolderPathsFullyQualified(LightmapRequest[] requestsToFixUp, string bakeOutputFolderPath)
        {
            for (int i = 0; i < requestsToFixUp.Length; i++)
            {
                ref LightmapRequest request = ref requestsToFixUp[i];
                request.outputFolderPath = Path.GetFullPath(request.outputFolderPath, bakeOutputFolderPath);
            }
        }

        internal static bool Bake(string bakeInputPath, string lightmapRequestsPath, string lightProbeRequestsPath, string bakeOutputFolderPath, BakeProgressState progressState)
        {
            // Read BakeInput and requests from LightBaker
            if (!BakeInputSerialization.Deserialize(bakeInputPath, out BakeInput bakeInput))
                return false;
            if (!BakeInputSerialization.Deserialize(lightmapRequestsPath, out LightmapRequestData lightmapRequestData))
                return false;
            if (!BakeInputSerialization.Deserialize(lightProbeRequestsPath, out ProbeRequestData probeRequestData))
                return false;

            // Change output folder paths from relative to absolute
            MakeOutputFolderPathsFullyQualified(probeRequestData.requests, bakeOutputFolderPath);
            MakeOutputFolderPathsFullyQualified(lightmapRequestData.requests, bakeOutputFolderPath);

            IntegrationSettings integrationSettings = IntegrationSettings.Default;

            // Setup state for baking
            using UnityComputeDeviceContext deviceContext = new();
            bool initOk = deviceContext.Initialize();
            Assert.IsTrue(initOk, "Failed to initialize DeviceContext.");
            using UnityComputeWorld world = new();
            RayTracingBackend backend = integrationSettings.Backend;
            const bool useLegacyBakingBehavior = true;
            const bool autoEstimateLUTRange = true;
            var worldResources = new World.ResourceLibrary();
            worldResources.Load();
            world.PopulateUsingBakeInput(deviceContext, backend, useLegacyBakingBehavior, autoEstimateLUTRange, in bakeInput, worldResources);

            LightmapBakeSettings lightmapBakeSettings = GetLightmapBakeSettings(bakeInput);
            // Build array of lightmap descriptors based on the atlassing data and instances.
            LightmapDesc[] lightmapDescriptors = PopulateLightmapDescsFromAtlassing(in lightmapRequestData.atlassing, in bakeInput.instanceData, world.Meshes, world.TerrainMeshOffset);
            SortInstancesByMeshAndResolution(lightmapDescriptors);

            ulong probeWorkSteps = CalculateWorkStepsForProbeRequests(in bakeInput, in probeRequestData);
            ulong lightmapWorkSteps = CalculateWorkStepsForLightmapRequests(in lightmapRequestData, lightmapDescriptors, lightmapBakeSettings);
            progressState.SetTotalWorkSteps(probeWorkSteps + lightmapWorkSteps);

            if (!ExecuteProbeRequests(in bakeInput, in probeRequestData, deviceContext, useLegacyBakingBehavior, world, bakeInput.lightingSettings.maxBounces, progressState))
                return false;

            if (lightmapRequestData.requests.Length <= 0)
                return true;

            // Populate resources structure
            LightmapResourceLibrary resources = new();
            resources.Load(world.RayTracingContext);

            if (ExecuteLightmapRequests(in lightmapRequestData, deviceContext, world, integrationSettings, useLegacyBakingBehavior, resources, progressState, lightmapDescriptors, lightmapBakeSettings) != Result.Success)
                return false;

            CoreUtils.Destroy(resources.UVFallbackBufferGenerationMaterial);

            return true;
        }

        internal static ulong CalculateWorkStepsForProbeRequests(in BakeInput bakeInput, in ProbeRequestData probeRequestData)
        {
            (uint directSampleCount, uint effectiveIndirectSampleCount) = GetProbeSampleCounts(bakeInput.lightingSettings.probeSampleCounts);
            ulong calculatedWorkSteps = 0;
            foreach (ProbeRequest probeRequest in probeRequestData.requests)
                calculatedWorkSteps += CalculateProbeWorkSteps(probeRequest.count, probeRequest.outputTypeMask, directSampleCount, effectiveIndirectSampleCount, bakeInput.lightingSettings.mixedLightingMode != MixedLightingMode.IndirectOnly,
                    bakeInput.lightingSettings.maxBounces);

            return calculatedWorkSteps;
        }

        private static ulong CalculateProbeWorkSteps(ulong count, ProbeRequestOutputType outputTypeMask, uint directSampleCount, uint effectiveIndirectSampleCount, bool usesProbeOcclusion, uint bounceCount)
        {
            ulong workSteps = 0;
            if (outputTypeMask.HasFlag(ProbeRequestOutputType.RadianceIndirect))
                workSteps += ProbeIntegrator.CalculateWorkSteps((uint)count, effectiveIndirectSampleCount, bounceCount);
            if (outputTypeMask.HasFlag(ProbeRequestOutputType.RadianceDirect))
                workSteps += ProbeIntegrator.CalculateWorkSteps((uint)count, directSampleCount, 0);
            if (outputTypeMask.HasFlag(ProbeRequestOutputType.Validity))
                workSteps += ProbeIntegrator.CalculateWorkSteps((uint)count, effectiveIndirectSampleCount, 0);
            if (outputTypeMask.HasFlag(ProbeRequestOutputType.LightProbeOcclusion) && usesProbeOcclusion)
                workSteps += ProbeIntegrator.CalculateWorkSteps((uint)count, effectiveIndirectSampleCount, 0);

            return workSteps;
        }

        internal static ulong CalculateWorkStepsForLightmapRequests(in LightmapRequestData lightmapRequestData, LightmapDesc[] lightmapDescriptors, LightmapBakeSettings lightmapBakeSettings)
        {
            ulong calculatedWorkSteps = 0;
            foreach (LightmapRequest r in lightmapRequestData.requests)
            {
                ref readonly var request = ref r;
                if (request.lightmapCount == 0)
                    continue;
                for (int lightmapIndex = 0; lightmapIndex < request.lightmapCount; lightmapIndex++)
                {
                    LightmapDesc currentLightmapDesc = lightmapDescriptors[lightmapIndex];
                    Dictionary<IntegratedOutputType, RequestedSubOutput> requestedLightmapTypes = GetRequestedIntegratedOutputTypes(request);
                    foreach (IntegratedOutputType lightmapType in requestedLightmapTypes.Keys)
                    {
                        uint sampleCount = lightmapBakeSettings.GetSampleCount(lightmapType);
                        foreach (Instance instance in currentLightmapDesc.Instances)
                        {
                            uint instanceWidth = (uint)instance.TexelSize.x;
                            uint instanceHeight = (uint)instance.TexelSize.y;

                            calculatedWorkSteps += CalculateIntegratedLightmapWorkSteps(sampleCount, instanceWidth * instanceHeight, lightmapType, lightmapBakeSettings.BounceCount, 1);
                        }
                    }
                    HashSet<LightmapRequestOutputType> requestedNonIntegratedLightmapTypes = GetRequestedNonIntegratedOutputTypes(request);
                    foreach (LightmapRequestOutputType _ in requestedNonIntegratedLightmapTypes)
                        calculatedWorkSteps += CalculateNonIntegratedLightmapWorkSteps(currentLightmapDesc.Resolution * currentLightmapDesc.Resolution);
                }
            }

            return calculatedWorkSteps;
        }

        private static ulong CalculateIntegratedLightmapWorkSteps(uint samplesPerTexel, uint chunkSize, IntegratedOutputType outputType, uint bounces, uint multiplier)
        {
            uint bouncesMultiplier = outputType == IntegratedOutputType.Indirect
                ? 0 == bounces ? 1 : bounces
                : 1;

            return samplesPerTexel*chunkSize*bouncesMultiplier*multiplier;
        }

        private static ulong CalculateNonIntegratedLightmapWorkSteps(uint lightmapResolution) => lightmapResolution;

        internal static LightmapBakeSettings GetLightmapBakeSettings(in BakeInput bakeInput)
        {
            // Lightmap settings
            LightmapBakeSettings lightmapBakeSettings = new()
            {
                AOSampleCount = math.max(0, bakeInput.lightingSettings.lightmapSampleCounts.indirectSampleCount),
                DirectSampleCount = math.max(0, bakeInput.lightingSettings.lightmapSampleCounts.directSampleCount),
                IndirectSampleCount = math.max(0, bakeInput.lightingSettings.lightmapSampleCounts.indirectSampleCount),
                BounceCount = math.max(0, bakeInput.lightingSettings.maxBounces),
                AOMaxDistance = math.max(0.0f, bakeInput.lightingSettings.aoDistance)
            };
            lightmapBakeSettings.ValiditySampleCount = lightmapBakeSettings.IndirectSampleCount;
            // Note: The new path tracing core uses the same samples for indirect illumination and the environment.
            // If a user requests increased env samples in the UI, to get equivalent quality as the old backends, we re-direct them to the indirect samples
            lightmapBakeSettings.IndirectSampleCount = math.max(lightmapBakeSettings.IndirectSampleCount, bakeInput.lightingSettings.lightmapSampleCounts.environmentSampleCount);
            return lightmapBakeSettings;
        }

        [Flags]
        private enum RequestedSubOutput
        {
            PrimaryTexture = 1 << 0,
            DirectionalityTexture = 1 << 1
        }

        private struct HashedInstanceIndex : IComparable<HashedInstanceIndex>
        {
            public Instance instance;
            public int texelCount;
            public int offsetX;
            public int offsetY;
            public int hashCode;

            public int CompareTo(HashedInstanceIndex other)
            {
                    int size = other.texelCount - texelCount;
                    if (size != 0)
                        return size;
                    int xOffset = other.offsetX - offsetX;
                    if (xOffset != 0)
                        return xOffset;
                    int yOffset = offsetY - other.offsetY;
                    if (yOffset != 0)
                        return yOffset;
                    return hashCode - other.hashCode;
            }
        };

        internal static void SortInstancesByMeshAndResolution(LightmapDesc[] lightmapDescriptors)
        {
            int lightmapDescIndex = 0;
            foreach (var lightmapDesc in lightmapDescriptors)
            {
                HashedInstanceIndex[] hashedInstances = new HashedInstanceIndex[lightmapDesc.Instances.Length];
                int instanceIndex = 0;
                foreach (var instance in lightmapDesc.Instances)
                {
                    int hashCode = System.HashCode.Combine(UVMesh.GetHashCode(instance.Mesh), instance.TexelSize.x, instance.TexelSize.y);
                    hashedInstances[instanceIndex] = new();
                    hashedInstances[instanceIndex].instance = instance;
                    hashedInstances[instanceIndex].texelCount = instance.TexelSize.x * instance.TexelSize.y;
                    hashedInstances[instanceIndex].offsetX = instance.TexelOffset.x;
                    hashedInstances[instanceIndex].offsetY = instance.TexelOffset.y;
                    hashedInstances[instanceIndex].hashCode = hashCode;
                    instanceIndex++;
                }
                Array.Sort(hashedInstances);
                instanceIndex = 0;
                foreach (var hashedInstance in hashedInstances)
                {
                    lightmapDescriptors[lightmapDescIndex].Instances[instanceIndex++] = hashedInstance.instance;
                }
                lightmapDescIndex++;
            }
        }

        // Gets all meshes used by the specified instances, ordered by the first instance they are used in.
        private static List<Mesh> GetMeshesInInstanceOrder(LightmapDesc[] lightmapDescriptors)
        {
            List<Mesh> sortedMeshes = new List<Mesh>();
            HashSet<Mesh> seenMeshes = new HashSet<Mesh>();
            foreach (var lightmapDesc in lightmapDescriptors)
            {
                foreach (var instance in lightmapDesc.Instances)
                {
                    if (seenMeshes.Add(instance.Mesh))
                    {
                        sortedMeshes.Add(instance.Mesh);
                    }
                }
            }
            return sortedMeshes;
        }

        private static bool AnyLightmapRequestHasOutput(LightmapRequest[] requests, LightmapRequestOutputType type)
        {
            foreach (var req in requests)
            {
                if (req.outputTypeMask.HasFlag(type))
                {
                    return true;
                }
            }

            return false;
        }

        private static bool EnsureInstanceInCacheAndClearExistingEntries(CommandBuffer cmd,
            LightmappingContext lightmappingContext,
            Instance instance)
        {
            var instances = new[] { instance };
            if (!lightmappingContext.ResourceCache.CacheIsHot(instances))
            {
                // Build the required resources for the current instance
                GraphicsHelpers.Flush(cmd); // need to flush as we are removing resources from the cache which might be in use
                lightmappingContext.ResourceCache.FreeResources(instances);

                if (!lightmappingContext.ResourceCache.AddResources(
                    instances, lightmappingContext.World.RayTracingContext, cmd,
                    lightmappingContext.IntegratorContext.UVFallbackBufferBuilder))
                {
                    return false;
                }
                // Flush as there can be a substantial amount of work done in the commandbuffer
                GraphicsHelpers.Flush(cmd);
            }

            return true;
        }

        private static bool GetInstanceUVResources(
            CommandBuffer cmd,
            LightmappingContext lightmappingContext,
            Instance instance,
            out UVMesh uvMesh,
            out UVAccelerationStructure uvAS,
            out UVFallbackBuffer uvFallbackBuffer)
        {
            uvMesh = default;
            uvAS = default;
            uvFallbackBuffer = default;

            if (!EnsureInstanceInCacheAndClearExistingEntries(cmd, lightmappingContext, instance))
                return false;

            bool gotResources = lightmappingContext.ResourceCache.GetResources(new[] { instance },
                out UVMesh[] uvMeshes, out UVAccelerationStructure[] uvAccelerationStructures, out UVFallbackBuffer[] uvFallbackBuffers);
            if (!gotResources)
                return false;

            uvMesh = uvMeshes[0];
            uvAS = uvAccelerationStructures[0];
            uvFallbackBuffer = uvFallbackBuffers[0];

            return true;
        }

        private static void PrepareLodInstances(CommandBuffer cmd, LightmappingContext lightmappingContext, Instance instance, bool isPathTracingPass)
        {
            if (instance.LodIdentifier.IsValid() && !instance.LodIdentifier.IsLodZero())
            {
                // AddLODInstances calls _pathTracingWorld.Add/RemoveInstance(...) that doesn't take a cmd buffer arg. Need to flush as cmdbuffer and immediate calls cannot be mixed.
                GraphicsHelpers.Flush(cmd);

                lightmappingContext.lightmapCurrentLodInstances = lightmappingContext.World.AddLODInstances(cmd, instance.LodIdentifier, isPathTracingPass);
                lightmappingContext.World.BuildAccelerationStructure(cmd);
            }
        }

        private static void ClearLodInstances(CommandBuffer cmd, LightmappingContext lightmappingContext, Instance instance)
        {
            if (instance.LodIdentifier.IsValid() && !instance.LodIdentifier.IsLodZero())
            {
                // RemoveLODInstances calls _pathTracingWorld.Add/RemoveInstance(...) that doesn't take a cmd buffer arg. Need to flush as cmdbuffer and immediate calls cannot be mixed.
                GraphicsHelpers.Flush(cmd);

                lightmappingContext.World.RemoveLODInstances(cmd, instance.LodIdentifier, lightmappingContext.lightmapCurrentLodInstances);
                lightmappingContext.World.BuildAccelerationStructure(cmd);
                lightmappingContext.lightmapCurrentLodInstances = null;
            }
        }

         private static Result IntegrateLightmapInstance(CommandBuffer cmd,
            int lightmapIndex,
            IntegratedOutputType integratedOutputType,
            Instance instance,
            LightmappingContext lightmappingContext,
            UVAccelerationStructure uvAS,
            UVFallbackBuffer uvFallbackBuffer,
            IntegrationSettings integrationSettings,
            LightmapBakeSettings lightmapBakeSettings,
            BakeProgressState progressState,
            LightmapIntegrationHelpers.GPUSync gpuSync,
            bool debugDispatches)
        {
            Debug.Assert(!lightmappingContext.ExpandedBufferNeedsUpdating(lightmapBakeSettings.ExpandedBufferSize), "The integration data must be allocated at this point.");

            // Bake the lightmap instances
            int bakeDispatches = 0;
            LightmapBakeState bakeState = new();
            bakeState.Init();
            uint samples = 0;
            bool isInstanceDone = false;
            uint dispatchCount = 0;
            uint instanceWidth = (uint)instance.TexelSize.x;
            uint instanceHeight = (uint)instance.TexelSize.y;

            // accumulate the instance
            Stopwatch instanceFlushStopwatch = Stopwatch.StartNew();
            Stopwatch dispatchStopwatch = Stopwatch.StartNew();
            do
            {
                if (debugDispatches)
                {
                    gpuSync.Sync(cmd);
                    dispatchStopwatch.Restart();
                    Console.WriteLine($"Begin pass {bakeDispatches} for lm: {lightmapIndex}, type: {integratedOutputType}, sample count: {bakeState.SampleIndex}, res: [{instance.TexelSize.x} x {instance.TexelSize.y}], offset: [{instance.TexelOffset.x} x {instance.TexelOffset.y}].");
                }

                bool isInstanceStart = (bakeState.SampleIndex == 0);
                if (isInstanceStart)
                {
                    bool isPathTracingPass = integratedOutputType == IntegratedOutputType.Indirect || integratedOutputType == IntegratedOutputType.DirectionalityIndirect;
                    PrepareLodInstances(cmd, lightmappingContext, instance, isPathTracingPass);
                }

                // Enqueue some baking work in the command buffer.
                cmd.BeginSample($"Bake {integratedOutputType}");
                dispatchCount++;

                uint passSamplesPerTexel = BakeLightmapDriver.AccumulateLightmapInstance(
                    bakeState,
                    instance,
                    lightmapBakeSettings,
                    integratedOutputType,
                    lightmappingContext,
                    uvAS,
                    uvFallbackBuffer,
                    out uint chunkSize,
                    out isInstanceDone);

                samples += instanceWidth * instanceHeight * passSamplesPerTexel;
                cmd.EndSample($"Bake {integratedOutputType}");

                if (isInstanceDone)
                {
                    ClearLodInstances(cmd, lightmappingContext, instance);
                }

                if (debugDispatches)
                {
                    gpuSync.Sync(cmd);
                    dispatchStopwatch.Stop();
                    Console.WriteLine($"Finished pass {bakeDispatches}. Elapsed ms: {dispatchStopwatch.ElapsedMilliseconds}");
                }

                bakeDispatches++;

                // Execute the baking work scheduled in BakeLightmaps.
                if (bakeDispatches % integrationSettings.MaxDispatchesPerFlush != 0 && !isInstanceDone)
                    continue;

                // Chip-off work steps based on the chunk done so far
                ulong completedWorkSteps =
                    CalculateIntegratedLightmapWorkSteps(passSamplesPerTexel, chunkSize, integratedOutputType, lightmapBakeSettings.BounceCount, integrationSettings.MaxDispatchesPerFlush);
                gpuSync.RequestAsyncReadback(cmd, _ => progressState.IncrementCompletedWorkSteps(completedWorkSteps));
                GraphicsHelpers.Flush(cmd);

                if (!debugDispatches)
                    continue;

                gpuSync.Sync(cmd);
                instanceFlushStopwatch.Stop();
                Console.WriteLine($"Bake dispatch flush -> Instance: {instance.Mesh.GetInstanceID()}, dispatches {dispatchCount}, Samples: \t{samples}\t. Elapsed ms:\t{instanceFlushStopwatch.ElapsedMilliseconds}");
                samples = 0;
                dispatchCount = 0;
                instanceFlushStopwatch.Restart();

                //LightmapIntegrationHelpers.WriteRenderTexture(cmd, $"Temp/lm{lightmapIndex}_type{lightmapType}_pass{bakeDispatches}.r2d", lightmappingContext.AccumulatedOutput, lightmappingContext.AccumulatedOutput.width, lightmappingContext.AccumulatedOutput.height);
            }
            while (isInstanceDone == false);

            return Result.Success;
        }

        internal static Result ExecuteLightmapRequests(in LightmapRequestData lightmapRequestData, UnityComputeDeviceContext deviceContext,
            UnityComputeWorld world, IntegrationSettings integrationSettings,
            bool useLegacyBakingBehavior, LightmapResourceLibrary resources,
            BakeProgressState progressState, LightmapDesc[] lightmapDescriptors, LightmapBakeSettings lightmapBakeSettings)
        {
            using var lightmappingContext = new LightmappingContext();

            bool debugDispatches = integrationSettings.DebugDispatches;

            (int width, int height)[] atlasSizes = lightmapRequestData.atlassing.m_AtlasSizes;
            int initialLightmapResolution = atlasSizes.Length > 0 ? atlasSizes[0].width : 1024;
            if (!lightmappingContext.Initialize(deviceContext, initialLightmapResolution, initialLightmapResolution, world, resources))
                return Result.InitializeFailure;
            lightmappingContext.IntegratorContext.Initialize(resources, useLegacyBakingBehavior, !useLegacyBakingBehavior);

            // Chart identification happens in multithreaded fashion on the CPU. We start it immediately so it can run in tandem with other work.
            bool usesChartIdentification = AnyLightmapRequestHasOutput(lightmapRequestData.requests, LightmapRequestOutputType.ChartIndex);
            using ParallelChartIdentification chartIdentification = usesChartIdentification ? new ParallelChartIdentification(GetMeshesInInstanceOrder(lightmapDescriptors)) : null;
            if (usesChartIdentification)
                chartIdentification.Start();

            if (debugDispatches)
            {
                for (int i = 0; i < lightmapDescriptors.Length; ++i)
                {
                    Console.WriteLine($"Desc:");
                    int instanceIndex = 0;
                    foreach (var instance in lightmapDescriptors[i].Instances)
                        Console.WriteLine($"  Instance[{instanceIndex++}]: {instance.Mesh.GetInstanceID()}, res: [{instance.TexelSize.x} x {instance.TexelSize.y}], offset: [{instance.TexelOffset.x} x {instance.TexelOffset.y}].");
                }
            }

            CommandBuffer cmd = lightmappingContext.GetCommandBuffer();
            using LightmapIntegrationHelpers.GPUSync gpuSync = new(); // used for sync points in debug mode
            gpuSync.Create();

            // process requests
            for (int requestIndex = 0; requestIndex < lightmapRequestData.requests.Length; requestIndex++)
            {
                ref readonly var request = ref lightmapRequestData.requests[requestIndex];

                if (request.lightmapCount == 0)
                    continue;

                var createDirResult = Directory.CreateDirectory(request.outputFolderPath);
                if (createDirResult.Exists == false)
                    return Result.CreateDirectoryFailure;

                Dictionary<IntegratedOutputType, RequestedSubOutput> integratedRequestOutputs = GetRequestedIntegratedOutputTypes(request);
                bool needsNormalsForDirectionality = request.outputTypeMask.HasFlag(LightmapRequestOutputType.DirectionalityIndirect) || request.outputTypeMask.HasFlag(LightmapRequestOutputType.DirectionalityDirect);
                bool writeNormals = request.outputTypeMask.HasFlag(LightmapRequestOutputType.Normal);

                // Request related bake settings
                for (int lightmapIndex = 0; lightmapIndex < request.lightmapCount; lightmapIndex++)
                {
                    LightmapDesc currentLightmapDesc = lightmapDescriptors[lightmapIndex];
                    lightmapBakeSettings.PushOff = currentLightmapDesc.PushOff;
                    int resolution = (int)currentLightmapDesc.Resolution;
                    lightmapBakeSettings.AntiAliasingSampleCount = currentLightmapDesc.AntiAliasingSampleCount;
                    UInt64 lightmapSize = (UInt64)resolution * (UInt64)resolution;
                    lightmapBakeSettings.ExpandedBufferSize = math.min(lightmapSize, LightmapRequest.TilingModeToLightmapExpandedBufferSize(request.tilingMode));

                    // allocate expanded buffer
                    if (lightmappingContext.ExpandedBufferNeedsUpdating(lightmapBakeSettings.ExpandedBufferSize))
                    {
                        GraphicsHelpers.Flush(cmd); // need to flush as we are removing resources from the cache which might be in use
                        if (!lightmappingContext.InitializeExpandedBuffer(lightmapBakeSettings.ExpandedBufferSize))
                            return Result.InitializeExpandedBufferFailure;
                        // The scratch buffer is used for tracing rays in the lightmap integrators it needs to be sufficiently large to ray trace an expanded buffer
                        uint scratchBufferSize = (uint)lightmapBakeSettings.ExpandedBufferSize;
                        lightmappingContext.InitializeTraceScratchBuffer(scratchBufferSize, 1, 1);
                        if (debugDispatches)
                            Console.WriteLine($"Built expanded buffer for {lightmapBakeSettings.ExpandedBufferSize} samples, lm w: {lightmappingContext.AccumulatedOutput.width}, lm h: {lightmappingContext.AccumulatedOutput.height}].");
                    }

                    ref RenderTexture accumulatedOutput = ref lightmappingContext.AccumulatedOutput;
                    if ((accumulatedOutput.width != resolution) || (accumulatedOutput.height != resolution))
                        if (!lightmappingContext.SetOutputResolution(resolution, resolution))
                            return Result.InitializeFailure;

                    // Bake normals output
                    RenderTexture normalBuffer = null;
                    if (needsNormalsForDirectionality || writeNormals)
                    {
                        lightmappingContext.ClearOutputs();

                        IRayTracingShader normalShader = resources.NormalAccumulationShader;
                        GraphicsBuffer compactedGBufferLength = lightmappingContext.CompactedGBufferLength;
                        GraphicsBuffer indirectDispatchBuffer = lightmappingContext.IndirectDispatchBuffer;
                        GraphicsBuffer indirectRayTracingDispatchBuffer = lightmappingContext.IndirectDispatchRayTracingBuffer;

                        uint maxChunkSize = (uint)lightmappingContext.ExpandedOutput.count;
                        var expansionShaders = resources.ExpansionHelpers;
                        var compactionKernel = expansionShaders.FindKernel("CompactGBuffer");
                        var populateCopyDispatchKernel = expansionShaders.FindKernel("PopulateCopyDispatch");
                        var copyToLightmapKernel = expansionShaders.FindKernel("AdditivelyCopyCompactedTo2D");
                        var populateNormalShaderDispatchKernel = expansionShaders.FindKernel("PopulateAccumulationDispatch");

                        expansionShaders.GetKernelThreadGroupSizes(copyToLightmapKernel, out uint copyThreadGroupSizeX, out uint copyThreadGroupSizeY, out uint copyThreadGroupSizeZ);
                        Debug.Assert(copyThreadGroupSizeY == 1 && copyThreadGroupSizeZ == 1);

                        foreach (var instance in currentLightmapDesc.Instances)
                        {
                            if (!GetInstanceUVResources(cmd, lightmappingContext, instance, out _, out var uvAS, out var uvFallbackBuffer))
                                return Result.AddResourcesToCacheFailure;

                            PrepareLodInstances(cmd, lightmappingContext, instance, false);
                            var instanceGeometryIndex = lightmappingContext.World.PathTracingWorld.GetAccelerationStructure().GeometryPool.GetInstanceGeometryIndex(instance.Mesh);

                            uint instanceWidth = (uint)instance.TexelSize.x;
                            uint instanceHeight = (uint)instance.TexelSize.y;
                            UInt64 instanceSize = (UInt64)instanceWidth * (UInt64)instanceHeight;
                            UInt64 chunkTexelOffset = 0;
                            do
                            {
                                uint2 chunkOffset = new uint2((uint)(chunkTexelOffset % (UInt64)instanceWidth), (uint)(chunkTexelOffset / (UInt64)instanceWidth));
                                uint remainingTexels = instanceWidth - chunkOffset.x + ((instanceHeight - 1) - chunkOffset.y) * instanceWidth;
                                uint chunkSize = math.min(maxChunkSize, remainingTexels);
                                Debug.Assert(remainingTexels > 0);

                                cmd.BeginSample($"Bake Normals");
                                // build a gBuffer for the chunk
                                ExpansionHelpers.GenerateGBuffer(
                                    cmd,
                                    lightmappingContext.IntegratorContext.GBufferShader,
                                    lightmappingContext.GBuffer,
                                    lightmappingContext.TraceScratchBuffer,
                                    lightmappingContext.IntegratorContext.SamplingResources,
                                    uvAS,
                                    uvFallbackBuffer,
                                    instance.TexelOffset,
                                    chunkOffset,
                                    chunkSize,
                                    AntiAliasingType.SuperSampling,
                                    0,
                                    1);
                                chunkTexelOffset += chunkSize;

                                // compact the gBuffer
                                ExpansionHelpers.CompactGBuffer(
                                    cmd,
                                    expansionShaders,
                                    compactionKernel,
                                    lightmappingContext.GBuffer,
                                    chunkSize,
                                    compactedGBufferLength,
                                    lightmappingContext.CompactedTexelIndices);

                                // now perform the normal generation for the chunk
                                // geometry pool bindings
                                Util.BindAccelerationStructure(cmd, normalShader, world.PathTracingWorld.GetAccelerationStructure());

                                var requiredSizeInBytes = normalShader.GetTraceScratchBufferRequiredSizeInBytes((uint)chunkSize, 1, 1);
                                if (requiredSizeInBytes > 0)
                                {
                                    var actualScratchBufferSize = (ulong)(lightmappingContext.TraceScratchBuffer.count * lightmappingContext.TraceScratchBuffer.stride);
                                    Debug.Assert(lightmappingContext.TraceScratchBuffer.stride == sizeof(uint));
                                    Debug.Assert(requiredSizeInBytes <= actualScratchBufferSize);
                                }

                                normalShader.SetMatrixParam(cmd, LightmapIntegratorShaderIDs.ShaderLocalToWorld, instance.LocalToWorldMatrix);
                                normalShader.SetMatrixParam(cmd, LightmapIntegratorShaderIDs.ShaderLocalToWorldNormals, instance.LocalToWorldMatrixNormals);
                                normalShader.SetIntParam(cmd, LightmapIntegratorShaderIDs.InstanceGeometryIndex, instanceGeometryIndex);
                                normalShader.SetIntParam(cmd, LightmapIntegratorShaderIDs.InstanceWidth, (int)instanceWidth);

                                normalShader.SetBufferParam(cmd, LightmapIntegratorShaderIDs.GBuffer, lightmappingContext.GBuffer);
                                normalShader.SetBufferParam(cmd, LightmapIntegratorShaderIDs.CompactedGBuffer, lightmappingContext.CompactedTexelIndices);
                                normalShader.SetBufferParam(cmd, LightmapIntegratorShaderIDs.ExpandedOutput, lightmappingContext.ExpandedOutput);
                                normalShader.SetIntParam(cmd, LightmapIntegratorShaderIDs.ExpandedTexelSampleWidth, 1);
                                normalShader.SetIntParam(cmd, LightmapIntegratorShaderIDs.ChunkOffsetX, (int)chunkOffset.x);
                                normalShader.SetIntParam(cmd, LightmapIntegratorShaderIDs.ChunkOffsetY, (int)chunkOffset.y);

                                ExpansionHelpers.PopulateAccumulationIndirectDispatch(cmd, normalShader, expansionShaders, populateNormalShaderDispatchKernel, 1, compactedGBufferLength, indirectRayTracingDispatchBuffer);
                                normalShader.SetIntParam(cmd, LightmapIntegratorShaderIDs.SampleOffset, 0);
                                normalShader.SetIntParam(cmd, LightmapIntegratorShaderIDs.MaxLocalSampleCount, 1);
                                cmd.BeginSample("Normal Generation");
                                normalShader.Dispatch(cmd, lightmappingContext.TraceScratchBuffer, indirectRayTracingDispatchBuffer);
                                cmd.EndSample("Normal Generation");

                                // copy back to the output
                                ExpansionHelpers.PopulateCopyToLightmapIndirectDispatch(cmd, expansionShaders, populateCopyDispatchKernel, copyThreadGroupSizeX, compactedGBufferLength, indirectDispatchBuffer);
                                ExpansionHelpers.CopyToLightmap(cmd, expansionShaders, copyToLightmapKernel, 1, (int)instanceWidth, instance.TexelOffset, chunkOffset, compactedGBufferLength, lightmappingContext.CompactedTexelIndices, lightmappingContext.ExpandedOutput, indirectDispatchBuffer, lightmappingContext.AccumulatedOutput);
                                cmd.EndSample("Bake Normals");
                            }
                            while (chunkTexelOffset < instanceSize);

                            ClearLodInstances(cmd, lightmappingContext, instance);
                        }

                        if (writeNormals)
                        {
                            if (LightmapIntegrationHelpers.WriteLightmap(cmd, accumulatedOutput, "normal", lightmapIndex, request.outputFolderPath) == false)
                                return Result.WriteToDiskFailure;
                        }

                        if (needsNormalsForDirectionality)
                        {
                            // Hang on to normal buffer for normalization of directionality
                            normalBuffer = LightmappingContext.MakeRenderTexture(accumulatedOutput.width, accumulatedOutput.height, "TempNormalBuffer");
                            cmd.CopyTexture(accumulatedOutput, normalBuffer);
                        }

                        ulong workSteps = CalculateNonIntegratedLightmapWorkSteps((uint) lightmapSize);
                        progressState.IncrementCompletedWorkSteps(workSteps);
                    }

                    // Bake occupancy output
                    if (request.outputTypeMask.HasFlag(LightmapRequestOutputType.Occupancy))
                    {
                        lightmappingContext.ClearOutputs();

                        foreach (var instance in currentLightmapDesc.Instances)
                        {
                            if (!GetInstanceUVResources(cmd, lightmappingContext, instance, out _, out _, out var uvFallbackBuffer))
                                return Result.AddResourcesToCacheFailure;

                            lightmappingContext.IntegratorContext.LightmapOccupancyIntegrator.Accumulate(
                                cmd,
                                instance.TexelSize,
                                instance.TexelOffset,
                                uvFallbackBuffer,
                                accumulatedOutput);
                        }

                        if (LightmapIntegrationHelpers.WriteLightmap(cmd, accumulatedOutput, "occupancy", lightmapIndex, request.outputFolderPath) == false)
                            return Result.WriteToDiskFailure;

                        ulong workSteps = CalculateNonIntegratedLightmapWorkSteps((uint) lightmapSize);
                        progressState.IncrementCompletedWorkSteps(workSteps);
                    }

                    // Bake chart index output
                    if (request.outputTypeMask.HasFlag(LightmapRequestOutputType.ChartIndex))
                    {
                        var rasterizer = lightmappingContext.ChartRasterizer;

                        cmd.SetRenderTarget(lightmappingContext.AccumulatedOutput);
                        cmd.ClearRenderTarget(false, true, new Color(-1, -1, -1, -1)); // -1 = No chart

                        // Keep track of how many charts we have rasterized, so we can ensure uniqueness across the entire lightmap.
                        uint chartIndexOffset = 0;

                        foreach (var instance in currentLightmapDesc.Instances)
                        {
                            if (!GetInstanceUVResources(cmd, lightmappingContext, instance, out var uvMesh, out _, out _))
                                return Result.AddResourcesToCacheFailure;

                            var vertexToChartId = chartIdentification.CompleteAndGetResult(instance.Mesh);
                            cmd.SetBufferData(lightmappingContext.ChartRasterizerBuffers.vertexToChartID, vertexToChartId.VertexChartIndices);

                            ChartRasterizer.PrepareRasterizeSoftware(cmd, uvMesh.Mesh,
                                lightmappingContext.ChartRasterizerBuffers.vertex, lightmappingContext.ChartRasterizerBuffers.vertexToOriginalVertex);
                            rasterizer.RasterizeSoftware(cmd, lightmappingContext.ChartRasterizerBuffers.vertex,
                                lightmappingContext.ChartRasterizerBuffers.vertexToOriginalVertex, lightmappingContext.ChartRasterizerBuffers.vertexToChartID,
                                uvMesh.Mesh.GetTotalIndexCount(), instance.NormalizedOccupiedST, chartIndexOffset, lightmappingContext.AccumulatedOutput);

                            chartIndexOffset += vertexToChartId.ChartCount;
                        }

                        // Readback texture
                        Vector4[] pixels = null;
                        cmd.RequestAsyncReadback(lightmappingContext.AccumulatedOutput, 0, request => pixels = request.GetData<Vector4>().ToArray());
                        cmd.WaitAllAsyncReadbackRequests();

                        GraphicsHelpers.Flush(cmd);

                        // Write it to disk
                        int[] chartIndices = new int[pixels.Length];
                        for (int i = 0; i < pixels.Length; i++)
                            chartIndices[i] = (int)pixels[i].x;
                        if (!SaveArrayToFile(request.outputFolderPath + $"/chartIndex{lightmapIndex}.int", chartIndices.Length, chartIndices))
                            return Result.WriteToDiskFailure;

                        ulong workSteps = CalculateNonIntegratedLightmapWorkSteps((uint)lightmapSize);
                        progressState.IncrementCompletedWorkSteps(workSteps);
                    }

                    // Bake all the integrator components requests
                    foreach (var integratedRequestOutputType in integratedRequestOutputs)
                    {
                        lightmappingContext.ClearOutputs();

                        IntegratedOutputType integratedOutputType = integratedRequestOutputType.Key;
                        foreach (var instance in currentLightmapDesc.Instances)
                        {
                            if (!GetInstanceUVResources(cmd, lightmappingContext, instance, out _, out var uvAS, out var uvFallbackBuffer))
                                return Result.AddResourcesToCacheFailure;

                            var result = IntegrateLightmapInstance(
                                cmd,
                                lightmapIndex,
                                integratedRequestOutputType.Key,
                                instance,
                                lightmappingContext,
                                uvAS,
                                uvFallbackBuffer,
                                integrationSettings,
                                lightmapBakeSettings,
                                progressState,
                                gpuSync,
                                debugDispatches
                            );

                            if (result != Result.Success)
                                return result;
                        }

                        // LightBaker outputs irradiance over PI instead of pure irradiance. Our new baker outputs irradiance. We divide by PI to match LightBaker.
                        const float oneOverPI = 1.0f / Mathf.PI;
                        if (integratedOutputType == IntegratedOutputType.Indirect || integratedOutputType == IntegratedOutputType.Direct)
                        {
                            Vector4 vOneOverPI = new Vector4(oneOverPI, oneOverPI, oneOverPI, 1.0f);
                            LightmapIntegrationHelpers.MultiplyRenderTexture(
                                cmd, resources.ComputeHelpers.ComputeHelperShader, LightmapIntegrationHelpers.ComputeHelpers.MultiplyKernel,
                                accumulatedOutput, accumulatedOutput.width, accumulatedOutput.height, vOneOverPI);
                        }

                        // Direct lights are multiplied by PI, divide by PI to match LightBaker.
                        bool hasDirectionality = integratedRequestOutputType.Value.HasFlag(RequestedSubOutput.DirectionalityTexture);
                        if (integratedOutputType == IntegratedOutputType.Direct && hasDirectionality)
                        {
                            Vector4 vOneOverPI = new Vector4(oneOverPI, oneOverPI, oneOverPI, oneOverPI);
                            LightmapIntegrationHelpers.MultiplyRenderTexture(
                                cmd, resources.ComputeHelpers.ComputeHelperShader, LightmapIntegrationHelpers.ComputeHelpers.MultiplyKernel,
                                lightmappingContext.AccumulatedDirectionalOutput, lightmappingContext.AccumulatedDirectionalOutput.width, lightmappingContext.AccumulatedDirectionalOutput.height, vOneOverPI);
                        }

                        // Copy out the results
                        if (hasDirectionality)
                        {
                            Debug.Assert(normalBuffer != null, "Normal buffer must be set when directionality is requested.");
                            Debug.Assert(needsNormalsForDirectionality);

                            // This uses the accumulatedOutput to get the sample count, so directionality must be normalized first since the sample count will be normalized when the lightmap is normalized
                            // Currently the directional output from LightBaker is not normalized - so for now we don't do it here either. JIRA: https://jira.unity3d.com/browse/LIGHT-1814
                            switch (integratedOutputType)
                            {
                                case IntegratedOutputType.Direct:
                                    lightmappingContext.IntegratorContext.LightmapDirectIntegrator.NormalizeDirectional(cmd, lightmappingContext.AccumulatedDirectionalOutput, accumulatedOutput, normalBuffer);
                                    break;
                                case IntegratedOutputType.Indirect:
                                    lightmappingContext.IntegratorContext.LightmapIndirectIntegrator.NormalizeDirectional(cmd, lightmappingContext.AccumulatedDirectionalOutput, accumulatedOutput, normalBuffer);
                                    break;
                            }
                        }

                        switch (integratedOutputType)
                        {
                            case IntegratedOutputType.AO:
                                lightmappingContext.IntegratorContext.LightmapAOIntegrator.Normalize(cmd, accumulatedOutput);
                                break;
                            case IntegratedOutputType.Direct:
                                lightmappingContext.IntegratorContext.LightmapDirectIntegrator.Normalize(cmd, accumulatedOutput);
                                break;
                            case IntegratedOutputType.Indirect:
                                lightmappingContext.IntegratorContext.LightmapIndirectIntegrator.Normalize(cmd, accumulatedOutput);
                                break;
                            case IntegratedOutputType.Validity:
                                lightmappingContext.IntegratorContext.LightmapValidityIntegrator.Normalize(cmd, accumulatedOutput);
                                break;
                            case IntegratedOutputType.ShadowMask:
                                lightmappingContext.IntegratorContext.LightmapShadowMaskIntegrator.Normalize(cmd, accumulatedOutput, lightmappingContext.AccumulatedDirectionalOutput);
                                break;
                        }

                        // Make sure all the work is done before writing to disk
                        GraphicsHelpers.Flush(cmd);

                        if (debugDispatches)
                        {
                            gpuSync.Sync(cmd);
                            Console.WriteLine($"Write {integratedOutputType} lightmap {lightmapIndex} to disk.");
                            //LightmapIntegrationHelpers.WriteRenderTexture(cmd, $"Temp/lm{lightmapIndex}_type{lightmapType}_final.r2dr", lightmappingContext.AccumulatedOutput, lightmappingContext.AccumulatedOutput.width, lightmappingContext.AccumulatedOutput.height);
                        }

                        // Write lightmap to disk. TODO: kick off a C# job so the compression and file IO happens in a different thread LIGHT-1772
                        bool outputPrimaryTexture = integratedRequestOutputType.Value.HasFlag(RequestedSubOutput.PrimaryTexture);
                        // We explicitly take into account whether to write for directionality, no directionality or for both.
                        if (outputPrimaryTexture)
                            if (LightmapIntegrationHelpers.WriteLightmap(cmd, accumulatedOutput, integratedOutputType, lightmapIndex, request.outputFolderPath) == false)
                                return Result.WriteToDiskFailure;

                        if (!integratedRequestOutputType.Value.HasFlag(RequestedSubOutput.DirectionalityTexture))
                            continue;

                        // Write 'direct/indirect directionality' if requested - it is baked in the same pass as 'direct/indirect'.
                        IntegratedOutputType directionalityLightmapType =
                            integratedOutputType == IntegratedOutputType.Direct ? IntegratedOutputType.DirectionalityDirect : IntegratedOutputType.DirectionalityIndirect;
                        if (LightmapIntegrationHelpers.WriteLightmap(cmd, lightmappingContext.AccumulatedDirectionalOutput, directionalityLightmapType, lightmapIndex, request.outputFolderPath) == false)
                            return Result.WriteToDiskFailure;
                    }

                    // Get rid of the normal output
                    if (normalBuffer is not null)
                        normalBuffer.Release();
                    CoreUtils.Destroy(normalBuffer);

                    // Write black outputs for the lightmap components we didn't bake. LightBaker does this too.
                    Texture2D blackOutput = new Texture2D(resolution, resolution, TextureFormat.RGBAFloat, false);
                    blackOutput.name = "BlackOutput";
                    blackOutput.hideFlags = HideFlags.HideAndDontSave;
                    blackOutput.SetPixels(new Color[resolution * resolution]);
                    blackOutput.Apply();
                    byte[] compressedBlack = blackOutput.EncodeToR2D();
                    try
                    {
                        // Environment is part of indirect irradiance.
                        if (request.outputTypeMask.HasFlag(LightmapRequestOutputType.IrradianceEnvironment))
                            File.WriteAllBytes(request.outputFolderPath + $"/irradianceEnvironment{lightmapIndex}.r2d", compressedBlack);
                        // occupiedTexels is needed for analytics, don't fail the bake if we cannot write it.
                        UInt64 occupiedTexels = (UInt64)lightmapRequestData.atlassing.m_EstimatedTexelCount;
                        if (request.outputTypeMask.HasFlag(LightmapRequestOutputType.Occupancy))
                            File.WriteAllBytes(request.outputFolderPath + $"/occupiedTexels{lightmapIndex}.UInt64", BitConverter.GetBytes(occupiedTexels));

                        // TODO: LIGHT-1767 generate actual overlapPixelIndex and uvOverlapInstanceIds data.
                        UInt64 overlapPixelIndex = 0; // zero sized dummy array data.
                        if (request.outputTypeMask.HasFlag(LightmapRequestOutputType.OverlapPixelIndex))
                            File.WriteAllBytes(request.outputFolderPath + $"/overlapPixelIndex{lightmapIndex}.uint", BitConverter.GetBytes(overlapPixelIndex));
                        UInt64 uvOverlapInstanceIds = 0; // zero sized dummy array data.
                        File.WriteAllBytes(request.outputFolderPath + $"/uvOverlapInstanceIds{lightmapIndex}.size_t", BitConverter.GetBytes(uvOverlapInstanceIds));
                    }
                    catch (Exception e)
                    {
                        CoreUtils.Destroy(blackOutput);
                        Debug.Assert(false, e.Message);
                        return Result.WriteToDiskFailure;
                    }
                    CoreUtils.Destroy(blackOutput);
                }
            }
            return Result.Success;
        }

        private static Dictionary<IntegratedOutputType, RequestedSubOutput> GetRequestedIntegratedOutputTypes(in LightmapRequest request)
        {
            // TODO handle lightmapOffset and lightmapCount from the request.
            LightmapRequestOutputType patchedRequest = request.outputTypeMask;
            if (request.outputTypeMask.HasFlag(LightmapRequestOutputType.IrradianceEnvironment) &&
                !request.outputTypeMask.HasFlag(LightmapRequestOutputType.IrradianceIndirect))
            {
                // If we are baking environment irradiance, we need to bake indirect irradiance too.
                patchedRequest |= LightmapRequestOutputType.IrradianceIndirect;
            }

            // Turn the bits set in the request bitmask into an array of LightmapRequestOutputType.
            HashSet<LightmapRequestOutputType> lightmapRequestOutputTypes = BitfieldToList((int)patchedRequest);

            // Make a dictionary of the lightmap types that are requested, the value indicates if directionality is requested for that stage (when it makes sense).
            // This accounts for the fact that for direct and indirect directionality is baked in the same pass. But for purposes of deciding what lightmap output
            // files to write we need to store that information as a bit flag.
            Dictionary<IntegratedOutputType, RequestedSubOutput> requestedLightmapTypes = new();
            void AddToRequestedLightmapTypes(IntegratedOutputType lightmapType, RequestedSubOutput type)
            {
                if (!requestedLightmapTypes.TryAdd(lightmapType, type))
                    requestedLightmapTypes[lightmapType] |= type;
            }

            foreach (var lightmapRequestOutputType in lightmapRequestOutputTypes)
            {
                if (!LightmapRequestOutputTypeToIntegratedOutputType(lightmapRequestOutputType, out IntegratedOutputType lightmapType))
                    continue;

                switch (lightmapType)
                {
                    case IntegratedOutputType.Indirect:
                        AddToRequestedLightmapTypes(IntegratedOutputType.Indirect, RequestedSubOutput.PrimaryTexture);
                        break;
                    case IntegratedOutputType.DirectionalityIndirect:
                        AddToRequestedLightmapTypes(IntegratedOutputType.Indirect, RequestedSubOutput.DirectionalityTexture);
                        break;
                    case IntegratedOutputType.Direct:
                        AddToRequestedLightmapTypes(IntegratedOutputType.Direct, RequestedSubOutput.PrimaryTexture);
                        break;
                    case IntegratedOutputType.DirectionalityDirect:
                        AddToRequestedLightmapTypes(IntegratedOutputType.Direct, RequestedSubOutput.DirectionalityTexture);
                        break;
                    default:
                        AddToRequestedLightmapTypes(lightmapType, RequestedSubOutput.PrimaryTexture);
                        break;
                }
            }

            return requestedLightmapTypes;
        }

        private static HashSet<LightmapRequestOutputType> GetRequestedNonIntegratedOutputTypes(in LightmapRequest request)
        {
            HashSet<LightmapRequestOutputType> outputTypes =
                BitfieldToList((int) (request.outputTypeMask & (LightmapRequestOutputType.Normal | LightmapRequestOutputType.Occupancy | LightmapRequestOutputType.ChartIndex)));
            bool needsNormalsForDirectionality = request.outputTypeMask.HasFlag(LightmapRequestOutputType.DirectionalityIndirect) || request.outputTypeMask.HasFlag(LightmapRequestOutputType.DirectionalityDirect);
            if (needsNormalsForDirectionality)
                outputTypes.Add(LightmapRequestOutputType.Normal);

            return outputTypes;
        }

        private static (uint directSampleCount, uint effectiveIndirectSampleCount) GetProbeSampleCounts(in SampleCount probeSampleCounts)
        {
            uint indirectSampleCount = probeSampleCounts.indirectSampleCount;
            uint environmentSampleCount = probeSampleCounts.environmentSampleCount;

            return (probeSampleCounts.directSampleCount, math.max(indirectSampleCount, environmentSampleCount));
        }

        internal static bool ExecuteProbeRequests(in BakeInput bakeInput, in ProbeRequestData probeRequestData, UnityComputeDeviceContext deviceContext,
            bool useLegacyBakingBehavior, UnityComputeWorld world, uint bounceCount, BakeProgressState progressState)
        {
            if (probeRequestData.requests.Length == 0)
                return true;

            using UnityComputeProbeIntegrator probeIntegrator = new(useLegacyBakingBehavior, !useLegacyBakingBehavior);
            probeIntegrator.SetProgressReporter(progressState);

            // Create input position buffer
            using NativeArray<float3> inputPositions = new(probeRequestData.positions, Allocator.Temp);
            var positionsBuffer = deviceContext.CreateBuffer((ulong)probeRequestData.positions.Length, sizeof(float) * 3);
            BufferSlice<float3> positionsBufferSlice = positionsBuffer.Slice<float3>();
            var positionsWriteEvent = deviceContext.CreateEvent();
            deviceContext.WriteBuffer(positionsBufferSlice, inputPositions, positionsWriteEvent);
            deviceContext.DestroyEvent(positionsWriteEvent);

            // Create input probe occlusion light indices buffer (shared for all requests)
            using NativeArray<int> inputPerProbeLightIndices = new(probeRequestData.occlusionLightIndices, Allocator.Temp);
            var perProbeLightIndicesBuffer = deviceContext.CreateBuffer((ulong)probeRequestData.occlusionLightIndices.Length, sizeof(int));
            BufferSlice<int> perProbeLightIndicesBufferSlice = perProbeLightIndicesBuffer.Slice<int>();
            deviceContext.WriteBuffer(perProbeLightIndicesBufferSlice, inputPerProbeLightIndices);

            (uint directSampleCount, uint effectiveIndirectSampleCount) = GetProbeSampleCounts(bakeInput.lightingSettings.probeSampleCounts);

            ProbeRequest[] probeRequests = probeRequestData.requests;
            for (int probeRequestIndex = 0; probeRequestIndex < probeRequests.Length; probeRequestIndex++)
            {
                // Read data from request and prepare integrator
                ref readonly ProbeRequest request = ref probeRequests[probeRequestIndex];
                int requestOffset = (int)request.positionOffset;
                int requestLength = (int)request.count;
                ulong floatBufferSize = sizeof(float) * request.count;
                float pushoff = request.pushoff;
                probeIntegrator.Prepare(deviceContext, world, positionsBuffer.Slice<Vector3>(), pushoff, (int)bounceCount);

                List<EventID> eventsToWaitFor = new();
                List<BufferID> buffersToDestroy = new();

                // Integrate indirect radiance
                using NativeArray<SphericalHarmonicsL2> outputIndirectRadiance = new(requestLength, Allocator.Persistent);
                if (request.outputTypeMask.HasFlag(ProbeRequestOutputType.RadianceIndirect))
                {
                    var shIndirectBuffer = deviceContext.CreateBuffer(request.count * 27, sizeof(float));
                    buffersToDestroy.Add(shIndirectBuffer);
                    var shIndirectBufferSlice = shIndirectBuffer.Slice<SphericalHarmonicsL2>();
                    var integrationResult = probeIntegrator.IntegrateIndirectRadiance(deviceContext, requestOffset, requestLength,
                        (int)effectiveIndirectSampleCount, request.ignoreIndirectEnvironment, shIndirectBufferSlice);
                    Assert.AreEqual(IProbeIntegrator.ResultType.Success, integrationResult.type, "IntegrateIndirectRadiance failed.");
                    var readEvent = deviceContext.CreateEvent();
                    deviceContext.ReadBuffer(shIndirectBufferSlice, outputIndirectRadiance, readEvent);
                    eventsToWaitFor.Add(readEvent);
                }

                // Integrate direct radiance
                using NativeArray<SphericalHarmonicsL2> outputDirectRadiance = new(requestLength, Allocator.Persistent);
                if (request.outputTypeMask.HasFlag(ProbeRequestOutputType.RadianceDirect))
                {
                    var shDirectBuffer = deviceContext.CreateBuffer(request.count * 27, sizeof(float));
                    buffersToDestroy.Add(shDirectBuffer);
                    var shDirectBufferSlice = shDirectBuffer.Slice<SphericalHarmonicsL2>();
                    var integrationResult = probeIntegrator.IntegrateDirectRadiance(deviceContext, requestOffset, requestLength,
                        (int)directSampleCount, request.ignoreDirectEnvironment, shDirectBufferSlice);
                    Assert.AreEqual(IProbeIntegrator.ResultType.Success, integrationResult.type, "IntegrateDirectRadiance failed.");
                    var readEvent = deviceContext.CreateEvent();
                    deviceContext.ReadBuffer(shDirectBufferSlice, outputDirectRadiance, readEvent);
                    eventsToWaitFor.Add(readEvent);
                }

                // Integrate validity
                using NativeArray<float> outputValidity = new(requestLength, Allocator.Persistent);
                if (request.outputTypeMask.HasFlag(ProbeRequestOutputType.Validity))
                {
                    var validityBuffer = deviceContext.CreateBuffer(request.count, sizeof(float));
                    buffersToDestroy.Add(validityBuffer);
                    var validityBufferSlice = validityBuffer.Slice<float>();
                    var integrationResult = probeIntegrator.IntegrateValidity(deviceContext, requestOffset, requestLength,
                        (int)effectiveIndirectSampleCount, validityBufferSlice);
                    Assert.AreEqual(IProbeIntegrator.ResultType.Success, integrationResult.type, "IntegrateValidity failed.");
                    var readEvent = deviceContext.CreateEvent();
                    deviceContext.ReadBuffer(validityBufferSlice, outputValidity, readEvent);
                    eventsToWaitFor.Add(readEvent);
                }

                // Integrate occlusion values
                const int maxOcclusionLightsPerProbe = 4;
                bool usesProbeOcclusion = bakeInput.lightingSettings.mixedLightingMode != MixedLightingMode.IndirectOnly;
                using NativeArray<float> outputOcclusion = new(requestLength * maxOcclusionLightsPerProbe, Allocator.Persistent, NativeArrayOptions.ClearMemory);
                if (request.outputTypeMask.HasFlag(ProbeRequestOutputType.LightProbeOcclusion) && usesProbeOcclusion)
                {
                    var occlusionBuffer = deviceContext.CreateBuffer(maxOcclusionLightsPerProbe * request.count, sizeof(float));
                    buffersToDestroy.Add(occlusionBuffer);
                    var occlusionBufferSlice = occlusionBuffer.Slice<float>();

                    var integrationResult = probeIntegrator.IntegrateOcclusion(deviceContext, requestOffset, requestLength,
                        (int)effectiveIndirectSampleCount, maxOcclusionLightsPerProbe, perProbeLightIndicesBufferSlice, occlusionBufferSlice);
                    Assert.AreEqual(IProbeIntegrator.ResultType.Success, integrationResult.type, "IntegrateOcclusion failed.");

                    EventID readEvent = deviceContext.CreateEvent();
                    deviceContext.ReadBuffer(occlusionBufferSlice, outputOcclusion, readEvent);
                    eventsToWaitFor.Add(readEvent);
                }

                // Gather occlusion containers / indices
                var outputOcclusionIndices = new LightProbeOcclusion[requestLength];
                for (int probeIdx = 0; probeIdx < requestLength; probeIdx++)
                {
                    ref var occlusion = ref outputOcclusionIndices[probeIdx];
                    occlusion.SetDefaultValues();
                }
                if (request.outputTypeMask.HasFlag(ProbeRequestOutputType.MixedLightOcclusion) && usesProbeOcclusion)
                {
                    // Build output data
                    for (int probeIdx = 0; probeIdx < requestLength; probeIdx++)
                    {
                        LightProbeOcclusion occlusion = new();
                        occlusion.SetDefaultValues();
                        for (int indirectLightIdx = 0; indirectLightIdx < maxOcclusionLightsPerProbe; indirectLightIdx++)
                        {
                            int bakeInputLightIdx = inputPerProbeLightIndices[probeIdx * maxOcclusionLightsPerProbe + indirectLightIdx];
                            if (bakeInputLightIdx >= 0)
                            {
                                sbyte occlusionMaskChannel = (sbyte)bakeInput.lightData[bakeInputLightIdx].shadowMaskChannel;

                                occlusion.SetProbeOcclusionLightIndex(indirectLightIdx, bakeInputLightIdx);
                                occlusion.SetOcclusion(indirectLightIdx, 0.0f);
                                occlusion.SetOcclusionMaskChannel(indirectLightIdx, occlusionMaskChannel);
                            }
                        }
                        outputOcclusionIndices[probeIdx] = occlusion;
                    }
                }

                // Flush and wait for all events to complete
                deviceContext.Flush();
                foreach (var evt in eventsToWaitFor)
                {
                    bool ok = deviceContext.Wait(evt);
                    Assert.IsTrue(ok);
                    deviceContext.DestroyEvent(evt);
                }

                // Cleanup temporary buffers
                foreach (var buffer in buffersToDestroy)
                {
                    deviceContext.DestroyBuffer(buffer);
                }

                // Write output data to disk
                // Write light probe data to disk, so the C++ side post processing stage can pick it up.
                // We can use the C# side post processing API in the future, once we can write the LDA from C#.
                string path = request.outputFolderPath;
                if (request.outputTypeMask.HasFlag(ProbeRequestOutputType.RadianceDirect))
                {
                    string filePath = path;
                    filePath += string.Format("/radianceDirect{0}.shl2", probeRequestIndex);
                    if (!SaveProbesToFileInterleaved(filePath, outputDirectRadiance))
                        return false;
                }
                if (request.outputTypeMask.HasFlag(ProbeRequestOutputType.RadianceIndirect))
                {
                    string filePath = path;
                    filePath += string.Format("/radianceIndirect{0}.shl2", probeRequestIndex);
                    if (!SaveProbesToFileInterleaved(filePath, outputIndirectRadiance))
                        return false;
                }
                if (request.outputTypeMask.HasFlag(ProbeRequestOutputType.Validity))
                {
                    string filePath = path;
                    filePath += string.Format("/validity{0}.float", probeRequestIndex);
                    if (!SaveArrayToFile(filePath, requestLength, outputValidity.ToArray()))
                        return false;
                }
                if (request.outputTypeMask.HasFlag(ProbeRequestOutputType.LightProbeOcclusion))
                {
                    string filePath = path;
                    filePath += string.Format("/lightProbeOcclusion{0}.vec4", probeRequestIndex);
                    if (!SaveArrayToFile(filePath, requestLength, outputOcclusion.ToArray()))
                        return false;
                }
                if (request.outputTypeMask.HasFlag(ProbeRequestOutputType.MixedLightOcclusion))
                {
                    // TODO(pema.malling): dummy data, the C++ side post processing stage needs this.
                    // https://jira.unity3d.com/browse/LIGHT-1102
                    string filePath = path;
                    filePath += string.Format("/mixedLightOcclusion{0}.occ", probeRequestIndex);
                    if (!SaveArrayToFile(filePath, requestLength, outputOcclusionIndices))
                        return false;
                }
            }

            // Cleanup created buffers
            deviceContext.DestroyBuffer(positionsBuffer);
            deviceContext.DestroyBuffer(perProbeLightIndicesBuffer);

            return true;
        }
    }
}
