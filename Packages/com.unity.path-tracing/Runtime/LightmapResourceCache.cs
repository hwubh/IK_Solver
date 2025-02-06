using System;
using System.Collections.Generic;
using UnityEngine.PathTracing.Integration;
using UnityEngine.Rendering;
using UnityEngine.Rendering.UnifiedRayTracing;

namespace UnityEngine.PathTracing.Lightmapping
{
    internal class LightmapIntegrationResourceCache : IDisposable
    {
        private Dictionary<int, UVMesh> _hashToUVMesh = new();
        private Dictionary<int, UVAccelerationStructure> _hashToUVAccelerationStructure = new();
        private Dictionary<int, UVFallbackBuffer> _hashToUVFallbackBuffer = new();

        public int UVMeshCount()
        {
            return _hashToUVMesh.Count;
        }

        public int UVAccelerationStructureCount()
        {
            return _hashToUVAccelerationStructure.Count;
        }

        public int UVFallbackBufferCount()
        {
            return _hashToUVFallbackBuffer.Count;
        }

        internal bool CacheIsHot(Instance[] instances)
        {
            foreach (Instance instance in instances)
            {
                // UVMesh
                int uvMeshHash = UVMesh.GetHashCode(instance.Mesh);
                if (!_hashToUVMesh.TryGetValue(uvMeshHash, out UVMesh uvMesh))
                    return false;

                // UVAccelerationStructure
                int uvASHash = UVAccelerationStructure.GetHashCode(uvMesh);
                if (!_hashToUVAccelerationStructure.TryGetValue(uvASHash, out UVAccelerationStructure uvAS))
                    return false;

                // UVFallbackBuffer
                int uvFBHash = UVFallbackBuffer.GetHashCode(instance.TexelSize.x, instance.TexelSize.y, uvMesh, uvAS);
                if (!_hashToUVFallbackBuffer.TryGetValue(uvFBHash, out UVFallbackBuffer uvFB))
                    return false;
            }
            return true;
        }

        internal bool AddResources(
            Instance[] instances,
            RayTracingContext context,
            CommandBuffer cmd,
            UVFallbackBufferBuilder uvFallbackBufferBuilder)
        {
            foreach (Instance instance in instances)
            {
                // UVMesh
                int uvMeshHash = UVMesh.GetHashCode(instance.Mesh);
                if (!_hashToUVMesh.TryGetValue(uvMeshHash, out UVMesh uvMesh))
                {
                    UVMesh newUVMesh = new();
                    if (!newUVMesh.Build(instance.Mesh))
                    {
                        newUVMesh?.Dispose();
                        return false;
                    }
                    _hashToUVMesh.Add(uvMeshHash, newUVMesh);
                    uvMesh = newUVMesh;
                }
                // UVAccelerationStructure
                int uvASHash = UVAccelerationStructure.GetHashCode(uvMesh);
                if (!_hashToUVAccelerationStructure.TryGetValue(uvASHash, out UVAccelerationStructure uvAS))
                {
                    cmd.BeginSample("Build UVAccelerationStructure");
                    UVAccelerationStructure newUVAS = new();
                    newUVAS.Build(cmd, context, uvMesh, BuildFlags.None);
                    _hashToUVAccelerationStructure.Add(uvASHash, newUVAS);
                    uvAS = newUVAS;
                    cmd.EndSample("Build UVAccelerationStructure");
                }
                // UVFallbackBuffer
                int uvFBHash = UVFallbackBuffer.GetHashCode(instance.TexelSize.x, instance.TexelSize.y, uvMesh, uvAS);
                if (!_hashToUVFallbackBuffer.TryGetValue(uvFBHash, out UVFallbackBuffer uvFB))
                {
                    UVFallbackBuffer newUVFB = new();
                    if (!newUVFB.Build(
                        cmd,
                        uvFallbackBufferBuilder,
                        instance.TexelSize.x,
                        instance.TexelSize.y,
                        uvMesh))
                    {
                        newUVFB?.Dispose();
                        return false;
                    }
                    _hashToUVFallbackBuffer.Add(uvFBHash, newUVFB);
                    uvFB = newUVFB;
                }
            }
            return true;
        }

        internal void FreeResources(Instance[] instancesToKeep)
        {
            // Build dictionary over resources to keep
            Dictionary<int, UVMesh> uvMeshesToKeep = new();
            Dictionary<int, UVAccelerationStructure> uvASToKeep = new();
            Dictionary<int, UVFallbackBuffer> uvFBToKeep = new();
            foreach (var instance in instancesToKeep)
            {
                int uvMeshHash = UVMesh.GetHashCode(instance.Mesh);
                if (_hashToUVMesh.TryGetValue(uvMeshHash, out UVMesh uvMesh))
                {
                    uvMeshesToKeep.Add(uvMeshHash, uvMesh);
                    _hashToUVMesh.Remove(uvMeshHash);
                    int uvASHash = UVAccelerationStructure.GetHashCode(uvMesh);
                    if (_hashToUVAccelerationStructure.TryGetValue(uvASHash, out UVAccelerationStructure uvAS))
                    {
                        uvASToKeep.Add(uvASHash, uvAS);
                        _hashToUVAccelerationStructure.Remove(uvASHash);
                        int uvFBHash = UVFallbackBuffer.GetHashCode(instance.TexelSize.x, instance.TexelSize.y, uvMesh, uvAS);
                        if (_hashToUVFallbackBuffer.TryGetValue(uvFBHash, out UVFallbackBuffer uvFB))
                        {
                            uvFBToKeep.Add(uvFBHash, uvFB);
                            _hashToUVFallbackBuffer.Remove(uvFBHash);
                        }
                    }
                }
            }

            // Dispose remaining resources
            Clear();

            // Restore resources to keep
            _hashToUVMesh = uvMeshesToKeep;
            _hashToUVAccelerationStructure = uvASToKeep;
            _hashToUVFallbackBuffer = uvFBToKeep;
        }

        internal bool GetResources(
            Instance[] instances,
            out UVMesh[] uvMeshes,
            out UVAccelerationStructure[] uvAccelerationStructures,
            out UVFallbackBuffer[] uvFallbackBuffers)
        {
            List<UVMesh> uvMeshList = new();
            List<UVAccelerationStructure> uvAccelerationStructureList = new();
            List<UVFallbackBuffer> uvFallbackBufferList = new();
            uvMeshes = null;
            uvAccelerationStructures = null;
            uvFallbackBuffers = null;
            foreach (var instance in instances)
            {
                int uvMeshHash = UVMesh.GetHashCode(instance.Mesh);
                if (!_hashToUVMesh.TryGetValue(uvMeshHash, out UVMesh uvMesh))
                    return false;
                uvMeshList.Add(uvMesh);
                int uvASHash = UVAccelerationStructure.GetHashCode(uvMesh);
                if (!_hashToUVAccelerationStructure.TryGetValue(uvASHash, out UVAccelerationStructure uvAS))
                    return false;
                uvAccelerationStructureList.Add(uvAS);
                int uvFBHash = UVFallbackBuffer.GetHashCode(instance.TexelSize.x, instance.TexelSize.y, uvMesh, uvAS);
                if (!_hashToUVFallbackBuffer.TryGetValue(uvFBHash, out UVFallbackBuffer uvFB))
                    return false;
                uvFallbackBufferList.Add(uvFB);
            }
            uvMeshes = uvMeshList.ToArray();
            uvAccelerationStructures = uvAccelerationStructureList.ToArray();
            uvFallbackBuffers = uvFallbackBufferList.ToArray();
            return true;
        }

        private void Clear()
        {
            foreach (var uvMesh in _hashToUVMesh)
                uvMesh.Value.Dispose();
            foreach (var uvAccelerationStructure in _hashToUVAccelerationStructure)
                uvAccelerationStructure.Value.Dispose();
            foreach (var uvFallbackBuffer in _hashToUVFallbackBuffer)
                uvFallbackBuffer.Value.Dispose();
            _hashToUVMesh.Clear();
            _hashToUVAccelerationStructure.Clear();
            _hashToUVFallbackBuffer.Clear();
        }

        public void Dispose()
        {
            Clear();
        }
    }
}
