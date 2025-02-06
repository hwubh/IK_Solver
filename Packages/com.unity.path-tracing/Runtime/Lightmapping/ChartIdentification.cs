using System;
using System.Collections.Generic;
using Unity.Jobs;
using Unity.Collections;
using Unity.Mathematics;

namespace UnityEngine.PathTracing.Lightmapping
{
    internal static class ChartIdentification
    {
        private static UInt32 FindRepWithPathCompression(Span<UInt32> reps, UInt32 vertexIdx)
        {
            UInt32 rep = reps[(int)vertexIdx];
            if (rep == vertexIdx)
                return vertexIdx;

            rep = FindRepWithPathCompression(reps, rep);
            reps[(int)vertexIdx] = rep;
            return rep;
        }

        private static UInt32 FindRepresentative(Span<UInt32> reps, UInt32 vertexIdx)
        {
            UInt32 rep = reps[(int)vertexIdx];
            return (rep == vertexIdx ? vertexIdx : FindRepresentative(reps, rep));
        }

        private static void Union(Span<UInt32> reps, UInt32 vertexIdx0, UInt32 vertexIdx1)
        {
            var rep0 = FindRepWithPathCompression(reps, vertexIdx0);
            var rep1 = FindRepWithPathCompression(reps, vertexIdx1);
            reps[(int)rep1] = rep0;
        }

        public static void UnionTriangleEdges(ReadOnlySpan<UInt32> triangleIndices, Span<UInt32> vertexChartIds)
        {
            Span<UInt32> reps = vertexChartIds;

            Debug.Assert(triangleIndices.Length % 3 == 0);

            int triangleCount = triangleIndices.Length / 3;

            for (int triIdx = 0; triIdx < triangleCount; ++triIdx)
            {
                int offset = triIdx * 3;
                UInt32 vertexIdx0 = triangleIndices[offset];
                UInt32 vertexIdx1 = triangleIndices[offset + 1];
                UInt32 vertexIdx2 = triangleIndices[offset + 2];
                Union(reps, vertexIdx0, vertexIdx1);
                Union(reps, vertexIdx1, vertexIdx2);
            }
        }

        public static void UnionDuplicateVertices(ReadOnlySpan<float2> vertexUvs, ReadOnlySpan<float3> vertexPositions, ReadOnlySpan<float3> vertexNormals, Span<UInt32> vertexChartIds)
        {
            var map = new Dictionary<(float2, float3, float3), uint>();
            for (uint i = 0; i < vertexChartIds.Length; ++i)
            {
                var tuple = (
                    vertexUvs[(int)i],
                    vertexPositions[(int)i],
                    vertexNormals[(int)i]);
                if (map.TryGetValue(tuple, out uint deduplicatedIndex))
                {
                    Union(vertexChartIds, i, deduplicatedIndex);
                }
                else
                {
                    map.Add(tuple, i);
                }
            }
        }

        public static void FindRepresentatives(Span<UInt32> vertexChartIds)
        {
            for (uint i = 0; i < vertexChartIds.Length; ++i)
            {
                vertexChartIds[(int)i] = FindRepresentative(vertexChartIds, i);
            }
        }

        public static void InitializeRepresentatives(Span<UInt32> vertexChartIds)
        {
            for (UInt32 i = 0; i < vertexChartIds.Length; ++i)
                vertexChartIds[(int)i] = i;
        }

        public static void Compact(Span<UInt32> vertexChartIds, out uint chartCount)
        {
            var map = new Dictionary<UInt32, UInt32>();
            for (int vertexIdx = 0; vertexIdx < vertexChartIds.Length; ++vertexIdx)
            {
                var chartId = vertexChartIds[vertexIdx];
                UInt32 newChartId;
                if (map.TryGetValue(chartId, out UInt32 compactedId))
                {
                    newChartId = compactedId;
                }
                else
                {
                    newChartId = (UInt32)map.Count;
                    map.Add(chartId, newChartId);
                }
                vertexChartIds[vertexIdx] = newChartId;
            }
            chartCount = (uint)map.Count;
        }
    }

    internal class ParallelChartIdentification : IDisposable
    {
        private readonly MeshChartIdentificationJob[] _jobs;
        private readonly JobHandle[] _jobHandles;
        private readonly NativeArray<UInt32>[] _outputVertexChartIndices;
        private readonly NativeArray<UInt32>[] _outputChartCounts;
        private readonly Dictionary<Mesh, uint> _meshToJobIdx;

        public struct MeshResult
        {
            public NativeArray<UInt32> VertexChartIndices;
            public uint ChartCount;
        };

        private struct MeshChartIdentificationJob : IJob
        {
            [ReadOnly, DeallocateOnJobCompletion]
            public NativeArray<UInt32> InputVertexIndexBuffer;

            [ReadOnly, DeallocateOnJobCompletion]
            public NativeArray<float2> InputVertexUvBuffer;

            [ReadOnly, DeallocateOnJobCompletion]
            public NativeArray<float3> InputVertexPositionBuffer;

            [ReadOnly, DeallocateOnJobCompletion]
            public NativeArray<float3> InputVertexNormalBuffer;

            public NativeArray<UInt32> OutputVertexChartIndicesBuffer;

            // Always a single element. Job system doesn't support scalar outputs.
            public NativeArray<UInt32> OutputChartCount;

            public void Execute()
            {
                ChartIdentification.InitializeRepresentatives(OutputVertexChartIndicesBuffer);
                ChartIdentification.UnionTriangleEdges(InputVertexIndexBuffer, OutputVertexChartIndicesBuffer);
                ChartIdentification.UnionDuplicateVertices(InputVertexUvBuffer, InputVertexPositionBuffer, InputVertexNormalBuffer, OutputVertexChartIndicesBuffer);
                ChartIdentification.FindRepresentatives(OutputVertexChartIndicesBuffer);
                ChartIdentification.Compact(OutputVertexChartIndicesBuffer, out uint outputChartCount);
                OutputChartCount[0] = outputChartCount;
            }
        }

        public ParallelChartIdentification(IList<Mesh> meshes)
        {
            _outputVertexChartIndices = new NativeArray<UInt32>[meshes.Count];
            _outputChartCounts = new NativeArray<UInt32>[meshes.Count];
            _jobs = new MeshChartIdentificationJob[meshes.Count];
            _jobHandles = new JobHandle[meshes.Count];
            _meshToJobIdx = new Dictionary<Mesh, uint>();

            for (uint meshIdx = 0; meshIdx < meshes.Count; ++meshIdx)
            {
                var mesh = meshes[(int)meshIdx];

                // Select uv buffer, prefer uv2
                var uvBuffer = mesh.uv2;
                if (uvBuffer == null || uvBuffer.Length == 0)
                    uvBuffer = mesh.uv;

                var inputVertexIndices = new NativeArray<Int32>(mesh.triangles, Allocator.TempJob).Reinterpret<UInt32>(sizeof(Int32));
                var inputVertexUvs = new NativeArray<Vector2>(uvBuffer, Allocator.TempJob).Reinterpret<float2>(sizeof(float) * 2);
                var inputVertexPositions = new NativeArray<Vector3>(mesh.vertices, Allocator.TempJob).Reinterpret<float3>(sizeof(float) * 3);
                var inputVertexNormals = new NativeArray<Vector3>(mesh.normals, Allocator.TempJob).Reinterpret<float3>(sizeof(float) * 3);
                var outputChartIndices = new NativeArray<UInt32>(mesh.vertexCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                var outputChartCount = new NativeArray<UInt32>(1, Allocator.TempJob);
                var job = new MeshChartIdentificationJob
                {
                    InputVertexIndexBuffer = inputVertexIndices,
                    InputVertexUvBuffer = inputVertexUvs,
                    InputVertexPositionBuffer = inputVertexPositions,
                    InputVertexNormalBuffer = inputVertexNormals,
                    OutputVertexChartIndicesBuffer = outputChartIndices,
                    OutputChartCount = outputChartCount,
                };

                _outputVertexChartIndices[meshIdx] = outputChartIndices;
                _outputChartCounts[meshIdx] = outputChartCount;
                _jobs[meshIdx] = job;
                _meshToJobIdx[mesh] = meshIdx;
            }
        }

        public void Start()
        {
            for (uint meshIdx = 0; meshIdx < _jobs.Length; ++meshIdx)
            {
                _jobHandles[meshIdx] = _jobs[meshIdx].ScheduleByRef();
            }
        }

        public MeshResult CompleteAndGetResult(Mesh mesh)
        {
            uint meshIdx = _meshToJobIdx[mesh];
            _jobHandles[meshIdx].Complete();
            var meshResult = new MeshResult
            {
                VertexChartIndices = _outputVertexChartIndices[meshIdx],
                ChartCount = _outputChartCounts[meshIdx][0],
            };
            return meshResult;
        }

        public void Dispose()
        {
            for (uint meshIdx = 0; meshIdx < _jobs.Length; ++meshIdx)
                _jobHandles[meshIdx].Complete();

            foreach (var chartIndexList in _outputVertexChartIndices)
                chartIndexList.Dispose();

            foreach (var chartCountList in _outputChartCounts)
                chartCountList.Dispose();
        }
    }
}
