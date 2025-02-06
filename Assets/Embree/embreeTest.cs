using System.Collections.Generic;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEditor.Embree;
using UnityEngine;
using UnityEngine.Rendering.UnifiedRayTracing;

public class embreeTest : MonoBehaviour
{
    public Mesh mesh;

    private GpuBvhPrimitiveDescriptor[] prims;

    internal class AABB
    {
        public float3 Min;
        public float3 Max;

        public AABB()
        {
            Min = new float3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
            Max = new float3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);
        }

        public AABB(float3 min, float3 max)
        {
            Min = min;
            Max = max;
        }

        public void Encapsulate(AABB aabb)
        {
            Min = math.min(Min, aabb.Min);
            Max = math.max(Max, aabb.Max);
        }

        public void Encapsulate(float3 point)
        {
            Min = math.min(Min, point);
            Max = math.max(Max, point);
        }

        public bool Contains(AABB rhs)
        {
            return rhs.Min.x >= Min.x && rhs.Min.y >= Min.y && rhs.Min.z >= Min.z &&
                   rhs.Max.x <= Max.x && rhs.Max.y <= Max.y && rhs.Max.z <= Max.z;
        }

        public bool IsValid()
        {
            return Min.x <= Max.x && Min.y <= Max.y && Min.z <= Max.z;
        }
    }

    struct Triangle
    {
        public float3 v0;
        public float3 v1;
        public float3 v2;
    };

    static int3 GetFaceIndices(List<int> indices, int triangleIdx)
    {
        return new int3(
            indices[3 * triangleIdx],
            indices[3 * triangleIdx + 1],
            indices[3 * triangleIdx + 2]);
    }

    static Triangle GetTriangle(List<Vector3> vertices, int3 idx)
    {
        Triangle tri;
        tri.v0 = vertices[idx.x];
        tri.v1 = vertices[idx.y];
        tri.v2 = vertices[idx.z];
        return tri;
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
    }

    GpuBvhBuildOptions ConvertFlagsToCpuBuild(BuildFlags flags, bool isTopLevel)
    {
        GpuBvhBuildQuality quality = GpuBvhBuildQuality.Medium;

        if ((flags & BuildFlags.PreferFastBuild) != 0 && (flags & BuildFlags.PreferFastTrace) == 0)
            quality = GpuBvhBuildQuality.Low;
        else if ((flags & BuildFlags.PreferFastTrace) != 0 && (flags & BuildFlags.PreferFastBuild) == 0)
            quality = GpuBvhBuildQuality.High;

        return new GpuBvhBuildOptions
        {
            quality = quality,
            minLeafSize = (flags & BuildFlags.MinimizeMemory) != 0 && !isTopLevel ? 4u : 1u,
            maxLeafSize = isTopLevel ? 1u : 4u,
            allowPrimitiveSplits = !isTopLevel && (flags & BuildFlags.MinimizeMemory) == 0,
            isTopLevel = isTopLevel
        };
    }

    private Color gizmosColor(int i) 
    {
        switch (i)
        {
            case 0: 
                {
                    return Color.red;
                }
            case 1:
                {
                    return Color.blue;
                }
            case 2:
                {
                    return Color.yellow;
                }
            case 3:
                {
                    return Color.green;
                }
            case 4:
                {
                    return Color.black;
                }
            case 5:
                {
                    return Color.cyan;
                }
            default: 
                {
                    return Color.white;
                }
        }
    }

    void OnDrawGizmos()
    {
        var go = GameObject.Find("Cube");
        mesh = go.GetComponent<MeshFilter>().sharedMesh;

        var indices = new List<int>();
        mesh.GetTriangles(indices, 0, false);
        var vertices = new List<Vector3>();
        mesh.GetVertices(vertices);
        var triangleCount = (uint)(mesh.GetSubMesh(0).indexCount / 3);

        prims = new GpuBvhPrimitiveDescriptor[triangleCount];
        for (int i = 0; i < triangleCount; ++i)
        {
            var triangleIndices = GetFaceIndices(indices, i);
            var triangle = GetTriangle(vertices, triangleIndices);

            AABB aabb = new AABB();
            aabb.Encapsulate(triangle.v0);
            aabb.Encapsulate(triangle.v1);
            aabb.Encapsulate(triangle.v2);

            prims[i].primID = (uint)i;
            prims[i].lowerBound = aabb.Min;
            prims[i].upperBound = aabb.Max;
        }

        var options = ConvertFlagsToCpuBuild(BuildFlags.PreferFastBuild, false);
        var bvhBlob = GpuBvh.Build(options, prims);

        //for (int i = 2; i < 3; i++)
        {
            int i = 5;
            Gizmos.color = gizmosColor(0);
            var lowerBound = new Vector3(math.asfloat(bvhBlob[16 * i + 4]), math.asfloat(bvhBlob[16 * i + 5]), math.asfloat(bvhBlob[16 * i + 6]));
            var upperBound = new Vector3(math.asfloat(bvhBlob[16 * i + 7]), math.asfloat(bvhBlob[16 * i + 8]), math.asfloat(bvhBlob[16 * i + 9]));
            Gizmos.DrawWireCube((lowerBound + upperBound) * 0.5f, upperBound - lowerBound);

            Gizmos.color = gizmosColor(1);
            lowerBound = new Vector3(math.asfloat(bvhBlob[16 * i + 10]), math.asfloat(bvhBlob[16 * i + 11]), math.asfloat(bvhBlob[16 * i + 12]));
            upperBound = new Vector3(math.asfloat(bvhBlob[16 * i + 13]), math.asfloat(bvhBlob[16 * i + 14]), math.asfloat(bvhBlob[16 * i + 15]));
            Gizmos.DrawWireCube((lowerBound + upperBound) * 0.5f, upperBound - lowerBound);
        }

        //var lowerBound = new Vector3(math.asfloat(bvhBlob[4]), math.asfloat(bvhBlob[5]), math.asfloat(bvhBlob[6]));
        //var upperBound = new Vector3(math.asfloat(bvhBlob[7]), math.asfloat(bvhBlob[8]), math.asfloat(bvhBlob[9]));

        //Gizmos.DrawWireCube((lowerBound + upperBound) * 0.5f, upperBound - lowerBound);

        //foreach (var prim in prims)
        //var prim = prims[3];
        //{
        //    Gizmos.DrawWireCube((prim.lowerBound + prim.upperBound) * 0.5f, prim.upperBound - prim.lowerBound);
        //}
    }
}
