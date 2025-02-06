#ifndef _PATHTRACING_STOCHASTICLIGHTMAPSAMPLING_HLSL_
#define _PATHTRACING_STOCHASTICLIGHTMAPSAMPLING_HLSL_

#include "PathTracingCommon.hlsl"

void FetchGeomAttributes(UnifiedRT::Hit hit, int geometryIndex, out float3 position, out float3 normal, out float3 faceNormal)
{
    position = normal = faceNormal = 0.0f;
    // hit.instanceID is set to be the same as the submeshIndex
    // geometryIndex always points to the first submesh, the others submeshes follow in order in the geometry array:
    // submeshGeomIndex = geometryIndex + randomUVHit.instanceID
    if (!hit.IsValid())
        return;
    UnifiedRT::HitGeomAttributes hitAttribs = UnifiedRT::FetchHitGeomAttributes(geometryIndex + hit.instanceID, hit.primitiveIndex, hit.uvBarycentrics);
    position = hitAttribs.position;
    normal = hitAttribs.normal;
    faceNormal = hitAttribs.faceNormal;
}

UnifiedRT::Ray MakeUVRay(float2 origin)
{
    UnifiedRT::Ray uvRay;
    uvRay.direction = float3(0.0, 0.0, 1.0);
    uvRay.tMax = 0.0002;
    uvRay.tMin = 0.0;
    uvRay.origin = float3(origin, -0.0001);
    return uvRay;
};

UnifiedRT::Hit LightmapSampleTexelOffset(
    float2 texelPos,
    float2 texelOffset,
    float2 resolution,
    float2 instanceScale,
    UnifiedRT::DispatchInfo dispatchInfo,
    UnifiedRT::RayTracingAccelStruct uvAccelStruct,
    half2 uvFallback)
{
    // No valid uv data for this texel?
    if (uvFallback.x < 0)
        return UnifiedRT::Hit::Invalid();

    // select a random point in the texel
    const float2 scaledResolution = resolution * instanceScale;
    const float2 texelSample = (texelPos + texelOffset) / scaledResolution;

    // shoot a random ray towards the texel
    const UnifiedRT::Ray uvRay = MakeUVRay(texelSample);
    UnifiedRT::Hit randomUVHit = UnifiedRT::TraceRayClosestHit(dispatchInfo, uvAccelStruct, 0xFFFFFFFF, uvRay, 0);
    if (randomUVHit.IsValid())
    {
        return randomUVHit;
    }
    else
    {
        // use fallback uv
        const float2 fallbackUvSample = (texelPos + uvFallback) / scaledResolution;
        const UnifiedRT::Ray uvRay = MakeUVRay(fallbackUvSample);
        return UnifiedRT::TraceRayClosestHit(dispatchInfo, uvAccelStruct, 0xFFFFFFFF, uvRay, 0);
    }
    return UnifiedRT::Hit::Invalid();
}

#endif
