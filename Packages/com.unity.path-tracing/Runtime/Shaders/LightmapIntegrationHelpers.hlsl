#ifndef _PATHTRACING_LIGHTMAPINTEGRATIONHELPERS_HLSL_
#define _PATHTRACING_LIGHTMAPINTEGRATIONHELPERS_HLSL_

#include "PathTracingCommon.hlsl"
#include "StochasticLightmapSampling.hlsl"

StructuredBuffer<HitEntry> g_GBuffer;
RWStructuredBuffer<uint> g_CompactedGBuffer;
RWStructuredBuffer<uint> g_CompactedGBufferLength; // This will contain the number of texels written.
int g_MaxLocalSampleCount; // Don't take more than this many samples per texel in this dispatch, sometimes the last dispatch will have fewer samples as the expanded sample count may not be a multiple of the total sample count
int g_ExpandedTexelSampleWidth; // The number of samples per texel in the expanded buffers
int g_InstanceWidth;
int g_ChunkOffsetX;
int g_ChunkOffsetY;
int g_InstanceGeometryIndex;
float4x4 g_ShaderLocalToWorld;
float4x4 g_ShaderLocalToWorldNormals;

bool GetExpandedSample(uint dispatchIndex, out uint localSampleOffset, out uint2 instanceTexelPos, inout float3 worldPosition, inout float3 worldNormal, inout float3 worldFaceNormal)
{
    localSampleOffset = dispatchIndex % g_ExpandedTexelSampleWidth;
    const uint compactedTexelIndex = dispatchIndex / g_ExpandedTexelSampleWidth;
    const uint texelIndex = g_CompactedGBuffer[compactedTexelIndex];
    const uint linearChunkOffset = g_ChunkOffsetY * g_InstanceWidth + g_ChunkOffsetX;
    const uint linearTexelIndex = texelIndex + linearChunkOffset;
    instanceTexelPos = uint2(linearTexelIndex % g_InstanceWidth, linearTexelIndex / g_InstanceWidth);

    if (localSampleOffset >= (uint)g_MaxLocalSampleCount)
        return false; // no more samples to process

    if (!g_GBuffer[texelIndex].IsValid())
        return false; // no intersection found, skip this sample

    UnifiedRT::Hit hit;
    hit.instanceID = g_GBuffer[texelIndex].instanceID;
    hit.primitiveIndex = g_GBuffer[texelIndex].primitiveIndex;
    hit.uvBarycentrics = g_GBuffer[texelIndex].barycentrics;

    FetchGeomAttributes(hit, g_InstanceGeometryIndex, worldPosition, worldNormal, worldFaceNormal);

    worldPosition = mul(g_ShaderLocalToWorld, float4(worldPosition, 1)).xyz;
    worldNormal = normalize(mul((float3x3)g_ShaderLocalToWorldNormals, worldNormal));
    worldFaceNormal = normalize(mul((float3x3)g_ShaderLocalToWorldNormals, worldFaceNormal));
    return true;
}
#endif
