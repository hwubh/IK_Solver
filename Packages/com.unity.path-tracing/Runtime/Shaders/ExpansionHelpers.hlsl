#ifndef _PATHTRACING_EXPANSIONHELPERS_HLSL_
#define _PATHTRACING_EXPANSIONHELPERS_HLSL_

#include "PathTracingCommon.hlsl"

StructuredBuffer<HitEntry> g_GBuffer;
RWStructuredBuffer<uint> g_CompactedGBuffer;
RWStructuredBuffer<uint> g_CompactedGBufferLength; // This will contain the number of texels written.

void CompactGBufferInternal(uint index)
{
    if (g_GBuffer[index].instanceID == -1)
        return; // Empty texel
    uint destinationIndex = 0;
    InterlockedAdd(g_CompactedGBufferLength[0], 1, destinationIndex);
    g_CompactedGBuffer[destinationIndex] = index;
}

#endif
