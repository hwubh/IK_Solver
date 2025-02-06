
#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Common.hlsl"
#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Sampling/Sampling.hlsl"

#include "Packages/com.unity.rendering.light-transport/Runtime/UnifiedRayTracing/FetchGeometry.hlsl"
#include "Packages/com.unity.rendering.light-transport/Runtime/UnifiedRayTracing/TraceRay.hlsl"

#include "PathTracingCommon.hlsl"

#include "PathTracingRandom.hlsl"

int                             g_BounceCount;
uint                            g_LightEvaluations;
int                             g_CountNEERayAsPathSegment; // This flag must be enabled for MIS to work properly.
int                             g_RenderedInstances;
int                             g_PathtracerAsGiPreviewMode;

#include "LightSampling.hlsl"

UNIFIED_RT_DECLARE_ACCEL_STRUCT(g_SceneAccelStruct);

#define RENDER_ALL 0
#define RENDER_ONLY_STATIC 1
#define RENDER_ALL_IN_CAMERA_RAYS_THEN_ONLY_STATIC 2

#define RAY_TERMINATION     0
#define RAY_SCATTERING      1

int ScatterDiffusely(PTHitGeom hitGeom, float3 V, inout PathTracingSampler rngState, out UnifiedRT::Ray bounceRay, out float brdfPdf)
{
    float2 u = float2(rngState.GetFloatSample(RAND_DIM_SURF_SCATTER_X), rngState.GetFloatSample(RAND_DIM_SURF_SCATTER_Y));

    bounceRay = (UnifiedRT::Ray)0;

    float3 rayDirection;
    if (!SampleDiffuseBrdf(u, hitGeom.worldFaceNormal, hitGeom.worldNormal, V, rayDirection, brdfPdf))
        return RAY_TERMINATION;

    bounceRay.origin = hitGeom.NextRayOrigin();
    bounceRay.direction = rayDirection;
    bounceRay.tMin = 0;
    bounceRay.tMax = K_T_MAX;

    return RAY_SCATTERING;
}

uint RayMask(bool isFirstHitSurface)
{
    if (g_RenderedInstances == RENDER_ALL)
        return DIRECT_RAY_VIS_MASK | INDIRECT_RAY_VIS_MASK;
    else if (g_RenderedInstances == RENDER_ALL_IN_CAMERA_RAYS_THEN_ONLY_STATIC)
        return isFirstHitSurface ? DIRECT_RAY_VIS_MASK | INDIRECT_RAY_VIS_MASK : INDIRECT_RAY_VIS_MASK;
    else // RENDER_ONLY_STATIC
        return isFirstHitSurface ? DIRECT_RAY_VIS_MASK : INDIRECT_RAY_VIS_MASK;
}

uint ShadowRayMask()
{
    return SHADOW_RAY_VIS_MASK;
}

float ComputeContinuationRayMISWeight(float lightPdf, float continuationRayPdf)
{
    // Emissive surfaces are also sampled explicitely as lights. Compute MIS weight to combine the two sampling strategies.
    #if (EMISSIVE_SAMPLING == LIGHT_SAMPLING)
    float misWeight = 0;
    #elif (EMISSIVE_SAMPLING == BRDF_SAMPLING)
    float misWeight = 1;
    #else
    float misWeight = continuationRayPdf > 0 ? PowerHeuristic(continuationRayPdf, lightPdf) : 1.0;
    #endif
    return misWeight;
}

float ComputeEmissiveTriangleMISWeight(UnifiedRT::RayTracingAccelStruct accelStruct, PTHitGeom hitGeom, int instanceIndex, float3 rayOrigin)
{
    float3 L = hitGeom.worldPosition - rayOrigin;
    float d = length(L);
    if (d > 0)
        L *= rcp(d);
    float cosTheta = dot(hitGeom.worldFaceNormal, L);
    // pdf to sample this as area light
    float weight = (d * d) / (hitGeom.triangleArea * cosTheta);
    // pdf to select the light source
    uint lightSampleCount = min(g_LightEvaluations, g_NumLights);
    weight *= (float)lightSampleCount;
    if (g_LightPickingMethod == LIGHT_PICKING_METHOD_POWER)
    {
        int lightIndex = FindEmissiveMeshLightIndex(instanceIndex);
        float pickingPdf = (lightIndex == 0) ? g_LightCdf[0] : g_LightCdf[lightIndex] - g_LightCdf[lightIndex - 1];
        weight *= pickingPdf;
    }
    else
        weight /= g_NumLights;

    // adjust pdf based on number of emissive triangles in the submesh that we hit
    int geometryIndex = g_AccelStructInstanceList[instanceIndex].geometryIndex;
    int numEmissiveTriangles = g_MeshList[geometryIndex].indexCount / uint(3);
    weight /= numEmissiveTriangles;

    return weight;
}

bool ShouldTreatAsBackface(UnifiedRT::Hit hitResult, MaterialProperties material)
{
    // Have we hit something that is considered a backface when double sided GI is taken into account?
    return !hitResult.isFrontFace && !material.doubleSidedGI;
}

bool ShouldTransmitRay(inout PathTracingSampler rngState, MaterialProperties material)
{
    bool result = false;
    if (material.isTransmissive)
    {
        // With proper support for IOR, the probability of refraction should be based on the materials fresnel.
        // We don't have this information, so we base it on average transmission color, which matches the old baker.
        // Additionally, we should divide the contribution of the ray by the probability of choosing either reflection or refraction,
        // but we intentionally don't do this, since the old baker didn't do it either.
        float transmissionProbability = dot(material.transmission, 1.0f) / 3.0f;
        if (rngState.GetFloatSample(RAND_DIM_TRANSMISSION) < transmissionProbability)
        {
            result = true;
        }
        else
        {
            result = false;
        }
    }
    return result;
}

#define TRACE_MISS 0
#define TRACE_HIT 1
#define TRACE_TRANSMISSION 2

struct PathIterator
{
    UnifiedRT::Ray ray;
    UnifiedRT::Hit hitResult;
    PTHitGeom hitGeo;
    MaterialProperties material;
    float lastScatterProbabilityDensity;
    float3 radianceSample;
    float3 throughput;
};

void AddRadiance(inout float3 radianceAccumulator, float3 throughput, float3 radianceSample)
{
    radianceAccumulator += throughput * radianceSample;
}

void InitPathIterator(out PathIterator iter, UnifiedRT::Ray primaryRay)
{
    iter = (PathIterator)0;
    iter.lastScatterProbabilityDensity = 0;
    iter.radianceSample = 0;
    iter.throughput = 1;
    iter.ray = primaryRay;
}

uint TraceBounceRay(inout PathIterator iterator, int bounceIndex, uint rayMask, UnifiedRT::DispatchInfo dispatchInfo, UnifiedRT::RayTracingAccelStruct accelStruct, inout PathTracingSampler rngState)
{
    bool isFirstRay = (bounceIndex == 0);
    uint traceResult = TRACE_MISS;

    // Trace the ray. For primary rays in LiveGI we want to respect the backfacing culling properties of the material. Bounce rays follow the culling behavior of the baker.
    int rayFlags = (bounceIndex == 0) ? UnifiedRT::kRayFlagCullBackFacingTriangles : UnifiedRT::kRayFlagNone;
    iterator.hitResult = UnifiedRT::TraceRayClosestHit(dispatchInfo, accelStruct, rayMask, iterator.ray, rayFlags);

    iterator.hitGeo = (PTHitGeom) 0;
    iterator.material = (MaterialProperties) 0;
    UnifiedRT::InstanceData instanceInfo = (UnifiedRT::InstanceData) 0;
    if (iterator.hitResult.IsValid())
    {
        instanceInfo = UnifiedRT::GetInstance(iterator.hitResult.instanceID);
        iterator.hitGeo = GetHitGeomInfo(instanceInfo, iterator.hitResult);
        iterator.hitGeo.FixNormals(iterator.ray.direction);

        // Evaluate material properties at hit location
        iterator.material = LoadMaterialProperties(instanceInfo, g_PathtracerAsGiPreviewMode && isFirstRay, iterator.hitGeo);
        traceResult = TRACE_HIT;
    }
    else
    {
        float3 envRadiance = g_EnvTex.SampleLevel(sampler_g_EnvTex, iterator.ray.direction, 0).xyz;

        // In the special case when the environment is zero/black, its importance sampling PDF is undefined.
        // This check ensures we don't evaluate the PDF in this case.
        if (any(envRadiance != 0.0f))
        {
#ifdef UNIFORM_ENVSAMPLING
            float envPdf = rcp(4 * PI);
#else
            float envPdf = GetSkyPDFFromValue(envRadiance);
#endif

            // apply intensity multiplier for the environment
            envRadiance *= g_EnvIntensityMultiplier;

            uint lightSampleCount = min(g_LightEvaluations, g_NumLights);
            envPdf *= (float) lightSampleCount;
            if (g_LightPickingMethod == LIGHT_PICKING_METHOD_POWER)
            {
                int lightIndex = g_NumLights - 1; // we expect env light to be stored last
                float pickingPdf = (lightIndex == 0) ? g_LightCdf[0] : g_LightCdf[lightIndex] - g_LightCdf[lightIndex - 1];
                envPdf *= pickingPdf;
            }
            else
                envPdf /= g_NumLights;

            if (!isFirstRay)
                envRadiance *= g_IndirectScale;

            AddRadiance(iterator.radianceSample, iterator.throughput, envRadiance * ComputeContinuationRayMISWeight(envPdf, iterator.lastScatterProbabilityDensity));
        }

        traceResult = TRACE_MISS;
    }

    if (traceResult == TRACE_HIT)
    {
        // If we hit a transmissive face, we should transmit or reflect.
        if (ShouldTransmitRay(rngState, iterator.material))
        {
            traceResult = TRACE_TRANSMISSION;
        }
        // We've hit a surface that should be treated as a backface, we should kill the ray instead of bouncing. This matches the old behavior.
        else if (ShouldTreatAsBackface(iterator.hitResult, iterator.material))
        {
            traceResult = TRACE_MISS;
        }
    }

    return traceResult;
}


void EvalDirectIllumination(inout PathIterator iterator, int bounceIndex, uint shadowRayMask, UnifiedRT::DispatchInfo dispatchInfo, UnifiedRT::RayTracingAccelStruct accelStruct, inout PathTracingSampler rngState)
{
    bool isFirstRay = (bounceIndex == 0);

    // Emission
    if (!g_PathtracerAsGiPreviewMode || !isFirstRay)
    {
        float lightPdf = ComputeEmissiveTriangleMISWeight(accelStruct, iterator.hitGeo, iterator.hitResult.instanceID, iterator.ray.origin);
        float3 emission = iterator.material.emissive;
        if (!isFirstRay)
            emission *= g_IndirectScale;
        AddRadiance(iterator.radianceSample, iterator.throughput, emission * ComputeContinuationRayMISWeight(lightPdf, iterator.lastScatterProbabilityDensity));
    }

    // Check if we should do NEE, respecting the max bounce count
    if (!g_CountNEERayAsPathSegment || bounceIndex < g_BounceCount)
    {
        const float3 radianceSample = EvalDirectIllumination(
            dispatchInfo, accelStruct, g_PathtracerAsGiPreviewMode && isFirstRay, shadowRayMask, iterator.hitGeo, iterator.material, min(g_LightEvaluations, MAX_LIGHT_EVALUATIONS), rngState);

        AddRadiance(iterator.radianceSample, iterator.throughput, radianceSample);
    }
}

uint TraceBounceRayAndEvalDirectIllumination(inout PathIterator iterator, int bounceIndex, uint rayMask, uint shadowRayMask, UnifiedRT::DispatchInfo dispatchInfo, UnifiedRT::RayTracingAccelStruct accelStruct, inout PathTracingSampler rngState)
{
    uint traceResult = TraceBounceRay(iterator, bounceIndex, rayMask, dispatchInfo, accelStruct, rngState);

    if (traceResult == TRACE_HIT)
        EvalDirectIllumination(iterator, bounceIndex, shadowRayMask, dispatchInfo, accelStruct, rngState);

    return traceResult;
}


bool Scatter(inout PathIterator iterator, inout PathTracingSampler rngState)
{
    float brdfPdf;
    int event = ScatterDiffusely(iterator.hitGeo, -iterator.ray.direction, rngState, iterator.ray, brdfPdf);
    if (event == RAY_TERMINATION)
        return false;

    // Here we assumes two things:
    // 1) We use cosine distribution for bounce.
    // 2) We never multiply the cosine density, cos(θ)/π.
    // This cancels out the cosine term of the rendering equation and the division by π
    // in the diffuse BRDF. Thus we only need to multiply by albedo below.
    iterator.throughput *= iterator.material.baseColor;

    iterator.lastScatterProbabilityDensity = brdfPdf;
    return true;
}

float3 EstimateRadiance(UnifiedRT::DispatchInfo dispatchInfo, UnifiedRT::Ray ray, inout PathTracingSampler rngState)
{
    UnifiedRT::RayTracingAccelStruct accelStruct = UNIFIED_RT_GET_ACCEL_STRUCT(g_SceneAccelStruct);

    PathIterator pathIter;
    InitPathIterator(pathIter, ray);

    int transparencyBounce = 0;
    // We start at bounce index 1, as bounce index is defined relative to the camera for this path tracer.
    // Since this function is used for baking, we already implicitly have the first "hit", and are about
    // to process the second path segment.
    for (int bounceIndex = 1; bounceIndex <= g_BounceCount && transparencyBounce < MAX_TRANSMISSION_BOUNCES; bounceIndex++)
    {
        uint pathRayMask = RayMask(bounceIndex == 0);
        uint shadowRayMask = ShadowRayMask();

        uint traceResult = TraceBounceRayAndEvalDirectIllumination(pathIter, bounceIndex, pathRayMask, shadowRayMask, dispatchInfo, accelStruct, rngState);

        if (traceResult == TRACE_MISS)
        {
            break;
        }

        if (traceResult == TRACE_TRANSMISSION)
        {
            bounceIndex--;
            transparencyBounce++;
            pathIter.ray.origin = pathIter.hitGeo.NextTransmissionRayOrigin();
            pathIter.throughput *= pathIter.material.transmission;
            rngState.NextBounce();
            continue;
        }

        if (!Scatter(pathIter, rngState))
            break;

        if (bounceIndex >= RUSSIAN_ROULETTE_MIN_BOUNCES)
        {
            float p = max(pathIter.throughput.x, max(pathIter.throughput.y, pathIter.throughput.z));
            if (rngState.GetFloatSample(RAND_DIM_RUSSIAN_ROULETTE) > p)
                break;
            else
                pathIter.throughput /= p;
        }

        rngState.NextBounce();
    }

    return pathIter.radianceSample;
}
