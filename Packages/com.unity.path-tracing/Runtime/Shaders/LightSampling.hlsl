#ifndef _PATHTRACING_LIGHTSAMPLING_HLSL_
#define _PATHTRACING_LIGHTSAMPLING_HLSL_

#include "PathTracingCommon.hlsl"
#include "PathTracingMaterials.hlsl"
#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Common.hlsl"
#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Sampling/Sampling.hlsl"
#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/CommonLighting.hlsl"
#include "PathTracingSkySampling.hlsl"

#define SOLID_ANGLE_SAMPLING
#define RESAMPLED_IMPORTANCE_SAMPLING
#define TRANSMISSION_IN_SHADOW_RAYS
#define USE_DISTANCE_ATTENUATION_LUT

#ifndef QRNG_METHOD_GLOBAL_SOBOL_BLUE_NOISE
// Unless we use global sobol, stratification improves light selection
#define STRATIFIED_LIGHT_PICKING
#endif

TextureCube<float4>             g_EnvTex;
SamplerState                    sampler_g_EnvTex;
float                           g_EnvIntensityMultiplier;

int                             g_LightPickingMethod;
StructuredBuffer<PTLight>       g_LightList;
StructuredBuffer<float>         g_LightCdf;
StructuredBuffer<float>         g_LightFalloff;
StructuredBuffer<float>         g_LightFalloffLUTRange;
uint                            g_LightFalloffLUTLength;
uint                            g_NumLights;
uint                            g_NumEmissiveMeshes;

// Light cookies
Texture2DArray<float4>          g_CookieAtlas;
TextureCubeArray<float4>        g_CubemapAtlas;
SamplerState                    sampler_g_CookieAtlas;
SamplerState                    sampler_g_CubemapAtlas;

// Exposure texture - 1x1 RG16F (r: exposure mult, g: exposure EV100)
TEXTURE2D(_ExposureTexture);
int g_PreExpose;
float g_IndirectScale;
float g_ExposureScale;
int g_MaxIntensity;

#define SHADEROPTIONS_PRE_EXPOSITION 1

// Copied from com.unity.render-pipelines.high-definition\Runtime\ShaderLibrary\ShaderVariables.hlsl
// URP-only projects will not have this file, so for now copy-pasting here.
float GetCurrentExposureMultiplier()
{
    if (g_PreExpose)
    {
        #if SHADEROPTIONS_PRE_EXPOSITION
        // g_ExposureScale is a scale used to perform range compression to avoid saturation of the content of the probes. It is 1.0 if we are not rendering probes.
        return LOAD_TEXTURE2D(_ExposureTexture, int2(0, 0)).x * g_ExposureScale;
        #else
        return g_ExposureScale;
        #endif
    }
    else
    {
        return 1.0;
    }
}

float3 ClampRadiance(float3 value)
{
    float intensity = Luminance(value) * GetCurrentExposureMultiplier();
    return intensity > g_MaxIntensity ? value * g_MaxIntensity / intensity : value;
}

bool CastShadowRay(UnifiedRT::DispatchInfo dispatchInfo,
    UnifiedRT::RayTracingAccelStruct accelStruct,
    float3 origin,
    float3 direction,
    float maxDistance,
    uint rayMask,
    out float3 attenuation)
{
    UnifiedRT::Ray shadowRay;
    shadowRay.origin = origin;
    shadowRay.direction = direction;
    shadowRay.tMin = 0.0;
    shadowRay.tMax = maxDistance;

    attenuation = 1.0f;

    #ifdef TRANSMISSION_IN_SHADOW_RAYS
    for (uint i = 0; i < MAX_TRANSMISSION_BOUNCES; i++)
    {
        UnifiedRT::Hit hitResult = UnifiedRT::TraceRayClosestHit(dispatchInfo, accelStruct, rayMask, shadowRay, 0);
        if (hitResult.IsValid())
        {
            UnifiedRT::InstanceData instance = UnifiedRT::GetInstance(hitResult.instanceID);
            PTHitGeom geometry = GetHitGeomInfo(instance, hitResult);
            geometry.FixNormals(direction);
            MaterialProperties material = LoadMaterialProperties(instance, false, geometry);

            // Transmissive material, continue ray and attenuate throughput
            if (material.isTransmissive)
            {
                attenuation *= saturate(material.transmission);
                shadowRay.origin = geometry.NextTransmissionRayOrigin();
                shadowRay.tMax = maxDistance - distance(origin, shadowRay.origin);
                if (all(attenuation < 0.000001f))
                    return false; // ray reached full attenuation so can be considered occluded
            }
            // Opaque material, so the ray is occluded
            else
            {
                return false;
            }
        }
        // Hit nothing, so the ray is not occluded
        else
        {
            return true;
        }
    }
    return true;
    #else
    return !UnifiedRT::TraceRayAnyHit(dispatchInfo, accelStruct, rayMask, shadowRay, 0);
    #endif
}

float3 CalculateJitteredLightVec(float shadowRadius, float2 rnd, float3 lightVec)
{
    float3x3 lightBasis = OrthoBasisFromVector(normalize(lightVec));
    float2 diskSample = MapSquareToDisk(rnd);
    float3 jitterOffset = float3(shadowRadius * diskSample, 0.0f);
    jitterOffset = mul(jitterOffset, lightBasis);
    return normalize(lightVec + jitterOffset);
}

bool CastJitteredShadowRay(UnifiedRT::DispatchInfo dispatchInfo,
    UnifiedRT::RayTracingAccelStruct accelStruct,
    float2 rnd,
    float shadowRadius,
    float3 lightVector, // Vector from ray `origin` to light if light is not a directional light. Otherwise, it is a unit vector pointing toward the directional light.
    float3 origin,
    float3 direction,
    float distance,
    uint shadowRayMask,
    out float3 attenuation)
{
    // jitter the light direction within the shadow radius
    direction = CalculateJitteredLightVec(shadowRadius, rnd, lightVector);
    return CastShadowRay(dispatchInfo, accelStruct, origin, direction, distance, shadowRayMask, attenuation);
}

struct SphQuad
{
    float3 o, x, y, z;
    float z0, z0sq;
    float x0, y0, y0sq;
    float x1, y1, y1sq;
    float b0, b1, b0sq, k;
    float S;
};

bool SampleRectangularLight(inout PathTracingSampler rngState, uint dimsOffset, out uint dimsUsed, float3 P, PTLight light, inout LightSample lightSample)
{
    float u = rngState.GetFloatSample(dimsOffset);
    float v = rngState.GetFloatSample(dimsOffset+1);
    dimsUsed = 2;

#ifndef SOLID_ANGLE_SAMPLING
    // Area sampling
    u -= 0.5f;
    v -= 0.5f;

    float3 position = light.position + u * light.width * light.right + v * light.height * light.up;
    lightSample.lightVector = position - P;
    lightSample.distanceToLight = length(lightSample.lightVector);
    lightSample.L = lightSample.lightVector * rcp(lightSample.distanceToLight);
    lightSample.weight = 0;
    lightSample.uv = 0;
    lightSample.materialIndex = -1;

    float cosTheta = -dot(lightSample.L, light.forward);
    if (cosTheta < 0.001)
        return false;

    float d = lightSample.distanceToLight;
    float lightArea = light.width * light.height;
    lightSample.weight = lightArea * cosTheta / (d * d);

    return true;

#else
    // Solid angle sampling
    SphQuad squad;
    {
        // compute local reference system ’R’
        squad.o = P;
        squad.x = light.right;
        squad.y = light.up;
        squad.z = cross(squad.x, squad.y);

        // Adjust the position of the light to be at the center of the rectangle
        light.position = light.position - 0.5 * light.width * light.right;
        light.position = light.position - 0.5 * light.height * light.up;

        // compute rectangle coords in local reference system
        float3 d = light.position - P;
        squad.z0 = dot(d, squad.z);

        // flip ’z’ to make it point against ’Q’
        if (squad.z0 > 0)
        {
            squad.z *= -1;
            squad.z0 *= -1;
        }

        float exl = light.width;
        float eyl = light.height;
        squad.z0sq = squad.z0 * squad.z0;
        squad.x0 = dot(d, squad.x);
        squad.y0 = dot(d, squad.y);
        squad.x1 = squad.x0 + exl;
        squad.y1 = squad.y0 + eyl;
        squad.y0sq = squad.y0 * squad.y0;
        squad.y1sq = squad.y1 * squad.y1;

        // create vectors to four vertices
        float3 v00 = float3 (squad.x0, squad.y0, squad.z0);
        float3 v01 = float3 (squad.x0, squad.y1, squad.z0);
        float3 v10 = float3 (squad.x1, squad.y0, squad.z0);
        float3 v11 = float3 (squad.x1, squad.y1, squad.z0);

        // compute normals to edges
        float3 n0 = normalize(cross(v00, v10));
        float3 n1 = normalize(cross(v10, v11));
        float3 n2 = normalize(cross(v11, v01));
        float3 n3 = normalize(cross(v01, v00));

        // compute internal angles (gamma_i)
        float g0 = acos(-dot(n0, n1));
        float g1 = acos(-dot(n1, n2));
        float g2 = acos(-dot(n2, n3));
        float g3 = acos(-dot(n3, n0));

        // compute predefined constants
        squad.b0 = n0.z;
        squad.b1 = n2.z;
        squad.b0sq = squad.b0 * squad.b0;
        squad.k = 2 * PI - g2 - g3;

        // compute solid angle from internal angles
        squad.S = g0 + g1 - squad.k;
    }

    // Generate sample
    lightSample = (LightSample)0;

    if (squad.S < 0.00001 || isnan(squad.S))
        return false;

    // 1. compute ’cu’
    float au = u * squad.S + squad.k;
    float fu = (cos(au) * squad.b0 - squad.b1) / sin(au);
    float cu = 1 / sqrt(fu * fu + squad.b0sq);// *(fu > 0 ? +1 : -1);
    cu = (fu > 0.0f) ? cu : -cu;
    cu = clamp(cu, -1, 1); // avoid NaNs

    // 2. compute ’xu’
    float xu = -(cu * squad.z0) / sqrt(1 - cu * cu);
    xu = clamp(xu, squad.x0, squad.x1); // avoid Infs

    // 3. compute ’yv’
    float d = sqrt(xu * xu + squad.z0sq);
    float h0 = squad.y0 / sqrt(d * d + squad.y0sq);
    float h1 = squad.y1 / sqrt(d * d + squad.y1sq);
    float hv = h0 + v * (h1 - h0);
    float hv2 = hv * hv;
    float eps = 0.0001;
    float yv = (hv2 < 1.0 - eps) ? (hv * d) / sqrt(1.0 - hv2) : squad.y1;

    // 4. transform (xu,yv,z0) to world coords
    float3 position = (squad.o + xu * squad.x + yv * squad.y + squad.z0 * squad.z);

    lightSample.lightVector = position - P;
    lightSample.distanceToLight = length(lightSample.lightVector);
    lightSample.L = lightSample.lightVector * rcp(lightSample.distanceToLight);
    lightSample.weight = squad.S;

    // Cookie texture coordinates
    lightSample.uv = float2((xu - squad.x0) / (squad.x1 - squad.x0) - 0.5, (yv - squad.y0) / (squad.y1 - squad.y0) - 0.5);
    lightSample.uv.y = 1.0 - lightSample.uv.y;
    lightSample.materialIndex = -1;

    float cosTheta = -dot(lightSample.L, light.forward);
    if (cosTheta < eps)
        return false;

    return true;
#endif
}

bool SampleDiscLight(inout PathTracingSampler rngState, uint dimsOffset, out uint dimsUsed, float3 P, PTLight light, inout LightSample lightSample)
{
    float u = rngState.GetFloatSample(dimsOffset);
    float v = rngState.GetFloatSample(dimsOffset + 1);
    dimsUsed = 2;

    float2 coord = SampleDiskUniform(u, v);
    const float radius = light.width;
    float3 posOnDisk = radius * (coord.x * light.right + coord.y * light.up);
    float3 position = light.position + posOnDisk;

    lightSample.lightVector = position - P;
    lightSample.distanceToLight = length(lightSample.lightVector);
    lightSample.L = lightSample.lightVector * rcp(lightSample.distanceToLight);
    lightSample.weight = 0;

    // Cookie texture coordinates
    float lightDiameter = 2 * light.width;
    float centerU = dot(posOnDisk, light.right) / (lightDiameter * Length2(light.right));
    float centerV = dot(posOnDisk, light.up) / (lightDiameter * Length2(light.up));
    lightSample.uv = float2(centerU - 0.5, centerV - 0.5);
    lightSample.uv.x = 1.0 - lightSample.uv.x;
    lightSample.materialIndex = -1;

    float cosTheta = -dot(lightSample.L, light.forward);
    if (cosTheta < 0.001)
        return false;

    float d = lightSample.distanceToLight;
    float lightArea = PI * light.width * light.width;
    lightSample.weight = lightArea * cosTheta / (d * d);
    return true;
}

bool SampleEmissiveMesh(UnifiedRT::RayTracingAccelStruct accelStruct, inout PathTracingSampler rngState, uint dimsOffset, out uint dimsUsed, float3 P, PTLight light, inout LightSample lightSample)
{
    float r1 = rngState.GetFloatSample(dimsOffset);
    float r2 = rngState.GetFloatSample(dimsOffset + 1);
    float r3 = rngState.GetFloatSample(dimsOffset + 2);
    dimsUsed = 3;

    // random barycentrics
    float r1_sqrt = sqrt(r1);
    float bary0 = 1 - r1_sqrt;
    float bary1 = (1 - r2) * r1_sqrt;
    float bary2 = r2 * r1_sqrt;

    int instanceIndex = light.height;
    int numPrimitives = light.attenuation.x;
    int primitiveIndex = (r3 * numPrimitives) % numPrimitives;

    // fetch the triangle vertices from the geometry pool

    int geometryIndex = g_AccelStructInstanceList[instanceIndex].geometryIndex;
    GeoPoolMeshChunk meshInfo = g_MeshList[geometryIndex];
    uint3 triangleVertexIndices = UnifiedRT::Internal::FetchTriangleIndices(meshInfo, primitiveIndex);

    GeoPoolVertex v1, v2, v3;
    v1 = UnifiedRT::Internal::FetchVertex(meshInfo, triangleVertexIndices.x);
    v2 = UnifiedRT::Internal::FetchVertex(meshInfo, triangleVertexIndices.y);
    v3 = UnifiedRT::Internal::FetchVertex(meshInfo, triangleVertexIndices.z);

    UnifiedRT::InstanceData instanceInfo = UnifiedRT::GetInstance(instanceIndex);
    v1.pos = mul(instanceInfo.localToWorld, float4(v1.pos, 1)).xyz;
    v2.pos = mul(instanceInfo.localToWorld, float4(v2.pos, 1)).xyz;
    v3.pos = mul(instanceInfo.localToWorld, float4(v3.pos, 1)).xyz;

    float3 geometricNormal = cross(v2.pos - v1.pos, v3.pos - v1.pos);

    // compute the random position
    float3 position = bary0 * v1.pos + bary1 * v2.pos + bary2 * v3.pos;

    lightSample.lightVector = position - P;
    lightSample.distanceToLight = length(lightSample.lightVector);
    lightSample.L = lightSample.lightVector * rcp(lightSample.distanceToLight);
    lightSample.distanceToLight *= 0.99;    // avoid self intersection with mesh light geometry
    lightSample.weight = 0;

    // Interpolate the texture coordinates if needed
    int materialIndex = instanceInfo.userMaterialID;
    lightSample.materialIndex = materialIndex;
    PTMaterial matInfo = g_MaterialList[materialIndex];

    if (matInfo.emissionTextureIndex != -1)
    {
        if (matInfo.albedoAndEmissionUVChannel == 1)
            lightSample.uv = bary0 * v1.uv1.xy + bary1 * v2.uv1.xy + bary2 * v3.uv1.xy;
        else
            lightSample.uv = bary0 * v1.uv0.xy + bary1 * v2.uv0.xy + bary2 * v3.uv0.xy;
    }
    else
        lightSample.uv = 0;

    float cosTheta = -dot(lightSample.L, normalize(geometricNormal));

    if (cosTheta < 0.001)
        return false;

    float d = lightSample.distanceToLight;

    float lightArea = 0.5 * length(geometricNormal);

    lightSample.weight = lightArea * cosTheta / (d * d);
    lightSample.weight *= numPrimitives; //uniform light selection pdf

    return true;
}

float2 PunctualLightCookieUVs(float3 L, PTLight light)
{
    // Handles spot, pyramid, box and directional lights.
    light.right *= 2.0f / light.width;
    light.up *= 2.0f / light.height;

    float3x3 lightToWorld = float3x3(light.right, light.up, light.forward);
    float3 positionLS = mul(-L, transpose(lightToWorld));

    float perspectiveZ = (light.type != BOX_LIGHT && light.type != DIRECTIONAL_LIGHT) ? positionLS.z : 1.0f;
    float2 positionCS = positionLS.xy / perspectiveZ;
    float2 positionNDC = positionCS * 0.5f + 0.5f;
    return positionNDC;
}

bool SamplePunctualLight(float3 P, PTLight light, out LightSample lightSample)
{
    lightSample.lightVector = light.type == DIRECTIONAL_LIGHT ? -light.forward : light.position - P;
    lightSample.distanceToLight = light.type == DIRECTIONAL_LIGHT ? K_T_MAX : length(lightSample.lightVector);
    lightSample.L = normalize(lightSample.lightVector);
    lightSample.weight = 1.0f;
    lightSample.materialIndex = -1;
    
    // Light cookie UVs (note: for pointlights we use the light vector to access the cookie)
    lightSample.uv = 0.0f;
    if (light.cookieIndex >= 0 && light.type != POINT_LIGHT)
    {
        lightSample.uv = PunctualLightCookieUVs(light.position - P, light);
    }

    return true;
}

bool SampleEnvironmentLight(inout PathTracingSampler rngState, uint dimsOffset, out uint dimsUsed, float3 P, PTLight light, out LightSample lightSample)
{

    lightSample = (LightSample)0;
    float r1 = rngState.GetFloatSample(dimsOffset);
    float r2 = rngState.GetFloatSample(dimsOffset+1);
    dimsUsed = 2;

#ifdef UNIFORM_ENVSAMPLING
    // Sample the environment with a random direction. Should only be used for reference / ground truth.
    lightSample.lightVector = SampleSphereUniform(r1, r2);
    lightSample.weight = 4 * PI;
#else
    float normalizationFactor = GetSkyPDFNormalizationFactor();

    // A normalization factor of zero means that the environment cubemap was zero, and in this case
    // its PDF isn't well-defined. It is safe to bail in this case, because the environment wouldn't
    // contribute anything anyway.
    if (normalizationFactor == 0.0f)
        return false;

    // Sample the environment CDF
    float2 u = SampleSky(r1, r2);
    lightSample.lightVector = MapUVToSkyDirection(u);

    float3 envValue = g_EnvTex.SampleLevel(sampler_g_EnvTex, lightSample.lightVector, 0).xyz;
    lightSample.weight = rcp(GetSkyPDFFromValue(envValue, normalizationFactor));
#endif

    lightSample.L = normalize(lightSample.lightVector);
    lightSample.distanceToLight = K_T_MAX;
    lightSample.uv = 0;
    lightSample.materialIndex = -1;

    return true;
}

bool SampleLightShape(UnifiedRT::RayTracingAccelStruct accelStruct, inout PathTracingSampler rngState, uint dimsOffset, out uint dimsUsed, float3 P, PTLight light, out LightSample lightSample)
{
    if (light.type == RECTANGULAR_LIGHT)
    {
        return SampleRectangularLight(rngState, dimsOffset, dimsUsed, P, light, lightSample);
    }
    else if (light.type == DISC_LIGHT)
    {
        return SampleDiscLight(rngState, dimsOffset, dimsUsed, P, light, lightSample);
    }
    else if (light.type == EMISSIVE_MESH)
    {
        return SampleEmissiveMesh(accelStruct, rngState, dimsOffset, dimsUsed, P, light, lightSample);
    }
    else if (light.type == ENVIRONMENT_LIGHT)
    {
        return SampleEnvironmentLight(rngState, dimsOffset, dimsUsed, P, light, lightSample);
    }
    else
    {
        dimsUsed = 0;
        return SamplePunctualLight(P, light, lightSample);
    }

    dimsUsed = 0;
    lightSample = (LightSample)0;
    return false;
}

// Find the smallest `i` between 0 and `cdfLength-1` such that `cdf[i] < rand < cdf[i+1]`.
// `cdf` must be normalized and `rand` must be between 0 and 1.
uint BinarySearchCdf(in StructuredBuffer<float> cdf, uint cdfLength, float rand)
{
    const uint maxSteps = log2((float)cdfLength) + 1;

    uint leftIdx = 0;
    uint rightIdx = cdfLength - 1;
    for (uint i = 0; i < maxSteps; ++i)
    {
        const uint candidateIdx  = (leftIdx + rightIdx) / 2;
        const float higherVal = cdf[candidateIdx];
        if (higherVal < rand)
        {
            leftIdx = candidateIdx + 1;
            continue;
        }

        const float lowerVal = candidateIdx == 0 ? 0.0f : cdf[candidateIdx - 1];
        if (rand < lowerVal)
        {
            rightIdx = candidateIdx - 1;
            continue;
        }

        return candidateIdx;
    }

    return UINT_MAX; // Unexpected.
}

void PickLightUniformly(float sample, out uint lightIndex, out float probabilityMass)
{
    lightIndex = g_NumLights * sample;
    // Guard in case the RNG returns 1 or higher
    lightIndex = lightIndex % g_NumLights;
    probabilityMass = 1.0f / g_NumLights;
}

void PickLightFromCDF(float sample, out uint lightIndex, out float probabilityMass)
{
    lightIndex = BinarySearchCdf(g_LightCdf, g_NumLights, sample);
    probabilityMass = lightIndex == 0 ? g_LightCdf[0] : g_LightCdf[lightIndex] - g_LightCdf[lightIndex - 1];
}

uint FindEmissiveMeshLightIndex(int instanceID)
{
    for (uint i = 0; i < g_NumEmissiveMeshes; i++)
        if (GetMeshLightInstanceID(g_LightList[i]) == instanceID)
            return i;
    return UINT_MAX; // Unexpected.
}

float SampleFalloff(int falloffIndex, float distance, float range)
{
    const float LUTRange = g_LightFalloffLUTRange[falloffIndex];
    if (distance > LUTRange)
        return 0.0f; // The distance is outside the range of the falloff LUT.
    const float normalizedSamplePosition = distance / LUTRange;
    const int sampleCount = g_LightFalloffLUTLength;
    const int LUTOffset = falloffIndex * sampleCount;
    const float index = normalizedSamplePosition * float(sampleCount);

    // compute the index pair
    const int loIndex = min(int(index), int(sampleCount - 1));
    const int hiIndex = min(int(index) + 1, int(sampleCount - 1));
    const float hiFraction = (index - float(loIndex));

    const float sampleLo = g_LightFalloff[LUTOffset+loIndex];
    const float sampleHi = g_LightFalloff[LUTOffset+hiIndex];

    // do the lerp
    return (1.0f - hiFraction) * sampleLo + hiFraction * sampleHi;
}

// TODO: Add support for angular falloff LUT, that is used in progressive lightmapper. https://jira.unity3d.com/browse/LIGHT-1770
// distances = {d, d^2, 1/d, d_proj}, where d_proj = dot(lightToSample, lightData.forward).
float PunctualLightAngleAttenuation(float4 distances, float rangeAttenuationScale, float rangeAttenuationBias, float lightAngleScale, float lightAngleOffset)
{
    float distRcp = distances.z;
    float distProj = distances.w;
    float cosFwd = distProj * distRcp;
    float attenuation = AngleAttenuation(cosFwd, lightAngleScale, lightAngleOffset);
    return Sq(attenuation);
}

float GetPunctualAttenuation(PTLight light, LightSample lightSample)
{
    float x = abs(dot(lightSample.lightVector, 2 * light.right / light.width));
    float y = abs(dot(lightSample.lightVector, 2 * light.up / light.height));
    float z = abs(dot(lightSample.lightVector, light.forward));
    
    if (light.type == BOX_LIGHT)
    {
        // box attenuation
        return ((z > 0) && (x < 1.0 && y < 1.0)) ? 1.0 : 0.0;
    }
    else if (light.type == PYRAMID_LIGHT)
    {
        // pyramid attenuation
        if (!((z > 0) && (x < z && y < z)))
        {
            return 0;
        }
    }
    
    // Punctual attenuation
    float attenuation = 1.0f;
    const float d = lightSample.distanceToLight;
    float4 distances = float4(d, Sq(d), rcp(d), -d * dot(lightSample.L, light.forward));

#ifdef USE_DISTANCE_ATTENUATION_LUT
    if (light.falloffIndex >= 0)
        attenuation *= SampleFalloff(light.falloffIndex, d, light.range);

    attenuation *= PunctualLightAngleAttenuation(distances, light.attenuation.x, light.attenuation.y, light.attenuation.z, light.attenuation.w);
#else
    attenuation *= PunctualLightAttenuation(distances, light.attenuation.x, light.attenuation.y, light.attenuation.z, light.attenuation.w);
#endif

    return attenuation;
}

float3 PointLightCookie(PTLight light, LightSample lightSample)
{
    if (light.cookieIndex >= 0)
    {
        float3x3 lightToWorld = float3x3(light.right, light.up, light.forward);
        float3 L = mul(-lightSample.L, transpose(lightToWorld));
        return g_CubemapAtlas.SampleLevel(sampler_g_CubemapAtlas, float4(L, light.cookieIndex), 0).xyz;
    }
    else
        return 1.0f;
}

float3 LightCookie(PTLight light, LightSample lightSample)
{
    if (light.cookieIndex >= 0)
    {
        return g_CookieAtlas.SampleLevel(sampler_g_CookieAtlas, float3(lightSample.uv, light.cookieIndex), 0).xyz;
    }
    else
        return 1.0f;
}

float3 AreaCookieAttenuation(PTLight light, LightSample lightSample)
{
    if (light.cookieIndex >= 0)
    {
        return g_CookieAtlas.SampleLevel(sampler_g_CookieAtlas, float3(lightSample.uv, light.cookieIndex), 0).xyz;
    }
    else
        return 1.0f;
}

float3 PunctualCookieAttenuation(PTLight light, LightSample lightSample)
{
    if (light.type == SPOT_LIGHT || light.type == BOX_LIGHT || light.type == PYRAMID_LIGHT)
        return LightCookie(light, lightSample);
    else if (light.type == POINT_LIGHT)
        return PointLightCookie(light, lightSample);
    else
        return 1.0;
}

float3 GetPunctualEmission(PTLight light, LightSample lightSample)
{
    float3 emission = light.intensity * GetPunctualAttenuation(light, lightSample);
    float3 cookieAttenuation = PunctualCookieAttenuation(light, lightSample);
    return emission * cookieAttenuation;
}

float3 GetDirectionalEmission(PTLight light, LightSample lightSample)
{
    return light.intensity * LightCookie(light, lightSample);
}

float3 GetRectangularLightEmission(PTLight light, LightSample lightSample)
{
    if (light.range < lightSample.distanceToLight)
        return 0.f;
    float3 emission = light.intensity;

    float3 cookieAttenuation = AreaCookieAttenuation(light, lightSample);
    return emission * cookieAttenuation;
}

float3 GetDiscLightEmission(PTLight light, LightSample lightSample)
{
    if (light.range < lightSample.distanceToLight)
        return 0.f;
    float3 emission = light.intensity;

    float3 cookieAttenuation = AreaCookieAttenuation(light, lightSample);
    return emission * cookieAttenuation;
}

float3 GetEmissiveMeshEmission(PTLight light, LightSample lightSample)
{
    PTMaterial matInfo = g_MaterialList[lightSample.materialIndex];

    float3 emission = 0;
    if (matInfo.emissionTextureIndex != -1)
    {
        emission = g_EmissionTextures.SampleLevel(sampler_g_EmissionTextures, float3(lightSample.uv, matInfo.emissionTextureIndex), 0).rgb;
    }
    else
    {
        emission = matInfo.emissionColor;
    }

    return emission;
}

float3 GetEnvironmentLightEmission(PTLight light, LightSample lightSample)
{
    // Sample the environment and apply intensity multiplier
    return g_EnvIntensityMultiplier * g_EnvTex.SampleLevel(sampler_g_EnvTex, lightSample.L, 0).xyz;
}

float3 GetEmission(PTLight light, LightSample lightSample)
{
    float3 emission = 0;
    if (light.type == DIRECTIONAL_LIGHT)
    {
        emission = GetDirectionalEmission(light, lightSample);
    }
    else if (light.type == SPOT_LIGHT || light.type == POINT_LIGHT || light.type == BOX_LIGHT || light.type == PYRAMID_LIGHT)
    {
        emission = GetPunctualEmission(light, lightSample);
    }
    else if (light.type == RECTANGULAR_LIGHT)
    {
        emission = GetRectangularLightEmission(light, lightSample);
    }
    else if (light.type == DISC_LIGHT)
    {
        emission = GetDiscLightEmission(light, lightSample);
    }
    else if (light.type == EMISSIVE_MESH)
    {
        emission = GetEmissiveMeshEmission(light, lightSample);
    }
    else if (light.type == ENVIRONMENT_LIGHT)
    {
        emission = GetEnvironmentLightEmission(light, lightSample);
    }
    return emission;
}

float ComputeNextEventEstimationMISWeight(int lightType, float3 L, float lightPdf, uint lightSampleCount, float3 worldNormal)
{
    if (lightType == EMISSIVE_MESH || lightType == ENVIRONMENT_LIGHT)
    {
        float cosTheta = dot(worldNormal, L);
        float brdfPdf = cosTheta / PI;
#if (EMISSIVE_SAMPLING == LIGHT_SAMPLING)
        float misWeight = 1;
#elif (EMISSIVE_SAMPLING == BRDF_SAMPLING)
        float misWeight = 0;
#else
        float misWeight = PowerHeuristic(lightPdf * lightSampleCount, brdfPdf);
#endif
        return misWeight;
    }
    return 1.0;
}

bool CullLight(PTHitGeom hitGeom, LightSample lightSample, float3 emmission)
{
    // Check if the hitpoint is facing the selected light sample
    if (dot(hitGeom.worldNormal, lightSample.L) < 0)
    {
        return true;
    }

    // skip if light Emission is zero
    if (Luminance(emmission) == 0)
    {
        return true;
    }

    return false;
}

PTLight FetchLight(int lightIndex)
{
    return g_LightList[lightIndex];
}

uint PackLightInfo(PTLight light)
{
    uint bitmask = 0;
    bitmask |= light.castsShadows;
    bitmask |= (light.type << 1);
    return bitmask;
}

void UnpackLightInfo(uint info, out int castsShadows, out int lightType)
{
    castsShadows = info & 1;
    lightType = info >> 1;
}

struct Reservoir
{
    float   totalWeights;
    uint    selectedLightInfo;          // packs the "cast shadows" flag and the light type
    float3  L;
    float   distanceToLight;            // We could remove this, but handling directional lights would require a special case.
    float   selectedCandidateSourcePDF; // for MIS
    float3  sampleEmission;             // We are caching this, to avoid re-evaluating the lightsample. We could drop it if we want even thinner reservoirs 

    void update(uint lightInfo, LightSample lightSample, float3 emission, float r)
    {
        float candidateTargetDensity = Luminance(emission);
        float candidateRISWeight = candidateTargetDensity * lightSample.weight;
        totalWeights += candidateRISWeight;

        if (r * totalWeights < candidateRISWeight)
        {
            selectedLightInfo = lightInfo;
            L = lightSample.L;
            distanceToLight = lightSample.distanceToLight;
            selectedCandidateSourcePDF = rcp(lightSample.weight);
            sampleEmission = emission;
        }
    }
};

float3 EvalDirectIllumination(
    UnifiedRT::DispatchInfo dispatchInfo, UnifiedRT::RayTracingAccelStruct accelStruct,
    bool isDirect, uint shadowRayMask, PTHitGeom hitGeom, MaterialProperties material, uint maxLightEvaluations, inout PathTracingSampler rngState)
{
    float3 shadowRayOrigin = hitGeom.NextRayOrigin();

    // Find how many lights we will evaluate
    uint iterationCount = min(maxLightEvaluations, g_NumLights);

#ifdef RESAMPLED_IMPORTANCE_SAMPLING
    Reservoir reservoir = (Reservoir) 0;
#endif
    float3 sampleSum = 0.0f;

    for (uint i = 0; i < iterationCount; ++i)
    {
        uint lightIndex;
        float lightProbabilityMass;

        float pickingSample = rngState.GetFloatSample(RAND_DIM_LIGHT_SELECTION + RAND_SAMPLES_PER_LIGHT * i);

#ifdef STRATIFIED_LIGHT_PICKING
        pickingSample = (i + pickingSample) / iterationCount;
#endif

        if (g_LightPickingMethod == LIGHT_PICKING_METHOD_POWER)
            PickLightFromCDF(pickingSample, lightIndex, lightProbabilityMass);
        else
            PickLightUniformly(pickingSample, lightIndex, lightProbabilityMass);

        PTLight light = FetchLight(lightIndex);

        LightSample lightSample;
        uint dimsOffset = RAND_DIM_LIGHT_SELECTION + RAND_SAMPLES_PER_LIGHT * i + 1;
        uint dimsUsed = 0;
        if (!SampleLightShape(accelStruct, rngState, dimsOffset, dimsUsed, hitGeom.worldPosition, light, lightSample))
            continue;

        float lightSelectionWeight = 1.0f / lightProbabilityMass;
        lightSample.weight *= lightSelectionWeight;

        // skip light if it contributes only to indirect illumination
        if (isDirect && !light.contributesToDirectLighting)
            continue;

        // skip lights based on light layer
        if ((light.layerMask & hitGeom.renderingLayerMask) == 0)
            continue;

        float3 emission = GetEmission(light, lightSample);

        if (!isDirect)
            emission *= light.indirectScale * g_IndirectScale;

        float3 radiance = emission * lightSample.weight;
        // skip lights that we can easilly cull before casting a shadow ray
        if (CullLight(hitGeom, lightSample, radiance))
            continue;

#ifdef RESAMPLED_IMPORTANCE_SAMPLING
        float r = rngState.GetFloatSample(RAND_DIM_LIGHT_SELECTION + RAND_SAMPLES_PER_LIGHT * i + 4);
        uint lightInfo = PackLightInfo(light);
        reservoir.update(lightInfo, lightSample, emission, r);
#else
        // cast a shadow ray
        float3 attenuation = 1.0f;
        if (light.castsShadows && !CastShadowRay(dispatchInfo, accelStruct, shadowRayOrigin, lightSample.L, lightSample.distanceToLight, shadowRayMask, attenuation))
            continue;

        float3 eval = EvalDiffuseBrdf(material.baseColor) * ClampedCosine(hitGeom.worldNormal, lightSample.L) * radiance * attenuation / iterationCount;
        eval *= ComputeNextEventEstimationMISWeight(light.type, lightSample.L, 1.0 / lightSample.weight, iterationCount, hitGeom.worldNormal);
        sampleSum += ClampRadiance(eval);
#endif
    }

#ifdef RESAMPLED_IMPORTANCE_SAMPLING
    float sampleTargetDensity = Luminance(reservoir.sampleEmission);
    if (reservoir.totalWeights > 0.0f && sampleTargetDensity > 0.0f)
    {
        // cast a shadow ray
        float3 attenuation = 1.0f;

        int castShadows;
        int lightType;
        UnpackLightInfo(reservoir.selectedLightInfo, castShadows, lightType);

        float distanceToLight = reservoir.distanceToLight;
        float3 L = reservoir.L;
        
        bool isShadowed = castShadows && !CastShadowRay(dispatchInfo, accelStruct, shadowRayOrigin, L, distanceToLight, shadowRayMask, attenuation);
        if (!isShadowed)
        {
            float risSampleWeight = (reservoir.totalWeights / float(iterationCount)) / sampleTargetDensity;
            float3 eval = EvalDiffuseBrdf(material.baseColor) * ClampedCosine(hitGeom.worldNormal, L) * reservoir.sampleEmission * attenuation * risSampleWeight;
            eval *= ComputeNextEventEstimationMISWeight(lightType, L, reservoir.selectedCandidateSourcePDF, iterationCount, hitGeom.worldNormal);
            sampleSum = ClampRadiance(eval);
        }
    }
#endif
    return sampleSum;
}

// Evaluate direct illumination from a single light given the index of the light. Used for baking.
void SampleDirectRadiance(
    UnifiedRT::DispatchInfo dispatchInfo,
    UnifiedRT::RayTracingAccelStruct accelStruct,
    uint shadowRayMask,
    float3 worldPosition,
    inout PathTracingSampler rngState,
    uint dimsOffset,
    out uint dimsUsed,
    PTLight light,
    bool receiveShadows,
    out float3 lightRadiance,
    out float3 lightDirection)
{
    lightRadiance = lightDirection = float3(0.0, 0.0, 0.0);

    LightSample lightSample;
    if (!SampleLightShape(accelStruct, rngState, dimsOffset, dimsUsed, worldPosition, light, lightSample))
        return;
    // always start at an even dimension
    if (dimsUsed % 2 == 1)
        dimsUsed++;
    dimsOffset += dimsUsed;

    const float3 emission = GetEmission(light, lightSample);
    const float3 radiance = emission * lightSample.weight;

    float3 attenuation = 1.0f;
    if (light.castsShadows && receiveShadows)
    {
        // cast a shadow ray
        if (light.shadowRadius > 0.0f)
        {
            const float r1 = rngState.GetFloatSample(dimsOffset);
            const float r2 = rngState.GetFloatSample(dimsOffset+1);
            dimsUsed += 2;
            if (!CastJitteredShadowRay(dispatchInfo, accelStruct, float2(r1, r2), light.shadowRadius, lightSample.lightVector, worldPosition, lightSample.L, lightSample.distanceToLight, shadowRayMask, attenuation))
                return;
        }
        else if (!CastShadowRay(dispatchInfo, accelStruct, worldPosition, lightSample.L, lightSample.distanceToLight, shadowRayMask, attenuation))
            return;
    }

    lightRadiance = radiance * attenuation;
    lightDirection = lightSample.L;
}

// Evaluate direct shadows from a single light given the index of the light. Used for baking.
bool IsLightVisibleFromPoint(
    UnifiedRT::DispatchInfo dispatchInfo,
    UnifiedRT::RayTracingAccelStruct accelStruct,
    uint shadowRayMask,
    float3 worldPosition,
    inout PathTracingSampler rngState,
    uint dimsOffset,
    out uint dimsUsed,
    PTLight light,
    bool receiveShadows,
    out float3 attenuation)
{
    attenuation = 1.0f;

    LightSample lightSample;
    if (!SampleLightShape(accelStruct, rngState, dimsOffset, dimsUsed, worldPosition, light, lightSample))
        return false;

    if (light.type != DIRECTIONAL_LIGHT && lightSample.distanceToLight >= light.range)
        return false;
    if (light.type == SPOT_LIGHT && GetPunctualAttenuation(light, lightSample) < 0.000001f)
        return false;

    if (light.castsShadows && receiveShadows)
    {
        // cast a shadow ray
        if (light.shadowRadius > 0.0f)
        {
            const float r1 = rngState.GetFloatSample(dimsOffset);
            const float r2 = rngState.GetFloatSample(dimsOffset + 1);
            dimsUsed += 2;
            return CastJitteredShadowRay(dispatchInfo, accelStruct, float2(r1, r2), light.shadowRadius, lightSample.lightVector, worldPosition, lightSample.L, lightSample.distanceToLight, shadowRayMask, attenuation);
        }
        else
            return CastShadowRay(dispatchInfo, accelStruct, worldPosition, lightSample.L, lightSample.distanceToLight, shadowRayMask, attenuation);
    }
    return true;
}
#endif
