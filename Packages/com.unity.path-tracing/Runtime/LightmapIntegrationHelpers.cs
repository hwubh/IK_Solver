using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using Unity.Mathematics;
using UnityEngine.Assertions;
using UnityEngine.Rendering;
using UnityEngine.Rendering.UnifiedRayTracing;

namespace UnityEngine.PathTracing.Lightmapping
{
    internal static class SuperSamplingHelpers
    {
        internal static class ShaderIDs
        {
            public static readonly int StochasticAntialiasing = Shader.PropertyToID("g_StochasticAntialiasing");
            public static readonly int ExplicitSampleOffsetX = Shader.PropertyToID("g_ExplicitSampleOffsetX");
            public static readonly int ExplicitSampleOffsetY = Shader.PropertyToID("g_ExplicitSampleOffsetY");

        }

        static internal float2 GetSuperSamplingOffset(uint currentSuperSampleIndex, uint superSampleWidth)
        {
            float2 superSamplingOffset;
            uint sqResolution = superSampleWidth * superSampleWidth;
            Debug.Assert(currentSuperSampleIndex < sqResolution);
            uint ssX = currentSuperSampleIndex % superSampleWidth;
            uint ssY = currentSuperSampleIndex / superSampleWidth;
            superSamplingOffset = (new float2(ssX, ssY) + 0.5f) / (float)superSampleWidth;
            return superSamplingOffset;
        }


        static internal void BindSuperSamplingParameters(CommandBuffer cmd, Rendering.UnifiedRayTracing.IRayTracingShader gBufferShader, float2 superSamplingParameters, bool stochasticAntialiasing)
        {
            Debug.Assert(superSamplingParameters.x >= 0.0f);
            Debug.Assert(superSamplingParameters.y >= 0.0f);
            Debug.Assert(superSamplingParameters.x <= 1.0f);
            Debug.Assert(superSamplingParameters.y <= 1.0f);
            gBufferShader.SetIntParam(cmd, ShaderIDs.StochasticAntialiasing, stochasticAntialiasing ? 1 : 0);
            gBufferShader.SetFloatParam(cmd, ShaderIDs.ExplicitSampleOffsetX, superSamplingParameters.x);
            gBufferShader.SetFloatParam(cmd, ShaderIDs.ExplicitSampleOffsetY, superSamplingParameters.y);
        }
    }

    internal static class LightmapIntegrationHelpers
    {
        internal class ComputeHelpers
        {
            internal ComputeShader ComputeHelperShader = null;

            internal static class ShaderIDs
            {
                public static readonly int TextureInOut = Shader.PropertyToID("g_TextureInOut");
                public static readonly int SampleCountInW = Shader.PropertyToID("g_SampleCountInW");
                public static readonly int VarianceInR = Shader.PropertyToID("g_VarianceInR");
                public static readonly int StandardErrorInR = Shader.PropertyToID("g_StandardErrorInR");
                public static readonly int MeanInR = Shader.PropertyToID("g_MeanInR");
                public static readonly int SourceTexture = Shader.PropertyToID("g_SourceTexture");
                public static readonly int OutputBuffer = Shader.PropertyToID("g_OutputBuffer");
                public static readonly int X = Shader.PropertyToID("g_X");
                public static readonly int Y = Shader.PropertyToID("g_Y");
                public static readonly int TextureOut = Shader.PropertyToID("g_TextureOut");
                public static readonly int TextureWidth = Shader.PropertyToID("g_TextureWidth");
                public static readonly int TextureHeight = Shader.PropertyToID("g_TextureHeight");
                public static readonly int MultiplicationFactor = Shader.PropertyToID("g_MultiplicationFactor");
                public static readonly int BoxFilterRadius = Shader.PropertyToID("g_BoxFilterRadius");
                public static readonly int StandardErrorThreshold = Shader.PropertyToID("g_StandardErrorThreshold");
                public static readonly int ChannelIndex = Shader.PropertyToID("g_ChannelIndex");
                public static readonly int ChannelValue = Shader.PropertyToID("g_ChannelValue");
                public static readonly int TemporaryRenderTarget = Shader.PropertyToID("g_TemporaryRenderTarget");
                public static readonly int SecondTemporaryRenderTarget = Shader.PropertyToID("g_SecondTemporaryRenderTarget");
                public static readonly int MultiplyTemporaryRenderTarget = Shader.PropertyToID("g_MultiplyTemporaryRenderTarget");
            }

            internal static int MultiplyKernel;
            internal static int BroadcastChannelKernel;
            internal static int SetChannelKernel;
            internal static int ReferenceBoxFilterKernel;
            internal static int ReferenceBoxFilterBlueChannelKernel;
            internal static int StandardErrorKernel;
            internal static int StandardErrorThresholdKernel;
            internal static int GetValueKernel;
            internal static int NormalizeByAlphaKernel;

            public void Load()
            {
#if UNITY_EDITOR
                const string packageFolder = "Packages/com.unity.path-tracing/";
                ComputeHelperShader = UnityEditor.AssetDatabase.LoadAssetAtPath<ComputeShader>(packageFolder + "Runtime/Shaders/ComputeHelpers.compute");
#endif
                if (ComputeHelperShader != null)
                {
                    MultiplyKernel = ComputeHelperShader.FindKernel("Multiply");
                    BroadcastChannelKernel = ComputeHelperShader.FindKernel("BroadcastChannel");
                    SetChannelKernel = ComputeHelperShader.FindKernel("SetChannel");
                    ReferenceBoxFilterKernel = ComputeHelperShader.FindKernel("ReferenceBoxFilter");
                    ReferenceBoxFilterBlueChannelKernel = ComputeHelperShader.FindKernel("ReferenceBoxFilterBlueChannel");
                    StandardErrorKernel = ComputeHelperShader.FindKernel("StandardError");
                    StandardErrorThresholdKernel = ComputeHelperShader.FindKernel("StandardErrorThreshold");
                    GetValueKernel = ComputeHelperShader.FindKernel("GetValue");
                    NormalizeByAlphaKernel = ComputeHelperShader.FindKernel("NormalizeByAlpha");
                }
            }
        }

        public class GPUSync : IDisposable
        {
            private RenderTexture _syncTexture;
            private Texture2D _readableTex;

            public void Create()
            {
                Debug.Assert(_syncTexture == null);
                _syncTexture = new RenderTexture(1, 1, 0, RenderTextureFormat.ARGB32)
                {
                    name = "_syncTexture",
                    enableRandomWrite = true,
                    hideFlags = HideFlags.HideAndDontSave
                };
                _syncTexture.Create();
                _readableTex = new Texture2D(_syncTexture.width, _syncTexture.height, TextureFormat.ARGB32, false, true)
                {
                    name = "_readableTex",
                    hideFlags = HideFlags.HideAndDontSave
                };
            }

            public void Sync(CommandBuffer cmd)
            {
                Debug.Assert(_syncTexture != null, "Create must be called before Sync.");
                Debug.Assert(_readableTex != null, "Create must be called before Sync.");
                cmd.SetRenderTarget(_syncTexture);
                cmd.ClearRenderTarget(false, true, new Color32(1, 2, 3, 4));
                cmd.RequestAsyncReadback(_syncTexture, delegate { });
                cmd.WaitAllAsyncReadbackRequests();
                GraphicsHelpers.Flush(cmd);
            }

            public void RequestAsyncReadback(CommandBuffer cmd, Action<AsyncGPUReadbackRequest> callback) =>
                cmd.RequestAsyncReadback(_syncTexture, callback);

            public void Dispose()
            {
                if (_syncTexture != null)
                {
                    _syncTexture.Release();
                    CoreUtils.Destroy(_syncTexture);
                    _syncTexture = null;
                }

                if (_readableTex == null)
                    return;

                CoreUtils.Destroy(_readableTex);
                _readableTex = null;
            }
        }

        private static string IntegratedOutputTypeToComponentName(IntegratedOutputType integratedOutputType)
        {
            switch (integratedOutputType)
            {
                case IntegratedOutputType.Direct: return "irradianceDirect";
                case IntegratedOutputType.Indirect: return "irradianceIndirect";
                case IntegratedOutputType.AO: return "ambientOcclusion";
                case IntegratedOutputType.Validity: return "validity";
                case IntegratedOutputType.DirectionalityDirect: return "directionalityDirect";
                case IntegratedOutputType.DirectionalityIndirect: return "directionalityIndirect";
                case IntegratedOutputType.ShadowMask: return "shadowmask";
                default:
                    {
                        Debug.Assert(false, "Missing lightmap type in LightmapTypeToComponentName");
                        return "Missing lightmap type in LightmapTypeToComponentName";
                    }
            }
        }

        static string BuildLightmapComponentPath(string outputType, int lightmapIndex, string path)
        {
            // Naming convention is path/componentName+LightmapIndex.r2d
            return $"{path}/{outputType}{lightmapIndex}.r2d";
        }

        public static bool WriteLightmap(CommandBuffer cmd, RenderTexture renderTex, string outputType, int lightmapIndex, string path)
        {
            try
            {
                DirectoryInfo directoryInfo = Directory.CreateDirectory(path);
                if (directoryInfo.Exists == false)
                    return false;

                string fullPath = BuildLightmapComponentPath(outputType, lightmapIndex, path);
                LightmapIntegrationHelpers.WriteRenderTexture(cmd, fullPath, renderTex);
                return true;
            }
            catch (Exception e)
            {
                Debug.Assert(false, e.Message);
                return false;
            }
        }

        public static bool WriteLightmap(CommandBuffer cmd, RenderTexture renderTex, IntegratedOutputType integratedOutputType, int lightmapIndex, string path)
        {
            return WriteLightmap(cmd, renderTex, IntegratedOutputTypeToComponentName(integratedOutputType),
                lightmapIndex, path);
        }

        private static void OutputUIntRequestData(string prefix, AsyncGPUReadbackRequest request)
        {
            string output = new("");
            Debug.Assert(!request.hasError);
            if (!request.hasError)
            {
                var src = request.GetData<uint>();
                output = $"{prefix}: ";
                for (int i = 0; i < src.Length; ++i)
                {
                    output += src[i];
                    if (i < src.Length - 1)
                        output += ", ";
                }
            }
            else
                output = "AsyncReadBack failed";
            System.Console.WriteLine(output);
        }

        public static void LogGraphicsBuffer(CommandBuffer cmd, GraphicsBuffer graphicsBuffer, string prefix)
        {
            Debug.Assert(graphicsBuffer.stride == 4);
            using GraphicsBuffer stagingBuffer = new GraphicsBuffer(GraphicsBuffer.Target.Structured | GraphicsBuffer.Target.CopyDestination, graphicsBuffer.count, graphicsBuffer.stride);
            cmd.CopyBuffer(graphicsBuffer, stagingBuffer);
            cmd.RequestAsyncReadback(stagingBuffer, (AsyncGPUReadbackRequest request) => { OutputUIntRequestData(prefix, request); });
            cmd.WaitAllAsyncReadbackRequests();
            Graphics.ExecuteCommandBuffer(cmd);
            cmd.Clear();
        }

        public static bool IsPow2(int value) => (value & (value - 1)) == 0 && value > 0;

        internal static GraphicsBuffer CreateDispatchDimensionBuffer()
        {
            return new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments | GraphicsBuffer.Target.Structured | GraphicsBuffer.Target.CopySource, 3, sizeof(uint));
        }

        private static void ReadbackTexture(Texture2D texture, AsyncGPUReadbackRequest request)
        {
            if (request.hasError)
            {
                Debug.Assert(request.hasError, "GPU readback request error detected.");
                return;
            }
            if (request.done)
            {
                var data = request.GetData<byte>();
                texture.LoadRawTextureData(data);
                return;
            }
            Debug.Assert(false, "GPU readback request not done.");
        }

        internal static byte[] EncodeToR2D(this Texture2D tex) // EncodeToR2DInternal is internal to the Editor, this enables us to call it from here
        {
            Type encoder = typeof(ImageConversion);

            MethodInfo encodeMethod = encoder.GetMethod("EncodeToR2DInternal", BindingFlags.Static | BindingFlags.NonPublic);
            if (encodeMethod == null)
                return Array.Empty<byte>();

            var encodeFunc = (Func<Texture2D, byte[]>)Delegate.CreateDelegate(typeof(Func<Texture2D, byte[]>), encodeMethod);

            return encodeFunc(tex);
        }

        internal static void WriteRenderTexture(CommandBuffer cmd, RenderTargetIdentifier renderTex, TextureFormat textureFormat, int width, int height, string path)
        {
            Texture2D readableTex = new Texture2D(width, height, textureFormat, false) { name = "readableTex (WriteRenderTexture)", hideFlags = HideFlags.HideAndDontSave};
            cmd.CopyTexture(renderTex, readableTex);
            cmd.RequestAsyncReadback(readableTex, 0, (AsyncGPUReadbackRequest request) => { ReadbackTexture(readableTex, request); });
            cmd.WaitAllAsyncReadbackRequests();
            Graphics.ExecuteCommandBuffer(cmd);
            cmd.Clear();

            if (!System.IO.Directory.Exists(System.IO.Path.GetDirectoryName(path)))
                System.IO.Directory.CreateDirectory(System.IO.Path.GetDirectoryName(path));
            System.IO.File.WriteAllBytes(path, readableTex.EncodeToR2D());
            CoreUtils.Destroy(readableTex);
        }

        internal static void WriteRenderTexture(CommandBuffer cmd, string path, RenderTexture renderTex)
        {
            Debug.Assert(renderTex.format == RenderTextureFormat.ARGBFloat);
            WriteRenderTexture(cmd, renderTex, TextureFormat.RGBAFloat, renderTex.width, renderTex.height, path);
        }

        public static double4 GetSum(int width, int height, RenderTexture renderTex)
        {
            RenderTexture previous = RenderTexture.active;
            RenderTexture.active = renderTex;
            Texture2D readableTex = new Texture2D(width, height, TextureFormat.RGBAFloat, false, true) { name = "GetSum.readableTex", hideFlags = HideFlags.HideAndDontSave };
            Assert.IsTrue(readableTex.isReadable);
            readableTex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
            RenderTexture.active = previous;
            Color[] colors = readableTex.GetPixels();
            double4 colorSum = new double4(0.0, 0.0, 0.0, 0.0);
            foreach (var color in colors)
                colorSum += new double4(color.r, color.g, color.b, color.a);
            CoreUtils.Destroy(readableTex);
            return colorSum;
        }

        public static float Luminance(Color color)
        {
            return color.r * 0.2126f + color.g * 0.7152f + color.b * 0.0722f;
        }

        public static Color GetValue(CommandBuffer cmd, ComputeShader computeShader, int getValueKernel, int sampleX, int sampleY, int width, int height, RenderTargetIdentifier renderTargetIdentifier)
        {
            using ComputeBuffer colorBuffer = new(4, sizeof(float));
            cmd.SetComputeBufferParam(computeShader, getValueKernel, ComputeHelpers.ShaderIDs.OutputBuffer, colorBuffer);
            cmd.SetComputeTextureParam(computeShader, getValueKernel, ComputeHelpers.ShaderIDs.SourceTexture, renderTargetIdentifier);
            cmd.SetComputeIntParam(computeShader, ComputeHelpers.ShaderIDs.TextureWidth, width);
            cmd.SetComputeIntParam(computeShader, ComputeHelpers.ShaderIDs.TextureHeight, height);
            cmd.SetComputeIntParam(computeShader, ComputeHelpers.ShaderIDs.X, sampleX);
            cmd.SetComputeIntParam(computeShader, ComputeHelpers.ShaderIDs.Y, sampleY);
            computeShader.GetKernelThreadGroupSizes(getValueKernel, out uint x, out uint y, out _);
            cmd.DispatchCompute(computeShader, getValueKernel, GraphicsHelpers.DivUp(width, x), GraphicsHelpers.DivUp(height, y), 1);
            Graphics.ExecuteCommandBuffer(cmd);
            cmd.Clear();
            float[] tempColorArray = new float[4];
            colorBuffer.GetData(tempColorArray);
            colorBuffer.Release();
            return new Color(tempColorArray[0], tempColorArray[1], tempColorArray[2], tempColorArray[3]);
        }

        public static void NormalizeByAlpha(CommandBuffer cmd, ComputeShader computeShader, int normalizeByAlphaKernel, int width, int height, RenderTargetIdentifier inOut)
        {
            cmd.SetComputeTextureParam(computeShader, normalizeByAlphaKernel, ComputeHelpers.ShaderIDs.TextureInOut, inOut);
            cmd.SetComputeIntParam(computeShader, ComputeHelpers.ShaderIDs.TextureWidth, width);
            cmd.SetComputeIntParam(computeShader, ComputeHelpers.ShaderIDs.TextureHeight, height);
            computeShader.GetKernelThreadGroupSizes(normalizeByAlphaKernel, out uint x, out uint y, out _);
            cmd.DispatchCompute(computeShader, normalizeByAlphaKernel, GraphicsHelpers.DivUp(width, x), GraphicsHelpers.DivUp(height, y), 1);
        }

        public static void MultiplyRenderTexture(CommandBuffer cmd, ComputeShader multiplyShader, int multiplyKernel, RenderTargetIdentifier inOut, int width, int height, Vector4 multiplicationFactor)
        {
            cmd.SetComputeTextureParam(multiplyShader, multiplyKernel, ComputeHelpers.ShaderIDs.TextureInOut, inOut);
            cmd.SetComputeIntParam(multiplyShader, ComputeHelpers.ShaderIDs.TextureWidth, width);
            cmd.SetComputeIntParam(multiplyShader, ComputeHelpers.ShaderIDs.TextureHeight, height);
            cmd.SetComputeVectorParam(multiplyShader, ComputeHelpers.ShaderIDs.MultiplicationFactor, multiplicationFactor);
            multiplyShader.GetKernelThreadGroupSizes(multiplyKernel, out uint x, out uint y, out _);
            cmd.DispatchCompute(multiplyShader, multiplyKernel, GraphicsHelpers.DivUp(width, x), GraphicsHelpers.DivUp(height, y), 1);
        }

        public static void MultiplyTexture(CommandBuffer cmd, ComputeShader multiplyShader, int multiplyKernel, Texture2D texture, Vector4 multiplicationFactor)
        {
            int renderTargetID = ComputeHelpers.ShaderIDs.MultiplyTemporaryRenderTarget;
            cmd.GetTemporaryRT(renderTargetID, texture.width, texture.height, 0, FilterMode.Point, RenderTextureFormat.ARGBFloat, RenderTextureReadWrite.Linear, 1, true);
            cmd.CopyTexture(texture, renderTargetID);
            LightmapIntegrationHelpers.MultiplyRenderTexture(cmd, multiplyShader, multiplyKernel, renderTargetID, texture.width, texture.height, multiplicationFactor);
            cmd.CopyTexture(renderTargetID, texture);
            cmd.ReleaseTemporaryRT(renderTargetID);
        }

        public static void StandardErrorRenderTexture(CommandBuffer cmd, ComputeShader standardError, int standardErrorKernel, RenderTargetIdentifier varianceInR, RenderTargetIdentifier sampleCountInW, RenderTargetIdentifier output, int width, int height)
        {
            cmd.SetComputeTextureParam(standardError, standardErrorKernel, ComputeHelpers.ShaderIDs.VarianceInR, varianceInR);
            cmd.SetComputeTextureParam(standardError, standardErrorKernel, ComputeHelpers.ShaderIDs.SampleCountInW, sampleCountInW);
            cmd.SetComputeTextureParam(standardError, standardErrorKernel, ComputeHelpers.ShaderIDs.TextureOut, output);
            cmd.SetComputeIntParam(standardError, ComputeHelpers.ShaderIDs.TextureWidth, width);
            cmd.SetComputeIntParam(standardError, ComputeHelpers.ShaderIDs.TextureHeight, height);
            standardError.GetKernelThreadGroupSizes(standardErrorKernel, out uint x, out uint y, out _);
            cmd.DispatchCompute(standardError, standardErrorKernel, GraphicsHelpers.DivUp(width, x), GraphicsHelpers.DivUp(height, y), 1);
        }

        public static void StandardErrorThresholdRenderTexture(CommandBuffer cmd, ComputeShader standardErrorThreshold, int StandardErrorThresholdKernel, RenderTargetIdentifier standardErrorInR, RenderTargetIdentifier meanInR, float standardErrorThresholdValue, RenderTargetIdentifier output, int width, int height)
        {
            cmd.SetComputeTextureParam(standardErrorThreshold, StandardErrorThresholdKernel, ComputeHelpers.ShaderIDs.StandardErrorInR, standardErrorInR);
            cmd.SetComputeTextureParam(standardErrorThreshold, StandardErrorThresholdKernel, ComputeHelpers.ShaderIDs.MeanInR, meanInR);
            cmd.SetComputeTextureParam(standardErrorThreshold, StandardErrorThresholdKernel, ComputeHelpers.ShaderIDs.TextureOut, output);
            cmd.SetComputeFloatParam(standardErrorThreshold, ComputeHelpers.ShaderIDs.StandardErrorThreshold, standardErrorThresholdValue);
            cmd.SetComputeIntParam(standardErrorThreshold, ComputeHelpers.ShaderIDs.TextureWidth, width);
            cmd.SetComputeIntParam(standardErrorThreshold, ComputeHelpers.ShaderIDs.TextureHeight, height);
            standardErrorThreshold.GetKernelThreadGroupSizes(StandardErrorThresholdKernel, out uint x, out uint y, out _);
            cmd.DispatchCompute(standardErrorThreshold, StandardErrorThresholdKernel, GraphicsHelpers.DivUp(width, x), GraphicsHelpers.DivUp(height, y), 1);
        }

        public static void BroadcastChannelRenderTexture(CommandBuffer cmd, ComputeShader broadcastChannelShader, int broadcastChannelKernel, RenderTargetIdentifier inOut, int width, int height, int channelIndex)
        {
            cmd.SetComputeTextureParam(broadcastChannelShader, broadcastChannelKernel, ComputeHelpers.ShaderIDs.TextureInOut, inOut);
            cmd.SetComputeIntParam(broadcastChannelShader, ComputeHelpers.ShaderIDs.TextureWidth, width);
            cmd.SetComputeIntParam(broadcastChannelShader, ComputeHelpers.ShaderIDs.TextureHeight, height);
            cmd.SetComputeIntParam(broadcastChannelShader, ComputeHelpers.ShaderIDs.ChannelIndex, channelIndex);
            broadcastChannelShader.GetKernelThreadGroupSizes(broadcastChannelKernel, out uint x, out uint y, out _);
            cmd.DispatchCompute(broadcastChannelShader, broadcastChannelKernel, GraphicsHelpers.DivUp(width, x), GraphicsHelpers.DivUp(height, y), 1);
        }

        public static void SetChannelRenderTexture(CommandBuffer cmd, ComputeShader setChannelShader, int setChannelKernel, RenderTargetIdentifier inOut, int width, int height, int channelIndex, float channelValue)
        {
            cmd.SetComputeTextureParam(setChannelShader, setChannelKernel, ComputeHelpers.ShaderIDs.TextureInOut, inOut);
            cmd.SetComputeIntParam(setChannelShader, ComputeHelpers.ShaderIDs.TextureWidth, width);
            cmd.SetComputeIntParam(setChannelShader, ComputeHelpers.ShaderIDs.TextureHeight, height);
            cmd.SetComputeIntParam(setChannelShader, ComputeHelpers.ShaderIDs.ChannelIndex, channelIndex);
            cmd.SetComputeFloatParam(setChannelShader, ComputeHelpers.ShaderIDs.ChannelValue, channelValue);
            setChannelShader.GetKernelThreadGroupSizes(setChannelKernel, out uint x, out uint y, out _);
            cmd.DispatchCompute(setChannelShader, setChannelKernel, GraphicsHelpers.DivUp(width, x), GraphicsHelpers.DivUp(height, y), 1);
        }

        public static void ReferenceBoxFilterRenderTexture(CommandBuffer cmd, ComputeShader referenceBoxFilterShader, int referenceBoxFilterKernel, RenderTargetIdentifier inOut, int width, int height, int radius)
        {
            cmd.GetTemporaryRT(ComputeHelpers.ShaderIDs.TextureOut, width, height, 0, FilterMode.Point, RenderTextureFormat.ARGBFloat, RenderTextureReadWrite.Linear, 1, true);
            cmd.SetComputeTextureParam(referenceBoxFilterShader, referenceBoxFilterKernel, ComputeHelpers.ShaderIDs.SourceTexture, inOut);
            cmd.SetComputeTextureParam(referenceBoxFilterShader, referenceBoxFilterKernel, ComputeHelpers.ShaderIDs.TextureOut, ComputeHelpers.ShaderIDs.TextureOut);
            cmd.SetComputeIntParam(referenceBoxFilterShader, ComputeHelpers.ShaderIDs.TextureWidth, width);
            cmd.SetComputeIntParam(referenceBoxFilterShader, ComputeHelpers.ShaderIDs.TextureHeight, height);
            cmd.SetComputeIntParam(referenceBoxFilterShader, ComputeHelpers.ShaderIDs.BoxFilterRadius, radius);
            referenceBoxFilterShader.GetKernelThreadGroupSizes(referenceBoxFilterKernel, out uint x, out uint y, out _);
            cmd.DispatchCompute(referenceBoxFilterShader, referenceBoxFilterKernel, GraphicsHelpers.DivUp(width, x), GraphicsHelpers.DivUp(height, y), 1);
            cmd.CopyTexture(ComputeHelpers.ShaderIDs.TextureOut, inOut);
            cmd.ReleaseTemporaryRT(ComputeHelpers.ShaderIDs.TextureOut);
        }

        public static void ReferenceBoxFilterBlueChannelRenderTexture(CommandBuffer cmd, ComputeShader referenceBoxFilterBlueChannelShader, int referenceBoxFilterBlueChannelKernel, RenderTargetIdentifier inOut, int width, int height, int radius, GraphicsBuffer indirectDispatchBuffer)
        {
            cmd.BeginSample("BoxFiltering");
            cmd.GetTemporaryRT(ComputeHelpers.ShaderIDs.TextureOut, width, height, 0, FilterMode.Point, RenderTextureFormat.ARGBFloat, RenderTextureReadWrite.Linear, 1, true);
            cmd.SetComputeTextureParam(referenceBoxFilterBlueChannelShader, referenceBoxFilterBlueChannelKernel, ComputeHelpers.ShaderIDs.SourceTexture, inOut);
            cmd.SetComputeTextureParam(referenceBoxFilterBlueChannelShader, referenceBoxFilterBlueChannelKernel, ComputeHelpers.ShaderIDs.TextureOut, ComputeHelpers.ShaderIDs.TextureOut);
            cmd.SetComputeIntParam(referenceBoxFilterBlueChannelShader, ComputeHelpers.ShaderIDs.TextureWidth, width);
            cmd.SetComputeIntParam(referenceBoxFilterBlueChannelShader, ComputeHelpers.ShaderIDs.TextureHeight, height);
            cmd.SetComputeIntParam(referenceBoxFilterBlueChannelShader, ComputeHelpers.ShaderIDs.BoxFilterRadius, radius);
            cmd.DispatchCompute(referenceBoxFilterBlueChannelShader, referenceBoxFilterBlueChannelKernel, indirectDispatchBuffer, 0);
            cmd.CopyTexture(ComputeHelpers.ShaderIDs.TextureOut, inOut); // TODO(jesper): dont do copy
            cmd.ReleaseTemporaryRT(ComputeHelpers.ShaderIDs.TextureOut);
            cmd.EndSample("BoxFiltering");
        }

        internal static Mesh TerrainDataToMesh(in LightBakerBridge.TerrainData terrainData, in LightBakerBridge.HeightmapData heightmapData, in LightBakerBridge.TerrainHoleData holeData)
        {
            var outMesh = TerrainToMesh.Convert(heightmapData.resolution, heightmapData.resolution, heightmapData.data, terrainData.heightmapScale, holeData.resolution, holeData.resolution, holeData.data);
            return outMesh;
        }

        // Computes the region of a lightmap that is occupied by the pixels of a specific instance,
        // given the lightmap size, the instance's scale and offset within the lightmap (lightmap ST),
        // and the bounding box of the instance's UVs.
        //
        // NOTE: Only considering the region defined by the instance's lightmap ST causes issues, because
        // these regions can overlap for multiple instances, when the instance's UV layout doesn't fully
        // cover the [0; 1] range. We pack based on tight bounding boxes of the instance's UVs,
        // but lightmap STs are with respect to the instance's entire UV layout.
        internal static void ComputeOccupiedTexelRegionForInstance(
            uint lightmapWidth,
            uint lightmapHeight,
            Vector4 instanceLightmapST,
            Vector2 uvBoundsSize,
            Vector2 uvBoundsOffset,
            out Vector4 normalizedOccupiedST,
            out Vector2Int occupiedTexelSize,
            out Vector2Int occupiedTexelOffset)
        {
            // Transform the "instance stamp" bounds to contain only the occupied region.
            float normalizedOccupiedWidth = instanceLightmapST.x * uvBoundsSize.x;
            float normalizedOccupiedHeight = instanceLightmapST.y * uvBoundsSize.y;

            float normalizedOccupiedOffsetX = instanceLightmapST.z + uvBoundsOffset.x * instanceLightmapST.x;
            float normalizedOccupiedOffsetY = instanceLightmapST.w + uvBoundsOffset.y * instanceLightmapST.y;

            normalizedOccupiedST = new Vector4(normalizedOccupiedWidth, normalizedOccupiedHeight, normalizedOccupiedOffsetX, normalizedOccupiedOffsetY);

            // Transform the occupied region to lightmap pixel coordinate space.
            float occupiedWidth = lightmapWidth * normalizedOccupiedWidth;
            float occupiedHeight = lightmapHeight * normalizedOccupiedHeight;

            float occupiedOffsetX = lightmapWidth * normalizedOccupiedOffsetX;
            float occupiedOffsetY = lightmapHeight * normalizedOccupiedOffsetY;

            // Round up to the nearest integer to ensure we cover all texels that the instance's UVs touch.
            occupiedTexelSize = new(Mathf.CeilToInt(occupiedWidth), Mathf.CeilToInt(occupiedHeight));

            // Clamp the occupied region to the lightmap bounds.
            float occupiedTexelOffsetXFloat = Mathf.Max(occupiedOffsetX, 0f);
            float occupiedTexelOffsetYFloat = Mathf.Max(occupiedOffsetY, 0f);
            Debug.Assert(Mathf.Abs(Mathf.Round(occupiedTexelOffsetXFloat) - occupiedTexelOffsetXFloat) < 0.001f, $"Expected offset (X) to be very close to zero. Was {occupiedTexelOffsetXFloat % 1}. Was the scene baked with the same resolution as it was atlassed for?");
            Debug.Assert(Mathf.Abs(Mathf.Round(occupiedTexelOffsetYFloat) - occupiedTexelOffsetYFloat) < 0.001f, $"Expected offset (Y) to be very close to zero. Was {occupiedTexelOffsetYFloat % 1}. Was the scene baked with the same resolution as it was atlassed for?");
            occupiedTexelOffset = new((int)Mathf.Round(occupiedTexelOffsetXFloat), (int)Mathf.Round(occupiedTexelOffsetYFloat));
        }

        // Computes the bounding box of a set of UV coordinates.
        internal static void ComputeUVBounds(IEnumerable<Vector2> uvs, out Vector2 size, out Vector2 offset)
        {
            Vector2 minUV = new Vector2(float.MaxValue, float.MaxValue);
            Vector2 maxUV = new Vector2(float.MinValue, float.MinValue);
            foreach (var uv in uvs)
            {
                minUV = Vector2.Min(minUV, uv);
                maxUV = Vector2.Max(maxUV, uv);
            }
            size = new Vector2(maxUV.x - minUV.x, maxUV.y - minUV.y);
            offset = minUV;
        }

        public class AdaptiveSample
        {
            public uint sampleCount;
            public float accumulatedLuminance;
            public float mean;
            public float meanSqr;
            public float variance;
            public float varianceFiltered;
            public float standardError;
            public bool active;
            override public string ToString()
            {
                string samplesString = new("");
                samplesString += sampleCount.ToString() + "\t";
                samplesString += accumulatedLuminance.ToString("F99").TrimEnd('0') + "\t";
                samplesString += mean.ToString("F99").TrimEnd('0') + "\t";
                samplesString += meanSqr.ToString("F99").TrimEnd('0') + "\t";
                samplesString += variance.ToString("F99").TrimEnd('0') + "\t";
                samplesString += varianceFiltered.ToString("F99").TrimEnd('0') + "\t";
                samplesString += standardError.ToString("F99").TrimEnd('0') + "\t";
                samplesString += active.ToString() + "\n";
                return samplesString;
            }
            public static string HeaderString()
            {
                string headerString = new("");
                headerString += "sampleCount\taccumulatedLuminance\tmean\tmeanSqr\tvariance\tvarianceFiltered\tstandardError\tactive\n";
                return headerString;
            }

            static public string SamplesToString(AdaptiveSample[] samples, int x, int y, float adaptiveThreshold)
            {
                if (samples.Length == 0)
                    return "";
                string samplesString = new($"data for pixel [{x}, {y}], threshold:\t{adaptiveThreshold}\n");
                samplesString += AdaptiveSample.HeaderString();
                foreach (AdaptiveSample sample in samples)
                {
                    samplesString += sample.ToString();
                }
                return samplesString;
            }
        }
    }
}
