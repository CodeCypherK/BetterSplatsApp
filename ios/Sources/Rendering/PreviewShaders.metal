#include <metal_stdlib>
using namespace metal;

// Fullscreen quad displaying the sensor-native 420f video buffer, oriented
// and aspect-filled for the portrait UI.
//
// The texture coordinates are computed on the CPU (VideoPreviewRenderer) and
// handed in, rather than being a hand-written table in here. They depend on
// the drawable size, which the shader does not know, and the rotation is
// error-prone enough to be worth writing where it can carry its derivation:
// the first version was a reflection about the anti-diagonal rather than a
// rotation, which reached a device as "the camera feed is upside down".

struct PreviewVertexOut {
    float4 position [[position]];
    float2 texcoord;
};

vertex PreviewVertexOut preview_vertex(uint vid [[vertex_id]],
                                       constant float2 *texcoords [[buffer(0)]]) {
    // Triangle strip covering clip space. Clip space is y-UP, and the
    // viewport maps y = +1 to the top row of the drawable.
    const float2 positions[4] = {
        float2(-1.0, -1.0),  // screen bottom-left
        float2(1.0, -1.0),   // screen bottom-right
        float2(-1.0, 1.0),   // screen top-left
        float2(1.0, 1.0),    // screen top-right
    };
    PreviewVertexOut out;
    out.position = float4(positions[vid], 0.0, 1.0);
    out.texcoord = texcoords[vid];
    return out;
}

fragment float4 preview_fragment(PreviewVertexOut in [[stage_in]],
                                 texture2d<float> lumaTexture [[texture(0)]],
                                 texture2d<float> chromaTexture [[texture(1)]]) {
    constexpr sampler s(address::clamp_to_edge, filter::linear);
    const float y = lumaTexture.sample(s, in.texcoord).r;
    const float2 cbcr = chromaTexture.sample(s, in.texcoord).rg - float2(0.5, 0.5);
    // BT.601 full-range.
    const float3 rgb = float3(
        y + 1.402 * cbcr.y,
        y - 0.344136 * cbcr.x - 0.714136 * cbcr.y,
        y + 1.772 * cbcr.x);
    return float4(clamp(rgb, 0.0, 1.0), 1.0);
}
