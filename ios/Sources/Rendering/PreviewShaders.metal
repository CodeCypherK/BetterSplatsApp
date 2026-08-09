#include <metal_stdlib>
using namespace metal;

// Fullscreen quad displaying the sensor-landscape 420f video buffer rotated
// 90° clockwise for portrait UI. Aspect handling is aspect-fill via the
// view; texcoords map (u,v) -> rotated sampling.

struct PreviewVertexOut {
    float4 position [[position]];
    float2 texcoord;
};

vertex PreviewVertexOut preview_vertex(uint vid [[vertex_id]]) {
    // Triangle strip covering clip space.
    const float2 positions[4] = {
        float2(-1.0, -1.0), float2(1.0, -1.0),
        float2(-1.0, 1.0), float2(1.0, 1.0),
    };
    // 90° CW rotation of the landscape sensor image into portrait:
    // screen (x right, y up) samples texture at (u = v_screen, v = u_screen).
    const float2 texcoords[4] = {
        float2(0.0, 1.0),  // bottom-left  <- image left edge bottom
        float2(0.0, 0.0),  // bottom-right <- image top
        float2(1.0, 1.0),
        float2(1.0, 0.0),
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
