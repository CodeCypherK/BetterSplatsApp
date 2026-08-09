#include <metal_stdlib>
using namespace metal;

// Reconstruction map rendering: instanced point sprites + line lists,
// shared MVP passed as vertex bytes at buffer(2).

struct MapVertexOut {
    float4 position [[position]];
    float4 color;
    float pointSize [[point_size]];
};

vertex MapVertexOut map_point_vertex(uint vid [[vertex_id]],
                                     const device float4* positions [[buffer(0)]],
                                     const device float4* colors [[buffer(1)]],
                                     constant float4x4& mvp [[buffer(2)]]) {
    MapVertexOut out;
    out.position = mvp * float4(positions[vid].xyz, 1.0);
    out.color = colors[vid];
    // Perspective-attenuated sprite size.
    out.pointSize = clamp(9.0 / max(out.position.w, 0.2), 2.0, 8.0);
    return out;
}

fragment float4 map_point_fragment(MapVertexOut in [[stage_in]],
                                   float2 coord [[point_coord]]) {
    const float2 centered = coord - float2(0.5, 0.5);
    if (dot(centered, centered) > 0.25) discard_fragment();
    return in.color;
}

vertex MapVertexOut map_line_vertex(uint vid [[vertex_id]],
                                    const device float4* positions [[buffer(0)]],
                                    const device float4* colors [[buffer(1)]],
                                    constant float4x4& mvp [[buffer(2)]]) {
    MapVertexOut out;
    out.position = mvp * float4(positions[vid].xyz, 1.0);
    out.color = colors[vid];
    out.pointSize = 1.0;
    return out;
}

fragment float4 map_line_fragment(MapVertexOut in [[stage_in]]) {
    return in.color;
}
