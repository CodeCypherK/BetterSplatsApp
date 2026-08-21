import ARKit
import Foundation
import SceneKit
import simd
import UIKit

private extension ARMeshGeometry {
    func vertex(at index: Int) -> SIMD3<Float> {
        let ptr = vertices.buffer.contents()
            .advanced(by: vertices.offset + vertices.stride * index)
        let f = ptr.assumingMemoryBound(to: Float.self)
        return SIMD3(f[0], f[1], f[2])
    }

    func faceIndices(at faceIndex: Int) -> [Int32] {
        let count = faces.indexCountPerPrimitive
        let offset = faces.offset + faces.stride * faceIndex
        let ptr = faces.buffer.contents().advanced(by: offset)
        var out = [Int32](repeating: 0, count: count)
        if faces.bytesPerIndex == 2 {
            let u16 = ptr.assumingMemoryBound(to: UInt16.self)
            for i in 0..<count { out[i] = Int32(u16[i]) }
        } else {
            let u32 = ptr.assumingMemoryBound(to: UInt32.self)
            for i in 0..<count { out[i] = Int32(bitPattern: u32[i]) }
        }
        return out
    }
}

/// Turns an `ARMeshAnchor` into a SceneKit wireframe so the room reads as a
/// meshed grid — cyan when photos cover it, orange when it is still thin.
enum CoverageMeshBuilder {
    static func geometry(from anchor: ARMeshAnchor, thin: Bool) -> SCNGeometry {
        let mesh = anchor.geometry
        let vertexCount = mesh.vertices.count
        let faceCount = mesh.faces.count

        var positions = [SCNVector3](repeating: .init(), count: vertexCount)
        for i in 0..<vertexCount {
            let v = mesh.vertex(at: i)
            positions[i] = SCNVector3(v.x, v.y, v.z)
        }

        let perFace = mesh.faces.indexCountPerPrimitive
        var indices = [Int32](repeating: 0, count: faceCount * perFace)
        for f in 0..<faceCount {
            let face = mesh.faceIndices(at: f)
            for j in 0..<perFace {
                indices[f * perFace + j] = face[j]
            }
        }

        let positionSource = SCNGeometrySource(vertices: positions)
        let element = SCNGeometryElement(indices: indices, primitiveType: .triangles)
        let lineElement = SCNGeometryElement(indices: indices, primitiveType: .triangles)

        let fill = SCNMaterial()
        fill.diffuse.contents = thin
            ? UIColor.systemOrange.withAlphaComponent(0.22)
            : UIColor.cyan.withAlphaComponent(0.14)
        fill.isDoubleSided = true
        fill.writesToDepthBuffer = false
        fill.readsFromDepthBuffer = false
        fill.lightingModel = .constant
        fill.transparency = thin ? 0.5 : 0.7

        let lines = SCNMaterial()
        lines.diffuse.contents = thin
            ? UIColor.systemOrange.withAlphaComponent(0.95)
            : UIColor.cyan.withAlphaComponent(0.9)
        lines.fillMode = .lines
        lines.isDoubleSided = true
        lines.lightingModel = .constant
        lines.writesToDepthBuffer = false

        let geometry = SCNGeometry(sources: [positionSource],
                                   elements: [element, lineElement])
        geometry.materials = [fill, lines]
        return geometry
    }

    static func photoMarkerNode(color: UIColor = .systemYellow) -> SCNNode {
        let root = SCNNode()
        let ball = SCNSphere(radius: 0.04)
        ball.firstMaterial?.diffuse.contents = color
        ball.firstMaterial?.lightingModel = .constant
        root.addChildNode(SCNNode(geometry: ball))

        let cone = SCNCone(topRadius: 0.001, bottomRadius: 0.07, height: 0.2)
        cone.firstMaterial?.diffuse.contents = color.withAlphaComponent(0.5)
        cone.firstMaterial?.lightingModel = .constant
        let coneNode = SCNNode(geometry: cone)
        coneNode.eulerAngles.x = -.pi / 2
        coneNode.position = SCNVector3(0, 0, -0.14)
        root.addChildNode(coneNode)
        return root
    }
}

struct CapturedPoseSample: Identifiable, Equatable {
    var id: UInt32
    var position: SIMD3<Float>
    var transform: simd_float4x4
}

struct CoverageStats: Equatable {
    var meshAnchorCount = 0
    var meshTriangles = 0
    var photos = 0
    var coveredFraction: Float = 0
    var thinSampleCount = 0
    var sampleCount = 0

    var coveredPercent: Int { Int((coveredFraction * 100).rounded()) }

    var headline: String {
        if photos == 0 { return "Walk the space — the mesh grows as you look" }
        if sampleCount == 0 { return "Meshing… keep the camera on surfaces" }
        if coveredFraction < 0.45 {
            return "Orange mesh needs photos — point and snap"
        }
        if coveredFraction < 0.75 {
            return "Filling in — hit the remaining orange patches"
        }
        return "Coverage looks solid"
    }
}

enum CoverageAnalyzer {
    static func analyze(anchors: [ARMeshAnchor],
                        photos: [CapturedPoseSample],
                        goodRadius: Float = 0.9)
        -> (stats: CoverageStats, thinAnchorIds: Set<UUID>) {
        var stats = CoverageStats()
        stats.meshAnchorCount = anchors.count
        stats.photos = photos.count
        var thinIds = Set<UUID>()
        guard !anchors.isEmpty else { return (stats, thinIds) }

        let photoPositions = photos.map(\.position)
        var covered = 0
        var total = 0
        var triangles = 0

        for anchor in anchors {
            let mesh = anchor.geometry
            triangles += mesh.faces.count
            let verts = mesh.vertices.count
            let step = max(1, verts / 48)
            var thinHere = 0
            var samplesHere = 0
            let M = anchor.transform
            for i in stride(from: 0, to: verts, by: step) {
                let local = mesh.vertex(at: i)
                let world = M * SIMD4<Float>(local.x, local.y, local.z, 1)
                let p = SIMD3(world.x, world.y, world.z)
                samplesHere += 1
                total += 1
                if photoPositions.isEmpty {
                    thinHere += 1
                    continue
                }
                var best = Float.greatestFiniteMagnitude
                for q in photoPositions {
                    best = min(best, simd_distance(p, q))
                }
                if best <= goodRadius { covered += 1 } else { thinHere += 1 }
            }
            if samplesHere > 0, Float(thinHere) / Float(samplesHere) > 0.5 {
                thinIds.insert(anchor.identifier)
            }
        }

        stats.meshTriangles = triangles
        stats.sampleCount = total
        stats.thinSampleCount = max(0, total - covered)
        stats.coveredFraction = total == 0 ? 0 : Float(covered) / Float(total)
        return (stats, thinIds)
    }
}
