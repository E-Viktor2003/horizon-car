#!/usr/bin/env python3
"""Convert `low_poly_small_car.glb` into `api/src/render/model_data.rs`.

The converter:

* applies the glTF node hierarchy to every vertex;
* scales metres to blocks (0.42) and puts the tyres on y = 0;
* splits the car into a body and four wheels with their own pivots;
* drops the interior, the steering wheel and the small hidden "handler"
  details (they are never visible from outside the car);
* merges coplanar triangle pairs into quads (draw_quad draws a quad a call);
* gives every quad the vanilla `block/white_concrete` sprite and the material
  colour as its tint, so the bundle needs no textures of its own.

Usage:
    python3 tools/glb2car.py low_poly_small_car.glb -o api/src/render/model_data.rs
"""
from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path

import numpy as np

SCALE = 0.42
# The model's lowest point is the tyre at world y = -0.033; lift it to 0.
LIFT = 0.033 * SCALE
# Nodes that are inside the car or are invisible small parts.
SKIP = ("Interior", "Steerwheel", "Handler")
SPRITE = "block/white_concrete"
UV = [4.0, 4.0, 12.0, 12.0]
MAX_QUADS = 2600

# glTF material -> 0xRRGGBB. Read from baseColorFactor; listed here so a
# material renamed in the asset still gets the same Minecraft colour.
COLORS = {
    0: 0x2F2F2F,  # tyre
    1: 0x6F6F6F,  # rim
    2: 0x8A8A8A,  # handler (dropped)
    3: 0x414141,  # interior panel (dropped)
    4: 0x2F2F2F,  # interior panel (dropped)
    5: 0x2D2D2D,  # bumpers
    6: 0x353E65,  # chassis (dark blue)
    7: 0xC79A4B,  # turn lights
    8: 0xC1C1C1,  # mirror
    9: 0x3A5A80,  # windows (the asset is glass; keep it opaque here)
    10: 0xF7F1DA,  # front lights
    11: 0xEB4141,  # brake lights
}


def read_glb(path: Path):
    data = path.read_bytes()
    magic, version, _ = struct.unpack_from("<III", data, 0)
    if magic != 0x46546C67:
        raise SystemExit(f"{path}: not a GLB")
    offset = 12
    chunks = []
    while offset < len(data):
        length, kind = struct.unpack_from("<II", data, offset)
        chunks.append((kind, offset + 8, length))
        offset += 8 + length
    doc = json.loads(data[chunks[0][1] : chunks[0][1] + chunks[0][2]])
    binary = data[chunks[1][1] : chunks[1][1] + chunks[1][2]]
    return doc, binary


def accessor(doc, binary, index):
    a = doc["accessors"][index]
    view = doc["bufferViews"][a["bufferView"]]
    start = view.get("byteOffset", 0) + a.get("byteOffset", 0)
    count = a["count"]
    ncomp = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[a["type"]]
    code = {5126: "f", 5123: "H", 5125: "I", 5121: "B"}[a["componentType"]]
    size = struct.calcsize(code)
    stride = view.get("byteStride", ncomp * size)
    out = []
    for i in range(count):
        out.append(struct.unpack_from("<" + code * ncomp, binary, start + i * stride))
    return out


def node_matrix(node):
    if "matrix" in node:
        return np.array(node["matrix"], dtype=np.float64).reshape(4, 4).T
    m = np.eye(4)
    if "scale" in node:
        m = np.diag(list(node["scale"]) + [1.0]) @ m
    if "rotation" in node:
        x, y, z, w = node["rotation"]
        r = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0],
                [0, 0, 0, 1],
            ]
        )
        m = r @ m
    if "translation" in node:
        t = np.eye(4)
        t[:3, 3] = node["translation"]
        m = t @ m
    return m


def wheel_of(name: str):
    for key in ("FR_Wheel", "RR_Wheel", "FL_Wheel", "RL_Wheel"):
        if name.startswith(key):
            return key[:2].lower()
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("glb", type=Path)
    parser.add_argument("-o", "--output", type=Path, required=True)
    args = parser.parse_args()

    doc, binary = read_glb(args.glb)
    nodes = doc["nodes"]
    world = {}

    def walk(index, parent):
        transform = parent @ node_matrix(nodes[index])
        world[index] = transform
        for child in nodes[index].get("children", []):
            walk(child, transform)

    for root in doc["scenes"][doc.get("scene", 0)]["nodes"]:
        walk(root, np.eye(4))

    triangles = []  # (part, material, [p0, p1, p2])
    pivots = {}
    for index, node in enumerate(nodes):
        name = str(node.get("name", ""))
        if "mesh" not in node or any(s in name for s in SKIP):
            continue
        part = wheel_of(name) or "body"
        transform = world[index]
        mesh = doc["meshes"][node["mesh"]]
        for primitive in mesh["primitives"]:
            material = primitive.get("material")
            if material is None or material not in COLORS:
                continue
            pos = np.array(
                [p for p in accessor(doc, binary, primitive["attributes"]["POSITION"])],
                dtype=np.float64,
            ).reshape(-1, 3)
            pos = np.hstack([pos, np.ones((len(pos), 1))]) @ transform.T
            pos = pos[:, :3] * SCALE
            pos[:, 1] += LIFT
            if part != "body":
                if part not in pivots:
                    lo, hi = pos.min(0), pos.max(0)
                    pivots[part] = ((lo + hi) / 2.0).tolist()
                pos -= np.array(pivots[part])
            indices = accessor(doc, binary, primitive["indices"])
            for i in range(0, len(indices), 3):
                tri = [pos[indices[i + k]] for k in range(3)]
                triangles.append((part, COLORS[material], tri))

    if "body" not in pivots:
        pivots["body"] = [0.0, 0.0, 0.0]
    for part in ("fl", "fr", "rl", "rr"):
        if part not in pivots:
            raise SystemExit(f"missing wheel {part}")

    quads = []
    for part in ("body", "fl", "fr", "rl", "rr"):
        part_tris = [t for t in triangles if t[0] == part]
        merged = merge(part_tris)
        quads.append((part, merged))
        print(f"{part}: {len(part_tris)} triangles -> {len(merged)} quads")

    total = sum(len(q) for _, q in quads)
    print(f"total: {total} quads (budget {MAX_QUADS})")
    if total > MAX_QUADS:
        raise SystemExit(f"over budget: {total} > {MAX_QUADS}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(render(quads, pivots), encoding="utf-8")
    print(f"wrote {args.output}")


def key(point, digits=4):
    return (round(point[0], digits), round(point[1], digits), round(point[2], digits))


def normal(tri):
    a, b, c = (np.array(p) for p in tri)
    n = np.cross(b - a, c - a)
    length = np.linalg.norm(n)
    return n / length if length > 1e-12 else None


def merge(tris):
    """Greedily merge coplanar triangles that share an edge into quads."""
    groups = {}
    for part, color, tri in tris:
        n = normal(tri)
        if n is None:
            continue
        plane = float(np.dot(n, tri[0]))
        k = (
            round(float(n[0]), 3),
            round(float(n[1]), 3),
            round(float(n[2]), 3),
            round(plane, 3),
            color,
        )
        groups.setdefault(k, (color, []))[1].append(tri)

    out = []
    for color, group in groups.values():
        edges = {}
        for i, tri in enumerate(group):
            for j in range(3):
                a = key(tri[j])
                b = key(tri[(j + 1) % 3])
                edges.setdefault((a, b), i)
        used = [False] * len(group)
        for i, tri in enumerate(group):
            if used[i]:
                continue
            partner = None
            for j in range(3):
                a = key(tri[j])
                b = key(tri[(j + 1) % 3])
                other = edges.get((b, a))
                if other is not None and other != i and not used[other]:
                    candidate = group[other]
                    # The shared edge of the candidate must be (b, a).
                    for k in range(3):
                        if key(candidate[k]) == b and key(candidate[(k + 1) % 3]) == a:
                            partner = (other, j, k)
                            break
                if partner:
                    break
            if partner is None:
                out.append(([tri[0], tri[1], tri[2], tri[2]], color))
                used[i] = True
                continue
            other, j, k = partner
            a, b = tri[j], tri[(j + 1) % 3]
            x = tri[(j + 2) % 3]
            y = group[other][(k + 2) % 3]
            out.append(([a, y, b, x], color))
            used[i] = True
            used[other] = True
    return out


def render(quads, pivots):
    lines = [
        "// @generated by tools/glb2car.py from low_poly_small_car.glb; do not edit.",
        "// The model is CC-BY-4.0 by scailman (see NOTICE).",
        "use super::Quad;",
        "",
        f'pub const SPRITE: &str = "{SPRITE}";',
        f"pub const UV: [f32; 4] = {rust_floats(UV)};",
        "",
    ]
    for part, items in quads:
        name = {
            "body": "BODY",
            "fl": "WHEEL_FL",
            "fr": "WHEEL_FR",
            "rl": "WHEEL_RL",
            "rr": "WHEEL_RR",
        }[part]
        lines.append(f"pub const {name}: &[Quad] = &[")
        for points, color in items:
            corners = ", ".join(rust_point(p) for p in points)
            lines.append(f"    Quad {{ p: [{corners}], color: 0x{color:06X} }},")
        lines.append("];")
        lines.append("")
    for part in ("fl", "fr", "rl", "rr"):
        lines.append(
            f"pub const PIVOT_{part.upper()}: [f32; 3] = {rust_point(pivots[part])};"
        )
    lines.append("")
    return "\n".join(lines)


def rust_point(p):
    return "[{:.4}, {:.4}, {:.4}]".format(float(p[0]), float(p[1]), float(p[2]))


def rust_floats(values):
    return "[" + ", ".join(f"{float(v):.1f}" for v in values) + "]"


if __name__ == "__main__":
    main()
