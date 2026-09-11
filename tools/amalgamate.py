#!/usr/bin/env python3
"""Single-header split/amalgamation for Puffysics.

Source of truth: src/puffysics/*.inl. Root and dist/ headers are
generated amalgamations. This script performs three jobs:

  split       cut root puffysics.cuh into src/puffysics/*.inl (pattern
              anchored, order fixed below) + write include/puffysics/puffysics.cuh
  amalgamate  inline the chunks (+ b3_art/atan_approx) into dist/ and root
              puffysics.cuh with a GENERATED banner
  check       redo both into temp dirs and byte-compare against the tree

Chunk boundaries are (file, start-regex) pairs; a chunk runs to the line
before the next chunk's start. Concatenated chunk bodies must equal the
original file byte-for-byte (asserted on split). Amalgamation is fully
deterministic: fixed order, LF endings, no timestamps.

Usage: tools/amalgamate.py split|amalgamate|check
"""
import os
import re
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src", "puffysics")
INCLUDE = os.path.join(ROOT, "include", "puffysics")
DIST = os.path.join(ROOT, "dist")

# (chunk file, start pattern or None for BOF, banner coverage note)
CHUNKS = [
    ("01_config.inl", None,
     "INTERNAL: includes, B3_HD/B3_INL, capacities, feature flags"),
    ("02_types.inl", r"^#define B3_STATIC 0$",
     "INTERNAL: enum constants, math/body/shape/contact/joint/world structs"),
    ("03_math.inl", r"^B3_HD B3_INL B3Vec3 b3_v\(float x, float y, float z\) \{$",
     "INTERNAL: vector/quaternion/matrix helpers, twist, hinge frame math"),
    ("04_world.inl",
     r"^B3_HD B3_INL B3BodyDef b3_default_body\(void\) \{$",
     "INTERNAL: defaults, world init, capacity API, body/shape creation, mass"),
    ("05_collision.inl", r"^B3_HD B3_INL B3Vec3 b3_closest_seg\(",
     "INTERNAL: manifolds, primitive tests, SAT box collision"),
    ("06_joints.inl", r"^B3_HD B3_INL B3Joint\* b3_add_joint\(",
     "INTERNAL: weld/revolute creation, setters, joint solvers, hinge cache"),
    ("07_packed.inl",
     r"^B3_HD B3_INL void b3_prepare_one_contact\(B3Contact\* c, const B3Body\* ba,$",
     "INTERNAL: packed-GS snapshots, interleaved solves, articulation hooks",
     r"^.*static_s\);$"),
    ("08_contacts.inl", r"^B3_HD B3_INL void b3_contact_from_mani\(",
     "INTERNAL: contact generation, warm start, contact sweeps, restitution"),
    ("09_step.inl", r"^B3_HD B3_INL void b3_integrate_velocity_state\(",
     "INTERNAL: integrators, substep driver, b3_step, CUDA kernel"),
]

WRAPPER_HEAD = """\
#pragma once
#ifndef PUFFYSICS_CORE_INCLUDED
#define PUFFYSICS_CORE_INCLUDED
// Puffysics development entry point (modular split of puffysics.cuh).
//
// PUBLIC API (stable surface; bodies below are INTERNAL implementation):
//   world: b3_world_init, b3_create_body, b3_add_shape, b3_create_sphere,
//     b3_create_capsule, b3_create_box, b3_create_box_local,
//     b3_finalize_mass, b3_set_inertial, b3_body/shape_capacity,
//     b3_clear_errors, b3_step_stats, b3_world_error
//   joints: b3_create_weld, b3_create_revolute, b3_joint_enable/set_motor,
//     b3_joint_enable/set_limits, b3_joint_enable/set_spring,
//     b3_joint_angle, b3_joint_speed, b3_joint_capacity
//   step: b3_step, b3_step_kernel, b3_find_contacts, b3_contact_capacity
//   mass: b3_shape_mass
// Topics below map to src/puffysics/*.inl; do not include .inl directly.
#define PUFFYSICS_SPLIT_OK
"""

GUARD = """\
#ifndef PUFFYSICS_SPLIT_OK
#error "Do not include this .inl directly; include puffysics.cuh"
#endif
"""

GENERATED_BANNER = """\
// GENERATED — do not edit. Source of truth: src/puffysics/*.inl
// (see include/puffysics/puffysics.cuh). Regenerate with tools/amalgamate.py.
"""


def banner(name, note):
    return "// ==== puffysics %s: %s ====\n" % (name, note)


def read_root():
    with open(os.path.join(ROOT, "puffysics.cuh"), "rb") as f:
        return f.read().decode("utf-8")


def split_chunks(text):
    lines = text.split("\n")
    starts = []
    for entry in CHUNKS:
        name, pat = entry[0], entry[1]
        look = entry[3] if len(entry) > 3 else None
        if pat is None:
            starts.append(0)
            continue
        hits = [i for i, ln in enumerate(lines) if re.match(pat, ln)
                and (look is None or (i + 1 < len(lines)
                    and re.match(look, lines[i + 1])))]
        assert len(hits) == 1, "%s: %d hits" % (name, len(hits))
        starts.append(hits[0])
    assert starts == sorted(starts), "chunk order violated"
    chunks = []
    for k, entry in enumerate(CHUNKS):
        name = entry[0]
        end = starts[k + 1] if k + 1 < len(starts) else len(lines)
        chunks.append((name, lines[starts[k]:end]))
    return chunks


def do_split():
    text = read_root()
    if text.startswith("// GENERATED"):
        sys.exit("refusing to split a generated amalgamation; "
                 "src/puffysics/*.inl is the source of truth")
    chunks = split_chunks(text)
    # Round trip: concatenated bodies must equal the original exactly.
    assert "\n".join(sum([c for _n, c in chunks], [])) == text, \
        "split is not a faithful partition"
    os.makedirs(SRC, exist_ok=True)
    os.makedirs(INCLUDE, exist_ok=True)
    for entry, (_n2, body) in zip(CHUNKS, chunks):
        name, note = entry[0], entry[2]
        out = banner(name, note) + GUARD + "\n".join(body)
        if not out.endswith("\n"):
            out += "\n"
        with open(os.path.join(SRC, name), "wb") as f:
            f.write(out.encode("utf-8"))
    wrapper = WRAPPER_HEAD
    for entry in CHUNKS:
        name = entry[0]
        wrapper += '#include "../../src/puffysics/%s"\n' % name
    wrapper += '#endif // PUFFYSICS_CORE_INCLUDED\n'
    with open(os.path.join(INCLUDE, "puffysics.cuh"), "wb") as f:
        f.write(wrapper.encode("utf-8"))
    print("split %d chunks from %d lines" % (len(chunks), len(text.split("\n"))))


INL_INCLUDE = re.compile(r'^#include "\.\./\.\./src/puffysics/(.+)"$')
ROOT_INCLUDE = re.compile(r'^#include "(b3_art\.cuh|atan_approx\.cuh)"$')
EMBED_GUARD = {"b3_art.cuh": "B3_ART_CUH",
               "atan_approx.cuh": "ATAN_APPROX_CUH"}


def inline_file(path, keep_guard=False):
    with open(path, "rb") as f:
        text = f.read().decode("utf-8")
    lines = text.split("\n")
    out = []
    i = 0
    while i < len(lines):
        ln = lines[i]
        if ln.strip() == "#pragma once":
            i += 1
            continue
        if not keep_guard and ln.strip() == "#ifndef PUFFYSICS_SPLIT_OK" \
                and lines[i + 1].strip() == \
                '#error "Do not include this .inl directly; include puffysics.cuh"' \
                and lines[i + 2].strip() == "#endif":
            i += 3
            continue
        if not keep_guard and ln.strip() in (
                "#ifndef B3_ART_CUH", "#define B3_ART_CUH",
                "#endif // B3_ART_CUH", "#ifndef ATAN_APPROX_CUH",
                "#define ATAN_APPROX_CUH", "#endif // ATAN_APPROX_CUH"):
            i += 1
            continue
        out.append(ln)
        i += 1
    return out


def amalgamate_text():
    with open(os.path.join(INCLUDE, "puffysics.cuh"), "rb") as f:
        wrapper = f.read().decode("utf-8")
    out = [GENERATED_BANNER.rstrip("\n"), ""]
    out.append("#pragma once")
    for ln in wrapper.split("\n"):
        m = INL_INCLUDE.match(ln.strip())
        if m:
            # drop the wrapper's own pragma (already emitted) and defines
            out.extend(inline_file(os.path.join(SRC, m.group(1))))
            continue
        if ln.strip() in ("#pragma once", "#define PUFFYSICS_SPLIT_OK"):
            continue
        if ln.startswith("// PUBLIC API") or ln.startswith("//   ") \
                or ln.startswith("// Topics below"):
            out.append(ln)
            continue
        if ln.startswith("// Puffysics development entry point"):
            continue
        if ln.strip() == "":
            continue
        out.append(ln)
    # resolve root module includes left inside chunks
    final = []
    for ln in out:
        m = ROOT_INCLUDE.match(ln.strip())
        if m:
            tag = EMBED_GUARD[m.group(1)]
            final.append("#ifndef %s" % tag)
            final.append("#define %s" % tag)
            final.extend(inline_file(os.path.join(ROOT, m.group(1))))
            final.append("#endif // %s" % tag)
        else:
            final.append(ln)
    text = "\n".join(final)
    if not text.endswith("\n"):
        text += "\n"
    return text


def do_amalgamate():
    text = amalgamate_text()
    os.makedirs(DIST, exist_ok=True)
    with open(os.path.join(DIST, "puffysics.cuh"), "wb") as f:
        f.write(text.encode("utf-8"))
    with open(os.path.join(ROOT, "puffysics.cuh"), "wb") as f:
        f.write(text.encode("utf-8"))
    print("amalgamated %d lines to dist/ and root" % len(text.split("\n")))

def do_check():
    with tempfile.TemporaryDirectory() as tmp:
        text = read_root()
        chunks = split_chunks(text)
        assert "\n".join(sum([c for _n, c in chunks], [])) == text
        for f in [os.path.join(SRC, entry[0]) for entry in CHUNKS]:
            assert os.path.isfile(f), "missing " + f
        regen = amalgamate_text()
        for target in [os.path.join(DIST, "puffysics.cuh"),
                       os.path.join(ROOT, "puffysics.cuh")]:
            with open(target, "rb") as f:
                assert f.read().decode("utf-8") == regen, \
                    "drift in " + target + " (run tools/amalgamate.py)"
    print("check OK: split faithful, dist/ and root current")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    if cmd == "split":
        do_split()
    elif cmd == "amalgamate":
        do_amalgamate()
    elif cmd == "check":
        do_check()
    else:
        sys.exit("usage: amalgamate.py split|amalgamate|check")
