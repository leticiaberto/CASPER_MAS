#!/usr/bin/env python3
"""
patch_actor.py — One-time model variant generator.

Run this script ONCE during development (not at launch time) to create
pre-coloured model folders.  Commit the generated folders; Gazebo loads
them as normal static models — no runtime patching needed.

Usage
─────
  # Generate all variants at once
  python3 patch_actor.py --models-dir src/human_adapters/models

  # Or generate a single variant
  python3 patch_actor.py --models-dir src/human_adapters/models \
      --actor-type WalkingActor --color red

    python3 patch_actor.py --models-dir src/human_adapters/models \
        --actor-type FemaleVisitor --color yellow

    python3 patch_actor.py --models-dir src/human_adapters/models \
        --actor-type WalkingActor --color red

    python3 patch_actor.py --models-dir src/human_adapters/models \
        --actor-type CasualFemale --color orange

Colour definitions
──────────────────
Edit the COLORS dict below to add or change variants before running.
"""

import argparse
import os
import re
import shutil
import sys
import xml.etree.ElementTree as ET

COLLADA_NS = "http://www.collada.org/2005/11/COLLADASchema"
ET.register_namespace("", COLLADA_NS)

# ── Define your colour variants here ─────────────────────────────────────────
# WalkingActor: RGB float tuples (baked into the COLLADA effect node)
# CasualFemale / FemaleVisitor: colour name strings that must match existing
#   texture filenames, e.g. female_casualsuit01_diffuse_red.png
COLORS = {
    "WalkingActor": {
        "red":    (0.8, 0.1, 0.1),
        "blue":   (0.1, 0.1, 0.8),
        "yellow": (0.9, 0.8, 0.1),
        "green":  (0.098, 0.255, 0.075),  # matches the original
    },
    "CasualFemale":  ["red", "blue", "yellow", "orange"],
    "FemaleVisitor": ["red", "blue", "yellow", "pink"],
    "FemaleVisitorSit": ["red", "blue", "yellow", "pink"],
    "MaleVisitorSit": ["red", "blue", "yellow", "white"],
}


# ── Per-type patching logic ───────────────────────────────────────────────────

def patch_walking_actor(src_dir, dst_dir, color):
    """
    Patch the sweater colour in walk.dae by finding the effect whose id
    contains 'sweater'. Prints available effect IDs if not found so you
    can identify the correct one and update SWEATER_EFFECT_ID below.
    """
    SWEATER_EFFECT_ID = "sweater"   # matched as a substring of effect id=

    r, g, b = COLORS["WalkingActor"][color]
    color_str = f"{r} {g} {b} 1"

    dae_src = os.path.join(src_dir, "meshes", "walk.dae")
    dae_dst = os.path.join(dst_dir, "meshes", "walk.dae")

    tree = ET.parse(dae_src)
    root = tree.getroot()

    found = False
    all_effect_ids = []
    for effect in root.iter(f"{{{COLLADA_NS}}}effect"):
        eid = effect.get("id") or ""
        all_effect_ids.append(eid)
        if SWEATER_EFFECT_ID in eid:
            for color_el in effect.iter(f"{{{COLLADA_NS}}}color"):
                if color_el.get("sid") in ("ambient", "diffuse"):
                    color_el.text = color_str
            found = True
            print(f"  walk.dae  matched effect id='{eid}' -> R={r:.3f} G={g:.3f} B={b:.3f}")
            break

    if not found:
        print(
            f"  WARNING: no effect id containing '{SWEATER_EFFECT_ID}' found in {dae_src}.\n"
            f"  Available effect ids: {all_effect_ids}\n"
            f"  Update SWEATER_EFFECT_ID in patch_actor.py to match the correct one.",
            file=sys.stderr,
        )

    tree.write(dae_dst, xml_declaration=True, encoding="utf-8")
    with open(dae_dst, "a") as f:
        f.write("\n")


def patch_casual_female(src_dir, dst_dir, color):
    """
    Patch the t-shirt texture path in casual_female.dae.

    The <init_from> may already contain a colour suffix from a previous run
    (e.g. female_casualsuit01_diffuse_orange.png).  Strip any existing suffix
    before applying the new one so re-running is always idempotent.

    Pattern matched:  female_casualsuit01_diffuse[_<anything>].png
    Replaced with:    female_casualsuit01_diffuse_<color>.png
    """
    BASE_NAME = "female_casualsuit01_diffuse"

    dae_src = os.path.join(src_dir, "meshes", "casual_female.dae")
    dae_dst = os.path.join(dst_dir, "meshes", "casual_female.dae")

    tree = ET.parse(dae_src)
    root = tree.getroot()

    found = False
    for image_el in root.iter(f"{{{COLLADA_NS}}}image"):
        if BASE_NAME in (image_el.get("id") or ""):
            init_from = image_el.find(f"{{{COLLADA_NS}}}init_from")
            if init_from is not None and init_from.text and BASE_NAME in init_from.text:
                # Strip any existing colour suffix, then append the new one
                new_path = re.sub(
                    rf"{re.escape(BASE_NAME)}(?:_\w+)?\.png",
                    f"{BASE_NAME}_{color}.png",
                    init_from.text,
                )
                init_from.text = new_path
                found = True
                print(f"  casual_female.dae -> {BASE_NAME}_{color}.png")
                break

    if not found:
        print(
            f"  WARNING: image id containing '{BASE_NAME}' not found in {dae_src}.",
            file=sys.stderr,
        )

    tree.write(dae_dst, xml_declaration=True, encoding="utf-8")
    with open(dae_dst, "a") as f:
        f.write("\n")


def patch_female_visitor(src_dir, dst_dir, color):
    """
    Patch map_Kd in FemaleVisitor.mtl.
    Strips any existing colour suffix so re-running is idempotent.
    """
    mtl_src = os.path.join(src_dir, "meshes", "FemaleVisitor.mtl")
    mtl_dst = os.path.join(dst_dir, "meshes", "FemaleVisitor.mtl")

    with open(mtl_src, encoding="utf-8") as fh:
        text = fh.read()

    # Match: map_Kd FemaleVisitor[_<anything>].png
    new_text, n = re.subn(
        r"(map_Kd\s+FemaleVisitor)(?:_\w+)?\.png",
        rf"\1_{color}.png",
        text,
    )
    if n == 0:
        print(
            f"  WARNING: 'map_Kd FemaleVisitor*.png' not found in {mtl_src}.",
            file=sys.stderr,
        )

    with open(mtl_dst, "w", encoding="utf-8") as fh:
        fh.write(new_text)
    print(f"  FemaleVisitor.mtl -> map_Kd FemaleVisitor_{color}.png")


def patch_female_visitor_sit(src_dir, dst_dir, color):
    """
    Patch map_Kd in FemaleVisitorSit.mtl.
    Strips any existing colour suffix so re-running is idempotent.
    """
    mtl_src = os.path.join(src_dir, "meshes", "FemaleVisitorSit.mtl")
    mtl_dst = os.path.join(dst_dir, "meshes", "FemaleVisitorSit.mtl")

    with open(mtl_src, encoding="utf-8") as fh:
        text = fh.read()

    # Match: map_Kd FemaleVisitorSit[_<anything>].png
    new_text, n = re.subn(
        r"(map_Kd\s+FemaleVisitorSit)(?:_\w+)?\.png",
        rf"\1_{color}.png",
        text,
    )
    if n == 0:
        print(
            f"  WARNING: 'map_Kd FemaleVisitorSit*.png' not found in {mtl_src}.",
            file=sys.stderr,
        )

    with open(mtl_dst, "w", encoding="utf-8") as fh:
        fh.write(new_text)
    print(f"  FemaleVisitorSit.mtl -> map_Kd FemaleVisitorSit_{color}.png")

def patch_male_visitor_sit(src_dir, dst_dir, color):
    """
    Patch map_Kd in MaleVisitorSit.mtl.
    Strips any existing colour suffix so re-running is idempotent.
    """
    mtl_src = os.path.join(src_dir, "meshes", "MaleVisitorSit.mtl")
    mtl_dst = os.path.join(dst_dir, "meshes", "MaleVisitorSit.mtl")

    with open(mtl_src, encoding="utf-8") as fh:
        text = fh.read()

    # Match: map_Kd MaleVisitorSit[_<anything>].png
    new_text, n = re.subn(
        r"(map_Kd\s+MaleVisitorSit)(?:_\w+)?\.png",
        rf"\1_{color}.png",
        text,
    )
    if n == 0:
        print(
            f"  WARNING: 'map_Kd MaleVisitorSit*.png' not found in {mtl_src}.",
            file=sys.stderr,
        )

    with open(mtl_dst, "w", encoding="utf-8") as fh:
        fh.write(new_text)
    print(f"  MaleVisitorSit.mtl -> map_Kd MaleVisitorSit_{color}.png")

# ── SDF + config patching ─────────────────────────────────────────────────────

def patch_sdf(src_dir, dst_dir, actor_type, model_variant):
    sdf_path = os.path.join(src_dir, "model.sdf")
    with open(sdf_path, encoding="utf-8") as fh:
        sdf = fh.read()

    sdf = sdf.replace(f"model://{actor_type}/", f"model://{model_variant}/")
    sdf = sdf.replace(f'name="{actor_type}"',   f'name="{model_variant}"')

    with open(os.path.join(dst_dir, "model.sdf"), "w", encoding="utf-8") as fh:
        fh.write(sdf)
    print(f"  model.sdf -> model://{model_variant}/...")


def patch_model_config(dst_dir, model_variant):
    config_path = os.path.join(dst_dir, "model.config")
    if not os.path.exists(config_path):
        return
    with open(config_path, encoding="utf-8") as fh:
        text = fh.read()
    text = re.sub(r"<name>.*?</name>", f"<name>{model_variant}</name>", text)
    with open(config_path, "w", encoding="utf-8") as fh:
        fh.write(text)


# ── Main generation logic ─────────────────────────────────────────────────────

PATCHERS = {
    "WalkingActor": patch_walking_actor,
    "CasualFemale": patch_casual_female,
    "FemaleVisitor": patch_female_visitor,
    "FemaleVisitorSit": patch_female_visitor_sit,  
    "MaleVisitorSit": patch_male_visitor_sit,
}


def generate_variant(models_dir, actor_type, color):
    src_dir       = os.path.join(models_dir, actor_type)
    model_variant = f"{actor_type}_{color}"
    dst_dir       = os.path.join(models_dir, model_variant)

    if not os.path.isdir(src_dir):
        print(f"ERROR: source model dir not found: {src_dir}", file=sys.stderr)
        sys.exit(1)

    print(f"Generating {model_variant} ...")

    if os.path.exists(dst_dir):
        shutil.rmtree(dst_dir)
    shutil.copytree(src_dir, dst_dir)

    PATCHERS[actor_type](src_dir, dst_dir, color)
    patch_sdf(src_dir, dst_dir, actor_type, model_variant)
    patch_model_config(dst_dir, model_variant)

    print(f"  -> {dst_dir}\n")


def main():
    parser = argparse.ArgumentParser(
        description="Generate pre-coloured Gazebo model variants (run once, then build)."
    )
    parser.add_argument(
        "--models-dir", required=True,
        help="Path to the models/ directory inside your ROS package source.",
    )
    parser.add_argument(
        "--actor-type",
        choices=list(PATCHERS.keys()),
        help="Generate variants for one actor type only (default: all).",
    )
    parser.add_argument(
        "--color",
        help="Generate a single colour variant (requires --actor-type).",
    )
    args = parser.parse_args()

    if args.color and not args.actor_type:
        parser.error("--color requires --actor-type")

    types_to_generate = [args.actor_type] if args.actor_type else list(PATCHERS.keys())

    for actor_type in types_to_generate:
        color_source = COLORS[actor_type]
        available = list(color_source.keys()) if isinstance(color_source, dict) else color_source
        colors = [args.color] if args.color else available

        for color in colors:
            generate_variant(models_dir=args.models_dir, actor_type=actor_type, color=color)

    print("Done. Run  colcon build  to install the new model folders.")


if __name__ == "__main__":
    main()
