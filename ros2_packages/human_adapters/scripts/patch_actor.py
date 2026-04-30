#!/usr/bin/env python3
"""
patch_actor.py — Patch walk.dae at spawn time (shirt colour).

Usage:
    python3 patch_actor.py \
        --input  walk.dae \
        --output agent_red.dae \
        --shirt-color 0.8 0.1 0.1
"""

import argparse
import sys
import xml.etree.ElementTree as ET

COLLADA_NS = "http://www.collada.org/2005/11/COLLADASchema"
ET.register_namespace("", COLLADA_NS)


def set_shirt_color(root, r: float, g: float, b: float):
    color_str = f"{r} {g} {b} 1"
    for effect in root.iter(f"{{{COLLADA_NS}}}effect"):
        if effect.get("id") == "sweater-green-effect":
            for color_el in effect.iter(f"{{{COLLADA_NS}}}color"):
                if color_el.get("sid") in ("ambient", "diffuse"):
                    color_el.text = color_str
            return
    print("WARNING: sweater-green-effect not found.", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input",  required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--shirt-color", nargs=3, type=float,
                        metavar=("R", "G", "B"), required=True)
    args = parser.parse_args()

    tree = ET.parse(args.input)
    root = tree.getroot()

    r, g, b = args.shirt_color
    print(f"[patch_actor] shirt color -> R={r:.3f} G={g:.3f} B={b:.3f}")
    set_shirt_color(root, r, g, b)

    tree.write(args.output, xml_declaration=True, encoding="utf-8")

    # ET strips the trailing newline — add it back so Gazebo's COLLADA
    # parser doesn't reject the file.
    with open(args.output, "a") as f:
        f.write("\n")

    print(f"[patch_actor] written: {args.output}")


if __name__ == "__main__":
    main()
