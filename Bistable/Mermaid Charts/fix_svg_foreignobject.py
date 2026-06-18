#!/usr/bin/env python3
"""Replace SVG foreignObject elements with native SVG text elements for Typst compatibility."""

import sys
import os
from lxml import etree

SVG_NS = "http://www.w3.org/2000/svg"
XLINK_NS = "http://www.w3.org/1999/xlink"

def get_text_lines(fo):
    return [t.strip() for t in fo.itertext() if t.strip()]

def make_text_element(fo):
    width = float(fo.get("width", "100"))
    height = float(fo.get("height", "20"))
    lines = get_text_lines(fo)
    if not lines:
        return None

    cx = width / 2
    el = etree.Element("{%s}text" % SVG_NS)
    el.set("text-anchor", "middle")
    el.set("dominant-baseline", "middle")

    if len(lines) == 1:
        el.set("x", str(cx))
        el.set("y", str(height / 2))
        el.text = lines[0]
    else:
        line_h = height / len(lines)
        el.set("x", str(cx))
        el.set("y", str(line_h / 2))
        for i, line in enumerate(lines):
            tspan = etree.SubElement(el, "{%s}tspan" % SVG_NS)
            tspan.set("x", str(cx))
            tspan.set("dy", "0" if i == 0 else str(line_h))
            tspan.text = line

    return el

def fix_svg(path):
    parser = etree.XMLParser(remove_blank_text=False)
    tree = etree.parse(path, parser)
    root = tree.getroot()

    fos = root.findall(".//{%s}foreignObject" % SVG_NS)
    replaced = 0
    for fo in fos:
        parent = fo.getparent()
        text_el = make_text_element(fo)
        if text_el is None:
            continue
        idx = list(parent).index(fo)
        parent.remove(fo)
        parent.insert(idx, text_el)
        replaced += 1

    tree.write(path, xml_declaration=True, encoding="UTF-8", pretty_print=False)
    print(f"  {os.path.basename(path)}: replaced {replaced} foreignObject(s)")

if __name__ == "__main__":
    svg_dir = os.path.dirname(os.path.abspath(__file__))
    svgs = [
        "LockingMethods.svg",
        "FastnerMaterial.svg",
        "OverConstrain.svg",
        "ShaftFasterer DecisionTree.svg",
    ]
    for name in svgs:
        fix_svg(os.path.join(svg_dir, name))
    print("Done.")
