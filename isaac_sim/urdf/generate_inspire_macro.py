"""Generate inspire_hand.urdf.xacro (a parameterized macro) from the dexsuite
inspire_hand_left.urdf, so the 6-DOF/5-finger hand can be mounted on each arm.

The macro: <xacro:inspire_hand prefix=".." side="left|right" parent="..">
  <origin .../></xacro:inspire_hand>
- drops the standalone "base" link/base_joint (the hand mounts on the arm),
- prefixes every link/joint name (avoids l/r collisions),
- rewrites mesh paths to absolute + by side (left_*/right_*).
"""
from lxml import etree

SRC = "/usr1/home/semathew/r2d3_isaac/isaac_sim/urdf/inspire_hand/inspire_hand_left.urdf"
OUT = "/usr1/home/semathew/r2d3_isaac/isaac_sim/urdf/inspire_hand.urdf.xacro"
ABS = "/usr1/home/semathew/r2d3_isaac/isaac_sim/urdf/inspire_hand"
XACRO = "http://www.ros.org/wiki/xacro"

root = etree.parse(SRC).getroot()
nsmap = {"xacro": XACRO}
new = etree.Element("robot", nsmap=nsmap)
new.set("name", "inspire_hand")
macro = etree.SubElement(new, "{%s}macro" % XACRO)
macro.set("name", "inspire_hand")
macro.set("params", "prefix side parent *origin")

# mount joint: arm hand link -> hand_base_link
mj = etree.SubElement(macro, "joint")
mj.set("name", "${prefix}hand_mount"); mj.set("type", "fixed")
etree.SubElement(mj, "parent").set("link", "${parent}")
etree.SubElement(mj, "child").set("link", "${prefix}hand_base_link")
etree.SubElement(mj, "{%s}insert_block" % XACRO).set("name", "origin")

for el in list(root):
    if el.tag == "link":
        nm = el.get("name")
        if nm == "base":
            continue
        el.set("name", "${prefix}" + nm)
        for mesh in el.iter("mesh"):
            fn = mesh.get("filename")
            fn = fn.replace("meshes/", ABS + "/meshes/").replace("left_", "${side}_")
            mesh.set("filename", fn)
        macro.append(el)
    elif el.tag == "joint":
        nm = el.get("name")
        if nm == "base_joint":
            continue
        el.set("name", "${prefix}" + nm)
        for pc in list(el.iter("parent")) + list(el.iter("child")):
            pc.set("link", "${prefix}" + pc.get("link"))
        for mim in el.iter("mimic"):
            mim.set("joint", "${prefix}" + mim.get("joint"))
        macro.append(el)

etree.ElementTree(new).write(OUT, pretty_print=True, xml_declaration=True, encoding="utf-8")
nlink = sum(1 for e in macro if e.tag == "link")
njoint = sum(1 for e in macro if e.tag == "joint")
print(f"wrote {OUT}: {nlink} links, {njoint} joints (incl mount)")
