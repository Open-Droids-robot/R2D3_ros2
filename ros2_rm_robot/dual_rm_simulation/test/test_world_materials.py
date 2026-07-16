"""Regression guard: every visual material in the Gz world files must define
``<diffuse>``.

gz-sim Harmonic's ogre2 renderer is PBR: ``<diffuse>`` is the albedo, and any
unspecified SDF material color defaults to black (0 0 0 1). A material that
only sets ``<ambient>`` therefore renders BLACK under a directional light --
the walls/boxes/cylinder in nav_empty.sdf all looked black in the simulator
while the ground plane (which defines diffuse) rendered fine.
"""
import unittest
from pathlib import Path
from xml.dom import minidom

WORLDS_DIR = Path(__file__).resolve().parent.parent / "worlds"


class TestWorldMaterials(unittest.TestCase):
    def test_visual_materials_define_diffuse(self):
        world_files = sorted(WORLDS_DIR.glob("*.sdf"))
        self.assertTrue(world_files, f"no world files found in {WORLDS_DIR}")
        missing = []
        for wf in world_files:
            dom = minidom.parse(str(wf))
            for visual in dom.getElementsByTagName("visual"):
                for mat in visual.getElementsByTagName("material"):
                    if not mat.getElementsByTagName("diffuse"):
                        model = visual.parentNode.parentNode  # link -> model
                        name = model.getAttribute("name") or "?"
                        missing.append(f"{wf.name}: model '{name}' "
                                       f"visual '{visual.getAttribute('name')}'")
        self.assertEqual(missing, [],
                         "materials without <diffuse> render BLACK under "
                         "ogre2 PBR (albedo defaults to 0 0 0):\n"
                         + "\n".join(missing))


if __name__ == "__main__":
    unittest.main()
