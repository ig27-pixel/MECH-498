"""Export RoboRoll pitch deck assets to PPTX.

This writes a lightweight, editable PowerPoint file without external packages.
The HTML deck remains the source for browser presenting; this PPTX is a
submission-friendly copy with the same story and generated mockup images.
"""

from __future__ import annotations

import html
import shutil
import zipfile
from pathlib import Path

from PIL import Image


ROOT = Path(__file__).resolve().parent
IMAGE_DIR = ROOT / "images"
OUT = ROOT / "roboroll_pitch_deck.pptx"
EMU = 914400
SLIDE_W = int(13.333333 * EMU)
SLIDE_H = int(7.5 * EMU)


def emu(inches: float) -> int:
    return int(inches * EMU)


def esc(text: str) -> str:
    return html.escape(text, quote=False)


SLIDES = [
    {
        "kind": "hero",
        "title": "RoboRoll Coatings",
        "subtitle": "Robotic wall painting for faster, safer, more repeatable interior finishing.",
        "image": "mockup_robot_hero.png",
    },
    {
        "eyebrow": "1. The problem",
        "title": "Interior painting is still a manual bottleneck.",
        "bullets": [
            "Repetitive wall coverage consumes crew hours.",
            "Quality changes with fatigue, reach, lighting, and setup consistency.",
            "Labor bottlenecks delay final construction turnover.",
        ],
        "image": "mockup_problem_scene.png",
    },
    {
        "eyebrow": "2. Why it matters",
        "title": "Construction needs focused automation that helps crews now.",
        "bullets": [
            "Painting appears across apartments, commercial spaces, and new builds.",
            "Large wall areas create predictable paths that a robot can execute.",
            "Fewer rework passes mean more predictable finishing timelines.",
        ],
        "image": "mockup_market_segments.png",
    },
    {
        "eyebrow": "3. The solution",
        "title": "A portable 4-DOF robot that follows planned wall paths.",
        "bullets": [
            "Base yaw aims the arm around the room.",
            "Shoulder, elbow, and wrist place the nozzle on target.",
            "Inverse kinematics converts wall points into smooth joint motion.",
        ],
        "image": "mockup_robot_architecture.png",
    },
    {
        "eyebrow": "4. Product demo",
        "title": "The simulated robot already paints across two walls.",
        "bullets": [
            "Wall 1: five horizontal color stripes.",
            "Wall 2: face outline, eyes, and smile arc.",
            "Curved paths prove more than straight-line coverage.",
        ],
        "images": ["mockup_product_lines.png", "mockup_product_smiley.png"],
    },
    {
        "eyebrow": "5. Why we win",
        "title": "RoboRoll starts with one focused construction task.",
        "bullets": [
            "Interior wall finishing is narrow enough for a first product.",
            "FK, IK, trajectory generation, and dynamics are already simulated.",
            "Seed funding turns the validated simulation into a portable prototype.",
        ],
        "image": "mockup_roadmap.png",
    },
    {"kind": "thanks", "title": "Thank You", "subtitle": "RoboRoll Coatings\nRobotic painting for the next generation of construction crews."},
    {"kind": "divider", "title": "Leave-Behind Slides", "subtitle": "The following slides provide the fuller technical, market, and roadmap story."},
    {
        "eyebrow": "A1. Beachhead market",
        "title": "Start with multifamily apartment turns.",
        "bullets": [
            "Similar room layouts create repeatable painting paths.",
            "Turnover schedules are tight, and labor availability affects move-in dates.",
            "Commercial interiors and new-build finishing are expansion markets.",
        ],
        "image": "mockup_market_segments.png",
    },
    {
        "eyebrow": "A2. Customer workflow",
        "title": "RoboRoll helps one operator supervise repeatable wall coverage.",
        "bullets": ["Position", "Calibrate", "Paint", "Verify"],
        "image": "mockup_robot_workflow.png",
    },
    {
        "eyebrow": "A3. Robot architecture",
        "title": "The first design is intentionally simple.",
        "bullets": [
            "4 revolute joints: base yaw, shoulder pitch, elbow pitch, wrist pitch.",
            "500 mm base height with 700 mm and 500 mm arm links.",
            "200 mm nozzle offset for reaching the wall surface.",
        ],
        "image": "mockup_nozzle_closeup.png",
    },
    {
        "eyebrow": "A4. Kinematics proof",
        "title": "The robot can turn wall targets into joint commands.",
        "bullets": [
            "Forward kinematics verifies the end-effector pose.",
            "Inverse kinematics solves reachable wall targets.",
            "Joint limits prevent unrealistic poses during planning.",
        ],
        "image": "mockup_calibration.png",
    },
    {
        "eyebrow": "A5. Demo path details",
        "title": "The demo combines coverage and detail work.",
        "bullets": [
            "343 rendered frames.",
            "Smoothstep interpolation between IK waypoints.",
            "Lift motions prevent unwanted diagonal paint marks.",
        ],
        "image": "mockup_path_planning.png",
    },
    {
        "eyebrow": "A6. Dynamics proof",
        "title": "The simulation includes physical reasoning.",
        "bullets": [
            "Passive motion checks kinetic, potential, and total energy behavior.",
            "PD plus gravity feedforward controls motion to a painting target.",
            "Torque and velocity plots support physically reasonable motion.",
        ],
        "image": "mockup_control_dashboard.png",
    },
    {
        "eyebrow": "A7. Development roadmap",
        "title": "The next milestones move from simulation to hardware.",
        "bullets": ["Paint delivery and nozzle control.", "Wall sensing and registration.", "Drywall repeatability testing.", "Operator setup workflow."],
        "image": "mockup_roadmap.png",
    },
    {
        "eyebrow": "A8. Risks and mitigations",
        "title": "The key risks are practical and testable.",
        "bullets": [
            "Surface quality: test nozzle spacing, flow rate, and overlap.",
            "Wall localization: add sensing and calibration before each room run.",
            "Setup time: design a simple operator workflow.",
            "Safety: limit speed and define a supervised work envelope.",
        ],
        "image": "mockup_calibration.png",
    },
    {
        "eyebrow": "A9. Funding use",
        "title": "Funding turns the validated simulation into a prototype.",
        "bullets": ["Build a first portable mechanism.", "Integrate paint delivery hardware.", "Add sensing and calibration.", "Run repeatability and finish-quality trials."],
        "image": "mockup_roadmap.png",
    },
    {
        "kind": "closing",
        "title": "A focused wedge into construction automation.",
        "subtitle": "RoboRoll begins with repeatable interior painting, proves value in controlled room-scale tasks, and expands toward broader finishing automation.",
    },
]


def text_shape(idx, x, y, w, h, text, size=28, color="17202A", bold=False):
    return f"""
      <p:sp>
        <p:nvSpPr><p:cNvPr id="{idx}" name="Text {idx}"/><p:cNvSpPr txBox="1"/><p:nvPr/></p:nvSpPr>
        <p:spPr><a:xfrm><a:off x="{x}" y="{y}"/><a:ext cx="{w}" cy="{h}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom><a:noFill/><a:ln><a:noFill/></a:ln></p:spPr>
        <p:txBody>
          <a:bodyPr wrap="square"/><a:lstStyle/>
          <a:p><a:r><a:rPr lang="en-US" sz="{size * 100}" b="{1 if bold else 0}"><a:solidFill><a:srgbClr val="{color}"/></a:solidFill></a:rPr><a:t>{esc(text)}</a:t></a:r></a:p>
        </p:txBody>
      </p:sp>"""


def bullets_shape(idx, x, y, w, h, bullets):
    paras = []
    for bullet in bullets:
        paras.append(
            f'<a:p><a:pPr marL="342900" indent="-228600"><a:buChar char="•"/></a:pPr>'
            f'<a:r><a:rPr lang="en-US" sz="2000"><a:solidFill><a:srgbClr val="334155"/></a:solidFill></a:rPr><a:t>{esc(bullet)}</a:t></a:r></a:p>'
        )
    return f"""
      <p:sp>
        <p:nvSpPr><p:cNvPr id="{idx}" name="Bullets {idx}"/><p:cNvSpPr txBox="1"/><p:nvPr/></p:nvSpPr>
        <p:spPr><a:xfrm><a:off x="{x}" y="{y}"/><a:ext cx="{w}" cy="{h}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom><a:noFill/><a:ln><a:noFill/></a:ln></p:spPr>
        <p:txBody><a:bodyPr wrap="square"/><a:lstStyle/>{''.join(paras)}</p:txBody>
      </p:sp>"""


def image_shape(idx, rel_id, x, y, w, h):
    return f"""
      <p:pic>
        <p:nvPicPr><p:cNvPr id="{idx}" name="Picture {idx}"/><p:cNvPicPr/><p:nvPr/></p:nvPicPr>
        <p:blipFill><a:blip r:embed="{rel_id}"/><a:stretch><a:fillRect/></a:stretch></p:blipFill>
        <p:spPr><a:xfrm><a:off x="{x}" y="{y}"/><a:ext cx="{w}" cy="{h}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom></p:spPr>
      </p:pic>"""


def image_box(path: Path, x, y, max_w, max_h):
    with Image.open(path) as img:
        iw, ih = img.size
    scale = min(max_w / iw, max_h / ih)
    w = int(iw * scale)
    h = int(ih * scale)
    return x + (max_w - w) // 2, y + (max_h - h) // 2, w, h


def slide_xml(slide, slide_num, media_map):
    shapes = []
    rels = []
    shape_id = 2
    bg = "F8FAFC"
    if slide.get("kind") in {"thanks", "divider", "closing"}:
        bg = "111827"

    shapes.append(f'<p:bg><p:bgPr><a:solidFill><a:srgbClr val="{bg}"/></a:solidFill><a:effectLst/></p:bgPr></p:bg>')

    if slide.get("kind") == "hero":
        img = slide["image"]
        rels.append((f"rId{len(rels)+1}", media_map[img]))
        shapes.append(image_shape(shape_id, rels[-1][0], 0, 0, SLIDE_W, SLIDE_H))
        shape_id += 1
        shapes.append(text_shape(shape_id, emu(0.7), emu(4.35), emu(6.8), emu(0.9), slide["title"], 46, "FFFFFF", True))
        shape_id += 1
        shapes.append(text_shape(shape_id, emu(0.75), emu(5.25), emu(6.6), emu(0.8), slide["subtitle"], 22, "E5EDF7"))
    elif slide.get("kind") in {"thanks", "divider", "closing"}:
        shapes.append(text_shape(shape_id, emu(1.25), emu(2.45), emu(10.8), emu(0.9), slide["title"], 44, "FFFFFF", True))
        shape_id += 1
        shapes.append(text_shape(shape_id, emu(1.55), emu(3.45), emu(10.2), emu(1.4), slide["subtitle"], 22, "E5EDF7"))
    else:
        shapes.append(text_shape(shape_id, emu(0.65), emu(0.5), emu(5.6), emu(0.35), slide["eyebrow"], 12, "1F6FEB", True))
        shape_id += 1
        shapes.append(text_shape(shape_id, emu(0.65), emu(0.92), emu(5.8), emu(1.6), slide["title"], 30, "17202A", True))
        shape_id += 1
        shapes.append(bullets_shape(shape_id, emu(0.68), emu(2.75), emu(5.6), emu(3.7), slide["bullets"]))
        shape_id += 1
        if "images" in slide:
            for row, img in enumerate(slide["images"]):
                rels.append((f"rId{len(rels)+1}", media_map[img]))
                x, y, w, h = image_box(IMAGE_DIR / img, emu(6.65), emu(0.55 + row * 3.25), emu(5.95), emu(2.8))
                shapes.append(image_shape(shape_id, rels[-1][0], x, y, w, h))
                shape_id += 1
        else:
            img = slide["image"]
            rels.append((f"rId{len(rels)+1}", media_map[img]))
            x, y, w, h = image_box(IMAGE_DIR / img, emu(6.65), emu(0.85), emu(5.95), emu(5.8))
            shapes.append(image_shape(shape_id, rels[-1][0], x, y, w, h))

    xml = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sld xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
  <p:cSld>
    {shapes[0]}
    <p:spTree>
      <p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>
      <p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>
      {''.join(shapes[1:])}
    </p:spTree>
  </p:cSld>
  <p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>
</p:sld>'''
    rel_xml = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
    for rid, target in rels:
        rel_xml += f'<Relationship Id="{rid}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="../media/{target}"/>'
    rel_xml += "</Relationships>"
    return xml, rel_xml


def build_pptx():
    media_names = sorted({img for slide in SLIDES for img in ([slide["image"]] if "image" in slide else slide.get("images", []))})
    media_map = {name: f"image{i+1}.png" for i, name in enumerate(media_names)}

    with zipfile.ZipFile(OUT, "w", compression=zipfile.ZIP_DEFLATED) as z:
        content = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Default Extension="png" ContentType="image/png"/>
  <Override PartName="/ppt/presentation.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.presentation.main+xml"/>
'''
        for i in range(1, len(SLIDES) + 1):
            content += f'  <Override PartName="/ppt/slides/slide{i}.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slide+xml"/>\n'
        content += "</Types>"
        z.writestr("[Content_Types].xml", content)
        z.writestr("_rels/.rels", '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="ppt/presentation.xml"/></Relationships>''')

        pres = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:presentation xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
  <p:sldIdLst>{''.join(f'<p:sldId id="{256+i}" r:id="rId{i}"/>' for i in range(1, len(SLIDES)+1))}</p:sldIdLst>
  <p:sldSz cx="{SLIDE_W}" cy="{SLIDE_H}" type="wide"/>
  <p:notesSz cx="6858000" cy="9144000"/>
</p:presentation>'''
        z.writestr("ppt/presentation.xml", pres)
        pres_rels = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        for i in range(1, len(SLIDES) + 1):
            pres_rels += f'<Relationship Id="rId{i}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slide" Target="slides/slide{i}.xml"/>'
        pres_rels += "</Relationships>"
        z.writestr("ppt/_rels/presentation.xml.rels", pres_rels)

        for name, target in media_map.items():
            z.write(IMAGE_DIR / name, f"ppt/media/{target}")

        for i, slide in enumerate(SLIDES, start=1):
            xml, rel_xml = slide_xml(slide, i, media_map)
            z.writestr(f"ppt/slides/slide{i}.xml", xml)
            z.writestr(f"ppt/slides/_rels/slide{i}.xml.rels", rel_xml)

    print(OUT)


if __name__ == "__main__":
    if OUT.exists():
        OUT.unlink()
    build_pptx()
