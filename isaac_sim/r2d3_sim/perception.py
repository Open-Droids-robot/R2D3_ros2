"""Open-vocabulary object detection (OWL-ViT) for the R2D3 head camera.

A COCO-pretrained detector (torchvision) does not recognise the simulated YCB
props — they fall outside its training distribution (a red mug classifies as
'chair'). OWL-ViT is CLIP-based and open-vocabulary, so it localises objects
from a text query ("a red mug") and transfers to the sim render.

`transformers`/`torch` segfault the Isaac **kit** process if imported in-process
(threading/CUDA-context conflict), so `detect()` runs the model in a clean
**subprocess** (this same file's ``__main__``) and reads the boxes back as JSON.
This keeps perception fully isolated from the simulator — a "perception service"
split. Needs ``pip install transformers`` (weights auto-download, ~600 MB, cached).

    from isaac_sim.r2d3_sim import perception
    dets = perception.detect(rgb, ["a red mug", "a bowl"])   # [(label, score, [x0,y0,x1,y1]), ...]
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile

import numpy as np

MODEL = "google/owlv2-base-patch16-ensemble"


def detect(rgb, queries, *, model: str = MODEL, threshold: float = 0.05):
    """Detect `queries` in an RGB frame (uint8 HxWx3). Returns a score-sorted list
    of (label, score, box[x0,y0,x1,y1]) — runs OWL-ViT in an isolated subprocess."""
    from PIL import Image
    d = tempfile.mkdtemp(prefix="r2d3_perc_")
    img_p = os.path.join(d, "rgb.png")
    out_p = os.path.join(d, "det.json")
    Image.fromarray(np.ascontiguousarray(rgb)).save(img_p)
    subprocess.run([sys.executable, __file__, img_p, out_p, model, str(threshold), "|".join(queries)],
                   check=True)
    with open(out_p) as f:
        raw = json.load(f)
    return [(x["label"], float(x["score"]), np.asarray(x["box"], dtype=float)) for x in raw]


def _worker(img_p: str, out_p: str, model_id: str, threshold: float, queries):
    """Subprocess entry: load OWL-ViT, detect, write JSON. Imports torch/transformers
    here so the parent (which may be the Isaac kit process) never imports them."""
    import torch
    from PIL import Image
    from transformers import Owlv2ForObjectDetection, Owlv2Processor

    img = Image.open(img_p).convert("RGB")
    W, H = img.size
    proc = Owlv2Processor.from_pretrained(model_id)
    model = Owlv2ForObjectDetection.from_pretrained(model_id).eval()
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    model = model.to(dev)
    inp = proc(text=[list(queries)], images=img, return_tensors="pt").to(dev)
    with torch.no_grad():
        out = model(**inp)
    res = proc.post_process_grounded_object_detection(
        out, threshold=threshold, target_sizes=torch.tensor([[H, W]]).to(dev))[0]
    boxes = res["boxes"].cpu().numpy()
    scores = res["scores"].cpu().numpy()
    if "text_labels" in res and res["text_labels"] is not None:
        names = list(res["text_labels"])
    else:
        names = [queries[int(i)] for i in res["labels"].cpu().numpy()]
    dets = [{"label": str(nm), "score": float(sc), "box": [float(b) for b in box]}
            for box, sc, nm in zip(boxes, scores, names)]
    dets.sort(key=lambda x: -x["score"])
    with open(out_p, "w") as f:
        json.dump(dets, f)


if __name__ == "__main__":
    _img_p, _out_p, _model_id, _thr, _qstr = sys.argv[1:6]
    _worker(_img_p, _out_p, _model_id, float(_thr), _qstr.split("|"))
