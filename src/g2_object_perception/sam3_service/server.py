"""SAM 3 分割服务:启动时加载一次模型,之后通过本机 HTTP 接收图片 + 英文短语,返回每个实例的掩码。

为什么单独做成服务:SAM 3 要 Python 3.12,ROS 2 Humble 是 Python 3.10,两者不能在一个进程里 import。

    cd ~/sam3_service
    .venv/bin/python server.py                    # 默认 127.0.0.1:8765
    .venv/bin/python server.py --threshold 0.4    # 放宽置信度

接口(都是 JSON):
    GET  /health   → {"ok": true, "device": ..., "load_s": ..., "gpu_mem_mb": ...}
    POST /segment  请求 {"image_jpg_b64": "...", "prompts": ["green pepper", "red block"], "threshold": 0.5}
                   或者:Content-Type: image/jpeg,请求体 = JPEG 原始字节,
                         头 X-Sam3-Request: {"prompts": [...], "threshold": 0.5}(实时用,不走 base64)
                   回复 {"ok": true, "width": W, "height": H,
                         "results": [{"prompt": "green pepper",
                                      "instances": [{"score": 0.91, "box_xyxy": [x0, y0, x1, y1],
                                                     "area_px": 1234, "mask_png_b64": "..."}]}],
                         "timing_ms": {"decode": .., "image": .., "text": [..], "encode": ..}}
同一张图的多个短语只算一次图像特征,所以一次请求里放多个短语比分开请求快。
"""
import argparse
import base64
import io
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# 本机终端设的是 all_proxy=socks://...,huggingface_hub 不认 socks://(只认 socks5:// 或 http://)。
# Clash 的 7897 端口也收 http,所以换个写法。只影响这个进程,不改终端。
for key in ("all_proxy", "ALL_PROXY"):
    if os.environ.get(key, "").startswith("socks://"):
        os.environ[key] = "http://" + os.environ[key][len("socks://"):]

import numpy as np
import torch
from huggingface_hub import try_to_load_from_cache
from PIL import Image

from sam3.model.sam3_image_processor import Sam3Processor
from sam3.model_builder import build_sam3_image_model

MAX_BODY_BYTES = 20 * 1024 * 1024       # 一张 JPEG 远小于这个,防止误传巨大文件
MAX_PROMPTS = 16


def load_model(threshold):
    t0 = time.monotonic()
    # 权重已经在 ~/.cache/huggingface 里:直接把路径交给 SAM 3,启动完全不联网。
    # 不这么做的话,build_sam3_image_model() 每次启动都会先联网问一下有没有新版本(HF_HUB_OFFLINE 对它无效)。
    cached = try_to_load_from_cache("facebook/sam3", "sam3.pt")
    if isinstance(cached, str):
        print(f"[sam3] 用本机权重 {cached}", flush=True)
        model = build_sam3_image_model(checkpoint_path=cached)
    else:
        print("[sam3] 本机没有权重,从 Hugging Face 下载(要先 hf auth login,约 3.4GB)", flush=True)
        model = build_sam3_image_model()
    processor = Sam3Processor(model, confidence_threshold=threshold)
    return processor, time.monotonic() - t0


def encode_mask(mask):
    """布尔掩码 → PNG(单通道 0/255)→ base64。PNG 对掩码压缩率很高。"""
    buf = io.BytesIO()
    Image.fromarray(mask.astype(np.uint8) * 255).save(buf, format="PNG", optimize=False)
    return base64.b64encode(buf.getvalue()).decode("ascii")


class Segmenter:
    def __init__(self, threshold, bf16):
        self.processor, self.load_s = load_model(threshold)
        self.default_threshold = threshold
        self.bf16 = bf16 and torch.cuda.is_available()
        self.lock = threading.Lock()                 # 只有一块 GPU:请求一个一个来

    def segment(self, image, prompts, threshold):
        timing = {}
        with self.lock, torch.autocast("cuda", dtype=torch.bfloat16, enabled=self.bf16):
            self.processor.confidence_threshold = threshold
            t0 = time.monotonic()
            state = self.processor.set_image(image)
            timing["image"] = round((time.monotonic() - t0) * 1000, 1)
            timing["text"] = []
            results = []
            for prompt in prompts:
                t1 = time.monotonic()
                out = self.processor.set_text_prompt(prompt=prompt, state=state)
                masks = out["masks"].squeeze(1).cpu().numpy()          # (N, H, W) bool
                boxes = out["boxes"].float().cpu().numpy()             # (N, 4) 像素 x0 y0 x1 y1
                scores = out["scores"].float().cpu().numpy()           # (N,)
                timing["text"].append(round((time.monotonic() - t1) * 1000, 1))
                order = np.argsort(-scores)
                results.append({"prompt": prompt, "instances": [
                    {"score": round(float(scores[i]), 4),
                     "box_xyxy": [round(float(v), 1) for v in boxes[i]],
                     "area_px": int(masks[i].sum()),
                     "mask": masks[i]} for i in order]})
            self.processor.reset_all_prompts(state)
        return results, timing

    def gpu_mem_mb(self):
        if not torch.cuda.is_available():
            return None
        return {"allocated": round(torch.cuda.memory_allocated() / 2**20),
                "peak": round(torch.cuda.max_memory_allocated() / 2**20)}


def make_handler(seg):
    class Handler(BaseHTTPRequestHandler):
        def _reply(self, code, body):
            data = json.dumps(body, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path != "/health":
                return self._reply(404, {"ok": False, "error": "只有 GET /health 和 POST /segment"})
            self._reply(200, {"ok": True, "model": "sam3", "device": str(seg.processor.device),
                              "bf16": seg.bf16, "load_s": round(seg.load_s, 1),
                              "threshold": seg.default_threshold, "gpu_mem_mb": seg.gpu_mem_mb()})

        def do_POST(self):
            if self.path != "/segment":
                return self._reply(404, {"ok": False, "error": "只有 GET /health 和 POST /segment"})
            try:
                length = int(self.headers.get("Content-Length", 0))
                if not 0 < length <= MAX_BODY_BYTES:
                    return self._reply(413, {"ok": False, "error": f"请求体大小 {length} 不合法"})
                body = self.rfile.read(length)
                if self.headers.get("Content-Type", "").startswith("image/jpeg"):
                    # 实时用:请求体就是 JPEG 原始字节,短语放在头里,省掉 base64(+33% 体积和一次拷贝)
                    req = json.loads(self.headers.get("X-Sam3-Request", "{}"))
                    jpg = body
                else:
                    req = json.loads(body)
                    jpg = base64.b64decode(req["image_jpg_b64"])
                prompts = req.get("prompts") or ([req["prompt"]] if req.get("prompt") else [])
                if not prompts or len(prompts) > MAX_PROMPTS or not all(isinstance(p, str) and p.strip() for p in prompts):
                    return self._reply(400, {"ok": False, "error": f"prompts 要是 1~{MAX_PROMPTS} 个非空英文短语"})
                threshold = float(req.get("threshold", seg.default_threshold))

                t0 = time.monotonic()
                image = Image.open(io.BytesIO(jpg)).convert("RGB")
                decode_ms = round((time.monotonic() - t0) * 1000, 1)

                results, timing = seg.segment(image, [p.strip() for p in prompts], threshold)
                t1 = time.monotonic()
                for r in results:
                    for inst in r["instances"]:
                        inst["mask_png_b64"] = encode_mask(inst.pop("mask"))
                timing.update(decode=decode_ms, encode=round((time.monotonic() - t1) * 1000, 1))
                self._reply(200, {"ok": True, "width": image.width, "height": image.height,
                                  "threshold": threshold, "results": results, "timing_ms": timing})
            except Exception as e:                       # 坏图片、缺字段、显存不够……都回错误,服务不退出
                self._reply(500, {"ok": False, "error": f"{type(e).__name__}: {e}"})

        def log_message(self, fmt, *args):               # 一行一个请求,不刷屏
            print(f"[sam3] {self.address_string()} {fmt % args}", flush=True)

    return Handler


def main():
    parser = argparse.ArgumentParser(description="SAM 3 本机分割服务")
    parser.add_argument("--host", default="127.0.0.1", help="只监听本机;不要改成 0.0.0.0,除非你确定要对局域网开放")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--threshold", type=float, default=0.5, help="默认置信度阈值,请求里可覆盖")
    parser.add_argument("--no-bf16", action="store_true", help="关闭 bf16 自动混合精度(更慢、更占显存)")
    args = parser.parse_args()

    print("[sam3] 加载模型……", flush=True)
    seg = Segmenter(args.threshold, bf16=not args.no_bf16)
    print(f"[sam3] 加载完成 {seg.load_s:.1f}s,显存 {seg.gpu_mem_mb()} MB,bf16={seg.bf16}", flush=True)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(seg))
    print(f"[sam3] 服务已启动 http://{args.host}:{args.port}  (Ctrl+C 退出)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
