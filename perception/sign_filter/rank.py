#!/usr/bin/env python3
"""embed.py 캐시를 프롬프트로 랭킹해 확인용 HTML 갤러리를 만든다.

원본을 다시 읽지 않으므로 프롬프트/임계값을 바꿔가며 몇 초 만에 재실행할 수 있다.
(상위 N장 썸네일 생성 때만 해당 이미지를 읽는다.)
"""
import argparse
import csv
import html
from pathlib import Path

import numpy as np
import torch
from PIL import Image, ImageDraw

POS = [
    "an indoor directional sign hanging from the ceiling",
    "a wayfinding signboard with arrows and text",
    "an emergency exit sign",
    "a subway station exit number sign",
    "a restroom sign",
    "a floor directory information board",
]
NEG = [
    "an empty indoor corridor",
    "store shelves full of products",
    "a crowd of people walking indoors",
    "a plain wall or ceiling",
    "an escalator or a staircase",
    "a shop storefront with a brand logo",
    "a blurry dark indoor photo",
]


def load_prompts(path, default):
    if not path:
        return default
    return [l.strip() for l in Path(path).read_text().splitlines() if l.strip()]


def tile_box(w, h, tile, rows, cols):
    """타일 인덱스 -> 원본 좌표 박스. 0 은 전체 이미지."""
    if tile == 0:
        return None
    r, c = divmod(tile - 1, cols)
    return (c * w // cols, r * h // rows, (c + 1) * w // cols, (r + 1) * h // rows)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--cache", required=True)
    p.add_argument("--out-dir", required=True)
    p.add_argument("--top", type=int, default=200)
    p.add_argument("--pos-file", default=None, help="줄당 프롬프트 1개 (긍정)")
    p.add_argument("--neg-file", default=None, help="줄당 프롬프트 1개 (부정)")
    p.add_argument("--thumb-width", type=int, default=420)
    args = p.parse_args()

    z = np.load(args.cache, allow_pickle=False)
    paths, devices, embeds, ok = z["paths"], z["devices"], z["embeds"], z["ok"]
    model_name, pretrained, rows, cols = z["meta"]
    rows, cols = int(rows), int(cols)
    print(f"[load] {len(paths)} images, embeds{embeds.shape}, model={model_name}/{pretrained}")

    pos = load_prompts(args.pos_file, POS)
    neg = load_prompts(args.neg_file, NEG)

    import open_clip
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    model, _, _ = open_clip.create_model_and_transforms(model_name, pretrained=pretrained)
    model = model.to(dev).eval()
    tok = open_clip.get_tokenizer(model_name)
    with torch.no_grad():
        txt = model.encode_text(tok(pos + neg).to(dev))
        txt = torch.nn.functional.normalize(txt.float(), dim=-1)
        scale = model.logit_scale.exp().item()

    img = torch.from_numpy(embeds.astype(np.float32)).to(dev)   # [N, T, D]
    logits = img @ txt.T * scale                                # [N, T, C]
    prob_pos = logits.softmax(dim=-1)[..., :len(pos)].sum(-1)   # [N, T]
    best_tile = prob_pos.argmax(dim=-1).cpu().numpy()
    score = prob_pos.max(dim=-1).values.cpu().numpy()
    score[~ok] = -1.0

    order = np.argsort(-score)
    out = Path(args.out_dir)
    (out / "thumbs").mkdir(parents=True, exist_ok=True)

    with open(out / "ranked.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["rank", "score", "best_tile", "device_id", "path"])
        for rk, i in enumerate(order):
            w.writerow([rk, f"{score[i]:.4f}", int(best_tile[i]), devices[i], paths[i]])

    cards = []
    for rk, i in enumerate(order[:args.top]):
        src = Path(paths[i])
        try:
            im = Image.open(src).convert("RGB")
        except Exception:
            continue
        box = tile_box(im.width, im.height, int(best_tile[i]), rows, cols)
        if box:
            ImageDraw.Draw(im).rectangle(box, outline=(255, 40, 40),
                                         width=max(4, im.width // 300))
        im.thumbnail((args.thumb_width, args.thumb_width * 2), Image.BICUBIC)
        name = f"{rk:04d}_{src.stem}.jpg"
        im.save(out / "thumbs" / name, quality=80)
        cards.append(
            f'<figure><a href="file://{html.escape(str(src))}" target="_blank">'
            f'<img src="thumbs/{name}" loading="lazy"></a>'
            f'<figcaption><b>#{rk}</b> score {score[i]:.3f} '
            f'&middot; tile {int(best_tile[i])} &middot; {html.escape(str(devices[i]))}<br>'
            f'<small>{html.escape(src.name)}</small></figcaption></figure>')

    doc = f"""<!doctype html><meta charset="utf-8">
<title>sign filter top {args.top}</title>
<style>
 body{{background:#111;color:#eee;font:14px system-ui;margin:24px}}
 h1{{font-size:18px}} .meta{{color:#9ab;margin-bottom:16px;line-height:1.6}}
 .grid{{display:grid;grid-template-columns:repeat(auto-fill,minmax(320px,1fr));gap:14px}}
 figure{{margin:0;background:#1b1b1b;border-radius:8px;padding:8px}}
 img{{width:100%;border-radius:4px;display:block}}
 figcaption{{margin-top:6px;color:#bbb}} small{{color:#888;word-break:break-all}}
 code{{color:#8fd}}
</style>
<h1>실내 안내 표지판 후보 상위 {len(cards)}장 / 전체 {len(paths)}장</h1>
<div class="meta">모델 <code>{html.escape(str(model_name))}/{html.escape(str(pretrained))}</code>
 &middot; 타일 {rows}x{cols}+전체 &middot; 빨간 박스 = 최고점 타일<br>
 긍정: {html.escape(' | '.join(pos))}<br>
 부정: {html.escape(' | '.join(neg))}</div>
<div class="grid">{''.join(cards)}</div>"""
    (out / "gallery.html").write_text(doc)
    print(f"[save] {out/'ranked.csv'}")
    print(f"[save] {out/'gallery.html'}  ({len(cards)} thumbs)")
    print(f"[score] max {score.max():.3f}  median {np.median(score[ok]):.3f}  "
          f"top{args.top} min {score[order[args.top-1]]:.3f}")


if __name__ == "__main__":
    main()
