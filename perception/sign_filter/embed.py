#!/usr/bin/env python3
"""NAVER LABS 실내 데이터셋을 서브샘플링해 CLIP 타일 임베딩 캐시를 만든다.

임베딩을 한 번 만들어 두면 rank.py 에서 프롬프트/임계값을 바꿔도
225GB 원본을 다시 읽지 않고 즉시 재랭킹할 수 있다.
"""
import argparse
import time
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from torch.utils.data import DataLoader, Dataset

MEAN = (0.48145466, 0.4578275, 0.40821073)
STD = (0.26862954, 0.26130258, 0.27577711)


def collect_records(dataset_root, stride):
    """kapture records_camera.txt 들을 읽어 device_id 별로 stride 간격 샘플링."""
    samples = []
    for rec in sorted(Path(dataset_root).rglob("records_camera.txt")):
        base = rec.parent / "records_data"
        by_device = {}
        for line in rec.read_text().splitlines():
            if line.startswith("#") or not line.strip():
                continue
            ts, device_id, image_path = (f.strip() for f in line.split(",", 2))
            by_device.setdefault(device_id, []).append((int(ts), image_path))
        for device_id, rows in sorted(by_device.items()):
            rows.sort()
            for _, image_path in rows[::stride]:
                samples.append((str(base / image_path), device_id))
    return samples


class TileDataset(Dataset):
    def __init__(self, samples, rows, cols, size):
        self.samples = samples
        self.rows, self.cols, self.size = rows, cols, size
        self.mean = torch.tensor(MEAN).view(3, 1, 1)
        self.std = torch.tensor(STD).view(3, 1, 1)

    def __len__(self):
        return len(self.samples)

    def _crops(self, im):
        w, h = im.size
        out = [im]
        for r in range(self.rows):
            for c in range(self.cols):
                box = (c * w // self.cols, r * h // self.rows,
                       (c + 1) * w // self.cols, (r + 1) * h // self.rows)
                out.append(im.crop(box))
        return out

    def __getitem__(self, i):
        path, _ = self.samples[i]
        try:
            im = Image.open(path).convert("RGB")
        except Exception:
            n = 1 + self.rows * self.cols
            return torch.zeros(n, 3, self.size, self.size), i, False
        tiles = []
        for crop in self._crops(im):
            t = crop.resize((self.size, self.size), Image.BICUBIC)
            t = torch.from_numpy(np.asarray(t)).permute(2, 0, 1).float().div_(255)
            tiles.append((t - self.mean) / self.std)
        return torch.stack(tiles), i, True


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dataset-root", default="/home/sangw/datasets/naver")
    p.add_argument("--out", required=True, help="출력 .npz 경로")
    p.add_argument("--stride", type=int, default=22, help="device_id 별 프레임 간격")
    p.add_argument("--rows", type=int, default=2)
    p.add_argument("--cols", type=int, default=3)
    p.add_argument("--model", default="ViT-L-14")
    p.add_argument("--pretrained", default="laion2b_s32b_b82k")
    p.add_argument("--batch", type=int, default=8, help="이미지 단위 배치 (타일은 x7)")
    p.add_argument("--workers", type=int, default=8)
    p.add_argument("--limit", type=int, default=0, help="0 이면 전체")
    args = p.parse_args()

    samples = collect_records(args.dataset_root, args.stride)
    if args.limit:
        samples = samples[:args.limit]
    n_tiles = 1 + args.rows * args.cols
    print(f"[scan] {len(samples)} images (stride={args.stride}), {n_tiles} tiles each")
    devices = {}
    for _, d in samples:
        devices[d] = devices.get(d, 0) + 1
    print(f"[scan] {len(devices)} device_ids, per-device min/max = "
          f"{min(devices.values())}/{max(devices.values())}")

    import open_clip
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    model, _, _ = open_clip.create_model_and_transforms(args.model, pretrained=args.pretrained)
    model = model.to(dev).eval()
    size = model.visual.image_size
    size = size[0] if isinstance(size, (tuple, list)) else size
    print(f"[model] {args.model}/{args.pretrained} on {dev}, input {size}px")

    ds = TileDataset(samples, args.rows, args.cols, size)
    dl = DataLoader(ds, batch_size=args.batch, num_workers=args.workers, shuffle=False)

    embeds = np.zeros((len(samples), n_tiles, model.visual.output_dim), dtype=np.float16)
    ok = np.zeros(len(samples), dtype=bool)
    done, t0 = 0, time.time()
    with torch.no_grad(), torch.autocast(dev, dtype=torch.float16, enabled=(dev == "cuda")):
        for tiles, idx, good in dl:
            b, t = tiles.shape[0], tiles.shape[1]
            feat = model.encode_image(tiles.view(b * t, *tiles.shape[2:]).to(dev))
            feat = torch.nn.functional.normalize(feat.float(), dim=-1).view(b, t, -1)
            embeds[idx.numpy()] = feat.cpu().numpy().astype(np.float16)
            ok[idx.numpy()] = good.numpy()
            done += b
            if done % (args.batch * 25) < args.batch:
                el = time.time() - t0
                print(f"[embed] {done}/{len(samples)}  {done/el:.1f} img/s  "
                      f"eta {(len(samples)-done)/max(done/el,1e-6)/60:.1f} min", flush=True)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out,
             paths=np.array([s[0] for s in samples]),
             devices=np.array([s[1] for s in samples]),
             embeds=embeds, ok=ok,
             meta=np.array([args.model, args.pretrained, str(args.rows), str(args.cols)]))
    print(f"[save] {out}  embeds{embeds.shape}  ok={int(ok.sum())}/{len(samples)}  "
          f"{out.stat().st_size/1e6:.1f} MB  elapsed {(time.time()-t0)/60:.1f} min")


if __name__ == "__main__":
    main()
