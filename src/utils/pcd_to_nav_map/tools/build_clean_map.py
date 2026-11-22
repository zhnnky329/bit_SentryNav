#!/usr/bin/env python3
"""Build a clean Nav2 PGM/YAML map from a LiDAR point cloud.

The free area is the convex hull of the floor points; obstacles come from a
height band above the floor.  Wall gaps are closed with a morphological
closing and tiny occupied components are removed.  The resulting map has:

  * 0   (black)  occupied obstacle cells
  * 255 (white)  free cells inside the mapped floor footprint
  * 205 (gray)   unknown cells outside the footprint

Usage:
  python3 build_clean_map.py map.pcd --output-prefix /path/to/map
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import ndimage as ndi
from scipy.spatial import ConvexHull


def read_pcd(path: Path) -> np.ndarray:
    """Read a PCD file with float32 x/y/z/(intensity) fields."""
    with path.open("rb") as stream:
        header: dict[str, str] = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"unterminated PCD header: {path}")
            text = line.decode("ascii", "ignore").strip()
            if not text or text.startswith("#"):
                continue
            key, _, value = text.partition(" ")
            header[key] = value
            if key == "DATA":
                break
        payload = stream.read()

    if header.get("DATA") != "binary":
        raise ValueError("only binary PCD files are supported")

    fields = header["FIELDS"].split()
    sizes = [int(item) for item in header["SIZE"].split()]
    types = header["TYPE"].split()
    counts = [int(item) for item in header.get("COUNT", " ".join("1" * len(fields))).split()]
    dtype = []
    for name, size_, type_, count in zip(fields, sizes, types, counts):
        code = {"F": "f", "U": "u", "I": "i"}.get(type_)
        if code is None:
            raise ValueError(f"unsupported PCD field type: {type_}")
        item = np.dtype(f"{code}{size_}")
        dtype.append((name, item) if count == 1 else (name, item, (count,)))
    records = np.frombuffer(payload, dtype=np.dtype(dtype))
    return np.stack(
        [records["x"], records["y"], records["z"]], axis=1
    ).astype(np.float64)


def write_pgm(path: Path, raster: np.ndarray) -> None:
    """Write an 8-bit binary PGM (row 0 is the top of the image)."""
    height, width = raster.shape
    with path.open("wb") as stream:
        stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        stream.write(np.ascontiguousarray(raster, dtype=np.uint8).tobytes())


def build(args: argparse.Namespace) -> None:
    xyz = read_pcd(args.pcd)
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]

    wall = (z >= args.wall_min_z) & (z < args.wall_max_z)
    floor = (z >= args.floor_min_z) & (z <= args.floor_max_z)
    if wall.sum() < 10:
        raise ValueError("too few wall points in the selected height band")
    if floor.sum() < 10:
        raise ValueError("too few floor points in the selected height band")

    x_min = float(x[floor].min() - args.padding)
    x_max = float(x[floor].max() + args.padding)
    y_min = float(y[floor].min() - args.padding)
    y_max = float(y[floor].max() + args.padding)
    width = int(np.ceil((x_max - x_min) / args.resolution))
    height = int(np.ceil((y_max - y_min) / args.resolution))

    occupied = np.zeros((height, width), dtype=bool)
    ix = ((x[wall] - x_min) / args.resolution).astype(int)
    iy = ((y[wall] - y_min) / args.resolution).astype(int)
    inside = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    occupied[iy[inside], ix[inside]] = True

    kernel = np.ones((args.close_kernel, args.close_kernel), dtype=bool)
    occupied = ndi.binary_closing(occupied, structure=kernel)

    labels, _ = ndi.label(occupied, structure=np.ones((3, 3), dtype=bool))
    sizes = np.bincount(labels.ravel())
    keep = sizes >= args.min_component
    keep[0] = False
    occupied = keep[labels]

    if args.dilate > 0:
        occupied = ndi.binary_dilation(
            occupied, structure=np.ones((3, 3), dtype=bool), iterations=args.dilate
        )

    floor_points = np.column_stack((x[floor], y[floor]))
    hull = ConvexHull(floor_points)
    polygon = [
        (
            int(round((px - x_min) / args.resolution)),
            int(round((py - y_min) / args.resolution)),
        )
        for px, py in floor_points[hull.vertices]
    ]
    mask = Image.new("L", (width, height), 0)
    ImageDraw.Draw(mask).polygon(polygon, fill=255)
    footprint = np.asarray(mask) > 0

    free = footprint & ~occupied
    raster = np.full((height, width), args.unknown_value, dtype=np.uint8)
    raster[free] = args.free_value
    raster[occupied] = args.occupied_value

    output_prefix = Path(args.output_prefix)
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    write_pgm(output_prefix.with_suffix(".pgm"), np.flipud(raster))

    descriptor = {
        "image": output_prefix.with_suffix(".pgm").name,
        "mode": "trinary",
        "resolution": args.resolution,
        "origin": [x_min, y_min, 0.0],
        "negate": 0,
        "occupied_thresh": args.occupied_thresh,
        "free_thresh": args.free_thresh,
    }
    with output_prefix.with_suffix(".yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(descriptor, stream, sort_keys=False)

    total = width * height
    print(
        f"map {width}x{height} @ {args.resolution} m/cell "
        f"({width * args.resolution:.1f}x{height * args.resolution:.1f} m), "
        f"occupied={100.0 * occupied.sum() / total:.2f}%, "
        f"free={100.0 * free.sum() / total:.1f}%, "
        f"unknown={100.0 * (total - occupied.sum() - free.sum()) / total:.1f}%"
    )
    print(f"written: {output_prefix.with_suffix('.pgm')}")
    print(f"written: {output_prefix.with_suffix('.yaml')}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcd", type=Path, help="input PCD file")
    parser.add_argument("--output-prefix", type=Path, required=True,
                        help="output path without extension")
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--padding", type=float, default=1.5)
    parser.add_argument("--floor-min-z", type=float, default=-0.65)
    parser.add_argument("--floor-max-z", type=float, default=0.0)
    parser.add_argument("--wall-min-z", type=float, default=0.0)
    parser.add_argument("--wall-max-z", type=float, default=3.0)
    parser.add_argument("--close-kernel", type=int, default=5)
    parser.add_argument("--min-component", type=int, default=20)
    parser.add_argument("--dilate", type=int, default=1)
    parser.add_argument("--occupied-value", type=int, default=0)
    parser.add_argument("--free-value", type=int, default=255)
    parser.add_argument("--unknown-value", type=int, default=205)
    parser.add_argument("--occupied-thresh", type=float, default=0.65)
    parser.add_argument("--free-thresh", type=float, default=0.196)
    build(parser.parse_args())


if __name__ == "__main__":
    main()
