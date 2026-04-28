#!/usr/bin/python3
"""Batch-extract raw/GT images from multiple ROS bag files.

Discovers .bag files under one or more directories (optionally recursive),
then sequentially calls extract_single_bag() for each bag. Each bag's output
is placed in an isolated subdirectory named after the bag stem to prevent
filename collisions.

Usage:
    # Discover bags under directories (recursive by default)
    python3 batch_extract.py --bag-dirs /path/to/bags

    # Non-recursive
    python3 batch_extract.py --bag-dirs /path/to/bags --no-recursive

    # Explicit file list
    python3 batch_extract.py --bag-files a.bag b.bag c.bag

    # Dry-run: list discovered bags without extracting
    python3 batch_extract.py --bag-dirs /path/to/bags --dry-run

    # Force re-extraction even when output already exists
    python3 batch_extract.py --bag-dirs /path/to/bags --force
"""
import argparse
import csv
import sys
import time
from pathlib import Path

# Import core extraction from sibling module.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from extract_bag_images import (  # noqa: E402
    DATA_ROOT,
    DEFAULT_CSV_DIR,
    DEFAULT_GT_DIR,
    DEFAULT_GT_POST_DIR,
    DEFAULT_GT_POST_SUFFIX,
    DEFAULT_RAW_DIR,
    build_minimal_rows,
    extract_single_bag,
    write_csv,
)

DEFAULT_BAGS_DIR = DATA_ROOT / "bags"


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Batch-extract images from multiple ROS bag files."
    )

    # --- Bag source (mutually exclusive) ---
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument(
        "--bag-dirs", nargs="+", type=str,
        help="Directories containing .bag files.",
    )
    src.add_argument(
        "--bag-files", nargs="+", type=str,
        help="Explicit list of .bag file paths.",
    )

    # --- Discovery options ---
    p.add_argument(
        "--no-recursive", dest="recursive", action="store_false", default=True,
        help="Do not search subdirectories (default: recursive).",
    )
    p.add_argument(
        "--pattern", default="*.bag",
        help="Glob pattern for bag files (default: *.bag).",
    )

    # --- Batch behaviour ---
    p.add_argument("--dry-run", action="store_true",
                    help="List discovered bags without extracting.")
    p.add_argument("--force", action="store_true",
                    help="Re-extract even when output subdirectory exists.")

    # --- Output roots ---
    p.add_argument("--raw-out-dir", default=str(DEFAULT_RAW_DIR))
    p.add_argument("--gt-out-dir", default=str(DEFAULT_GT_DIR))
    p.add_argument("--csv-dir", default=str(DEFAULT_CSV_DIR))
    p.add_argument("--gt-post-dir", default=str(DEFAULT_GT_POST_DIR))
    p.add_argument("--gt-post-suffix", default=DEFAULT_GT_POST_SUFFIX)

    # --- Passthrough to extract_single_bag ---
    p.add_argument(
        "--gt-topic-regex",
        default=r"(image_gt|_gt/.*/image_raw|_gt/image_raw|/gt/|ground_truth|seg)",
    )
    p.add_argument("--raw-topic-regex", default=r"(image_raw|/raw/)")
    p.add_argument("--cameras", default="front,left,rear,right")
    p.add_argument("--include-unknown-camera", action="store_true")
    p.add_argument("--max-frames-per-topic", type=int, default=0)
    p.add_argument("--start-offset-sec", type=float, default=0.0)
    p.add_argument("--duration-sec", type=float, default=0.0)
    p.add_argument("--overwrite", action="store_true")

    return p.parse_args(argv)


def discover_bags(dirs, pattern, recursive):
    """Find .bag files under the given directories."""
    bags = []
    for d in dirs:
        d = Path(d)
        if not d.is_dir():
            print(f"[WARN] Not a directory, skipping: {d}")
            continue
        if recursive:
            bags.extend(sorted(d.rglob(pattern)))
        else:
            bags.extend(sorted(d.glob(pattern)))
    return bags


def make_unique_stem(bag_path, all_bags):
    """Create a unique subdirectory name for this bag.

    If multiple bags share the same stem, incorporate the parent
    directory name to disambiguate.
    """
    stem = bag_path.stem
    same_stem = [b for b in all_bags if b.stem == stem]
    if len(same_stem) <= 1:
        return stem
    # Prepend parent directory name for uniqueness.
    return f"{bag_path.parent.name}_{stem}"


def write_batch_manifest(csv_dir, all_records, gt_post_dir, gt_post_suffix):
    """Merge all per-bag records into a single batch_manifest.csv."""
    if not all_records:
        return None
    fields = ["bag", "timestamp", "camera", "raw", "gt", "gt_post"]
    rows = build_minimal_rows(all_records, gt_post_dir, gt_post_suffix)
    # Inject bag source into rows.
    bag_map = {}
    for rec in all_records:
        try:
            rel_path = str(Path(rec["output_path"]).relative_to(DATA_ROOT))
        except ValueError:
            rel_path = str(rec["output_path"])
        bag_map[rel_path] = rec["bag"]
    for row in rows:
        key = row.get("raw") or row.get("gt") or ""
        row["bag"] = bag_map.get(key, "")
    path = Path(csv_dir) / "batch_manifest.csv"
    write_csv(path, fields, rows)
    return path


def main():
    args = parse_args()

    # --- Discover bags ---
    if args.bag_files:
        bags = [Path(f) for f in args.bag_files]
        missing = [b for b in bags if not b.exists()]
        if missing:
            print(f"[ERROR] Bag files not found: {missing}")
            sys.exit(1)
    else:
        bags = discover_bags(args.bag_dirs, args.pattern, args.recursive)

    if not bags:
        print("[ERROR] No .bag files found.")
        sys.exit(1)

    bags = sorted(set(bags))  # deduplicate
    total = len(bags)
    print(f"=== Batch Extract: {total} bag(s) ===")
    for i, b in enumerate(bags, 1):
        print(f"  [{i}/{total}] {b}")
    print()

    if args.dry_run:
        print("[DRY-RUN] No extraction performed.")
        return

    cameras = [x.strip() for x in args.cameras.split(",") if x.strip()]
    raw_root = Path(args.raw_out_dir)
    gt_root = Path(args.gt_out_dir)
    csv_dir = Path(args.csv_dir)
    gt_post_dir = Path(args.gt_post_dir)

    all_records = []
    summary = []
    t0 = time.time()

    for i, bag in enumerate(bags, 1):
        sub = make_unique_stem(bag, bags)

        raw_sub = raw_root / sub
        gt_sub = gt_root / sub

        # Skip check: if both raw and gt subdirs exist and are non-empty.
        if not args.force:
            raw_exists = raw_sub.exists() and any(raw_sub.iterdir())
            gt_exists = gt_sub.exists() and any(gt_sub.iterdir())
            if raw_exists or gt_exists:
                print(f"[{i}/{total}] SKIP (already exists): {bag.name}")
                print(f"         Use --force to re-extract.\n")
                summary.append({"bag": bag.name, "status": "skipped"})
                continue

        prefix = f"[{i}/{total}] "
        try:
            result = extract_single_bag(
                bag_path=bag,
                raw_out_dir=str(raw_sub),
                gt_out_dir=str(gt_sub),
                csv_dir=str(csv_dir),
                csv_prefix=sub,
                gt_post_dir=str(gt_post_dir / sub),
                gt_post_suffix=args.gt_post_suffix,
                gt_topic_regex=args.gt_topic_regex,
                raw_topic_regex=args.raw_topic_regex,
                cameras=cameras,
                include_unknown_camera=args.include_unknown_camera,
                max_frames_per_topic=args.max_frames_per_topic,
                start_offset_sec=args.start_offset_sec,
                duration_sec=args.duration_sec,
                overwrite=args.overwrite,
                log_prefix=prefix,
            )
            all_records.extend(result["records"])
            summary.append({
                "bag": bag.name, "status": "ok",
                "saved": result["saved"], "skipped": result["skipped"],
            })
        except Exception as exc:
            print(f"{prefix}ERROR processing {bag.name}: {exc}\n")
            summary.append({"bag": bag.name, "status": f"error: {exc}"})

        print()  # blank line between bags

    elapsed = time.time() - t0

    # --- Batch manifest ---
    manifest_path = write_batch_manifest(
        csv_dir, all_records, gt_post_dir, args.gt_post_suffix)

    # --- Summary ---
    print("=" * 60)
    print(f"Batch complete.  {total} bags, {elapsed:.1f}s elapsed.")
    ok = [s for s in summary if s["status"] == "ok"]
    skipped = [s for s in summary if s["status"] == "skipped"]
    errors = [s for s in summary if s["status"].startswith("error")]
    total_saved = sum(s.get("saved", 0) for s in ok)
    print(f"  OK: {len(ok)}  Skipped: {len(skipped)}  Errors: {len(errors)}")
    print(f"  Total images saved: {total_saved}")
    if manifest_path:
        print(f"  Batch manifest: {manifest_path}")
    if errors:
        print("  Failed bags:")
        for e in errors:
            print(f"    - {e['bag']}: {e['status']}")
    print("=" * 60)


if __name__ == "__main__":
    main()
