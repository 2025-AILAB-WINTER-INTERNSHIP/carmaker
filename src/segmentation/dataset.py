"""Raw image와 class-id mask를 PyTorch tensor로 바꾸는 Dataset."""

from __future__ import annotations

from collections import defaultdict
from typing import Callable, Optional, Tuple

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader

try:
    from .adapters import DatasetAdapter, SegmentationSample
except ImportError:
    # `python3 src/segmentation/train.py`처럼 직접 실행할 때를 위한 fallback.
    from adapters import DatasetAdapter, SegmentationSample


ImageMaskTransform = Callable[[np.ndarray, np.ndarray], Tuple[np.ndarray, np.ndarray]]


class SegmentationDataset(Dataset):
    """Adapter가 찾은 image/mask pair를 학습 가능한 tensor로 변환한다."""

    def __init__(
        self,
        adapter: DatasetAdapter,
        image_size: tuple[int, int] = (512, 512),
        transform: Optional[ImageMaskTransform] = None,
    ) -> None:
        self.adapter = adapter
        self.samples = list(adapter.samples())
        self.image_size = image_size
        self.transform = transform

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, index: int) -> dict:
        sample = self.samples[index]

        # 원본 파일은 OpenCV로 읽고, mask는 class id를 보존해야 한다.
        image = self._read_image(sample)
        mask = self._read_mask(sample)

        # image와 mask는 반드시 같은 크기로 맞춰야 loss 계산이 가능하다.
        image, mask = self._resize_pair(image, mask)
        if self.transform:
            image, mask = self.transform(image, mask)

        # image: HWC uint8(0..255) -> CHW float32(0..1)
        image_tensor = torch.from_numpy(image.transpose(2, 0, 1)).float() / 255.0

        # mask: HW class id -> HW int64. CrossEntropyLoss target 형식이다.
        mask_tensor = torch.from_numpy(mask).long()

        return {
            "image": image_tensor,
            "mask": mask_tensor,
            "image_path": str(sample.image_path),
            "mask_path": str(sample.mask_path),
            "camera": sample.camera,
        }

    @staticmethod
    def _read_image(sample: SegmentationSample) -> np.ndarray:
        image = cv2.imread(str(sample.image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise FileNotFoundError(f"Could not read image: {sample.image_path}")
        # OpenCV는 BGR로 읽기 때문에 PyTorch/TensorBoard에서 보기 좋은 RGB로 변환한다.
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    @staticmethod
    def _read_mask(sample: SegmentationSample) -> np.ndarray:
        mask = cv2.imread(str(sample.mask_path), cv2.IMREAD_UNCHANGED)
        if mask is None:
            raise FileNotFoundError(f"Could not read mask: {sample.mask_path}")
        if mask.ndim == 3:
            # GT가 3채널로 저장되어도 실제 학습에는 class id 1채널만 사용한다.
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        return mask.astype(np.int64)

    def _resize_pair(self, image: np.ndarray, mask: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        width, height = self.image_size
        if image.shape[1] == width and image.shape[0] == height:
            return image, mask

        # image는 자연스러운 보간을 위해 bilinear, mask는 class id 보존을 위해 nearest 사용.
        image = cv2.resize(image, (width, height), interpolation=cv2.INTER_LINEAR)
        mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
        return image, mask


def split_dataset(
    dataset: Dataset,
    val_ratio: float,
    seed: int,
    test_ratio: float = 0.0,
    stratify_by_camera: bool = False,
    split_by_scenario: bool = True,
    manual_split: Optional[dict[str, list[str]]] = None,
) -> tuple[Dataset, Dataset, Dataset, str, dict]:
    """Dataset을 train/validation/test로 나누되 seed로 split을 고정한다."""
    total = len(dataset)
    if val_ratio < 0 or test_ratio < 0 or val_ratio + test_ratio >= 1:
        raise ValueError("val_ratio and test_ratio must be non-negative and sum to less than 1.0")

    if manual_split:
        samples = getattr(dataset, "samples", None)
        if samples is not None:
            return _split_dataset_manually(dataset, manual_split)

    if split_by_scenario:
        samples = getattr(dataset, "samples", None)
        if samples is not None:
            return _split_dataset_by_scenario(dataset, val_ratio, test_ratio, seed)

    if stratify_by_camera:
        samples = getattr(dataset, "samples", None)
        if samples is not None:
            return _split_dataset_by_camera(dataset, val_ratio, test_ratio, seed)

    val_count = int(round(total * val_ratio))
    test_count = int(round(total * test_ratio))
    train_count = total - val_count - test_count
    generator = torch.Generator().manual_seed(seed)
    subsets = torch.utils.data.random_split(
        dataset,
        [train_count, val_count, test_count],
        generator=generator,
    )
    report = (
        f"### Random Split Report\n"
        f"- **Train**: {train_count} images\n"
        f"- **Val**: {val_count} images\n"
        f"- **Test**: {test_count} images\n"
    )
    return subsets[0], subsets[1], subsets[2], report, {}

def _split_dataset_manually(
    dataset: Dataset,
    manual_split: dict[str, list[str]],
) -> tuple[Dataset, Dataset, Dataset, str, dict]:
    """Splits dataset manually based on scenario names."""
    samples = getattr(dataset, "samples")
    scenario_groups: dict[str, list[int]] = defaultdict(list)
    for index, sample in enumerate(samples):
        scenario = sample.metadata.get("scenario", "unknown") if sample.metadata else "unknown"
        scenario_groups[scenario].append(index)

    train_indices: list[int] = []
    val_indices: list[int] = []
    test_indices: list[int] = []

    # Track which scenarios were assigned to which split for the report
    assigned_scenarios = {"train": [], "val": [], "test": []}

    for split_name in ["train", "val", "test"]:
        target_scenarios = manual_split.get(split_name, [])
        for scenario in target_scenarios:
            if scenario in scenario_groups:
                indices = scenario_groups[scenario]
                if split_name == "train":
                    train_indices.extend(indices)
                elif split_name == "val":
                    val_indices.extend(indices)
                else:
                    test_indices.extend(indices)
                assigned_scenarios[split_name].append((scenario, len(indices)))
            else:
                print(f"[warn] Manual split scenario '{scenario}' not found in dataset.")

    total_images = len(dataset)
    report = [f"### Manual Scenario-based Split Report", f"- Total images: {total_images}", ""]
    report.append("| Split | Scenario Name | Images |")
    report.append("| :--- | :--- | :--- |")

    for split_name in ["train", "val", "test"]:
        display_name = split_name.capitalize()
        for scenario, count in assigned_scenarios[split_name]:
            report.append(f"| {display_name} | `{scenario}` | {count} |")

    report.append(f"\n**Summary:**")
    report.append(f"- **Train**: {len(train_indices)} images ({len(train_indices)/total_images:.1%})")
    report.append(f"- **Val**: {len(val_indices)} images ({len(val_indices)/total_images:.1%})")
    report.append(f"- **Test**: {len(test_indices)} images ({len(test_indices)/total_images:.1%})")

    report_md = "\n".join(report)
    print(f"\n{report_md}\n")

    split_info = {
        "train_scenarios": [s[0] for s in assigned_scenarios["train"]],
        "val_scenarios": [s[0] for s in assigned_scenarios["val"]],
        "test_scenarios": [s[0] for s in assigned_scenarios["test"]],
    }

    return (
        torch.utils.data.Subset(dataset, train_indices),
        torch.utils.data.Subset(dataset, val_indices),
        torch.utils.data.Subset(dataset, test_indices),
        report_md,
        split_info,
    )


def _split_dataset_by_scenario(
    dataset: Dataset,
    val_ratio: float,
    test_ratio: float,
    seed: int,
) -> tuple[Dataset, Dataset, Dataset, str, dict]:
    """Splits data by scenario to prevent leakage between frames."""
    samples = getattr(dataset, "samples")
    scenario_groups: dict[str, list[int]] = defaultdict(list)
    for index, sample in enumerate(samples):
        scenario = sample.metadata.get("scenario", "unknown") if sample.metadata else "unknown"
        scenario_groups[scenario].append(index)

    scenarios = sorted(scenario_groups.keys())
    generator = torch.Generator().manual_seed(seed)
    perm = torch.randperm(len(scenarios), generator=generator).tolist()
    shuffled_scenarios = [scenarios[i] for i in perm]

    total_images = len(dataset)
    # 목표 수치 설정
    targets = {
        "val": int(round(total_images * val_ratio)),
        "test": int(round(total_images * test_ratio)),
    }
    targets["train"] = total_images - targets["val"] - targets["test"]

    current_counts = {"train": 0, "val": 0, "test": 0}
    split_indices = {"train": [], "val": [], "test": []}
    split_scenarios = {"train": [], "val": [], "test": []}

    # Greedy allocation: 각 시나리오를 넣었을 때 전체 target 대비 MSE가 최소화되는 split을 선택
    for scenario in shuffled_scenarios:
        indices = scenario_groups[scenario]
        count = len(indices)

        best_split = "train"
        min_mse = float("inf")

        for split in ["train", "val", "test"]:
            if targets[split] <= 0:
                continue

            # 이 시나리오를 해당 split에 넣는다고 가정했을 때의 MSE 계산
            temp_counts = current_counts.copy()
            temp_counts[split] += count

            mse = sum((temp_counts[s] - targets[s]) ** 2 for s in ["train", "val", "test"])
            if mse < min_mse:
                min_mse = mse
                best_split = split

        current_counts[best_split] += count
        split_indices[best_split].extend(indices)
        split_scenarios[best_split].append((scenario, count))

    report = [
        f"### Scenario-based Split Report (Seed: {seed})",
        f"- Total images: {total_images}",
        f"- Total scenarios: {len(scenarios)}",
        "",
    ]
    report.append("| Split | Scenario Name | Images |")
    report.append("| :--- | :--- | :--- |")

    for s_name in ["train", "val", "test"]:
        display_name = s_name.capitalize()
        for scenario, count in split_scenarios[s_name]:
            report.append(f"| {display_name} | `{scenario}` | {count} |")

    train_indices = split_indices["train"]
    val_indices = split_indices["val"]
    test_indices = split_indices["test"]

    report.append(f"\n**Summary:**")
    report.append(f"- **Train**: {len(train_indices)} images ({len(train_indices)/total_images:.1%}, target: {targets['train']})")
    report.append(f"- **Val**: {len(val_indices)} images ({len(val_indices)/total_images:.1%}, target: {targets['val']})")
    report.append(f"- **Test**: {len(test_indices)} images ({len(test_indices)/total_images:.1%}, target: {targets['test']})")

    report_md = "\n".join(report)
    print(f"\n{report_md}\n")

    split_info = {
        "train_scenarios": [s[0] for s in split_scenarios["train"]],
        "val_scenarios": [s[0] for s in split_scenarios["val"]],
        "test_scenarios": [s[0] for s in split_scenarios["test"]],
    }

    return (
        torch.utils.data.Subset(dataset, train_indices),
        torch.utils.data.Subset(dataset, val_indices),
        torch.utils.data.Subset(dataset, test_indices),
        report_md,
        split_info,
    )


def _split_dataset_by_camera(
    dataset: Dataset,
    val_ratio: float,
    test_ratio: float,
    seed: int,
) -> tuple[Dataset, Dataset, Dataset, str, dict]:
    """Splits data stratified by camera."""
    samples = getattr(dataset, "samples")
    groups: dict[str, list[int]] = defaultdict(list)
    for index, sample in enumerate(samples):
        camera = getattr(sample, "camera", "") or "unknown"
        groups[camera].append(index)

    generator = torch.Generator().manual_seed(seed)
    train_indices: list[int] = []
    val_indices: list[int] = []
    test_indices: list[int] = []

    report = [f"### Camera-stratified Split Report (Seed: {seed})", ""]
    report.append("| Camera | Train | Val | Test | Total |")
    report.append("| :--- | :--- | :--- | :--- | :--- |")

    for camera in sorted(groups):
        indices = groups[camera]
        count = len(indices)
        permutation = torch.randperm(count, generator=generator).tolist()
        shuffled = [indices[i] for i in permutation]

        v_cnt = int(round(count * val_ratio))
        t_cnt = int(round(count * test_ratio))
        tr_cnt = count - v_cnt - t_cnt

        train_indices.extend(shuffled[:tr_cnt])
        val_indices.extend(shuffled[tr_cnt : tr_cnt + v_cnt])
        test_indices.extend(shuffled[tr_cnt + v_cnt :])
        report.append(f"| {camera} | {tr_cnt} | {v_cnt} | {t_cnt} | {count} |")

    report.append(f"\n**Total Summary:**")
    report.append(f"- **Train**: {len(train_indices)} images")
    report.append(f"- **Val**: {len(val_indices)} images")
    report.append(f"- **Test**: {len(test_indices)} images")

    report_md = "\n".join(report)
    print(f"\n{report_md}\n")

    split_info = {
        "cameras": sorted(groups.keys()),
    }

    return (
        torch.utils.data.Subset(dataset, train_indices),
        torch.utils.data.Subset(dataset, val_indices),
        torch.utils.data.Subset(dataset, test_indices),
        report_md,
        split_info,
    )
