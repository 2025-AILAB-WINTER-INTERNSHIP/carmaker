#!/usr/bin/env python3
"""CarMaker localization EKF 지표를 rosbag 또는 실시간 ROS topic에서 계산한다.

Metrics:
  - Position RMSE [m]
  - Yaw RMSE [deg]
  - NEES for pose state [x, y, yaw]

Examples:
  rosrun carmaker_localization evaluate_ekf.py bag ekf_eval.bag

  rosrun carmaker_localization evaluate_ekf.py live --csv
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import os
import sys
import threading
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple


DEFAULT_GT_TOPIC = "/carmaker/dynamic_info"
DEFAULT_ODOM_TOPIC = "/localization/odom"
DEFAULT_MAX_DT = 0.05
DEFAULT_CSV = "auto"

# covariance가 0 또는 거의 singular일 때 NEES 계산이 터지지 않도록 작은 대각 성분을 더한다.
POSE_COV_EPS = 1e-9
DISABLED_OUTPUT_VALUES = {"", "none", "false", "off", "no"}


def _run_id() -> str:
    # 같은 날 여러 번 실험해도 결과 파일명이 겹치지 않도록 초 단위 timestamp를 붙인다.
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _package_root() -> Path:
    # ROS 환경에서는 rospkg로 패키지 루트를 찾고, 일반 Python 실행에서는 현재 파일 위치를 기준으로 찾는다.
    try:
        import rospkg  # type: ignore

        return Path(rospkg.RosPack().get_path("carmaker_localization"))
    except Exception:
        return Path(__file__).resolve().parents[1]


def _default_output_dir() -> Path:
    # EKF 평가 산출물은 패키지 내부 data/csv 폴더로 모은다.
    return _package_root() / "data" / "csv"


def _is_disabled_output(value: Optional[str]) -> bool:
    return value is None or str(value).strip().lower() in DISABLED_OUTPUT_VALUES


def _unique_path(path: Path) -> Path:
    # overwrite를 명시하지 않은 경우 기존 파일을 보존하기 위해 숫자 suffix를 붙인다.
    if not path.exists():
        return path
    stem = path.stem
    suffix = path.suffix
    parent = path.parent
    for index in range(1, 10000):
        candidate = parent / f"{stem}_{index:03d}{suffix}"
        if not candidate.exists():
            return candidate
    raise RuntimeError(f"Could not find a free output path for {path}")


def _resolve_output_path(value: Optional[str], default_name: str, overwrite: bool) -> str:
    # value가 auto이면 data/csv/default_name으로 저장한다.
    raw_value = "auto" if value is None else str(value).strip()
    if raw_value.lower() == "auto":
        path = _default_output_dir() / default_name
    else:
        path = Path(raw_value).expanduser()
        # 확장자가 없거나 구분자로 끝나는 값은 파일이 아니라 폴더로 해석한다.
        if raw_value.endswith(("/", "\\")) or path.suffix == "":
            path = path / default_name

    path.parent.mkdir(parents=True, exist_ok=True)
    if not overwrite:
        path = _unique_path(path)
    return str(path)


@dataclass
class GtSample:
    # GT는 CarMaker DynamicsInfo의 후륜축 위치와 차량 yaw를 그대로 사용한다.
    time: float
    x: float
    y: float
    yaw: float


@dataclass
class OdomSample:
    # EKF odom은 pose와 pose covariance를 함께 보관해야 NEES까지 계산할 수 있다.
    time: float
    x: float
    y: float
    yaw: float
    covariance: Sequence[float]


@dataclass
class MetricRow:
    # CSV 한 줄에 대응되는 매칭 샘플 결과.
    time: float
    x_gt: float
    y_gt: float
    yaw_gt: float
    x_ekf: float
    y_ekf: float
    yaw_ekf: float
    pos_err: float
    yaw_err_deg: float
    nees: Optional[float]


@dataclass
class MetricAccumulator:
    # RMSE는 전체 샘플을 다시 순회하지 않도록 제곱합을 누적한다.
    rows: List[MetricRow] = field(default_factory=list)
    total_gt_samples: int = 0
    total_odom_samples: int = 0
    unmatched_samples: int = 0
    invalid_samples: int = 0
    pos_sq_sum: float = 0.0
    yaw_sq_sum: float = 0.0
    nees_values: List[float] = field(default_factory=list)

    def add_row(self, row: MetricRow, yaw_err_rad: float) -> None:
        self.rows.append(row)
        self.pos_sq_sum += row.pos_err * row.pos_err
        self.yaw_sq_sum += yaw_err_rad * yaw_err_rad
        if row.nees is not None and math.isfinite(row.nees):
            self.nees_values.append(row.nees)

    @property
    def dropped_samples(self) -> int:
        return self.unmatched_samples + self.invalid_samples

    def summary(self) -> Dict[str, float]:
        n = len(self.rows)
        nees_n = len(self.nees_values)
        return {
            "samples": float(n),
            "total_gt_samples": float(self.total_gt_samples),
            "total_odom_samples": float(self.total_odom_samples),
            "dropped_samples": float(self.dropped_samples),
            "unmatched_samples": float(self.unmatched_samples),
            "invalid_samples": float(self.invalid_samples),
            "nees_samples": float(nees_n),
            "position_rmse_m": math.sqrt(self.pos_sq_sum / n) if n else math.nan,
            "yaw_rmse_deg": math.degrees(math.sqrt(self.yaw_sq_sum / n)) if n else math.nan,
            "mean_nees": (sum(self.nees_values) / nees_n) if nees_n else math.nan,
            "median_nees": _percentile(self.nees_values, 50.0),
            "nees_p95": _percentile(self.nees_values, 95.0),
        }


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _quaternion_to_yaw(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _stamp_to_sec(stamp: Any) -> float:
    if stamp is None:
        return math.nan
    if hasattr(stamp, "to_sec"):
        return float(stamp.to_sec())
    return float(getattr(stamp, "secs", 0)) + float(getattr(stamp, "nsecs", 0)) * 1e-9


def _message_time(msg: Any, fallback_time: Optional[float] = None) -> float:
    # ROS message header stamp를 우선 사용하고, 비어 있으면 bag read time 또는 now를 사용한다.
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    stamp_sec = _stamp_to_sec(stamp)
    if math.isfinite(stamp_sec) and stamp_sec > 0.0:
        return stamp_sec
    if fallback_time is not None:
        return float(fallback_time)
    return math.nan


def _is_finite_values(values: Sequence[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _gt_from_msg(msg: Any, fallback_time: Optional[float] = None) -> Optional[GtSample]:
    try:
        sample = GtSample(
            time=_message_time(msg, fallback_time),
            x=float(msg.RearAxle_x),
            y=float(msg.RearAxle_y),
            yaw=float(msg.Car_Yaw),
        )
    except (AttributeError, TypeError, ValueError):
        return None
    if not _is_finite_values((sample.time, sample.x, sample.y, sample.yaw)):
        return None
    return sample


def _odom_from_msg(msg: Any, fallback_time: Optional[float] = None) -> Optional[OdomSample]:
    try:
        pose = msg.pose.pose
        sample = OdomSample(
            time=_message_time(msg, fallback_time),
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=float(_quaternion_to_yaw(pose.orientation)),
            covariance=list(msg.pose.covariance),
        )
    except (AttributeError, TypeError, ValueError):
        return None
    if len(sample.covariance) < 36:
        return None
    if not _is_finite_values((sample.time, sample.x, sample.y, sample.yaw)):
        return None
    return sample


def _pose_covariance_3x3(cov: Sequence[float]) -> List[List[float]]:
    # ROS pose covariance는 6x6 row-major이며 yaw는 회전 z축 index 5에 해당한다.
    return [
        [float(cov[0]) + POSE_COV_EPS, float(cov[1]), float(cov[5])],
        [float(cov[6]), float(cov[7]) + POSE_COV_EPS, float(cov[11])],
        [float(cov[30]), float(cov[31]), float(cov[35]) + POSE_COV_EPS],
    ]


def _inverse_3x3(m: Sequence[Sequence[float]]) -> Optional[List[List[float]]]:
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]

    c00 = e * i - f * h
    c01 = -(d * i - f * g)
    c02 = d * h - e * g
    c10 = -(b * i - c * h)
    c11 = a * i - c * g
    c12 = -(a * h - b * g)
    c20 = b * f - c * e
    c21 = -(a * f - c * d)
    c22 = a * e - b * d

    det = a * c00 + b * c01 + c * c02
    if not math.isfinite(det) or abs(det) < 1e-15:
        return None

    inv_det = 1.0 / det
    return [
        [c00 * inv_det, c10 * inv_det, c20 * inv_det],
        [c01 * inv_det, c11 * inv_det, c21 * inv_det],
        [c02 * inv_det, c12 * inv_det, c22 * inv_det],
    ]


def _quadratic_form(vec: Sequence[float], mat: Sequence[Sequence[float]]) -> float:
    tmp = [
        mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2],
        mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2],
        mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2],
    ]
    return vec[0] * tmp[0] + vec[1] * tmp[1] + vec[2] * tmp[2]


def _compute_nees(dx: float, dy: float, yaw_err: float, covariance: Sequence[float]) -> Optional[float]:
    # NEES = e^T P^-1 e. P가 비정상이면 RMSE 샘플은 유지하고 NEES만 제외한다.
    if not _is_finite_values(covariance):
        return None
    p_pose = _pose_covariance_3x3(covariance)
    inv_p = _inverse_3x3(p_pose)
    if inv_p is None:
        return None
    nees = _quadratic_form((dx, dy, yaw_err), inv_p)
    if not math.isfinite(nees):
        return None
    return nees


def _compute_row(gt: GtSample, odom: OdomSample) -> Optional[Tuple[MetricRow, float]]:
    # GT 후륜축 pose와 EKF 후륜축 pose의 오차를 한 샘플 단위로 계산한다.
    dx = gt.x - odom.x
    dy = gt.y - odom.y
    yaw_err = _wrap_to_pi(gt.yaw - odom.yaw)
    if not _is_finite_values((dx, dy, yaw_err)):
        return None

    nees = _compute_nees(dx, dy, yaw_err, odom.covariance)
    row = MetricRow(
        time=gt.time,
        x_gt=gt.x,
        y_gt=gt.y,
        yaw_gt=gt.yaw,
        x_ekf=odom.x,
        y_ekf=odom.y,
        yaw_ekf=odom.yaw,
        pos_err=math.hypot(dx, dy),
        yaw_err_deg=abs(math.degrees(yaw_err)),
        nees=nees,
    )
    return row, yaw_err


def _percentile(values: Sequence[float], pct: float) -> float:
    finite_values = sorted(v for v in values if math.isfinite(v))
    if not finite_values:
        return math.nan
    if len(finite_values) == 1:
        return finite_values[0]
    rank = (pct / 100.0) * (len(finite_values) - 1)
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return finite_values[lo]
    frac = rank - lo
    return finite_values[lo] * (1.0 - frac) + finite_values[hi] * frac


def _format_value(value: float) -> str:
    if not math.isfinite(value):
        return "nan"
    if abs(value - round(value)) < 1e-9:
        return str(int(round(value)))
    return f"{value:.6f}"


def _format_summary(summary: Dict[str, float]) -> str:
    keys = [
        "samples",
        "total_gt_samples",
        "total_odom_samples",
        "dropped_samples",
        "unmatched_samples",
        "invalid_samples",
        "nees_samples",
        "position_rmse_m",
        "yaw_rmse_deg",
        "mean_nees",
        "median_nees",
        "nees_p95",
    ]
    return "\n".join(f"{key}: {_format_value(summary[key])}" for key in keys)


def _write_summary(path: str, summary: Dict[str, float]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(_format_summary(summary))
        f.write("\n")


def _write_csv(path: str, rows: Sequence[MetricRow]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "time",
                "x_gt",
                "y_gt",
                "yaw_gt",
                "x_ekf",
                "y_ekf",
                "yaw_ekf",
                "pos_err",
                "yaw_err_deg",
                "nees",
            ]
        )
        for row in rows:
            writer.writerow(
                [
                    f"{row.time:.9f}",
                    f"{row.x_gt:.9f}",
                    f"{row.y_gt:.9f}",
                    f"{row.yaw_gt:.9f}",
                    f"{row.x_ekf:.9f}",
                    f"{row.y_ekf:.9f}",
                    f"{row.yaw_ekf:.9f}",
                    f"{row.pos_err:.9f}",
                    f"{row.yaw_err_deg:.9f}",
                    "" if row.nees is None else f"{row.nees:.9f}",
                ]
            )


def _nearest_odom(gt_time: float, odom_times: Sequence[float], odoms: Sequence[OdomSample]) -> Optional[OdomSample]:
    # bag 후처리에서는 GT timestamp 기준으로 가장 가까운 odom 하나를 이진 탐색으로 찾는다.
    if not odom_times:
        return None
    right = bisect.bisect_left(odom_times, gt_time)
    candidates = []
    if right < len(odom_times):
        candidates.append(right)
    if right > 0:
        candidates.append(right - 1)
    if not candidates:
        return None
    best_index = min(candidates, key=lambda idx: abs(odom_times[idx] - gt_time))
    return odoms[best_index]


def run_bag(args: argparse.Namespace) -> int:
    # 저장된 bag에서 GT와 EKF odom을 읽고, 전체 실험 구간의 최종 지표를 계산한다.
    try:
        import rosbag  # type: ignore
    except ImportError as exc:
        print(f"ERROR: rosbag import failed: {exc}", file=sys.stderr)
        return 2

    if not os.path.isfile(args.bag_file):
        print(f"ERROR: bag file not found: {args.bag_file}", file=sys.stderr)
        return 2

    gts: List[GtSample] = []
    odoms: List[OdomSample] = []

    with rosbag.Bag(args.bag_file, "r") as bag:
        _, topic_info = bag.get_type_and_topic_info()
        if args.gt_topic not in topic_info:
            print(f"ERROR: GT topic not found in bag: {args.gt_topic}", file=sys.stderr)
            return 2
        if args.odom_topic not in topic_info:
            print(f"ERROR: odom topic not found in bag: {args.odom_topic}", file=sys.stderr)
            return 2

        for topic, msg, t in bag.read_messages(topics=[args.gt_topic, args.odom_topic]):
            fallback = t.to_sec()
            if topic == args.gt_topic:
                sample = _gt_from_msg(msg, fallback)
                if sample is not None:
                    gts.append(sample)
            elif topic == args.odom_topic:
                sample = _odom_from_msg(msg, fallback)
                if sample is not None:
                    odoms.append(sample)

    odoms.sort(key=lambda sample: sample.time)
    odom_times = [sample.time for sample in odoms]

    acc = MetricAccumulator(total_gt_samples=len(gts), total_odom_samples=len(odoms))
    for gt in gts:
        # max_dt 밖의 odom은 같은 시점의 추정값으로 보지 않고 drop한다.
        odom = _nearest_odom(gt.time, odom_times, odoms)
        if odom is None or abs(odom.time - gt.time) > args.max_dt:
            acc.unmatched_samples += 1
            continue

        result = _compute_row(gt, odom)
        if result is None:
            acc.invalid_samples += 1
            continue
        row, yaw_err = result
        acc.add_row(row, yaw_err)

    run_id = _run_id()
    summary = acc.summary()
    print(_format_summary(summary))

    if not args.no_csv:
        csv_path = _resolve_output_path(args.csv, f"ekf_metrics_{run_id}.csv", args.overwrite)
        _write_csv(csv_path, acc.rows)
        print(f"csv: {csv_path}")
    if not _is_disabled_output(args.summary_output):
        summary_path = _resolve_output_path(
            args.summary_output,
            f"ekf_metrics_summary_{run_id}.txt",
            args.overwrite,
        )
        _write_summary(summary_path, summary)
        print(f"summary: {summary_path}")

    if len(acc.rows) < args.min_samples:
        print(
            f"ERROR: only {len(acc.rows)} matched samples; required at least {args.min_samples}.",
            file=sys.stderr,
        )
        return 1
    return 0


class LiveEvaluator:
    # 실시간 모드는 두 topic의 메시지를 queue에 쌓고 가까운 timestamp끼리 즉시 매칭한다.
    def __init__(self, args: argparse.Namespace, rospy: Any) -> None:
        from carmaker_msgs.msg import DynamicsInfo  # type: ignore
        from nav_msgs.msg import Odometry  # type: ignore
        from std_msgs.msg import Float64  # type: ignore

        self.args = args
        self.rospy = rospy
        self.lock = threading.Lock()
        self.gt_queue: Deque[GtSample] = deque()
        self.odom_queue: Deque[OdomSample] = deque()
        self.acc = MetricAccumulator()
        self.csv_file = None
        self.csv_writer = None
        self.last_print_time = rospy.Time.now()

        if not _is_disabled_output(args.csv):
            csv_path = _resolve_output_path(
                args.csv,
                f"ekf_live_metrics_{_run_id()}.csv",
                args.overwrite,
            )
            self.csv_file = open(csv_path, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                [
                    "time",
                    "x_gt",
                    "y_gt",
                    "yaw_gt",
                    "x_ekf",
                    "y_ekf",
                    "yaw_ekf",
                    "pos_err",
                    "yaw_err_deg",
                    "nees",
                ]
            )
            rospy.loginfo("EKF metrics CSV: %s", csv_path)

        prefix = args.publish_prefix.rstrip("/")
        self.publishers = {
            "position_rmse_m": rospy.Publisher(f"{prefix}/position_rmse", Float64, queue_size=10),
            "yaw_rmse_deg": rospy.Publisher(f"{prefix}/yaw_rmse_deg", Float64, queue_size=10),
            "mean_nees": rospy.Publisher(f"{prefix}/mean_nees", Float64, queue_size=10),
            "latest_position_error_m": rospy.Publisher(f"{prefix}/latest_position_error", Float64, queue_size=10),
            "latest_yaw_error_deg": rospy.Publisher(f"{prefix}/latest_yaw_error_deg", Float64, queue_size=10),
            "latest_nees": rospy.Publisher(f"{prefix}/latest_nees", Float64, queue_size=10),
            "samples": rospy.Publisher(f"{prefix}/samples", Float64, queue_size=10),
            "dropped_samples": rospy.Publisher(f"{prefix}/dropped_samples", Float64, queue_size=10),
        }

        rospy.Subscriber(args.gt_topic, DynamicsInfo, self.gt_callback, queue_size=100)
        rospy.Subscriber(args.odom_topic, Odometry, self.odom_callback, queue_size=100)
        self.timer = rospy.Timer(rospy.Duration(args.print_period), self.print_timer_callback)
        rospy.on_shutdown(self.close)

    def gt_callback(self, msg: Any) -> None:
        fallback = self.rospy.Time.now().to_sec()
        sample = _gt_from_msg(msg, fallback)
        with self.lock:
            self.acc.total_gt_samples += 1
            if sample is None:
                self.acc.invalid_samples += 1
                return
            self.gt_queue.append(sample)
            self._trim_queues()
            self._process_queues()

    def odom_callback(self, msg: Any) -> None:
        fallback = self.rospy.Time.now().to_sec()
        sample = _odom_from_msg(msg, fallback)
        with self.lock:
            self.acc.total_odom_samples += 1
            if sample is None:
                return
            self.odom_queue.append(sample)
            self._trim_queues()
            self._process_queues()

    def _trim_queues(self) -> None:
        # 오래 쌓인 미매칭 메시지는 메모리 증가를 막기 위해 queue-size 기준으로 제거한다.
        while len(self.odom_queue) > self.args.queue_size:
            self.odom_queue.popleft()
        while len(self.gt_queue) > self.args.queue_size:
            self.gt_queue.popleft()
            self.acc.unmatched_samples += 1

    def _process_queues(self) -> None:
        # GT 하나를 기준으로 max_dt 이내의 가장 가까운 odom을 찾아 1:1로 소비한다.
        while self.gt_queue and self.odom_queue:
            gt = self.gt_queue[0]

            while self.odom_queue and self.odom_queue[0].time < gt.time - self.args.max_dt:
                self.odom_queue.popleft()
            if not self.odom_queue:
                return

            best_index = min(
                range(len(self.odom_queue)),
                key=lambda idx: abs(self.odom_queue[idx].time - gt.time),
            )
            odom = self.odom_queue[best_index]
            best_dt = abs(odom.time - gt.time)

            if best_dt <= self.args.max_dt:
                self.gt_queue.popleft()
                for _ in range(best_index + 1):
                    self.odom_queue.popleft()
                self._add_match(gt, odom)
                continue

            if self.odom_queue[-1].time >= gt.time + self.args.max_dt:
                self.gt_queue.popleft()
                self.acc.unmatched_samples += 1
                continue

            return

    def _add_match(self, gt: GtSample, odom: OdomSample) -> None:
        result = _compute_row(gt, odom)
        if result is None:
            self.acc.invalid_samples += 1
            return
        row, yaw_err = result
        self.acc.add_row(row, yaw_err)

        if self.csv_writer is not None:
            self.csv_writer.writerow(
                [
                    f"{row.time:.9f}",
                    f"{row.x_gt:.9f}",
                    f"{row.y_gt:.9f}",
                    f"{row.yaw_gt:.9f}",
                    f"{row.x_ekf:.9f}",
                    f"{row.y_ekf:.9f}",
                    f"{row.yaw_ekf:.9f}",
                    f"{row.pos_err:.9f}",
                    f"{row.yaw_err_deg:.9f}",
                    "" if row.nees is None else f"{row.nees:.9f}",
                ]
            )
            if self.csv_file is not None:
                self.csv_file.flush()

        self._publish(row)

    def _publish(self, latest: MetricRow) -> None:
        # 누적 RMSE/NEES와 최신 순간 오차를 모두 topic으로 내보낸다.
        from std_msgs.msg import Float64  # type: ignore

        summary = self.acc.summary()
        for key, value in (
            ("position_rmse_m", summary["position_rmse_m"]),
            ("yaw_rmse_deg", summary["yaw_rmse_deg"]),
            ("mean_nees", summary["mean_nees"]),
            ("latest_position_error_m", latest.pos_err),
            ("latest_yaw_error_deg", latest.yaw_err_deg),
            ("latest_nees", math.nan if latest.nees is None else latest.nees),
            ("samples", summary["samples"]),
            ("dropped_samples", summary["dropped_samples"]),
        ):
            msg = Float64()
            msg.data = value
            self.publishers[key].publish(msg)

    def print_timer_callback(self, _event: Any) -> None:
        if self.args.quiet:
            return
        with self.lock:
            summary = self.acc.summary()
        self.rospy.loginfo("\n%s", _format_summary(summary))

    def close(self) -> None:
        if self.csv_file is not None:
            self.csv_file.close()


def run_live(args: argparse.Namespace) -> int:
    try:
        import rospy  # type: ignore
    except ImportError as exc:
        print(f"ERROR: rospy import failed: {exc}", file=sys.stderr)
        return 2

    rospy.init_node(args.node_name, anonymous=False)
    rospy.loginfo(
        "EKF metrics live evaluator: gt=%s odom=%s max_dt=%.3fs",
        args.gt_topic,
        args.odom_topic,
        args.max_dt,
    )
    LiveEvaluator(args, rospy)
    rospy.spin()
    return 0


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compute EKF Position RMSE, Yaw RMSE, and NEES from bag or live ROS topics."
    )
    subparsers = parser.add_subparsers(dest="mode", required=True)

    bag_parser = subparsers.add_parser("bag", help="Evaluate metrics from a rosbag.")
    bag_parser.add_argument("bag_file", help="Input .bag file.")
    bag_parser.add_argument("--gt-topic", default=DEFAULT_GT_TOPIC, help=f"GT topic. Default: {DEFAULT_GT_TOPIC}")
    bag_parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC, help=f"EKF odom topic. Default: {DEFAULT_ODOM_TOPIC}")
    bag_parser.add_argument("--max-dt", type=float, default=DEFAULT_MAX_DT, help="Max GT/odom match dt in seconds.")
    bag_parser.add_argument(
        "--csv",
        nargs="?",
        const="auto",
        default=DEFAULT_CSV,
        help="Output CSV path or directory. Use 'auto' for data/csv timestamp file. Default: auto.",
    )
    bag_parser.add_argument("--no-csv", action="store_true", help="Do not write per-sample CSV.")
    bag_parser.add_argument(
        "--summary-output",
        nargs="?",
        const="auto",
        default="",
        help="Optional text summary path or directory. Use without value for data/csv timestamp file.",
    )
    bag_parser.add_argument("--min-samples", type=int, default=1, help="Fail if fewer matched samples are produced.")
    bag_parser.add_argument("--overwrite", action="store_true", help="Overwrite output files if the path exists.")
    bag_parser.set_defaults(func=run_bag)

    live_parser = subparsers.add_parser("live", help="Evaluate metrics online from ROS topics.")
    live_parser.add_argument("--gt-topic", default=DEFAULT_GT_TOPIC, help=f"GT topic. Default: {DEFAULT_GT_TOPIC}")
    live_parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC, help=f"EKF odom topic. Default: {DEFAULT_ODOM_TOPIC}")
    live_parser.add_argument("--max-dt", type=float, default=DEFAULT_MAX_DT, help="Max GT/odom match dt in seconds.")
    live_parser.add_argument(
        "--csv",
        nargs="?",
        const="auto",
        default="",
        help="Optional live CSV path or directory. Use without value for data/csv timestamp file.",
    )
    live_parser.add_argument("--print-period", type=float, default=1.0, help="Summary log period in seconds.")
    live_parser.add_argument("--publish-prefix", default="/localization/eval", help="Output metric topic prefix.")
    live_parser.add_argument("--queue-size", type=int, default=2000, help="Max unmatched message queue length.")
    live_parser.add_argument("--node-name", default="ekf_metrics_evaluator", help="ROS node name.")
    live_parser.add_argument("--quiet", action="store_true", help="Disable periodic summary logs.")
    live_parser.add_argument("--overwrite", action="store_true", help="Overwrite output files if the path exists.")
    live_parser.set_defaults(func=run_live)

    args = parser.parse_args(argv)
    if args.max_dt <= 0.0:
        parser.error("--max-dt must be positive")
    if getattr(args, "print_period", 1.0) <= 0.0:
        parser.error("--print-period must be positive")
    if getattr(args, "queue_size", 1) <= 0:
        parser.error("--queue-size must be positive")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    sys.exit(main())
