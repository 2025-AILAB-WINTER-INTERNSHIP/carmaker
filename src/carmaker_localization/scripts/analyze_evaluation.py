#!/usr/bin/env python3
import json
import os
import re
import sys

try:
    import numpy as np
    import pandas as pd
    import yaml
    import matplotlib.pyplot as plt
    import matplotlib.cm as cm
    from matplotlib.colors import Normalize
    from matplotlib.patches import Patch, Rectangle
    from matplotlib.collections import PatchCollection
except ImportError as e:
    print(f"[-] Missing python dependency: {e}")
    print("[*] Please install required libraries: pip install pandas numpy matplotlib pyyaml")
    sys.exit(1)

HEATMAPS = [
    ("mean_observed_count", "Mean Observed Feature Count", "Observed Features", "YlGnBu"),
    ("landmark_count", "Mean Landmark Feature Count", "Landmark Features", "YlGnBu"),
    ("attempts", "Registration Attempts", "Attempts", "YlGnBu"),
    ("success_count", "Registration Success Count", "Success Count", "YlGnBu"),
    ("success_rate", "Registration Success Rate", "Success Rate", "RdYlGn"),
    ("mean_fitness", "Mean Fitness Score", "Fitness Score", "plasma"),
    ("mean_iterations", "Mean ICP Iterations", "Iterations", "magma"),
    ("mean_latency_ms", "Mean ICP Latency", "Latency (ms)", "cividis"),
    ("longitudinal_rmse", "GT Pose Frame Longitudinal RMSE", "GT pose frame RMSE (m)", "viridis_r"),
    ("lateral_rmse", "GT Pose Frame Lateral RMSE", "GT pose frame RMSE (m)", "viridis_r"),
    ("yaw_rmse", "GT Pose Frame Yaw RMSE", "GT pose frame RMSE (rad)", "viridis_r"),
    ("yaw_rmse_deg", "GT Pose Frame Yaw RMSE", "GT pose frame RMSE (deg)", "viridis_r"),
    ("cov_trace", "Mean Covariance Trace", "Trace", "inferno"),
    ("cov_determinant", "Mean Covariance Determinant", "Determinant", "inferno"),
    ("cov_xx", "Mean Covariance X Variance", "Variance (m^2)", "Reds"),
    ("cov_yy", "Mean Covariance Y Variance", "Variance (m^2)", "Reds"),
    ("cov_yawyaw", "Mean Covariance Yaw Variance", "Variance (rad^2)", "Reds"),
    ("cov_yawyaw_deg2", "Mean Covariance Yaw Variance", "Variance (deg^2)", "Reds"),
    ("cov_xyaw", "Mean X-Yaw Covariance", "Covariance (m*rad)", "coolwarm"),
    ("cov_yyaw", "Mean Y-Yaw Covariance", "Covariance (m*rad)", "coolwarm"),
    ("cov_xyaw_deg", "Mean X-Yaw Covariance", "Covariance (m*deg)", "coolwarm"),
    ("cov_yyaw_deg", "Mean Y-Yaw Covariance", "Covariance (m*deg)", "coolwarm"),
    ("abs_cov_xyaw", "Mean Absolute X-Yaw Covariance", "|Covariance| (m*rad)", "Reds"),
    ("abs_cov_yyaw", "Mean Absolute Y-Yaw Covariance", "|Covariance| (m*rad)", "Reds"),
    ("abs_cov_xyaw_deg", "Mean Absolute X-Yaw Covariance", "|Covariance| (m*deg)", "Reds"),
    ("abs_cov_yyaw_deg", "Mean Absolute Y-Yaw Covariance", "|Covariance| (m*deg)", "Reds"),
    ("std_x", "Mean ICP X Standard Deviation", "Std Dev (m)", "Reds"),
    ("std_y", "Mean ICP Y Standard Deviation", "Std Dev (m)", "Reds"),
    ("std_position", "Mean ICP Position Standard Deviation", "Std Dev (m)", "Reds"),
    ("std_position_cm", "Mean ICP Position Standard Deviation", "Std Dev (cm)", "Reds"),
    ("std_yaw", "Mean ICP Yaw Standard Deviation", "Std Dev (rad)", "Reds"),
    ("std_yaw_deg", "Mean ICP Yaw Standard Deviation", "Std Dev (deg)", "Reds"),
    ("yaw_translation_coupling", "Mean Yaw-Translation Coupling", "Coupling (m*rad)", "Reds"),
    ("yaw_translation_coupling_deg", "Mean Yaw-Translation Coupling", "Coupling (m*deg)", "Reds"),
]

LEGACY_COLUMN_ALIASES = {
    "map_feature_count": "landmark_count",
}

DETAIL_COLUMNS = [
    "grid_i", "grid_j", "cell_x", "cell_y", "mean_observed_count", "landmark_count",
    "attempts", "success_count", "success_rate", "mean_fitness", "mean_iterations", "mean_latency_ms",
    "longitudinal_rmse", "lateral_rmse", "yaw_rmse", "yaw_rmse_deg",
    "std_x", "std_y", "std_position", "std_position_cm", "std_yaw", "std_yaw_deg",
    "yaw_translation_coupling", "yaw_translation_coupling_deg", "cov_yawyaw_deg2",
    "cov_xx", "cov_xy", "cov_xyaw", "cov_yx", "cov_yy", "cov_yyaw",
    "cov_yawx", "cov_yawy", "cov_yawyaw", "cov_trace", "cov_determinant",
]

def resolve_package_dir():
    script_package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if os.path.exists(os.path.join(script_package_dir, "config", "localization_params.yaml")):
        return script_package_dir

    try:
        import rospkg
        return rospkg.RosPack().get_path("carmaker_localization")
    except ImportError:
        return script_package_dir


def load_config(package_dir):
    cfg = {
        "center_x": 0.0,
        "center_y": -4.33,
        "grid_step": 1.0,
        "output_csv_path": "grid_registration_results.csv",
        "output_dir": "docs",
        "landmarks_csv_path": "landmarks.csv",
        "landmark_file": "config/Test.osm",
    }
    yaml_path = os.path.join(package_dir, "config", "localization_params.yaml")
    if not os.path.exists(yaml_path):
        return cfg

    try:
        with open(yaml_path, "r", encoding="utf-8") as f:
            params = yaml.safe_load(f) or {}
        eval_params = params.get("evaluator", {})
        for key in cfg:
            if key in eval_params:
                cfg[key] = eval_params[key]
        
        reg_params = params.get("feature_registration", {})
        landmark_file = reg_params.get("landmark_file", "") or eval_params.get("map_file", "")
        if landmark_file:
            cfg["landmark_file"] = landmark_file

        cfg["landmarks_csv_path"] = eval_params.get(
            "landmarks_csv_path",
            eval_params.get("map_features_csv_path", cfg["landmarks_csv_path"]),
        )
    except Exception as e:
        print(f"[-] Failed to load yaml parameters: {e}")
    cfg["grid_step"] = max(float(cfg["grid_step"]), 1e-3)
    return cfg


def resolve_config_path(package_dir, path):
    return path if os.path.isabs(path) else os.path.join(package_dir, path)


def normalize_schema(df):
    for legacy, current in LEGACY_COLUMN_ALIASES.items():
        if current not in df.columns and legacy in df.columns:
            df[current] = df[legacy]
    return df


def add_standard_deviation_columns(df):
    variance_to_std = {
        "cov_xx": "std_x",
        "cov_yy": "std_y",
        "cov_yawyaw": "std_yaw",
    }
    for variance_column, std_column in variance_to_std.items():
        if variance_column not in df.columns:
            df[std_column] = np.nan
            continue
        variances = pd.to_numeric(df[variance_column], errors="coerce").replace([np.inf, -np.inf], np.nan)
        df[std_column] = np.sqrt(variances.where(variances >= 0.0, np.nan))

    if "cov_xx" in df.columns and "cov_yy" in df.columns:
        cov_xx = pd.to_numeric(df["cov_xx"], errors="coerce").replace([np.inf, -np.inf], np.nan)
        cov_yy = pd.to_numeric(df["cov_yy"], errors="coerce").replace([np.inf, -np.inf], np.nan)
        position_variance = cov_xx + cov_yy
        df["std_position"] = np.sqrt(position_variance.where(position_variance >= 0.0, np.nan))
        df["std_position_cm"] = df["std_position"] * 100.0
    else:
        df["std_position"] = np.nan
        df["std_position_cm"] = np.nan

    if "cov_yawyaw" in df.columns:
        yaw_variance = pd.to_numeric(df["cov_yawyaw"], errors="coerce").replace([np.inf, -np.inf], np.nan)
        yaw_variance = yaw_variance.where(yaw_variance >= 0.0, np.nan)
        rad_to_deg = 180.0 / np.pi
        df["cov_yawyaw_deg2"] = yaw_variance * (rad_to_deg ** 2)
        df["std_yaw_deg"] = np.sqrt(yaw_variance) * rad_to_deg
    else:
        df["cov_yawyaw_deg2"] = np.nan
        df["std_yaw_deg"] = np.nan
    return df


def add_yaw_degree_columns(df):
    if "yaw_rmse" in df.columns:
        yaw_rmse = pd.to_numeric(df["yaw_rmse"], errors="coerce").replace([np.inf, -np.inf], np.nan)
        df["yaw_rmse_deg"] = yaw_rmse * 180.0 / np.pi
    else:
        df["yaw_rmse_deg"] = np.nan
    return df


def add_covariance_coupling_columns(df):
    rad_to_deg = 180.0 / np.pi
    for column in ("cov_xyaw", "cov_yyaw"):
        abs_column = f"abs_{column}"
        deg_column = f"{column}_deg"
        abs_deg_column = f"abs_{column}_deg"
        if column in df.columns:
            values = pd.to_numeric(df[column], errors="coerce").replace([np.inf, -np.inf], np.nan)
            df[abs_column] = values.abs()
            df[deg_column] = values * rad_to_deg
            df[abs_deg_column] = values.abs() * rad_to_deg
        else:
            df[abs_column] = np.nan
            df[deg_column] = np.nan
            df[abs_deg_column] = np.nan
    if "cov_xyaw" in df.columns and "cov_yyaw" in df.columns:
        cov_xyaw = pd.to_numeric(df["cov_xyaw"], errors="coerce").replace([np.inf, -np.inf], np.nan)
        cov_yyaw = pd.to_numeric(df["cov_yyaw"], errors="coerce").replace([np.inf, -np.inf], np.nan)
        df["yaw_translation_coupling"] = np.sqrt(cov_xyaw.pow(2) + cov_yyaw.pow(2))
        df["yaw_translation_coupling_deg"] = df["yaw_translation_coupling"] * rad_to_deg
    else:
        df["yaw_translation_coupling"] = np.nan
        df["yaw_translation_coupling_deg"] = np.nan
    return df


def format_value(value, digits=6):
    return "N/A" if pd.isna(value) else f"{value:.{digits}f}"


def metric_stats(df, column):
    values = df[column].replace([np.inf, -np.inf], np.nan).dropna()
    if values.empty:
        return {"mean": np.nan, "min": np.nan, "median": np.nan, "max": np.nan}
    return {
        "mean": float(values.mean()),
        "min": float(values.min()),
        "median": float(values.median()),
        "max": float(values.max()),
    }


def weighted_mean(group, column, weight_column):
    if column not in group.columns or weight_column not in group.columns:
        return np.nan
    values = group[column].replace([np.inf, -np.inf], np.nan)
    weights = group[weight_column].replace([np.inf, -np.inf], np.nan).fillna(0.0)
    mask = values.notna() & (weights > 0.0)
    if not mask.any():
        return np.nan
    return float((values[mask] * weights[mask]).sum() / weights[mask].sum())


def weighted_rmse(group, column, weight_column):
    if column not in group.columns or weight_column not in group.columns:
        return np.nan
    values = group[column].replace([np.inf, -np.inf], np.nan)
    weights = group[weight_column].replace([np.inf, -np.inf], np.nan).fillna(0.0)
    mask = values.notna() & (weights > 0.0)
    if not mask.any():
        return np.nan
    return float(np.sqrt(((values[mask] ** 2) * weights[mask]).sum() / weights[mask].sum()))


def aggregate_duplicate_grids(df):
    required = {"grid_i", "grid_j", "cell_x", "cell_y", "attempts", "success_count"}
    if not required.issubset(df.columns):
        return df

    rows = []
    for (_, _), group in df.groupby(["grid_i", "grid_j"], sort=True):
        attempts = group["attempts"].replace([np.inf, -np.inf], np.nan).fillna(0.0).sum()
        successes = group["success_count"].replace([np.inf, -np.inf], np.nan).fillna(0.0).sum()
        row = {
            "grid_i": int(group["grid_i"].iloc[0]),
            "grid_j": int(group["grid_j"].iloc[0]),
            "cell_x": float(group["cell_x"].iloc[0]),
            "cell_y": float(group["cell_y"].iloc[0]),
            "attempts": float(attempts),
            "success_count": float(successes),
            "success_rate": float(successes / attempts) if attempts > 0.0 else 0.0,
            "mean_observed_count": weighted_mean(group, "mean_observed_count", "attempts"),
            "landmark_count": weighted_mean(group, "landmark_count", "attempts"),
            "mean_fitness": weighted_mean(group, "mean_fitness", "success_count"),
            "mean_iterations": weighted_mean(group, "mean_iterations", "success_count"),
            "mean_latency_ms": weighted_mean(group, "mean_latency_ms", "success_count"),
            "longitudinal_rmse": weighted_rmse(group, "longitudinal_rmse", "success_count"),
            "lateral_rmse": weighted_rmse(group, "lateral_rmse", "success_count"),
            "yaw_rmse": weighted_rmse(group, "yaw_rmse", "success_count"),
        }
        for column in (
            "cov_xx", "cov_xy", "cov_xyaw", "cov_yx", "cov_yy", "cov_yyaw",
            "cov_yawx", "cov_yawy", "cov_yawyaw", "cov_trace", "cov_determinant",
        ):
            row[column] = weighted_mean(group, column, "success_count")
        rows.append(row)

    aggregated = pd.DataFrame(rows)
    ordered = [column for column in df.columns if column in aggregated.columns]
    extras = [column for column in aggregated.columns if column not in ordered]
    return aggregated[ordered + extras]


def parse_osm_to_landmarks(osm_path):
    import xml.etree.ElementTree as ET
    import math
    try:
        tree = ET.parse(osm_path)
    except Exception as e:
        print(f"[-] Failed to parse OSM file {osm_path}: {e}")
        return None

    root = tree.getroot()
    nodes = {}
    for node in root.findall('node'):
        nid_str = node.get('id')
        if not nid_str:
            continue
        nid = int(nid_str)
        local_x, local_y = None, None
        for tag in node.findall('tag'):
            if tag.get('k') == 'local_x':
                local_x = float(tag.get('v'))
            elif tag.get('k') == 'local_y':
                local_y = float(tag.get('v'))
        if local_x is not None and local_y is not None:
            nodes[nid] = (local_x, local_y)

    sampled_points = []
    
    def sample_segment(x1, y1, x2, y2, res=0.05):
        points = []
        dx = x2 - x1
        dy = y2 - y1
        dist = math.hypot(dx, dy)
        if dist == 0.0:
            return [(x1, y1)]
        steps = int(math.ceil(dist / res))
        for step in range(steps + 1):
            t = step / steps
            points.append((x1 + t * dx, y1 + t * dy))
        return points

    for way in root.findall('way'):
        tags = {tag.get('k'): tag.get('v') for tag in way.findall('tag') if tag.get('k') and tag.get('v')}
        class_id = None
        if tags.get('amenity') == 'ev_charging' or tags.get('type') == 'ev_charging':
            class_id = 2
        elif tags.get('type') in ('parking_spot', 'lane', 'line'):
            class_id = 1

        if class_id is None:
            continue

        nd_refs = [int(nd.get('ref')) for nd in way.findall('nd') if nd.get('ref')]
        if not nd_refs:
            continue

        for i in range(len(nd_refs) - 1):
            ref1, ref2 = nd_refs[i], nd_refs[i+1]
            if ref1 in nodes and ref2 in nodes:
                p1, p2 = nodes[ref1], nodes[ref2]
                pts = sample_segment(p1[0], p1[1], p2[0], p2[1], res=0.05)
                for pt in pts:
                    sampled_points.append((class_id, pt[0], pt[1]))

        if class_id == 2 and nd_refs[0] != nd_refs[-1]:
            ref1, ref2 = nd_refs[-1], nd_refs[0]
            if ref1 in nodes and ref2 in nodes:
                p1, p2 = nodes[ref1], nodes[ref2]
                pts = sample_segment(p1[0], p1[1], p2[0], p2[1], res=0.05)
                for pt in pts:
                    sampled_points.append((class_id, pt[0], pt[1]))

    if not sampled_points:
        return None

    voxel_size = 0.05
    voxels = {}
    for cid, x, y in sampled_points:
        vx = int(round(x / voxel_size))
        vy = int(round(y / voxel_size))
        key = (cid, vx, vy)
        if key not in voxels:
            voxels[key] = (vx * voxel_size, vy * voxel_size)

    rows = []
    for (cid, _, _), (x, y) in voxels.items():
        rows.append({"class_id": cid, "x": x, "y": y})

    return pd.DataFrame(rows)


def load_landmarks(package_dir, cfg):
    path = resolve_config_path(package_dir, cfg["landmarks_csv_path"])
    if not os.path.exists(path) and cfg["landmarks_csv_path"] == "landmarks.csv":
        path = resolve_config_path(package_dir, "map_features.csv")
    if os.path.exists(path):
        return pd.read_csv(path)

    landmark_file = cfg.get("landmark_file", "config/Test.osm")
    osm_path = resolve_config_path(package_dir, landmark_file)
    if os.path.exists(osm_path):
        print(f"[!] {cfg['landmarks_csv_path']} not found. Auto-generating from OSM file: {osm_path}")
        landmarks_df = parse_osm_to_landmarks(osm_path)
        if landmarks_df is not None:
            csv_out = resolve_config_path(package_dir, cfg["landmarks_csv_path"])
            landmarks_df.to_csv(csv_out, index=False)
            print(f"[+] Re-generated and saved landmarks CSV to: {csv_out}")
            return landmarks_df

    return None


def write_debug_html(json_path, html_path):
    data = None
    try:
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except json.JSONDecodeError as e:
        with open(json_path, "r", encoding="utf-8") as f:
            raw = f.read()
        fixed_raw = re.sub(r'([,{])([A-Za-z_][A-Za-z0-9_]*):', r'\1"\2":', raw)
        try:
            data = json.loads(fixed_raw)
            print(f"[!] Parsed legacy non-standard ICP debug JSON: {json_path}")
        except json.JSONDecodeError:
            raise RuntimeError(
                f"Failed to parse ICP debug JSON '{json_path}' at line {e.lineno}, "
                f"column {e.colno}: {e.msg}"
            ) from e

    package_dir = resolve_package_dir()
    template_path = os.path.join(package_dir, "resources", "icp_debug_template.html")
    if not os.path.exists(template_path):
        template_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "resources", "icp_debug_template.html")
        if not os.path.exists(template_path):
            raise FileNotFoundError(f"ICP debug HTML template not found at: {template_path}")

    with open(template_path, "r", encoding="utf-8") as f:
        template = f.read()

    payload = json.dumps(data, separators=(",", ":"))
    html_content = template.replace("__DEBUG_DATA_PLACEHOLDER__", payload)

    with open(html_path, "w", encoding="utf-8") as f:
        f.write(html_content)


def write_debug_html_reports(output_dir):
    for name in sorted(os.listdir(output_dir)):
        if not name.startswith("icp_debug_") or not name.endswith(".json"):
            continue
        json_path = os.path.join(output_dir, name)
        html_path = os.path.join(output_dir, name[:-5] + ".html")
        write_debug_html(json_path, html_path)


def plot_heatmap(df, column, title, colorbar_label, cmap, output_path, cfg, landmarks=None, filter_perfect=False):
    grid_step = cfg["grid_step"]
    fig, ax = plt.subplots(figsize=(10, 8))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    for spine in ax.spines.values():
        spine.set_color('#e5e8eb')
        spine.set_linewidth(0.8)

    values = df[column].replace([np.inf, -np.inf], np.nan)
    if filter_perfect and "success_rate" in df.columns:
        values = values.copy()
        values[df["success_rate"] < 0.9999] = np.nan
    finite = values.dropna()
    if column == "success_rate":
        vmin, vmax = 0.0, 1.0
    else:
        vmin = float(finite.min()) if not finite.empty else 0.0
        vmax = float(finite.max()) if not finite.empty else 1.0
        if vmin == vmax:
            vmin = 0.0
            vmax = 1.0 if vmax == 0.0 else vmax

    try:
        colormap = plt.colormaps[cmap]
    except AttributeError:
        colormap = cm.get_cmap(cmap)
    norm = Normalize(vmin=vmin, vmax=vmax)

    patches = []
    colors = []
    for index, row in df.iterrows():
        patches.append(Rectangle((row["cell_x"] - grid_step / 2.0, row["cell_y"] - grid_step / 2.0), grid_step, grid_step))
        val = values.loc[index]
        colors.append((0.9, 0.9, 0.9, 0.65) if pd.isna(val) else colormap(norm(val)))

    ax.add_collection(PatchCollection(patches, facecolors=colors, edgecolors="#e5e8eb", linewidths=0.5))

    feature_handles = []
    feature_labels = []
    if landmarks is not None and not landmarks.empty:
        for class_id in sorted(landmarks["class_id"].dropna().unique()):
            class_df = landmarks[landmarks["class_id"] == class_id]
            handle = ax.scatter(
                class_df["x"], class_df["y"],
                s=5, alpha=0.65, edgecolors="white", linewidths=0.3,
                label=f"Landmark feature class {int(class_id)}",
            )
            feature_handles.append(handle)
            feature_labels.append(f"Landmark feature class {int(class_id)}")

    cell_handle = ax.scatter(df["cell_x"], df["cell_y"], color="black", marker="+", s=18, linewidths=1.0, zorder=5)
    pad_handle = ax.scatter(
        [cfg["center_x"]], [cfg["center_y"]],
        color="#ff5722", marker="*", s=90, edgecolor="white", linewidths=0.8, zorder=6
    )
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, linestyle="--", alpha=0.2, color="#cccccc")
    ax.legend(
        [cell_handle, pad_handle, Patch(facecolor=(0.9, 0.9, 0.9, 0.65), edgecolor="#e5e8eb")] + feature_handles,
        ["GT RearAxle grid center", "Charging pad center", "N/A"] + feature_labels,
        loc="lower right",
        framealpha=0.65,
        facecolor="white",
        edgecolor="#e5e8eb",
        markerscale=2.0,
    )

    if not df.empty:
        ax.set_xlim(df["cell_x"].min() - grid_step, df["cell_x"].max() + grid_step)
        ax.set_ylim(df["cell_y"].min() - grid_step, df["cell_y"].max() + grid_step)

    sm = cm.ScalarMappable(cmap=colormap, norm=norm)
    sm.set_array([])
    fig.colorbar(sm, ax=ax, label=colorbar_label)
    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    plt.close(fig)


def write_summary(df, output_path, cfg):
    metrics = [
        "mean_observed_count", "landmark_count", "success_rate",
        "mean_fitness", "mean_iterations", "mean_latency_ms",
        "longitudinal_rmse", "lateral_rmse", "yaw_rmse", "yaw_rmse_deg",
        "std_x", "std_y", "std_position", "std_position_cm", "std_yaw", "std_yaw_deg",
        "yaw_translation_coupling", "yaw_translation_coupling_deg",
        "cov_yawyaw_deg2", "abs_cov_xyaw", "abs_cov_yyaw",
        "abs_cov_xyaw_deg", "abs_cov_yyaw_deg",
        "cov_trace", "cov_determinant",
    ]
    valid_grid_count = int((df["attempts"] > 0).sum())
    no_success_count = int((df["success_count"] <= 0).sum())

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("# ICP Evaluation Summary\n\n")
        f.write(
            "Grid cells are centered on the charging pad frame origin defined by "
            f"`center_x={cfg['center_x']:.3f}`, `center_y={cfg['center_y']:.3f}`, "
            f"`grid_step={cfg['grid_step']:.3f}`. Each cell center is the GT RearAxle landmarks "
            "position for static evaluation and the intended spawn/stop position for bag evaluation. "
            "Static evaluation estimates pose from the landmarks visible at each GT position and sampled "
            "GT yaw. Bag evaluation estimates pose from segmentation BEV features. Longitudinal, lateral, "
            "and yaw RMSE are computed in the GT pose frame; static rows aggregate all sampled GT yaw angles "
            "for the cell. Heatmap overlays show landmarks used by ICP, not simulator "
            "or segmentation BEV sensor observations.\n\n"
        )
        f.write(f"- Total grid cells: {len(df)}\n")
        f.write(f"- Valid grid cells: {valid_grid_count}\n")
        f.write(f"- Cells without successful ICP registration: {no_success_count}\n\n")
        f.write("| Metric | Mean | Min | Median | Max |\n")
        f.write("| --- | ---: | ---: | ---: | ---: |\n")
        for metric in metrics:
            stats = metric_stats(df, metric)
            f.write(
                f"| {metric} | {format_value(stats['mean'])} | {format_value(stats['min'])} | "
                f"{format_value(stats['median'])} | {format_value(stats['max'])} |\n"
            )


def write_detail(df, output_path):
    for column in DETAIL_COLUMNS:
        if column not in df.columns:
            df[column] = np.nan
    detail = df[DETAIL_COLUMNS].copy()
    detail = detail.sort_values(["success_rate", "grid_j", "grid_i"], ascending=[True, True, True])
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("# ICP Evaluation Detail\n\n")
        f.write("| " + " | ".join(DETAIL_COLUMNS) + " |\n")
        f.write("| " + " | ".join(["---:"] * len(DETAIL_COLUMNS)) + " |\n")
        for _, row in detail.iterrows():
            values = []
            for column in DETAIL_COLUMNS:
                if column in ("grid_i", "grid_j"):
                    values.append(str(int(row[column])))
                elif column in ("attempts", "success_count"):
                    values.append(str(int(row[column])))
                else:
                    values.append(format_value(row[column], 4))
            f.write("| " + " | ".join(values) + " |\n")


def main():
    package_dir = resolve_package_dir()
    cfg = load_config(package_dir)
    csv_path = cfg["output_csv_path"]
    if not os.path.isabs(csv_path):
        csv_path = os.path.join(package_dir, csv_path)
    if len(sys.argv) >= 2:
        csv_path = sys.argv[1]

    if not os.path.exists(csv_path):
        print(f"[-] CSV file not found: {csv_path}")
        sys.exit(1)

    df = normalize_schema(pd.read_csv(csv_path))
    if df.empty:
        print(f"[-] CSV file has no rows: {csv_path}")
        sys.exit(1)
    for column in ("mean_observed_count", "landmark_count"):
        if column not in df.columns:
            df[column] = np.nan
    df = aggregate_duplicate_grids(df)
    df = add_standard_deviation_columns(df)
    df = add_yaw_degree_columns(df)
    df = add_covariance_coupling_columns(df)

    output_dir = resolve_config_path(package_dir, cfg["output_dir"])
    os.makedirs(output_dir, exist_ok=True)
    landmarks = load_landmarks(package_dir, cfg)

    for column, title, label, cmap in HEATMAPS:
        if column in df.columns:
            plot_heatmap(df, column, title, label, cmap, os.path.join(output_dir, f"heatmap_{column}.png"), cfg, landmarks)
            if column in ("longitudinal_rmse", "lateral_rmse", "yaw_rmse", "yaw_rmse_deg"):
                plot_heatmap(
                    df, column, f"{title}", label, cmap,
                    os.path.join(output_dir, f"heatmap_{column}_perfect.png"),
                    cfg, landmarks, filter_perfect=True
                )

    write_summary(df, os.path.join(output_dir, "evaluation_summary.md"), cfg)
    write_detail(df, os.path.join(output_dir, "evaluation_detail.md"))
    write_debug_html_reports(output_dir)
    print(f"[+] Evaluation heatmaps and markdown reports written to: {output_dir}")


if __name__ == "__main__":
    main()
