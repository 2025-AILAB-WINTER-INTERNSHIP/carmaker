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
    ("cov_trace", "Mean Covariance Trace", "Trace", "inferno"),
    ("cov_determinant", "Mean Covariance Determinant", "Determinant", "inferno"),
    ("cov_xx", "Mean Covariance X Variance", "Variance (m^2)", "Reds"),
    ("cov_yy", "Mean Covariance Y Variance", "Variance (m^2)", "Reds"),
    ("cov_yawyaw", "Mean Covariance Yaw Variance", "Variance (rad^2)", "Reds"),
]

LEGACY_COLUMN_ALIASES = {
    "map_feature_count": "landmark_count",
}

DETAIL_COLUMNS = [
    "grid_i", "grid_j", "cell_x", "cell_y", "mean_observed_count", "landmark_count",
    "attempts", "success_count", "success_rate", "mean_fitness", "mean_iterations", "mean_latency_ms",
    "longitudinal_rmse", "lateral_rmse", "yaw_rmse",
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
    }
    yaml_path = os.path.join(package_dir, "config", "localization_params.yaml")
    if not os.path.exists(yaml_path):
        return cfg

    try:
        with open(yaml_path, "r", encoding="utf-8") as f:
            params = yaml.safe_load(f) or {}
        eval_params = params.get("evaluator", {})
        for key in cfg:
            cfg[key] = eval_params.get(key, cfg[key])
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


def load_landmarks(package_dir, cfg):
    path = resolve_config_path(package_dir, cfg["landmarks_csv_path"])
    if not os.path.exists(path) and cfg["landmarks_csv_path"] == "landmarks.csv":
        path = resolve_config_path(package_dir, "map_features.csv")
    if not os.path.exists(path):
        return None
    return pd.read_csv(path)


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


def plot_heatmap(df, column, title, colorbar_label, cmap, output_path, cfg, landmarks=None):
    grid_step = cfg["grid_step"]
    fig, ax = plt.subplots(figsize=(10, 8))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    for spine in ax.spines.values():
        spine.set_color('#e5e8eb')
        spine.set_linewidth(0.8)

    values = df[column].replace([np.inf, -np.inf], np.nan)
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
        loc="upper right",
        framealpha=1.0,
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
        "longitudinal_rmse", "lateral_rmse", "yaw_rmse",
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

    output_dir = resolve_config_path(package_dir, cfg["output_dir"])
    os.makedirs(output_dir, exist_ok=True)
    landmarks = load_landmarks(package_dir, cfg)

    for column, title, label, cmap in HEATMAPS:
        if column in df.columns:
            plot_heatmap(df, column, title, label, cmap, os.path.join(output_dir, f"heatmap_{column}.png"), cfg, landmarks)

    write_summary(df, os.path.join(output_dir, "evaluation_summary.md"), cfg)
    write_detail(df, os.path.join(output_dir, "evaluation_detail.md"))
    write_debug_html_reports(output_dir)
    print(f"[+] Evaluation heatmaps and markdown reports written to: {output_dir}")


if __name__ == "__main__":
    main()
