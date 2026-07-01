"""
Analiza UAANET ns-3 CSV izlaza.

Ulaz:
  CSV koji generise uaanet_aodv_1.cc.
  Jedan red = jedan ns-3 run za jedan nUavs i jedan RNG run.

Izlaz:
  - summary_by_protocol_nuavs.csv
  - 4-subplot grafici za C2 control, C2 command, Telemetry i Video/Image
  - routing overhead grafici

Primjer:
  python3 analyze_aodv_scaling_fixed.py \
    --csv results/aodv_scaling/aodv_scaling_results.csv \
    --out-dir results/aodv_scaling/analysis
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


TRAFFIC_TYPES: list[tuple[str, str]] = [
    ("c2Control", "C2 kontrola"),
    ("c2Command", "C2 komanda"),
    ("telemetry", "Telemetrija"),
    ("video", "Video/Image"),
]

METRIC_PLOTS: list[tuple[str, str, str, str]] = [
    ("delayMs", "Prosječno kašnjenje (ms)", "Prosječni delay po tipu saobraćaja", "delay_4traffic_subplots.png"),
    ("pdr", "PDR (%)", "PDR po tipu saobraćaja", "pdr_4traffic_subplots.png"),
    ("loss", "Packet loss (%)", "Packet loss po tipu saobraćaja", "loss_4traffic_subplots.png"),
    ("throughputKbps", "Throughput (kbps)", "Throughput po tipu saobraćaja", "throughput_4traffic_subplots.png"),
    ("offeredLoadKbps", "Offered load (kbps)", "Ponuđeno opterećenje po tipu saobraćaja", "offered_load_4traffic_subplots.png"),
]

ROUTING_PLOTS: list[tuple[str, str, str, str]] = [
    ("routing_offeredLoadKbps", "Routing offered load (kbps)", "Routing overhead load", "routing_offered_load.png"),
    ("routingBytesPerDeliveredAppByte", "Routing bytes / delivered app byte", "Normalizovani routing overhead po byte-u", "routing_bytes_per_delivered_app_byte.png"),
    ("routingPacketsPerDeliveredAppPacket", "Routing packets / delivered app packet", "Normalizovani routing overhead po paketu", "routing_packets_per_delivered_app_packet.png"),
]

ID_COLUMNS = {"protocol", "run", "nGcs", "nUavs", "nNodes"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Crtanje UAANET grafika iz ns-3 CSV fajla.")
    parser.add_argument("--csv", required=True, help="Ulazni CSV iz ns-3 simulacija")
    parser.add_argument(
        "--out-dir",
        default="results/aodv_scaling/analysis",
        help="Folder za summary CSV i PNG grafike",
    )
    parser.add_argument("--dpi", type=int, default=200, help="DPI za PNG grafike")
    parser.add_argument(
        "--no-error-bars",
        action="store_true",
        help="Isključi std error barove na graficima",
    )
    return parser.parse_args()


def require_columns(df: pd.DataFrame, columns: Iterable[str]) -> None:
    missing = [col for col in columns if col not in df.columns]
    if missing:
        raise ValueError(f"CSV nema potrebne kolone: {missing}")


def flatten_columns(columns: pd.MultiIndex) -> list[str]:
    flat: list[str] = []
    for col in columns:
        parts = [str(part) for part in col if str(part) != ""]
        flat.append("_".join(parts))
    return flat


def prepare_dataframe(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    require_columns(df, ["protocol", "run", "nUavs"])

    # Osiguraj numeričke tipove za sve kolone osim protocol.
    for col in df.columns:
        if col != "protocol":
            df[col] = pd.to_numeric(df[col], errors="coerce")

    return df


def metric_columns_present(df: pd.DataFrame) -> list[str]:
    wanted: list[str] = []

    for prefix, _label in TRAFFIC_TYPES:
        for suffix, _ylabel, _title, _filename in METRIC_PLOTS:
            col = f"{prefix}_{suffix}"
            if col in df.columns:
                wanted.append(col)

        # Ove kolone su korisne u summary CSV-u, iako ih ne crtamo kao glavne grafike.
        for suffix in ["flowCount", "successfulFlows", "txPackets", "rxPackets"]:
            col = f"{prefix}_{suffix}"
            if col in df.columns:
                wanted.append(col)

    for col, _ylabel, _title, _filename in ROUTING_PLOTS:
        if col in df.columns:
            wanted.append(col)

    for col in ["routing_flowEntries", "routing_activeEntries", "routing_txPackets", "routing_rxPackets", "routing_txBytes", "routing_rxBytes"]:
        if col in df.columns:
            wanted.append(col)

    # Bez Remote-ID metrika u ovoj fazi.
    return list(dict.fromkeys(wanted))


def make_summary(df: pd.DataFrame, metric_cols: list[str], out_dir: Path) -> pd.DataFrame:
    summary = (
        df.groupby(["protocol", "nUavs"], dropna=False)[metric_cols]
        .agg(["mean", "std", "count"])
        .reset_index()
    )
    summary.columns = flatten_columns(summary.columns)

    # Kod jednog runa std bude NaN; za crtanje je praktičnije 0.
    for col in summary.columns:
        if col.endswith("_std"):
            summary[col] = summary[col].fillna(0.0)

    summary_path = out_dir / "summary_by_protocol_nuavs.csv"
    summary.to_csv(summary_path, index=False)
    return summary


def plot_traffic_metric(summary: pd.DataFrame, suffix: str, ylabel: str, title: str, out_path: Path, dpi: int, show_error_bars: bool) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    axes_flat = axes.flatten()

    protocols = sorted(summary["protocol"].dropna().astype(str).unique())
    handles: list = []
    labels: list[str] = []

    for ax, (prefix, traffic_label) in zip(axes_flat, TRAFFIC_TYPES):
        mean_col = f"{prefix}_{suffix}_mean"
        std_col = f"{prefix}_{suffix}_std"
        count_col = f"{prefix}_{suffix}_count"

        ax.set_title(traffic_label)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("Broj UAV čvorova")
        ax.set_ylabel(ylabel)

        if mean_col not in summary.columns:
            ax.text(0.5, 0.5, f"Nema kolone\n{prefix}_{suffix}", ha="center", va="center", transform=ax.transAxes)
            continue

        for protocol in protocols:
            sub = summary[summary["protocol"].astype(str) == protocol].sort_values("nUavs")
            sub = sub.dropna(subset=[mean_col])
            if sub.empty:
                continue

            yerr = None
            if show_error_bars and std_col in sub.columns:
                yerr = sub[std_col].fillna(0.0)

            line = ax.errorbar(
                sub["nUavs"],
                sub[mean_col],
                yerr=yerr,
                marker="o",
                capsize=4,
                label=protocol,
            )

            if protocol not in labels:
                handles.append(line)
                labels.append(protocol)

        if suffix in {"pdr", "loss"}:
            ax.set_ylim(0, 100)
        else:
            ax.set_ylim(bottom=0)

        if count_col in summary.columns:
            # Diskretan x-axis je čitljiviji za nUavs vrijednosti.
            ax.set_xticks(sorted(summary["nUavs"].dropna().unique()))

    fig.suptitle(title, fontsize=14, y=0.99)
    if handles:
        fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.96), ncol=max(1, len(labels)))
        fig.subplots_adjust(top=0.86)
    else:
        fig.subplots_adjust(top=0.90)

    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)


def plot_single_routing_metric(summary: pd.DataFrame, metric: str, ylabel: str, title: str, out_path: Path, dpi: int, show_error_bars: bool) -> None:
    mean_col = f"{metric}_mean"
    std_col = f"{metric}_std"

    if mean_col not in summary.columns:
        print(f"Preskačem {metric}: nema kolone {mean_col}")
        return

    plt.figure(figsize=(8, 5))

    protocols = sorted(summary["protocol"].dropna().astype(str).unique())
    for protocol in protocols:
        sub = summary[summary["protocol"].astype(str) == protocol].sort_values("nUavs")
        sub = sub.dropna(subset=[mean_col])
        if sub.empty:
            continue

        yerr = None
        if show_error_bars and std_col in sub.columns:
            yerr = sub[std_col].fillna(0.0)

        plt.errorbar(
            sub["nUavs"],
            sub[mean_col],
            yerr=yerr,
            marker="o",
            capsize=4,
            label=protocol,
        )

    plt.xlabel("Broj UAV čvorova")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True, alpha=0.3)
    plt.xticks(sorted(summary["nUavs"].dropna().unique()))
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=dpi)
    plt.close()


def main() -> None:
    args = parse_args()
    csv_path = Path(args.csv)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    df = prepare_dataframe(csv_path)
    metric_cols = metric_columns_present(df)
    if not metric_cols:
        raise ValueError("Nema prepoznatih UAANET metrika u CSV-u.")

    summary = make_summary(df, metric_cols, out_dir)
    show_error_bars = not args.no_error_bars

    for suffix, ylabel, title, filename in METRIC_PLOTS:
        plot_traffic_metric(
            summary=summary,
            suffix=suffix,
            ylabel=ylabel,
            title=title,
            out_path=out_dir / filename,
            dpi=args.dpi,
            show_error_bars=show_error_bars,
        )

    for metric, ylabel, title, filename in ROUTING_PLOTS:
        plot_single_routing_metric(
            summary=summary,
            metric=metric,
            ylabel=ylabel,
            title=title,
            out_path=out_dir / filename,
            dpi=args.dpi,
            show_error_bars=show_error_bars,
        )

    print(f"Učitano redova: {len(df)}")
    print(f"Broj protocol/nUavs grupa: {len(summary)}")
    print(f"Summary CSV: {out_dir / 'summary_by_protocol_nuavs.csv'}")
    print(f"Grafici: {out_dir}")


if __name__ == "__main__":
    main()
