#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_odom.py - Graficas de /odom a partir de ros2 bags (ROS 2 Humble).

"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
except ImportError as exc:  # pragma: no cover
    print("[ERROR] No se pudo importar ROS 2 (rosbag2_py / rclpy).\n"
          f"        Detalle: {exc}\n"
          "        En Ubuntu:  source /opt/ros/humble/setup.bash\n"
          "        En Windows: usa plot_odom_win.py (no necesita ROS).")
    sys.exit(2)

# ---------------------------------------------------------------------------
# Estilo (mismo que error_odom.py del grupo)
# ---------------------------------------------------------------------------
COLOR_TRAY = "#B85042"
COLOR_REF = "#5C6D3A"
COLOR_CMD = "#D9C5A0"
COLOR_INI = "#2E7D32"
COLOR_FIN = "#C62828"


# ---------------------------------------------------------------------------
# Lectura de bags
# ---------------------------------------------------------------------------


@dataclass
class OdomSample:
    t: float          # segundos (header stamp)
    x: float
    y: float
    yaw: float        # rad, sin desenvolver


@dataclass
class CmdSample:
    t: float
    linear_x: float
    angular_z: float


@dataclass
class BagData:
    path: Path
    storage: str
    message_count: int
    topics: dict
    samples: list
    cmd: list


def load_metadata(bag_dir: Path) -> dict:
    meta_path = bag_dir / "metadata.yaml"
    if not meta_path.is_file():
        raise FileNotFoundError(f"no existe {meta_path}")
    with open(meta_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)["rosbag2_bagfile_information"]


def read_bag(bag_dir: Path, odom_topic: str = "/odom") -> BagData:
    info = load_metadata(bag_dir)
    storage = info.get("storage_identifier", "sqlite3")
    topics = {}
    for item in info.get("topics_with_message_count", []):
        tm = item["topic_metadata"]
        topics[tm["name"]] = tm["type"]

    reader = SequentialReader()
    try:
        reader.open(
            StorageOptions(uri=str(bag_dir), storage_id=storage),
            ConverterOptions(input_serialization_format="cdr",
                             output_serialization_format="cdr"),
        )
    except Exception as exc:
        raise SystemExit(f"{bag_dir.name}: no se pudo abrir el bag "
                         f"(storage={storage}): {exc}")

    samples = []
    cmd = []
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        try:
            if topic == odom_topic:
                msg = deserialize_message(data, Odometry)
                stamp = msg.header.stamp
                t = stamp.sec + stamp.nanosec * 1e-9
                if t == 0.0:                      # header sin sello
                    t = timestamp * 1e-9
                q = msg.pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                samples.append(OdomSample(t=t,
                                          x=msg.pose.pose.position.x,
                                          y=msg.pose.pose.position.y,
                                          yaw=yaw))
            elif topic == "/cmd_vel":
                msg = deserialize_message(data, Twist)
                cmd.append(CmdSample(t=timestamp * 1e-9,
                                     linear_x=msg.linear.x,
                                     angular_z=msg.angular.z))
        except Exception as exc:
            print(f"  [aviso] mensaje de {topic} ilegible en {bag_dir.name}: {exc}")

    if not samples:
        raise SystemExit(f"{bag_dir.name}: no hay mensajes en {odom_topic}")

    samples.sort(key=lambda s: s.t)
    cmd.sort(key=lambda c: c.t)
    return BagData(path=bag_dir, storage=storage,
                   message_count=int(info.get("message_count", 0)),
                   topics=topics, samples=samples, cmd=cmd)


def find_bags(root: Path):
    bags = []
    if (root / "metadata.yaml").is_file():
        bags.append(root)
    for child in sorted(root.iterdir()):
        if child.is_dir() and (child / "metadata.yaml").is_file():
            bags.append(child)
    return bags


# ---------------------------------------------------------------------------
# Metricas
# ---------------------------------------------------------------------------


def compute_metrics(bag: BagData) -> dict:
    s = bag.samples
    t = np.array([p.t for p in s], dtype=float)
    x = np.array([p.x for p in s], dtype=float)
    y = np.array([p.y for p in s], dtype=float)
    yaw = np.unwrap(np.array([p.yaw for p in s], dtype=float))

    path = float(np.sum(np.hypot(np.diff(x), np.diff(y)))) if len(s) > 1 else 0.0
    net = float(math.hypot(x[-1] - x[0], y[-1] - y[0]))

    h0 = float(s[0].yaw)
    u = (x - x[0]) * math.cos(h0) + (y - y[0]) * math.sin(h0)
    v = -(x - x[0]) * math.sin(h0) + (y - y[0]) * math.cos(h0)
    lat_max = float(np.max(np.abs(v)))
    lat_final = float(v[-1])

    rotation = float(np.degrees(yaw[-1] - yaw[0]))
    yaw_final = float(math.degrees(s[-1].yaw))     # lo que publica /odom

    if abs(rotation) > 45.0 and path < 0.20:
        tipo = "giro"
    elif path >= 0.20:
        tipo = "recta"
    else:
        tipo = "otro"

    cmd_window = None
    if bag.cmd:
        moving = [c for c in bag.cmd
                  if abs(c.linear_x) > 1e-6 or abs(c.angular_z) > 1e-6]
        if moving:
            cmd_window = (moving[0].t, moving[-1].t)

    return {
        "bag": bag.path.name,
        "storage": bag.storage,
        "mensajes_odom": len(s),
        "duracion_s": float(t[-1] - t[0]),
        "x_final_m": float(x[-1]),
        "y_final_m": float(y[-1]),
        "distancia_recorrida_m": path,
        "desplazamiento_neto_m": net,
        "desvio_lateral_max_cm": lat_max * 100.0,
        "desvio_lateral_final_cm": lat_final * 100.0,
        "giro_acumulado_deg": rotation,
        "giro_final_deg": yaw_final,
        "tipo_sugerido": tipo,
        "_t": t, "_x": x, "_y": y, "_yaw": yaw, "_u": u, "_v": v,
        "_h0": h0, "_cmd_window": cmd_window,
    }


# ---------------------------------------------------------------------------
# Graficas
# ---------------------------------------------------------------------------


def _caja(ax, texto):
    ax.annotate(texto, xy=(0.02, 0.98), xycoords="axes fraction",
                va="top", ha="left", fontsize=9,
                bbox=dict(boxstyle="round,pad=0.4", fc="wheat", alpha=0.75))


def plot_xy(m: dict, out_png: Path, dpi: int = 150) -> None:
    x, y = m["_x"], m["_y"]
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(x, y, color=COLOR_TRAY, linewidth=2.0, label="trayectoria /odom")
    if m["tipo_sugerido"] == "recta":
        u_end = float(m["_u"][-1])
        h0 = m["_h0"]
        ax.plot([x[0], x[0] + u_end * math.cos(h0)],
                [y[0], y[0] + u_end * math.sin(h0)],
                linestyle="--", color=COLOR_REF, linewidth=1.2,
                label="recta ideal (sin desvio)")
    ax.scatter([x[0]], [y[0]], c=COLOR_INI, s=60, zorder=5, label="inicio")
    ax.scatter([x[-1]], [y[-1]], c=COLOR_FIN, s=60, zorder=5, label="fin")

    span = max(float(np.ptp(x)), float(np.ptp(y)), 0.10)
    cx = float(x.min() + x.max()) / 2.0
    cy = float(y.min() + y.max()) / 2.0
    margin = 0.08 * span
    ax.set_xlim(cx - span / 2 - margin, cx + span / 2 + margin)
    ax.set_ylim(cy - span / 2 - margin, cy + span / 2 + margin)
    ax.set_aspect("equal", adjustable="box")

    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", fontsize=9)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Trayectoria /odom - {m['bag']}")

    _caja(ax,
          f"tipo: {m['tipo_sugerido']}\n"
          f"distancia: {m['distancia_recorrida_m'] * 100:.1f} cm\n"
          f"neto: {m['desplazamiento_neto_m'] * 100:.1f} cm\n"
          f"desvio lateral max: {m['desvio_lateral_max_cm']:.1f} cm\n"
          f"giro: {m['giro_acumulado_deg']:+.1f} deg")

    fig.savefig(out_png, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


def plot_theta(m: dict, out_png: Path, dpi: int = 150) -> None:
    t = m["_t"] - m["_t"][0]
    theta = np.degrees(m["_yaw"] - m["_yaw"][0])
    fig, ax = plt.subplots(figsize=(8, 4.5))

    window = m["_cmd_window"]
    if window:
        ax.axvspan(window[0] - m["_t"][0], window[1] - m["_t"][0],
                   color=COLOR_CMD, alpha=0.35, lw=0, label="ventana de /cmd_vel")

    ax.plot(t, theta, color=COLOR_TRAY, linewidth=1.6, label="theta(t) /odom")
    ax.axhline(0.0, color=COLOR_REF, linewidth=1.0, alpha=0.8)

    if abs(m["giro_acumulado_deg"]) > 180.0:
        for ref in (-360.0, 360.0):
            ax.axhline(ref, color=COLOR_REF, linestyle="--", linewidth=1.0,
                       alpha=0.8)
        ax.text(t[-1], -360.0, " -360 deg", va="bottom", ha="right",
                fontsize=8, color=COLOR_REF)

    ax.grid(True, alpha=0.3)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("theta [deg]")
    ax.set_title(f"Orientacion theta(t) - {m['bag']}")
    ax.legend(loc="best", fontsize=9)

    _caja(ax,
          f"giro acumulado: {m['giro_acumulado_deg']:+.1f} deg\n"
          f"theta final /odom: {m['giro_final_deg']:+.1f} deg\n"
          f"duracion: {m['duracion_s']:.1f} s")

    fig.savefig(out_png, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

CSV_COLUMNS = [
    "bag", "storage", "tipo_sugerido", "mensajes_odom", "duracion_s",
    "x_final_m", "y_final_m", "distancia_recorrida_m", "desplazamiento_neto_m",
    "desvio_lateral_max_cm", "desvio_lateral_final_cm",
    "giro_acumulado_deg", "giro_final_deg",
]


def resolve_bag(arg: str, base: Path) -> Path:
    p = Path(arg)
    if not p.is_absolute():
        p = (base / arg).resolve()
    if p.is_file() and p.suffix == ".db3":
        p = p.parent
    return p


def main(argv=None) -> int:
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="Genera las graficas x-y y theta(t) de bags ROS 2.")
    parser.add_argument("bags", nargs="*", help="carpetas de bags (por defecto: --all)")
    parser.add_argument("--all", action="store_true",
                        help="procesa todas las subcarpetas con metadata.yaml")
    parser.add_argument("--dir", default=None,
                        help="carpeta donde buscar los bags con --all (por defecto: la del script)")
    parser.add_argument("--out", default="plots", help="carpeta de salida (por defecto: plots)")
    parser.add_argument("--dpi", type=int, default=150, help="resolucion de los PNG")
    parser.add_argument("--topic", default="/odom", help="topic de odometria")
    args = parser.parse_args(argv)

    out_dir = Path(args.out)
    if not out_dir.is_absolute():
        out_dir = here / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.all or not args.bags:
        root = Path(args.dir).resolve() if args.dir else here
        bag_dirs = find_bags(root)
    else:
        bag_dirs = [resolve_bag(b, here) for b in args.bags]

    if not bag_dirs:
        print(f"No se encontraron bags en {root if (args.all or not args.bags) else here}")
        return 1

    print(f"Bags encontrados: {len(bag_dirs)}")
    print(f"Salida: {out_dir}\n")

    rows = []
    for bag_dir in bag_dirs:
        try:
            bag = read_bag(bag_dir, odom_topic=args.topic)
        except (FileNotFoundError, SystemExit) as exc:
            print(f"[omitido] {bag_dir.name}: {exc}")
            continue

        m = compute_metrics(bag)
        plot_xy(m, out_dir / f"{bag.path.name}_xy.png", dpi=args.dpi)
        plot_theta(m, out_dir / f"{bag.path.name}_theta.png", dpi=args.dpi)
        rows.append(m)
        print(f"  {bag.path.name:10s} {m['tipo_sugerido']:5s} "
              f"{m['mensajes_odom']:5d} msgs  "
              f"dist={m['distancia_recorrida_m'] * 100:7.1f} cm  "
              f"lat={m['desvio_lateral_max_cm']:5.1f} cm  "
              f"giro={m['giro_acumulado_deg']:+7.1f} deg")

    if not rows:
        print("\nNo se genero ninguna grafica.")
        return 1

    csv_path = out_dir / "resumen.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for m in rows:
            writer.writerow({k: (round(m[k], 4) if isinstance(m[k], float) else m[k])
                             for k in CSV_COLUMNS})

    print(f"\nGraficas y resumen en: {out_dir}")
    print(f"  resumen.csv: {len(rows)} bags")
    return 0


if __name__ == "__main__":
    sys.exit(main())
