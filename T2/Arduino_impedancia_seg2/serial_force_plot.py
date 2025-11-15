"""Real-time force telemetry visualizer for the Arduino impedance controller.

Usage example:
    python serial_force_plot.py --port COM6 --baud 9600 --window 60 --save resultados.csv

Requirements:
    pip install pyserial matplotlib

The Arduino sketch must emit lines in the format:
    DATA,<ms>,<raw>,<raw_avg>,<kg>,<kg_filt>,<kg_ref>,<brazo_deg>
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

try:
    import serial  # type: ignore
except ImportError as exc:  # pragma: no cover - handled at runtime
    raise SystemExit(
        "pyserial is required. Install it with 'pip install pyserial'."
    ) from exc


ROLLOVER_MS = 2 ** 32  # millis() rollover guard (~49 days on Arduino)


@dataclass
class TelemetrySample:
    t_rel: float
    raw: int
    raw_avg: int
    kg: float
    kg_filt: float
    kg_ref: float
    brazo_deg: int


def parse_data_line(line: str) -> Optional[Tuple[int, int, int, float, float, float, int]]:
    if not line.startswith("DATA"):
        return None

    parts = line.split(',')
    if len(parts) != 8:
        return None

    try:
        timestamp_ms = int(parts[1])
        raw = int(parts[2])
        raw_avg = int(parts[3])
        kg = float(parts[4])
        kg_filt = float(parts[5])
        kg_ref = float(parts[6])
        brazo_deg = int(float(parts[7]))
    except ValueError:
        return None

    return timestamp_ms, raw, raw_avg, kg, kg_filt, kg_ref, brazo_deg


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        help="Serial port (example: COM6 on Windows, /dev/ttyUSB0 on Linux)",
        required=True,
    )
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument(
        "--window",
        type=float,
        default=30.0,
        help="Time window in seconds to keep in the plot (use <=0 for all)",
    )
    parser.add_argument(
        "--save",
        type=str,
        default=None,
        help="Optional CSV output file for later analysis",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.1,
        help="Serial read timeout in seconds",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print non-telemetry serial lines to stdout",
    )

    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    except serial.SerialException as exc:
        raise SystemExit(f"Could not open serial port {args.port!r}: {exc}") from exc

    # Give the Arduino time to reset after opening the port.
    time.sleep(2.0)
    ser.reset_input_buffer()

    print(
        "Recibiendo telemetría desde {port} a {baud} baudios. Cierra la ventana del gráfico para detener.".format(
            port=args.port, baud=args.baud
        )
    )

    times: Deque[float] = deque()
    kg_vals: Deque[float] = deque()
    kg_filt_vals: Deque[float] = deque()
    kg_ref_vals: Deque[float] = deque()
    samples_log: List[TelemetrySample] = []

    origin_ms: Optional[int] = None
    last_timestamp_ms: Optional[int] = None
    rollover_offset = 0

    fig, ax = plt.subplots(figsize=(10, 5))
    line_meas, = ax.plot([], [], label="Fuerza medida (kg)")
    line_filt, = ax.plot([], [], label="Fuerza filtrada (kg)", linestyle="--")
    line_ref, = ax.plot([], [], label="Referencia (kg)", linestyle=":")

    ax.set_xlabel("Tiempo [s]")
    ax.set_ylabel("Fuerza [kg]")
    ax.grid(True, which="both", linestyle=":", linewidth=0.5)
    legend = ax.legend(loc="upper right")

    text_brazo = ax.text(0.02, 0.95, "", transform=ax.transAxes, va="top")

    def pump_serial() -> bool:
        nonlocal origin_ms, last_timestamp_ms, rollover_offset
        updated = False

        while ser.in_waiting:
            raw_line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not raw_line:
                continue

            parsed = parse_data_line(raw_line)
            if parsed is None:
                if args.verbose:
                    print(raw_line)
                continue

            timestamp_ms, raw, raw_avg, kg, kg_filt, kg_ref, brazo_deg = parsed

            # Handle millis() rollover explicitly.
            if last_timestamp_ms is not None and timestamp_ms < last_timestamp_ms:
                rollover_offset += ROLLOVER_MS
            absolute_ms = timestamp_ms + rollover_offset

            if origin_ms is None:
                origin_ms = absolute_ms

            t_rel = (absolute_ms - origin_ms) / 1000.0

            times.append(t_rel)
            kg_vals.append(kg)
            kg_filt_vals.append(kg_filt)
            kg_ref_vals.append(kg_ref)

            samples_log.append(
                TelemetrySample(
                    t_rel=t_rel,
                    raw=raw,
                    raw_avg=raw_avg,
                    kg=kg,
                    kg_filt=kg_filt,
                    kg_ref=kg_ref,
                    brazo_deg=brazo_deg,
                )
            )

            last_timestamp_ms = timestamp_ms

            # Keep memory bounded
            if args.window > 0:
                while times and times[-1] - times[0] > args.window:
                    times.popleft()
                    kg_vals.popleft()
                    kg_filt_vals.popleft()
                    kg_ref_vals.popleft()

            text_brazo.set_text(f"Ángulo brazo: {brazo_deg:.0f}°")
            updated = True

        return updated

    def update_plot(_frame: int):
        pump_serial()
        if not times:
            return line_meas, line_filt, line_ref, legend, text_brazo

        x = list(times)
        y_meas = list(kg_vals)
        y_filt = list(kg_filt_vals)
        y_ref = list(kg_ref_vals)

        line_meas.set_data(x, y_meas)
        line_filt.set_data(x, y_filt)
        line_ref.set_data(x, y_ref)

        if args.window > 0:
            x_max = max(args.window, x[-1])
            ax.set_xlim(max(0.0, x_max - args.window), x_max)
        else:
            ax.set_xlim(0.0, max(10.0, x[-1]))

        y_all = y_meas + y_filt + y_ref
        y_min = min(y_all)
        y_max = max(y_all)
        span = max(0.2, y_max - y_min)
        margin = span * 0.1
        ax.set_ylim(y_min - margin, y_max + margin)

        return line_meas, line_filt, line_ref, legend, text_brazo

    ani = FuncAnimation(fig, update_plot, interval=100, blit=False)

    try:
        plt.show()
    finally:
        ser.close()

    if args.save and samples_log:
        with open(args.save, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "time_s",
                    "fsr_raw",
                    "fsr_raw_avg",
                    "kg_measured",
                    "kg_filtered",
                    "kg_reference",
                    "brazo_deg",
                ]
            )
            for sample in samples_log:
                writer.writerow(
                    [
                        f"{sample.t_rel:.3f}",
                        sample.raw,
                        sample.raw_avg,
                        f"{sample.kg:.4f}",
                        f"{sample.kg_filt:.4f}",
                        f"{sample.kg_ref:.4f}",
                        sample.brazo_deg,
                    ]
                )

        print(f"Datos guardados en {args.save}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)

