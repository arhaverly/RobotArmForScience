from __future__ import annotations

from dataclasses import dataclass
import os
import sys
import time
import tkinter as tk
from tkinter import filedialog

# Use Bio-Logic's official OEM Python API shipped with EC-Lab package.
OEM_PYTHON_DIR = r"C:\EC-Lab Development Package\Examples\Python"
if OEM_PYTHON_DIR not in sys.path:
    sys.path.insert(0, OEM_PYTHON_DIR)

import kbio.kbio_types as KBIO
from kbio.c_utils import c_is_64b
from kbio.kbio_api import KBIO_api
from kbio.kbio_tech import ECC_parm
from kbio.kbio_tech import get_experiment_data
from kbio.kbio_tech import get_info_data
from kbio.kbio_tech import make_ecc_parm
from kbio.kbio_tech import make_ecc_parms

ADDRESS = "USB0"
CHANNEL = 1
OUTPUT_CSV_DEFAULT = r"biologic\pulse_results.csv"

# Pulse-current settings (CP technique)
REPEAT_COUNT = 2
RECORD_DT_SECONDS = 0.1
RECORD_DE_V = 0.01
CURRENT_RANGE = "I_RANGE_10mA"
POLL_INTERVAL_SECONDS = 0.1

LIB_DIR = os.environ.get("ECLIB_DIR", r"C:\EC-Lab Development Package\lib")
DLL_FILE = "EClib64.dll" if c_is_64b else "EClib.dll"
DLL_PATH = os.path.join(LIB_DIR, DLL_FILE)

CP_PARMS = {
    "current_step": ECC_parm("Current_step", float),
    "step_duration": ECC_parm("Duration_step", float),
    "vs_init": ECC_parm("vs_initial", bool),
    "nb_steps": ECC_parm("Step_number", int),
    "record_dt": ECC_parm("Record_every_dT", float),
    "record_de": ECC_parm("Record_every_dE", float),
    "repeat": ECC_parm("N_Cycles", int),
    "i_range": ECC_parm("I_Range", int),
}


@dataclass
class CurrentPulseStep:
    current_a: float
    duration_s: float
    vs_initial: bool = False


# Example pulse sequence:
# - +1 mA for 2 s
# - +2 mA for 1 s
# - +0.5 mA (vs initial) for 3 s
PULSE_STEPS = [
    CurrentPulseStep(0.0001, 2.0, False),
    CurrentPulseStep(0.0002, 1.0, False),
    CurrentPulseStep(0.00005, 3.0, True),
]


def choose_output_csv() -> str | None:
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    selected = filedialog.asksaveasfilename(
        title="Choose Pulse output CSV",
        defaultextension=".csv",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        initialfile=os.path.basename(OUTPUT_CSV_DEFAULT),
    )
    root.destroy()
    return selected or None


def select_cp_file(board_type: int) -> str:
    if board_type == KBIO.BOARD_TYPE.ESSENTIAL.value:
        return "cp.ecc"
    if board_type == KBIO.BOARD_TYPE.PREMIUM.value:
        return "cp4.ecc"
    if board_type == KBIO.BOARD_TYPE.DIGICORE.value:
        return "cp5.ecc"
    return "cp.ecc"


def select_firmware_paths(board_type: int) -> tuple[str, str]:
    if board_type == KBIO.BOARD_TYPE.ESSENTIAL.value:
        return "kernel.bin", "Vmp_ii_0437_a6.xlx"
    if board_type == KBIO.BOARD_TYPE.PREMIUM.value:
        return "kernel4.bin", "vmp_iv_0395_aa.xlx"
    if board_type == KBIO.BOARD_TYPE.DIGICORE.value:
        return "kernel.bin", ""
    return "kernel.bin", "Vmp_ii_0437_a6.xlx"


def run_pulse(output_csv: str) -> None:
    output_dir = os.path.dirname(output_csv)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    api = KBIO_api(DLL_PATH)
    print("EC-Lib version:", api.GetLibVersion())

    idn, device_info = api.Connect(ADDRESS)
    print(f"Connected on {ADDRESS}")
    print("Device info:", device_info)

    try:
        board_type = api.GetChannelBoardType(idn, CHANNEL)
        firmware, fpga = select_firmware_paths(board_type)
        channel_map = api.channel_map({CHANNEL})
        api.LoadFirmware(idn, channel_map, firmware=firmware, fpga=fpga, force=True)
        print("Firmware loaded.")

        channel_info = api.GetChannelInfo(idn, CHANNEL)
        print("Channel info:", channel_info)
        if not channel_info.is_kernel_loaded:
            raise RuntimeError("Kernel is not loaded on selected channel.")

        tech_file = select_cp_file(board_type)
        cp_params = []
        for idx, step in enumerate(PULSE_STEPS):
            cp_params.append(make_ecc_parm(api, CP_PARMS["current_step"], step.current_a, idx))
            cp_params.append(make_ecc_parm(api, CP_PARMS["step_duration"], step.duration_s, idx))
            cp_params.append(make_ecc_parm(api, CP_PARMS["vs_init"], step.vs_initial, idx))

        cp_params.extend(
            [
                make_ecc_parm(api, CP_PARMS["nb_steps"], len(PULSE_STEPS) - 1),
                make_ecc_parm(api, CP_PARMS["record_dt"], RECORD_DT_SECONDS),
                make_ecc_parm(api, CP_PARMS["record_de"], RECORD_DE_V),
                make_ecc_parm(api, CP_PARMS["i_range"], KBIO.I_RANGE[CURRENT_RANGE].value),
                make_ecc_parm(api, CP_PARMS["repeat"], REPEAT_COUNT),
            ]
        )

        ecc_parms = make_ecc_parms(api, *cp_params)

        api.LoadTechnique(idn, CHANNEL, tech_file, ecc_parms, first=True, last=True, display=False)
        api.StartChannel(idn, CHANNEL)
        print(f"Pulse current started on channel {CHANNEL} using {tech_file}")

        count = 0
        last_key = None
        with open(output_csv, "w", encoding="utf-8", newline="") as f:
            f.write("time_s,Ewe_V,I_A,cycle\n")
            while True:
                data = api.GetData(idn, CHANNEL)
                status, tech_name = get_info_data(api, data)
                for output in get_experiment_data(api, data, tech_name, board_type):
                    t = output.get("t")
                    ewe = output.get("Ewe")
                    iwe = output.get("Iwe")
                    cycle = output.get("cycle")
                    if t is None or ewe is None or iwe is None:
                        continue

                    key = (float(t), int(cycle) if cycle is not None else -1)
                    if key == last_key:
                        continue
                    last_key = key
                    cycle_text = str(int(cycle)) if cycle is not None else ""
                    f.write(f"{float(t)},{float(ewe)},{float(iwe)},{cycle_text}\n")
                    f.flush()
                    count += 1
                    print(
                        f"t={float(t):.3f}s, Ewe={float(ewe):.6f} V, "
                        f"I={float(iwe):.6e} A, cycle={cycle_text}"
                    )
                if status == "STOP":
                    break
                time.sleep(POLL_INTERVAL_SECONDS)

        print(f"Pulse run finished. Saved {count} points to {output_csv}")
    finally:
        api.Disconnect(idn)
        print("Disconnected.")


if __name__ == "__main__":
    output_path = choose_output_csv()
    if output_path is None:
        print("Measurement canceled: no output file selected.")
    else:
        run_pulse(output_path)
