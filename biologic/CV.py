from __future__ import annotations

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
OUTPUT_CSV_DEFAULT = r"biologic\cv_results.csv"

# CV settings
START_VOLTAGE_V = 0.0
VERTEX1_VOLTAGE_V = 0.8
VERTEX2_VOLTAGE_V = -0.2
FINAL_VOLTAGE_V = 0.0
SCAN_RATE_MV_S = 20.0
CV_CYCLES = 2
RECORD_DE_V = 0.005
POLL_INTERVAL_SECONDS = 0.1

LIB_DIR = os.environ.get("ECLIB_DIR", r"C:\EC-Lab Development Package\lib")
DLL_FILE = "EClib64.dll" if c_is_64b else "EClib.dll"
DLL_PATH = os.path.join(LIB_DIR, DLL_FILE)

CV_PARMS = {
    "vs_initial": ECC_parm("vs_initial", bool),
    "voltage_step": ECC_parm("Voltage_step", float),
    "scan_rate": ECC_parm("Scan_Rate", float),
    "scan_number": ECC_parm("Scan_number", int),
    "record_de": ECC_parm("Record_every_dE", float),
    "average_over_de": ECC_parm("Average_over_dE", bool),
    "n_cycles": ECC_parm("N_Cycles", int),
    "begin_measuring_i": ECC_parm("Begin_measuring_I", float),
    "end_measuring_i": ECC_parm("End_measuring_I", float),
    "i_range": ECC_parm("I_Range", int),
    "e_range": ECC_parm("E_Range", int),
    "bandwidth": ECC_parm("Bandwidth", int),
}


def choose_output_csv() -> str | None:
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    selected = filedialog.asksaveasfilename(
        title="Choose CV output CSV",
        defaultextension=".csv",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        initialfile=os.path.basename(OUTPUT_CSV_DEFAULT),
    )
    root.destroy()
    return selected or None


def select_cv_file(board_type: int) -> str:
    if board_type == KBIO.BOARD_TYPE.ESSENTIAL.value:
        return "cv.ecc"
    if board_type == KBIO.BOARD_TYPE.PREMIUM.value:
        return "cv4.ecc"
    if board_type == KBIO.BOARD_TYPE.DIGICORE.value:
        return "cv5.ecc"
    return "cv.ecc"


def select_firmware_paths(board_type: int) -> tuple[str, str]:
    if board_type == KBIO.BOARD_TYPE.ESSENTIAL.value:
        return "kernel.bin", "Vmp_ii_0437_a6.xlx"
    if board_type == KBIO.BOARD_TYPE.PREMIUM.value:
        return "kernel4.bin", "vmp_iv_0395_aa.xlx"
    if board_type == KBIO.BOARD_TYPE.DIGICORE.value:
        return "kernel.bin", ""
    return "kernel.bin", "Vmp_ii_0437_a6.xlx"


def parse_cv_row(api: KBIO_api, data, board_type: int):
    current_values, data_info, data_record = data
    index = 0

    for _ in range(data_info.NbRows):
        next_index = index + data_info.NbCols
        t_high, t_low, *row = data_record[index:next_index]
        index = next_index

        if len(row) < 4:
            continue

        t_rel = (t_high << 32) + t_low
        t = current_values.TimeBase * t_rel

        ec = api.ConvertChannelNumericIntoSingle(row[0], board_type)
        current = api.ConvertChannelNumericIntoSingle(row[1], board_type)
        ewe = api.ConvertChannelNumericIntoSingle(row[2], board_type)
        cycle = row[3]

        yield {
            "t": float(t),
            "Ec": float(ec),
            "I": float(current),
            "Ewe": float(ewe),
            "cycle": int(cycle),
        }


def run_cv(output_csv: str) -> None:
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

        tech_file = select_cv_file(board_type)
        # Configure CV sweep sequence: Ei -> E1 -> E2 -> Ei -> Ef
        vs_initial = [False, False, False, False, False]
        voltage_steps = [
            START_VOLTAGE_V,
            VERTEX1_VOLTAGE_V,
            VERTEX2_VOLTAGE_V,
            START_VOLTAGE_V,
            FINAL_VOLTAGE_V,
        ]
        scan_rates = [SCAN_RATE_MV_S] * 5

        cv_params = []
        for idx, value in enumerate(vs_initial):
            cv_params.append(make_ecc_parm(api, CV_PARMS["vs_initial"], value, idx))
        for idx, value in enumerate(voltage_steps):
            cv_params.append(make_ecc_parm(api, CV_PARMS["voltage_step"], value, idx))
        for idx, value in enumerate(scan_rates):
            cv_params.append(make_ecc_parm(api, CV_PARMS["scan_rate"], value, idx))

        cv_params.extend(
            [
                make_ecc_parm(api, CV_PARMS["scan_number"], 2),
                make_ecc_parm(api, CV_PARMS["record_de"], RECORD_DE_V),
                make_ecc_parm(api, CV_PARMS["average_over_de"], False),
                make_ecc_parm(api, CV_PARMS["n_cycles"], CV_CYCLES),
                make_ecc_parm(api, CV_PARMS["begin_measuring_i"], 0.0),
                make_ecc_parm(api, CV_PARMS["end_measuring_i"], 1.0),
                make_ecc_parm(api, CV_PARMS["i_range"], KBIO.I_RANGE["I_RANGE_AUTO"].value),
                make_ecc_parm(api, CV_PARMS["e_range"], KBIO.E_RANGE["E_RANGE_10V"].value),
                make_ecc_parm(api, CV_PARMS["bandwidth"], KBIO.BANDWIDTH["BW_5"].value),
            ]
        )

        ecc_parms = make_ecc_parms(api, *cv_params)

        api.LoadTechnique(idn, CHANNEL, tech_file, ecc_parms, first=True, last=True, display=False)
        api.StartChannel(idn, CHANNEL)
        print(f"CV started on channel {CHANNEL} using {tech_file}")

        count = 0
        last_key = None
        with open(output_csv, "w", encoding="utf-8", newline="") as f:
            f.write("time_s,Ec_V,Ewe_V,I_A,cycle\n")
            while True:
                data = api.GetData(idn, CHANNEL)
                status, tech_name = get_info_data(api, data)
                for output in parse_cv_row(api, data, board_type):
                    key = (output["t"], output["cycle"])
                    if key == last_key:
                        continue
                    last_key = key
                    f.write(
                        f'{output["t"]},{output["Ec"]},{output["Ewe"]},{output["I"]},{output["cycle"]}\n'
                    )
                    f.flush()
                    count += 1
                    print(
                        f't={output["t"]:.3f}s, Ewe={output["Ewe"]:.6f} V, '
                        f'I={output["I"]:.6e} A, cycle={output["cycle"]}'
                    )
                if status == "STOP":
                    break
                time.sleep(POLL_INTERVAL_SECONDS)

        print(f"CV finished. Saved {count} points to {output_csv}")
    finally:
        api.Disconnect(idn)
        print("Disconnected.")


if __name__ == "__main__":
    output_path = choose_output_csv()
    if output_path is None:
        print("Measurement canceled: no output file selected.")
    else:
        run_cv(output_path)
