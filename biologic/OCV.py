from __future__ import annotations

import os
import sys
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
MEASUREMENT_SECONDS = 30.0
RECORD_DT_SECONDS = 0.5
OUTPUT_CSV_DEFAULT = r"biologic\ocv_results.csv"

LIB_DIR = os.environ.get("ECLIB_DIR", r"C:\EC-Lab Development Package\lib")
DLL_FILE = "EClib64.dll" if c_is_64b else "EClib.dll"
DLL_PATH = os.path.join(LIB_DIR, DLL_FILE)

OCV_PARMS = {
    "duration": ECC_parm("Rest_time_T", float),
    "record_dt": ECC_parm("Record_every_dT", float),
    "record_de": ECC_parm("Record_every_dE", float),
    "e_range": ECC_parm("E_Range", int),
}


def choose_output_csv() -> str | None:
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    selected = filedialog.asksaveasfilename(
        title="Choose OCV output CSV",
        defaultextension=".csv",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        initialfile=os.path.basename(OUTPUT_CSV_DEFAULT),
    )
    root.destroy()
    return selected or None


def select_ocv_file(board_type: int) -> str:
    if board_type == KBIO.BOARD_TYPE.ESSENTIAL.value:
        return "ocv.ecc"
    if board_type == KBIO.BOARD_TYPE.PREMIUM.value:
        return "ocv4.ecc"
    if board_type == KBIO.BOARD_TYPE.DIGICORE.value:
        return "ocv5.ecc"
    return "ocv.ecc"


def select_firmware_paths(board_type: int) -> tuple[str, str]:
    if board_type == KBIO.BOARD_TYPE.ESSENTIAL.value:
        return "kernel.bin", "Vmp_ii_0437_a6.xlx"
    if board_type == KBIO.BOARD_TYPE.PREMIUM.value:
        return "kernel4.bin", "vmp_iv_0395_aa.xlx"
    if board_type == KBIO.BOARD_TYPE.DIGICORE.value:
        return "kernel.bin", ""
    return "kernel.bin", "Vmp_ii_0437_a6.xlx"


def run_ocv(output_csv: str) -> None:
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

        tech_file = select_ocv_file(board_type)
        p_duration = make_ecc_parm(api, OCV_PARMS["duration"], MEASUREMENT_SECONDS)
        p_record_dt = make_ecc_parm(api, OCV_PARMS["record_dt"], RECORD_DT_SECONDS)
        p_record_de = make_ecc_parm(api, OCV_PARMS["record_de"], 0.005)
        p_erange = make_ecc_parm(api, OCV_PARMS["e_range"], KBIO.E_RANGE["E_RANGE_10V"].value)
        ecc_parms = make_ecc_parms(api, p_duration, p_record_dt, p_record_de, p_erange)

        api.LoadTechnique(idn, CHANNEL, tech_file, ecc_parms, first=True, last=True, display=False)
        api.StartChannel(idn, CHANNEL)
        print(f"OCV started on channel {CHANNEL} using {tech_file}")

        count = 0
        with open(output_csv, "w", encoding="utf-8", newline="") as f:
            f.write("time_s,Ewe_V\n")
            while True:
                data = api.GetData(idn, CHANNEL)
                status, tech_name = get_info_data(api, data)
                for output in get_experiment_data(api, data, tech_name, board_type):
                    t = output.get("t")
                    ewe = output.get("Ewe")
                    if t is not None and ewe is not None:
                        f.write(f"{t},{ewe}\n")
                        f.flush()
                        count += 1
                        print(f"t={float(t):.3f}s, Ewe={float(ewe):.6f} V")
                if status == "STOP":
                    break

        print(f"OCV finished. Saved {count} points to {output_csv}")
    finally:
        api.Disconnect(idn)
        print("Disconnected.")


if __name__ == "__main__":
    output_path = choose_output_csv()
    if output_path is None:
        print("Measurement canceled: no output file selected.")
    else:
        run_ocv(output_path)
