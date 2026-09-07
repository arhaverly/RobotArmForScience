from __future__ import annotations

from PyExpLabSys.drivers.bio_logic import SP150

ECLIB_DLL = r"C:\EC-Lab Development Package\lib\EClib64.dll"
ADDRESS = b"USB0"
CONNECT_TIMEOUT_SECONDS = 5


def connect_sp150() -> tuple[SP150, str]:
    address_text = ADDRESS.decode("ascii", errors="replace")
    instrument = SP150(address=ADDRESS, EClib_dll_path=ECLIB_DLL)
    version = instrument.get_lib_version()
    if isinstance(version, bytes):
        version = version.decode("ascii", errors="replace")
    print("EC-Lib version:", version)

    instrument.connect(timeout=CONNECT_TIMEOUT_SECONDS)
    instrument.test_connection()
    print(f"Connected successfully on {address_text}, device id={instrument.id_number}")
    return instrument, address_text


def main() -> None:
    connect_sp150()


if __name__ == "__main__":
    main()