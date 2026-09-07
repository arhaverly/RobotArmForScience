from __future__ import annotations

from PyExpLabSys.drivers.bio_logic import SP150

ECLIB_DLL = r"C:\EC-Lab Development Package\lib\EClib64.dll"
ADDRESS = b"USB0"
CONNECT_TIMEOUT_SECONDS = 5


def disconnect_sp150() -> None:
    address_text = ADDRESS.decode("ascii", errors="replace")
    instrument = SP150(address=ADDRESS, EClib_dll_path=ECLIB_DLL)

    version = instrument.get_lib_version()
    if isinstance(version, bytes):
        version = version.decode("ascii", errors="replace")
    print("EC-Lib version:", version)

    try:
        instrument.connect(timeout=CONNECT_TIMEOUT_SECONDS)
        print(f"Connected on {address_text}, device id={instrument.id_number}")
    except Exception as exc:
        raise RuntimeError(f"Unable to connect on {address_text}: {exc}") from exc

    try:
        instrument.disconnect()
        print("Disconnected successfully.")
    except Exception as exc:
        raise RuntimeError(f"Failed to disconnect on {address_text}: {exc}") from exc


def main() -> None:
    disconnect_sp150()


if __name__ == "__main__":
    main()