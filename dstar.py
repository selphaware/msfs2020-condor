# dstar.py  — Python 3.13 compatible
# Control the Darkstar SCRAMJET via Input Events using Python-SimConnect.

from __future__ import annotations

from ctypes import (
    windll, HRESULT, c_char_p, c_double, c_int, c_void_p, POINTER
)
from ctypes.wintypes import HANDLE, DWORD
from typing import Optional

from SimConnect import SimConnect  # pip install SimConnect


def _get_hsimconnect(sm: SimConnect) -> HANDLE:
    """
    Robustly fetch the SimConnect HANDLE from the wrapper.
    """
    for attr in ("hSimConnect", "hsc", "_hSimConnect", "_hsc"):
        h = getattr(sm, attr, None)
        if h:
            return h
    raise RuntimeError("Could not find SimConnect handle on SimConnect() instance")


def exec_calc(sm: SimConnect, code: str) -> None:
    """
    Execute RPN/calculator code using SimConnect_ExecuteCalculatorCode.
    Works on Python 3.13 (HRESULT from ctypes, not wintypes).
    """
    dll = windll.SimConnect
    hsc = _get_hsimconnect(sm)

    # HRESULT SimConnect_ExecuteCalculatorCode(HANDLE, const char*,
    #                                          double*, int*, char*, DWORD)
    dll.SimConnect_ExecuteCalculatorCode.argtypes = [
        HANDLE, c_char_p, POINTER(c_double), POINTER(c_int), c_char_p, DWORD
    ]
    dll.SimConnect_ExecuteCalculatorCode.restype = HRESULT

    # Use NULLs for optional out params
    null_dbl_p = POINTER(c_double)()
    null_int_p = POINTER(c_int)()
    null_char_p = c_char_p()

    hr = dll.SimConnect_ExecuteCalculatorCode(
        hsc, code.encode("ascii"), null_dbl_p, null_int_p, null_char_p, DWORD(0)
    )
    if hr != 0:
        raise OSError(f"ExecuteCalculatorCode failed, HRESULT=0x{hr:08X}")


def send_input_event(sm: SimConnect, name: str, value: Optional[int] = None) -> None:
    """
    Fire an Input Event (B:/K: input event namespace) via calculator code.
    Examples:
        send_input_event(sm, "ENGINE_Transition_On")
        send_input_event(sm, "ENGINE_Transition_Set", 1)
    """
    rpn = f"{int(value)} (>K:{name})" if value is not None else f"(>K:{name})"
    exec_calc(sm, rpn)


def set_darkstar_scram(sm: SimConnect, on: bool) -> None:
    """
    Convenience wrapper for the Darkstar SCRAMJET switch.
    """
    # Prefer explicit On/Off; Set/Toggle also exist in the behavior graph.
    send_input_event(sm, "ENGINE_Transition_On" if on else "ENGINE_Transition_Off")


if __name__ == "__main__":
    # Start MSFS, load into the Darkstar cockpit first.
    sm = SimConnect()
    import pdb
    pdb.set_trace()
    # ON
    set_darkstar_scram(sm, True)
    # OFF
    set_darkstar_scram(sm, False)

    # Alternates:
    # send_input_event(sm, "ENGINE_Transition_Set", 1)  # ON
    # send_input_event(sm, "ENGINE_Transition_Set", 0)  # OFF
    # send_input_event(sm, "ENGINE_Transition_Toggle")
