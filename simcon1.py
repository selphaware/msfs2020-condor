# simcon1.py
# pip install SimConnect
from __future__ import annotations
import time
from typing import Optional, Callable
from SimConnect import SimConnect, AircraftRequests, AircraftEvents

__author__ = "Usman Ahmad"

AXIS_MAX = 16383  # -16383..+16383 for trim axis
FTPS_TO_KT = 1.0 / 1.687809857  # ft/s -> knots

class MSFSController:
    def __init__(self) -> None:
        self.sm = SimConnect()
        self.aq = AircraftRequests(self.sm, _time=200)
        self.ae = AircraftEvents(self.sm)

        # Resolve events safely (some builds don’t return events via .find())
        self._evt_parking_set     = self._evt("PARKING_BRAKE_SET")
        self._evt_parking_toggle  = self._evt("PARKING_BRAKES")
        self._evt_eng_start       = self._evt("ENGINE_AUTO_START")
        self._evt_eng_shutdown    = self._evt("ENGINE_AUTO_SHUTDOWN")
        self._evt_trim_set        = self._evt("ELEVATOR_TRIM_SET", "AXIS_ELEV_TRIM_SET")
        self._evt_elevator_axis = self._evt("AXIS_ELEVATOR_SET", "ELEVATOR_SET")
        self._evt_aileron_axis  = self._evt("AXIS_AILERONS_SET", "AILERON_SET")
        self._evt_rudder_axis   = self._evt("AXIS_RUDDER_SET", "RUDDER_SET")
        self._evt_batt_on       = self._evt("MASTER_BATTERY_ON")
        self._evt_batt_off      = self._evt("MASTER_BATTERY_OFF")
        self._evt_batt_toggle   = self._evt("TOGGLE_MASTER_BATTERY")

        self._evt_avionics_set  = self._evt("AVIONICS_MASTER_SET")
        self._evt_avionics_tog  = self._evt("TOGGLE_AVIONICS_MASTER")

    def _evt(self, *names: str) -> Optional[Callable]:
        for n in names:
            try:
                e = self.ae.find(n)
                if callable(e):
                    return e
            except Exception:
                pass
        return None

    # ---------- Parking brake ----------
    def parking_brake_toggle(self) -> None:
        if callable(self._evt_parking_set):
            self._evt_parking_set(1)
            return
        # Fallback: toggle only if currently off
        pos = self.aq.get("BRAKE PARKING POSITION") or 0.0  # 0 off, 1 on
        if pos < 0.5 and callable(self._evt_parking_toggle):
            self._evt_parking_toggle()

    # ---------- Engine auto start / stop ----------
    def engine_on(self) -> None:
        if not callable(self._evt_eng_start):
            raise RuntimeError("ENGINE_AUTO_START event not available")
        self._evt_eng_start()

    def engine_off(self) -> None:
        if not callable(self._evt_eng_shutdown):
            raise RuntimeError("ENGINE_AUTO_SHUTDOWN event not available")
        self._evt_eng_shutdown()

    # ---------- Elevator trim ----------
    def _get_trim_percent(self) -> Optional[float]:
        try:
            pct = self.aq.get("ELEVATOR TRIM PCT")
            if pct is None:
                return None
            return float(max(-100.0, min(100.0, pct)))
        except Exception:
            return None

    def set_trim_percent(self, target_pct: float) -> None:
        target_pct = max(-100.0, min(100.0, float(target_pct)))
        axis_value = int((target_pct / 100.0) * AXIS_MAX)
        evt = self._evt_trim_set or self._evt("ELEVATOR_TRIM_SET", "AXIS_ELEV_TRIM_SET")
        if not callable(evt):
            raise RuntimeError("Trim event not available")
        evt(axis_value)

    def nudge_trim(self, delta_pct: float) -> None:
        current = self._get_trim_percent()
        if current is None:
            current = 0.0
        self.set_trim_percent(current + float(delta_pct))

    def trim_increase(self, delta_pct: float) -> None:
        self.nudge_trim(abs(delta_pct))

    def trim_decrease(self, delta_pct: float) -> None:
        self.nudge_trim(-abs(delta_pct))

    def close(self) -> None:
        try:
            self.sm.exit()
        except Exception:
            pass

    def _get_num_engines(self) -> int:
        try:
            n = int(self.aq.get("NUMBER OF ENGINES") or 1)
            return max(1, min(n, 8))
        except Exception:
            return 1

    def set_throttle_percent(self, target_pct: float, engine: int | None = None, allow_reverse: bool = False) -> None:
        """
        Set throttle to a given percent.
        - engine=None: apply to all engines found via NUMBER OF ENGINES
        - engine=1..N: apply to a specific engine index (1-based)
        - allow_reverse=True: permit -100..+100 (uses AXIS_THROTTLE_SET only)
        """
        if allow_reverse:
            # Use combined axis event, which supports negative values for reverse (if aircraft supports it)
            pct = max(-100.0, min(100.0, float(target_pct)))
            axis_value = int(round((pct / 100.0) * AXIS_MAX))
            if not callable(self._evt_throttle_axis):
                raise RuntimeError("AXIS_THROTTLE_SET event not available")
            self._evt_throttle_axis(axis_value)
            return

        # Forward thrust only (0..100) via engine-specific events when possible
        pct = max(0.0, min(100.0, float(target_pct)))
        val_0_16383 = int(round((pct / 100.0) * AXIS_MAX))

        def _send_engine(e_idx: int) -> bool:
            evt = self._evt(f"THROTTLE{e_idx}_SET")
            if callable(evt):
                evt(val_0_16383)
                return True
            return False

        if engine is None:
            n = self._get_num_engines()
            sent_any = False
            for i in range(1, n + 1):
                sent_any |= _send_engine(i)
            if not sent_any:
                # Fallback: combined axis
                if not callable(self._evt_throttle_axis):
                    raise RuntimeError("No throttle events available")
                self._evt_throttle_axis(val_0_16383)
        else:
            if not _send_engine(int(engine)):
                if not callable(self._evt_throttle_axis):
                    raise RuntimeError(f"THROTTLE{engine}_SET and AXIS_THROTTLE_SET not available")
                self._evt_throttle_axis(val_0_16383)

    def _pct_to_axis(self, pct: float) -> int:
        # Clamp [-100,100] and map to [-16383, +16383]
        p = max(-100.0, min(100.0, float(pct)))
        return int(round((p / 100.0) * AXIS_MAX))

    def _send_axis_or_raise(self, evt, value: int, name: str) -> None:
        if not callable(evt):
            raise RuntimeError(f"{name} event not available")
        evt(value)

    def set_elevator_percent(self, pct: float, invert: bool = False) -> None:
        """
        Elevator (pitch) input: -100..+100
        + = pull back (nose up) on most setups (set invert=True if reversed)
        """
        if invert:
            pct = -pct
        self._send_axis_or_raise(self._evt_elevator_axis, self._pct_to_axis(pct), "Elevator")

    # Alias to match your wording "set the elevation"
    def set_elevation_percent(self, pct: float, invert: bool = False) -> None:
        self.set_elevator_percent(pct, invert=invert)

    def set_aileron_percent(self, pct: float, invert: bool = False) -> None:
        """
        Aileron (roll) input: -100..+100
        - = roll left, + = roll right (set invert=True if reversed)
        """
        if invert:
            pct = -pct
        self._send_axis_or_raise(self._evt_aileron_axis, self._pct_to_axis(pct), "Aileron")

    def set_rudder_percent(self, pct: float, invert: bool = False) -> None:
        """
        Rudder (yaw) input: -100..+100
        - = yaw left, + = yaw right (set invert=True if reversed)
        """
        if invert:
            pct = -pct
        self._send_axis_or_raise(self._evt_rudder_axis, self._pct_to_axis(pct), "Rudder")

    # Replace/augment helpers inside MSFSController
    def _simvar_name(self, name: str) -> str:
        """Normalize to the underscore style used by Python-SimConnect."""
        return name.replace(" ", "_")

    def _get_var(self, name: str, attempts: int = 20, interval: float = 0.05):
        """
        Read a SimVar with brief retries until a real value (not None) arrives.
        Name should be in SDK style with spaces or underscores; we'll normalize.
        """
        key = self._simvar_name(name)
        # Prefer AircraftRequests.find(...).get() which is reliable in this lib
        try:
            req = self.aq.find(key)
        except Exception:
            req = None

        for _ in range(max(1, attempts)):
            try:
                if req is not None:
                    v = req.get()
                else:
                    v = self.aq.get(key)
            except Exception:
                v = None
            if v is not None:
                return v
            time.sleep(interval)
        return None  # caller decides the default

    # ----------- Basic states -----------
    def get_parking_brake_position(self) -> float:
        """0.0 = off, 1.0 = on."""
        v = self._get_var("BRAKE_PARKING_POSITION")
        return float(v) if v is not None else 0.0

    def is_on_ground(self) -> bool:
        v = self._get_var("SIM_ON_GROUND")
        return bool(int(v)) if v is not None else False

    # ----------- Altitude / position / attitude -----------
    def get_altitudes(self) -> dict:
        return {
            "msl": float(self._get_var("PLANE_ALTITUDE") or 0.0),
            "agl": float(self._get_var("ALTITUDE_AGL") or 0.0),
            "indicated": float(self._get_var("INDICATED_ALTITUDE") or 0.0),
        }

    def get_position(self) -> dict:
        return {
            "lat": float(self._get_var("PLANE_LATITUDE") or 0.0),
            "lon": float(self._get_var("PLANE_LONGITUDE") or 0.0),
        }

    def get_attitude(self) -> dict:
        return {
            "pitch": float(self._get_var("PLANE_PITCH_DEGREES") or 0.0),
            "bank": float(self._get_var("PLANE_BANK_DEGREES") or 0.0),
            "hdg_true": float(self._get_var("PLANE_HEADING_DEGREES_TRUE") or 0.0),
            "hdg_mag": float(self._get_var("PLANE_HEADING_DEGREES_MAGNETIC") or 0.0),
        }

    def get_vertical_speed_fpm(self) -> float:
        return float(self._get_var("VERTICAL_SPEED") or 0.0)

    def get_speeds(self) -> dict:
        gs_ftps = float(self._get_var("GROUND_VELOCITY") or 0.0)  # ft/s per SDK
        return {
            "ias": float(self._get_var("AIRSPEED_INDICATED") or 0.0),  # kt
            "tas": float(self._get_var("AIRSPEED_TRUE") or 0.0),       # kt
            "mach": float(self._get_var("AIRSPEED_MACH") or 0.0),
            "gs": gs_ftps * FTPS_TO_KT,                                # kt
        }

    # ----------- Engines / throttle -----------
    def _get_num_engines(self) -> int:
        v = self._get_var("NUMBER_OF_ENGINES")
        try:
            n = int(v) if v is not None else 1
        except Exception:
            n = 1
        return max(1, min(n, 8))

    def get_engine_combustion(self, engine: int | None = None):
        """
        True if running (combustion). If engine=None, list for all engines.
        """
        n = self._get_num_engines()
        def _one(i: int) -> bool:
            v = self._get_var(f"GENERAL_ENG_COMBUSTION:{i}")
            return bool(int(v)) if v is not None else False
        if engine is None:
            return [_one(i) for i in range(1, n + 1)]
        return _one(int(engine))

    def is_any_engine_on(self) -> bool:
        return any(self.get_engine_combustion())

    def get_throttle_percent(self, engine: int | None = None):
        """
        Throttle lever position percent(s) 0..100. If engine=None, list for all.
        """
        n = self._get_num_engines()
        def _one(i: int) -> float:
            v = self._get_var(f"GENERAL_ENG_THROTTLE_LEVER_POSITION:{i}")
            return float(v) if v is not None else 0.0
        if engine is None:
            return [_one(i) for i in range(1, n + 1)]
        return _one(int(engine))

    # ----------- Gear / flaps / trim -----------
    def get_gear_handle_position(self) -> float:
        """0.0 = up, 1.0 = down."""
        v = self._get_var("GEAR_HANDLE_POSITION")
        return float(v) if v is not None else 0.0

    def get_flaps_percent(self) -> float:
        pos_0_1 = float(self._get_var("FLAPS_HANDLE_POSITION") or 0.0)
        return max(0.0, min(100.0, pos_0_1 * 100.0))

    def get_trim_percent(self) -> float:
        v = self._get_var("ELEVATOR_TRIM_PCT")
        if v is None:
            return 0.0
        try:
            return float(max(-100.0, min(100.0, float(v) * 100)))
        except Exception:
            return 0.0

    # ----------- Control surface deflections -----------
    def get_control_surface_percents(self) -> dict:
        def to_pct(x):
            try:
                return float(max(-100.0, min(100.0, (float(x) or 0.0) * 100.0)))
            except Exception:
                return 0.0
        return {
            "elevator": to_pct(self._get_var("ELEVATOR_POSITION")),
            "aileron":  to_pct(self._get_var("AILERON_POSITION")),
            "rudder":   to_pct(self._get_var("RUDDER_POSITION")),
        }

    # ----------- Electrical -----------
    def get_electrical(self) -> dict:
        return {
            "master_battery": bool(int(self._get_var("ELECTRICAL_MASTER_BATTERY") or 0)),
            "avionics_master": bool(int(self._get_var("AVIONICS_MASTER_SWITCH") or 0)),
        }

    # ----------- Master Battery -----------
    def master_battery_on(self, index: int = 1) -> None:
        """
        Turn ON the (master) battery. For most default aircraft, index=1 is fine.
        Falls back to toggle if needed.
        """
        if callable(self._evt_batt_on):
            self._evt_batt_on(int(index))
            return
        # Fallback: only toggle if currently off
        cur = bool(int(self._get_var("ELECTRICAL_MASTER_BATTERY") or 0))
        if not cur and callable(self._evt_batt_toggle):
            self._evt_batt_toggle(int(index))

    def master_battery_off(self, index: int = 1) -> None:
        """
        Turn OFF the (master) battery. For most default aircraft, index=1 is fine.
        Falls back to toggle if needed.
        """
        if callable(self._evt_batt_off):
            self._evt_batt_off(int(index))
            return
        cur = bool(int(self._get_var("ELECTRICAL_MASTER_BATTERY") or 0))
        if cur and callable(self._evt_batt_toggle):
            self._evt_batt_toggle(int(index))

    # ----------- Avionics Master -----------
    def avionics_master_on(self) -> None:
        """Set avionics master ON (fallback to toggle)."""
        if callable(self._evt_avionics_set):
            self._evt_avionics_set(1)
            return
        cur = bool(int(self._get_var("AVIONICS_MASTER_SWITCH") or 0))
        if not cur and callable(self._evt_avionics_tog):
            self._evt_avionics_tog()

    def avionics_master_off(self) -> None:
        """Set avionics master OFF (fallback to toggle)."""
        if callable(self._evt_avionics_set):
            self._evt_avionics_set(0)
            return
        cur = bool(int(self._get_var("AVIONICS_MASTER_SWITCH") or 0))
        if cur and callable(self._evt_avionics_tog):
            self._evt_avionics_tog()


if __name__ == "__main__":
    ctl = MSFSController()
    try:
        # Examples:
        # ctl.parking_brake_on()
        # ctl.parking_brake_off()
        # ctl.engine_on()
        # ctl.engine_off()
        ctl.trim_increase(5.0)
        ctl.trim_decrease(2.0)
    finally:
        ctl.close()
