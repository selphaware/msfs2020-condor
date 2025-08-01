# simcon1.py
# pip install SimConnect
from __future__ import annotations
from typing import Optional, Callable
from SimConnect import SimConnect, AircraftRequests, AircraftEvents

AXIS_MAX = 16383  # -16383..+16383 for trim axis

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
    def parking_brake_on(self) -> None:
        if callable(self._evt_parking_set):
            self._evt_parking_set(1)
            return
        # Fallback: toggle only if currently off
        pos = self.aq.get("BRAKE PARKING POSITION") or 0.0  # 0 off, 1 on
        if pos < 0.5 and callable(self._evt_parking_toggle):
            self._evt_parking_toggle()

    def parking_brake_off(self) -> None:
        if callable(self._evt_parking_set):
            self._evt_parking_set(0)
            return
        pos = self.aq.get("BRAKE PARKING POSITION") or 0.0
        if pos >= 0.5 and callable(self._evt_parking_toggle):
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
