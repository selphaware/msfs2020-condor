#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
msfs_control.py

PEP8-compliant helpers to control Microsoft Flight Simulator 2020
via SimConnect (MSFS SDK). Provides generic send/get functions and
convenience wrappers for common cockpit interactions and telemetry.

Requires:
    pip install SimConnect  # sometimes published as `python-simconnect`

Tested with the 'SimConnect' Python wrapper (python-simconnect).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional, Callable
import logging
import time
from math import fmod

try:
    # The widely used Python wrapper for MSFS SimConnect
    from SimConnect import SimConnect, AircraftEvents, AircraftRequests  # type: ignore
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency: install with `pip install SimConnect`.\n"
        "Package is sometimes published as `python-simconnect`."
    ) from exc


KNOT_MACH = 666.739  # 1 Mach = 666.739 Knots

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

LOGGER = logging.getLogger("msfs_control")
if not LOGGER.handlers:
    _handler = logging.StreamHandler()
    _fmt = logging.Formatter(
        "[%(levelname)s] %(asctime)s %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    _handler.setFormatter(_fmt)
    LOGGER.addHandler(_handler)
LOGGER.setLevel(logging.INFO)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

AXIS_MAX = 16383
AXIS_MIN = -16383


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp value to [lo, hi]."""
    if lo > hi:
        raise ValueError(f"clamp bounds inverted: lo={lo}, hi={hi}")
    return max(lo, min(hi, value))


def percent_to_unipolar_axis(percent: float) -> int:
    """
    Map 0..100 (%) to 0..16383 (integer).

    Args:
        percent: Desired percentage [0..100].
    """
    p = clamp(percent, 0.0, 100.0) / 100.0
    return int(round(p * AXIS_MAX))


def percent_to_bipolar_axis(percent: float) -> int:
    """
    Map -100..+100 (%) to -16383..+16383 (integer).

    Args:
        percent: Desired percentage [-100..100].
    """
    p = clamp(percent, -100.0, 100.0) / 100.0
    # scale into full symmetric range [-16383, +16383]
    return int(round(p * AXIS_MAX))


# ---------------------------------------------------------------------------
# Connection
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class MSFSConnection:
    """
    Thin wrapper around SimConnect, providing generic send/get and
    convenience actions/queries.
    """

    request_timeout_ms: int = 1000
    _sm: Optional[SimConnect] = field(default=None, init=False, repr=False)
    _events: Optional[AircraftEvents] = field(default=None, init=False, repr=False)
    _requests: Optional[AircraftRequests] = field(default=None, init=False, repr=False)

    # -- lifecycle -----------------------------------------------------------

    def connect(self, retries: int = 1, backoff_s: float = 0.5) -> None:
        """
        Open a SimConnect session and prepare helpers.

        Args:
            retries: number of additional attempts if the first connect fails.
            backoff_s: seconds to wait between retries.
        """
        if self._sm is not None:
            LOGGER.debug("Already connected to SimConnect.")
            return

        attempt = 0
        last_exc: Optional[Exception] = None
        while attempt <= retries:
            try:
                LOGGER.info("Connecting to MSFS via SimConnect (attempt %d)...", attempt + 1)
                self._sm = SimConnect()
                self._events = AircraftEvents(self._sm)
                self._requests = AircraftRequests(self._sm, _time=self.request_timeout_ms)
                LOGGER.info("Connected.")
                return
            except Exception as exc:  # noqa: BLE001
                last_exc = exc
                LOGGER.warning("SimConnect connection failed: %s", exc)
                self._sm = None
                self._events = None
                self._requests = None
                if attempt < retries:
                    time.sleep(backoff_s)
                attempt += 1

        raise ConnectionError("Unable to connect to SimConnect.") from last_exc

    def close(self) -> None:
        """Close the SimConnect session."""
        if self._sm is not None:
            try:
                LOGGER.info("Closing SimConnect session.")
                self._sm.exit()
            except Exception as exc:  # noqa: BLE001
                LOGGER.warning("Error while closing SimConnect: %s", exc)
        self._sm = None
        self._events = None
        self._requests = None

    # Context manager for safe usage
    def __enter__(self) -> "MSFSConnection":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # -- core API ------------------------------------------------------------

    def send_event(self, event_id: str, value: Optional[int] = None) -> None:
        """
        Send a Client Event by its MSFS SDK Event ID.

        Args:
            event_id: Exact Client Event name (e.g., 'GEAR_TOGGLE',
                      'AXIS_THROTTLE_SET', 'PARKING_BRAKES_SET').
            value: Optional integer parameter (range depends on the event).
                   If omitted, the event will be fired with no parameter.
        """
        self._ensure_connected()

        assert self._events is not None  # for type checkers
        try:
            ev = self._events.find(event_id)
        except Exception as exc:  # noqa: BLE001
            msg = f"Unknown/unsupported event ID: {event_id!r}"
            LOGGER.error("%s (%s)", msg, exc)
            raise ValueError(msg) from exc

        if value is None:
            LOGGER.debug("Sending event %s()", event_id)
            ev()
        else:
            if not isinstance(value, int):
                raise TypeError(f"Event value must be int, got {type(value).__name__}")
            LOGGER.debug("Sending event %s(%s)", event_id, value)
            ev(value)

    def get_simvar(self, name: str, units: Optional[str] = None) -> Any:
        """
        Read a SimVar by name. Units are optional but recommended.

        Args:
            name: SimVar name exactly as in the MSFS SDK
                  (e.g., 'PLANE ALTITUDE', 'PLANE PITCH DEGREES').
            units: Desired units string where applicable
                   (e.g., 'Feet', 'Knots', 'Degrees').
        """
        self._ensure_connected()

        assert self._requests is not None  # for type checkers
        try:
            if units is None:
                val = self._requests.get(name)
            else:
                val = self._requests.get(name, units)
        except Exception as exc:  # noqa: BLE001
            msg = f"Failed to read SimVar {name!r} (units={units!r})"
            LOGGER.error("%s (%s)", msg, exc)
            raise ValueError(msg) from exc

        LOGGER.debug("Read SimVar %s (%s): %s", name, units, val)
        return val

    # -- convenience ---------------------------------------------------------

    def set_axis_percent(self, event_id: str, percent: float, *, bipolar: bool = False) -> None:
        """
        Convert percent to axis value and send to an AXIS_* event.

        Args:
            event_id: e.g. 'AXIS_ELEVATOR_SET', 'AXIS_THROTTLE_SET'
            percent: 0..100 for unipolar, -100..100 for bipolar
            bipolar: whether to map using bipolar range.
        """
        value = (
            percent_to_bipolar_axis(percent) if bipolar else percent_to_unipolar_axis(percent)
        )
        self.send_event(event_id, value)

    def wait_for_simvar(
        self,
        name: str,
        predicate: Callable[[Any], bool],
        units: Optional[str] = None,
        timeout_s: float = 5.0,
        poll_interval_s: float = 0.1,
    ) -> Any:
        """
        Poll a SimVar until `predicate(value)` is True or timeout elapses.

        Returns the last value read (which satisfies the predicate or the final sample).
        """
        deadline = time.monotonic() + timeout_s
        last_val: Any = None
        while True:
            last_val = self.get_simvar(name, units)
            if predicate(last_val):
                return last_val
            if time.monotonic() >= deadline:
                LOGGER.warning("wait_for_simvar timeout on %r (last=%r)", name, last_val)
                return last_val
            time.sleep(poll_interval_s)

    # -- internals -----------------------------------------------------------

    def _ensure_connected(self) -> None:
        """Raise if not connected; guides user to call connect()."""
        if self._sm is None or self._events is None or self._requests is None:
            raise RuntimeError(
                "Not connected to SimConnect. Call MSFSConnection.connect() first "
                "or use `with MSFSConnection() as msfs: ...`."
            )

    def get_airspeed_true(self) -> float:  # knots
        return self.get_simvar("AIRSPEED_TRUE")

    def get_airspeed_mach(self) -> float:  # mach
        return self.get_simvar("AIRSPEED_MACH")

    # Engines
    def set_engines(self, on: bool) -> None:
        """Start or stop engines using auto-start/auto-shutdown."""
        if on:
            self.send_event("ENGINE_AUTO_START")
        else:
            self.send_event("ENGINE_AUTO_SHUTDOWN")

    # Master battery
    def get_master_battery(self) -> bool:
        return bool(self.get_simvar("ELECTRICAL_MASTER_BATTERY"))

    def set_master_battery(self, on: bool) -> None:
        """
        Set master battery on/off.
        Uses 'MASTER_BATTERY_SET' if available; otherwise toggles to reach state.
        """
        if (on and not self.get_master_battery()) or \
            (not on and self.get_master_battery()):
                self.send_event("TOGGLE_MASTER_BATTERY")

    def get_on_ground(self) -> bool:
        return bool(self.get_simvar("SIM_ON_GROUND"))

    # Throttle (0..100 %)
    def set_throttle_percent(self, percent: float) -> None:
        """Set throttle (all engines) as percent [0..100]."""
        value = percent_to_unipolar_axis(percent)
        # Commonly supported:
        self.send_event("THROTTLE_SET", value)

    # Aileron (−100..+100 %)
    def set_aileron_percent(self, percent: float) -> None:
        """Set aileron deflection as percent [-100..100]."""
        value = percent_to_bipolar_axis(percent)
        self.send_event("AXIS_AILERONS_SET", value)

    # Rudder (−100..+100 %)
    def set_rudder_percent(self, percent: float) -> None:
        """Set rudder deflection as percent [-100..100]."""
        value = percent_to_bipolar_axis(percent)
        self.send_event("AXIS_RUDDER_SET", value)

    # Elevator (−100..+100 %)
    def set_elevator_percent(self, percent: float) -> None:
        """Set elevator deflection as percent [-100..100]."""
        value = percent_to_bipolar_axis(percent)
        self.send_event("AXIS_ELEVATOR_SET", value)

    # Elevator trim (−100..+100 %)
    def set_trim_percent(self, percent: float) -> None:
        """Set elevator trim as percent [-100..100]."""
        # Prefer axis event; some aircraft accept ELEVATOR_TRIM_SET (0..16383) too.
        self.send_event("ELEVATOR_TRIM_SET", percent_to_bipolar_axis(percent))

    def get_parking_brakes(self) -> bool:
        return bool(self.get_simvar("BRAKE_PARKING_INDICATOR"))

    def set_parking_brakes(self, on: bool) -> None:
        if (on and not self.get_parking_brakes()) or \
            (not on and self.get_parking_brakes()):
                self.send_event("PARKING_BRAKES")

    def get_landing_gear_down(self) -> bool:
        return bool(self.get_simvar("GEAR_HANDLE_POSITION"))

    def set_landing_gear_down(self, on: bool) -> None:
        if (on and not self.get_landing_gear_down()) or \
            (not on and self.get_landing_gear_down()):
                self.send_event("GEAR_TOGGLE")

    def get_aileron_percent(self) -> float:
        # AILERON POSITION returns -1..+1
        pos = float(self.get_simvar("AILERON_POSITION"))
        return clamp(pos * 100.0, -100.0, 100.0)

    def get_rudder_percent(self) -> float:
        pos = float(self.get_simvar("RUDDER_POSITION"))
        return clamp(pos * 100.0, -100.0, 100.0)

    def get_elevator_percent(self) -> float:
        pos = float(self.get_simvar("ELEVATOR_POSITION"))
        return clamp(pos * 100.0, -100.0, 100.0)

    def get_trim_percent(self) -> float:
        # ELEVATOR TRIM POSITION returns radians or normalized per aircraft; try degrees
        # If degrees not supported, wrapper will still convert appropriately.
        val = float(self.get_simvar("ELEVATOR_TRIM_PCT")) * 100
        return val

    def get_flaps_percent(self) -> float:
        # FLAPS HANDLE PERCENT returns 0..1
        pos = float(self.get_simvar("FLAPS_HANDLE_PERCENT"))
        return pos * 100

    def _wait_for_simvar(self, name: str, units: str | None = None, poll_interval_s: float = 0.01):
        """Keep polling a simvar until it's non-None and can be cast to float."""
        while True:
            try:
                val = self.get_simvar(name) if units is None else self.get_simvar(name, units)
                if val is not None:
                    return float(val)
            except (TypeError, ValueError):
                pass
            time.sleep(poll_interval_s)

    # Flight metrics
    def get_altitude_ft(self) -> float:
        return self._wait_for_simvar("PLANE_ALTITUDE")

    def get_aoa_deg(self) -> float:
        return self._wait_for_simvar("INCIDENCE_ALPHA")

    def get_ias_kt(self) -> float:
        return self._wait_for_simvar("AIRSPEED_INDICATED")

    def get_tas_kt(self) -> float:
        return self._wait_for_simvar("AIRSPEED_TRUE")

    def get_ground_speed_kt(self) -> float:
        return self._wait_for_simvar("GROUND_VELOCITY")

    def get_vertical_speed_fpm(self) -> float:
        return self._wait_for_simvar("VERTICAL_SPEED")

    def get_pitch_deg(self) -> float:
        return self._wait_for_simvar("PLANE_PITCH_DEGREES")

    def get_bank_deg(self) -> float:
        return self._wait_for_simvar("PLANE_BANK_DEGREES")

    def get_heading_true_deg(self) -> float:
        return self._wait_for_simvar("PLANE_HEADING_DEGREES_TRUE")

    def get_heading_magnetic_deg(self) -> float:
        return self._wait_for_simvar("PLANE_HEADING_DEGREES_MAGNETIC")

    # Control positions (as percents)
    def get_throttle_percent(self) -> float:
        # Average engine 1 position; adjust if you need per-engine.
        val = float(self.get_simvar("GENERAL_ENG_THROTTLE_LEVER_POSITION:1"))
        return val


def takeoff(con: MSFSConnection, target_altitude_ft: float, *, climb_kias: float = 180.0):
    """Automated takeoff using only provided get/set methods, with logs."""

    def wait_until(pred, poll_s=0.1, on_poll=None):
        """Spin until pred() is True; call on_poll() each tick if provided."""
        while True:
            if pred():
                return
            if on_poll:
                on_poll()
            time.sleep(poll_s)

    # Record starting ground altitude for AGL calc (MSL snapshot at brake release)
    ground_alt = con.get_altitude_ft()
    print(f"[INIT] Ground MSL snapshot: {ground_alt:.1f} ft")

    # 1) Battery ON
    if not con.get_master_battery():
        print("[STEP 1] Master battery OFF → turning ON")
        con.set_master_battery(True)
    else:
        print("[STEP 1] Master battery already ON")

    # 2) Engines ON (use autostart; idempotent if already running on many aircraft)
    if con.get_on_ground():
        print("[STEP 2] Requesting ENGINE AUTOSTART")
        con.set_engines(True)
    else:
        print("[STEP 2] Not on ground (already airborne?) — skipping ENGINE AUTOSTART")

    # 3) Parking brake OFF
    if con.get_parking_brakes():
        print("[STEP 3] Parking brake ON → releasing")
        con.set_parking_brakes(False)
    else:
        print("[STEP 3] Parking brake already released")

    # 4) Throttle 100%
    print("[STEP 4] Throttle → 100%")
    con.set_throttle_percent(100.0)

    # 5) Rotate at 160 KIAS → elevator -20% (nose up)
    last_log_t = time.monotonic()
    def _poll_speed_log():
        nonlocal last_log_t
        t = time.monotonic()
        if t - last_log_t >= 1.0:
            ias = con.get_ias_kt()
            alt = con.get_altitude_ft()
            agl = alt - ground_alt
            vsi = con.get_vertical_speed_fpm()
            print(f"[ACCEL] IAS={ias:.1f} kt | ALT={alt:.0f} ft | AGL={agl:.0f} ft | VS={vsi:.0f} fpm")
            last_log_t = t

    print("[STEP 5] Waiting for rotate speed 160 KIAS…")
    wait_until(lambda: con.get_ias_kt() >= 160.0, poll_s=0.1, on_poll=_poll_speed_log)

    print("[ROTATE] Reaching 160 KIAS → elevator = -20% (nose up)")
    con.set_elevator_percent(-20.0)

    # 6) Gear up at 500 ft AGL
    print("[CLIMB] Holding -20% elevator until 1,000 ft AGL. Gear will retract at 500 ft AGL.")
    def _agl() -> float:
        return con.get_altitude_ft() - ground_alt

    # — Gear at 500 AGL
    wait_until(lambda: _agl() >= 500.0, poll_s=0.1, on_poll=_poll_speed_log)
    if con.get_landing_gear_down():
        print("[GEAR] 500 ft AGL reached → Gear UP")
        con.set_landing_gear_down(False)
    else:
        print("[GEAR] Gear already UP at 500 ft AGL")

    # — Keep holding -20% elevator until 1,000 AGL to cleanly climb out
    wait_until(lambda: _agl() >= 1000.0, poll_s=0.1, on_poll=_poll_speed_log)
    print("[CLIMB] 1,000 ft AGL reached → switching to IAS-hold climb")

    # 6.5) Simple IAS-hold climb (elevator-only) until target altitude
    # Control law: elevator% = bias + Kp*(target - IAS), clamped
    # Notes:
    #   * Negative elevator = nose up (slower IAS); positive = nose down (faster IAS)
    #   * Choose gentle gains to avoid porpoising.
    bias = -10.0         # base nose-up for climb
    kp = 0.25            # % elevator per knot error (start gentle)
    elev_min, elev_max = -25.0, +15.0
    last_ctrl_log_t = time.monotonic()

    def _climb_step():
        nonlocal last_ctrl_log_t
        ias = con.get_ias_kt()
        alt = con.get_altitude_ft()
        agl = alt - ground_alt
        vsi = con.get_vertical_speed_fpm()
        err = (climb_kias - ias)  # +ve if slow → nose down (more + elevator)
        elev_cmd = max(elev_min, min(elev_max, bias + kp * err))
        con.set_elevator_percent(elev_cmd)

        t = time.monotonic()
        if t - last_ctrl_log_t >= 1.0:
            pitch = con.get_pitch_deg()
            print(f"[IAS-HOLD] IAS tgt={climb_kias:.0f} | IAS={ias:.1f} kt | ALT={alt:.0f} ft "
                  f"(AGL {agl:.0f}) | VS={vsi:.0f} fpm | PITCH={pitch:.1f}° | ELEV_CMD={elev_cmd:.1f}%")
            last_ctrl_log_t = t

    # Climb loop until target MSL altitude reached
    while con.get_altitude_ft() < target_altitude_ft:
        _climb_step()
        time.sleep(0.2)  # control update rate ~5 Hz

    # 7) Level off: elevator to 0% (neutral)
    print(f"[LEVEL] Target altitude {target_altitude_ft:.0f} ft reached → elevator = 0% (level)")
    con.set_elevator_percent(0.0)
    print("[DONE] Takeoff + initial climb complete.")


def takeoff2(
    con: MSFSConnection,
    target_altitude_ft: float,
    *,
    rotate_kias: float = 160.0,
    target_pitch_deg: float = 12.0,   # rotation pitch target
    climb_kias: float = 180.0         # IAS to hold after 1,000 ft AGL
):
    """Stabilized takeoff with smooth rotation and IAS-hold climb (get/set only)."""

    # ---------------- helpers ----------------
    def wait_until(pred, poll_s=0.1, on_poll=None):
        while True:
            if pred():
                return
            if on_poll:
                on_poll()
            time.sleep(poll_s)

    def clamp(v, lo, hi):
        return lo if v < lo else hi if v > hi else v

    # --------- snapshot for AGL & logging ----------
    ground_alt = con.get_altitude_ft()
    print(f"[INIT] Ground MSL snapshot: {ground_alt:.1f} ft")

    # 1) Battery ON
    if not con.get_master_battery():
        print("[STEP 1] Master battery OFF → turning ON")
        con.set_master_battery(True)
    else:
        print("[STEP 1] Master battery already ON")

    # 2) Engines ON
    if con.get_on_ground():
        print("[STEP 2] Requesting ENGINE AUTOSTART")
        con.set_engines(True)
    else:
        print("[STEP 2] Not on ground (already airborne?) — skipping ENGINE AUTOSTART")

    # 3) Parking brake OFF
    if con.get_parking_brakes():
        print("[STEP 3] Parking brake ON → releasing")
        con.set_parking_brakes(False)
    else:
        print("[STEP 3] Parking brake already released")

    # 4) Throttle 100%
    print("[STEP 4] Throttle → 100%")
    con.set_throttle_percent(100.0)

    # 5) Rotate at rotate_kias → smooth pitch-hold (nose up = NEGATIVE elevator)
    last_log_t = time.monotonic()
    def _poll_speed_log():
        nonlocal last_log_t
        t = time.monotonic()
        if t - last_log_t >= 1.0:
            ias  = con.get_ias_kt()
            alt  = con.get_altitude_ft()
            agl  = alt - ground_alt
            vsi  = con.get_vertical_speed_fpm()
            pitch= con.get_pitch_deg()
            elev = con.get_elevator_percent()
            print(f"[ACCEL] IAS={ias:.1f} kt | ALT={alt:.0f} ft | AGL={agl:.0f} ft | "
                  f"VS={vsi:.0f} fpm | PITCH={pitch:+.1f}° | ELEV={elev:+.1f}%")
            last_log_t = t

    print(f"[STEP 5] Waiting for rotate speed {rotate_kias:.0f} KIAS…")
    wait_until(lambda: con.get_ias_kt() >= rotate_kias, poll_s=0.1, on_poll=_poll_speed_log)

    # ---- Smooth rotation & pitch hold to 1,000 ft AGL ----
    print(f"[ROTATE] {rotate_kias:.0f} KIAS reached → begin smooth rotation (pitch-hold ≈ {target_pitch_deg:.0f}°)")
    elev_cmd = con.get_elevator_percent()
    elev_min, elev_max = -30.0, +15.0      # safety limits (NEG = nose up)
    max_step_per_tick = 3.0                # % per control update (rate limit)
    dt = 0.05                              # 20 Hz control during rotation
    Kp = 1.2                               # % elevator per deg pitch error
    Kd = 0.9                               # % elevator per deg/s pitch rate
    last_pitch = con.get_pitch_deg()
    last_t = time.monotonic()

    # Small initial nudge (not a big step) to start rotation
    elev_cmd = clamp(elev_cmd - 8.0, elev_min, elev_max)
    con.set_elevator_percent(elev_cmd)

    print("[CLIMB] Pitch-hold active until 1,000 ft AGL. Gear will retract at 500 ft AGL.")
    gear_up_done = False

    def _agl(): return con.get_altitude_ft() - ground_alt

    while _agl() < 1000.0:
        # state
        t = time.monotonic()
        pitch = con.get_pitch_deg()
        dt_s = max(1e-3, t - last_t)
        pitch_rate = (pitch - last_pitch) / dt_s
        last_pitch, last_t = pitch, t

        # control: make elevator more NEGATIVE for more nose-up
        pitch_err = target_pitch_deg - pitch          # + if we need more nose-up
        raw_cmd = elev_cmd - (Kp * pitch_err) + (Kd * pitch_rate)

        # rate limit & clamp
        delta = clamp(raw_cmd - elev_cmd, -max_step_per_tick, +max_step_per_tick)
        elev_cmd = clamp(elev_cmd + delta, elev_min, elev_max)
        con.set_elevator_percent(elev_cmd)

        # gear at 500 AGL (once)
        if not gear_up_done and _agl() >= 500.0:
            if con.get_landing_gear_down():
                print("[GEAR] 500 ft AGL → Gear UP")
                con.set_landing_gear_down(False)
            else:
                print("[GEAR] Already UP at 500 ft AGL")
            gear_up_done = True

        _poll_speed_log()
        time.sleep(dt)

    print("[CLIMB] 1,000 ft AGL reached → switching to IAS-hold climb")

    # 6) IAS-hold climb (gentle) until target MSL altitude
    #    elevator% = bias + kp*(climb_kias - IAS); trim assist keeps elevator near a small negative
    bias = -8.0              # base nose-up
    kp_ias = 0.22            # % elevator per knot error
    elev_min, elev_max = -25.0, +15.0
    trim_target_elev = -5.0  # aim to keep commanded elevator near this
    last_ctrl_log_t = time.monotonic()
    last_trim_update = time.monotonic()
    trim_update_interval = 1.5  # seconds

    while con.get_altitude_ft() < target_altitude_ft:
        ias = con.get_ias_kt()
        alt = con.get_altitude_ft()
        agl = alt - ground_alt
        vsi = con.get_vertical_speed_fpm()

        err = (climb_kias - ias)  # + if slow → more nose-down (POS elevator)
        elev_cmd = clamp(bias + kp_ias * err, elev_min, elev_max)
        con.set_elevator_percent(elev_cmd)

        # Trim assist: adjust absolute trim to keep elevator around trim_target_elev
        if time.monotonic() - last_trim_update >= trim_update_interval:
            current_elev = con.get_elevator_percent()
            current_trim = con.get_trim_percent()
            trim_err = trim_target_elev - current_elev
            trim_step = clamp(0.3 * trim_err, -2.0, 2.0)         # small, gentle
            new_trim = clamp(current_trim + trim_step, -100.0, 100.0)
            if abs(trim_step) > 0.05:
                con.set_trim_percent(new_trim)                   # absolute trim position
                print(f"[TRIM] Trim {current_trim:+.1f}% → {new_trim:+.1f}% (elev {current_elev:+.1f}%)")
            last_trim_update = time.monotonic()

        # 1 Hz status
        t = time.monotonic()
        if t - last_ctrl_log_t >= 1.0:
            pitch = con.get_pitch_deg()
            print(f"[IAS-HOLD] IAS tgt={climb_kias:.0f} | IAS={ias:.1f} kt | ALT={alt:.0f} ft "
                  f"(AGL {agl:.0f}) | VS={vsi:.0f} fpm | PITCH={pitch:+.1f}° | ELEV={elev_cmd:+.1f}%")
            last_ctrl_log_t = t

        time.sleep(0.2)  # ~5 Hz

    # 7) Level at target: neutralize elevator (leave throttle to user or AP)
    print(f"[LEVEL] Target altitude {target_altitude_ft:.0f} ft reached → elevator = 0% (level)")
    con.set_elevator_percent(0.0)
    print("[DONE] Takeoff + initial climb complete.")


def takeoff3(
    con: MSFSConnection,
    target_altitude_ft: float,
    *,
    climb_kias: float = 180.0,
    rotate_kias: float = 160.0,
    max_aoa_deg: float = 12.0,         # hard ceiling for |AOA|
    max_pitch_deg: float = 15.0,       # don't let pitch exceed this during initial climb
    elevator_slew_per_s: float = 35.0, # % elevator per second (rate limit)
    control_rate_hz: float = 10.0,     # control loop frequency
):
    """Automated takeoff with smooth rotation and AOA-limited climb (elevator-only)."""

    def wait_until(pred, poll_s=0.1, on_poll=None):
        """Spin until pred() is True; call on_poll() each tick if provided."""
        while True:
            if pred():
                return
            if on_poll:
                on_poll()
            time.sleep(poll_s)

    dt = 1.0 / control_rate_hz
    slew_step = elevator_slew_per_s * dt  # max delta-elevator per cycle (in %)

    # --- helpers ------------------------------------------------------------

    def _agl(ground_ft: float) -> float:
        return con.get_altitude_ft() - ground_ft

    def _get_aoa_deg_safe() -> Optional[float]:
        """Try a direct AOA read; if unavailable, approximate using FPA."""
        try:
            # If your wrapper exposes this directly, prefer it.
            return float(con.get_aoa_deg())  # SimVar 'INCIDENCE ALPHA' (degrees)
        except Exception:
            try:
                pitch = float(con.get_pitch_deg())
                vsi_fpm = float(con.get_vertical_speed_fpm())
                ias_kt = float(con.get_ias_kt())
                if ias_kt < 50.0:
                    return None  # unreliable before real airflow
                speed_fps = ias_kt * 1.68781
                vsi_fps = vsi_fpm / 60.0
                # Flight path angle ≈ atan(VS / Speed)
                fpa_deg = math.degrees(math.atan2(vsi_fps, speed_fps))
                return pitch - fpa_deg
            except Exception:
                return None

    def _ramp_elevator_to(target_pct: float, get_current: Callable[[], float]) -> float:
        """Slew-limit the change from current elevator to target; returns new command."""
        current = get_current()
        delta = target_pct - current
        if abs(delta) <= slew_step:
            return target_pct
        return current + (slew_step if delta > 0 else -slew_step)

    # state carried across the control loop
    prev_elev_cmd = 0.0
    prev_ias = None
    prev_t = None

    # Record starting ground altitude for AGL calc (MSL snapshot at brake release)
    ground_alt = con.get_altitude_ft()
    print(f"[INIT] Ground MSL snapshot: {ground_alt:.1f} ft")

    # 1) Battery ON
    if not con.get_master_battery():
        print("[STEP 1] Master battery OFF → turning ON")
        con.set_master_battery(True)
    else:
        print("[STEP 1] Master battery already ON")

    # 2) Engines ON (autostart; idempotent on many aircraft)
    if con.get_on_ground():
        print("[STEP 2] Requesting ENGINE AUTOSTART")
        con.set_engines(True)
    else:
        print("[STEP 2] Not on ground (already airborne?) — skipping ENGINE AUTOSTART")

    # 3) Parking brake OFF
    if con.get_parking_brakes():
        print("[STEP 3] Parking brake ON → releasing")
        con.set_parking_brakes(False)
    else:
        print("[STEP 3] Parking brake already released")

    # 4) Throttle 100%
    print("[STEP 4] Throttle → 100%")
    con.set_throttle_percent(100.0)

    # 5) Accelerate to rotate speed with periodic logging
    last_log_t = time.monotonic()

    def _poll_speed_log():
        nonlocal last_log_t
        t = time.monotonic()
        if t - last_log_t >= 1.0:
            ias = con.get_ias_kt()
            alt = con.get_altitude_ft()
            agl = alt - ground_alt
            vsi = con.get_vertical_speed_fpm()
            print(f"[ACCEL] IAS={ias:.1f} kt | ALT={alt:.0f} ft | AGL={agl:.0f} ft | VS={vsi:.0f} fpm")
            last_log_t = t

    print(f"[STEP 5] Waiting for rotate speed {rotate_kias:.0f} KIAS…")
    wait_until(lambda: con.get_ias_kt() >= rotate_kias, poll_s=0.1, on_poll=_poll_speed_log)

    # Smooth rotation: target initial pitch ~10°, not a hard -20% elevator step
    import math
    target_pitch_deg = 10.0
    print(f"[ROTATE] Reaching {rotate_kias:.0f} KIAS → smooth rotate to ~{target_pitch_deg:.0f}° pitch")

    rotate_timeout_s = 6.0  # give it a few seconds to lift gently
    t_end = time.monotonic() + rotate_timeout_s
    while time.monotonic() < t_end and con.get_on_ground():
        pitch = con.get_pitch_deg()
        pitch_err = target_pitch_deg - pitch

        # Simple P controller on pitch for rotation phase, negative elevator is nose-up
        kp_pitch = -1.2  # % elevator per deg pitch error (negative for nose-up command)
        raw_cmd = kp_pitch * pitch_err

        # AOA guard during rotation
        aoa = _get_aoa_deg_safe()
        if aoa is not None and aoa > max_aoa_deg:
            # Reduce nose-up (drive elevator more positive)
            excess = aoa - max_aoa_deg
            raw_cmd += +1.0 * excess  # push gently

        # Pitch hard ceiling
        if pitch > max_pitch_deg:
            raw_cmd += +2.0 * (pitch - max_pitch_deg)

        # Slew-limit around current elevator command
        new_cmd = _ramp_elevator_to(raw_cmd, lambda: prev_elev_cmd)
        con.set_elevator_percent(new_cmd)
        prev_elev_cmd = new_cmd

        time.sleep(dt)

    # 6) Gear up at 500 ft AGL, hold gentle pitch until 1,000 AGL
    print("[CLIMB] Holding gentle climb; gear retracts at 500 ft AGL, IAS-hold at 1,000 ft AGL.")

    # — Gear at 500 AGL
    wait_until(lambda: _agl(ground_alt) >= 500.0, poll_s=0.1, on_poll=_poll_speed_log)
    if con.get_landing_gear_down():
        print("[GEAR] 500 ft AGL reached → Gear UP")
        con.set_landing_gear_down(False)
    else:
        print("[GEAR] Gear already UP at 500 ft AGL")

    # — Keep climbing to 1,000 AGL with pitch guard
    while _agl(ground_alt) < 1000.0:
        pitch = con.get_pitch_deg()
        pitch_err = target_pitch_deg - pitch
        kp_pitch = -0.8
        raw_cmd = kp_pitch * pitch_err

        # AOA & pitch guards
        aoa = _get_aoa_deg_safe()
        if aoa is not None and aoa > max_aoa_deg:
            raw_cmd += +1.0 * (aoa - max_aoa_deg)
        if pitch > max_pitch_deg:
            raw_cmd += +2.0 * (pitch - max_pitch_deg)

        # Slew-limit and send
        new_cmd = _ramp_elevator_to(raw_cmd, lambda: prev_elev_cmd)
        con.set_elevator_percent(new_cmd)
        prev_elev_cmd = new_cmd

        _poll_speed_log()
        time.sleep(dt)

    print("[CLIMB] 1,000 ft AGL reached → switching to IAS-hold climb (smooth).")

    # 6.5) Smooth IAS-hold climb (elevator-only) until target altitude
    # Control law (PD + protectors):
    #   elev_cmd = bias + Kp*(IAS_err) + Kd*(d/dt IAS_err)
    # Sign convention reminder: negative elevator = nose up (slower IAS)
    bias = -8.0            # base nose-up for climb
    kp = 0.18              # % elevator per knot error
    kd = 0.08              # % per (knot/s) derivative to damp porpoising
    elev_min, elev_max = -22.0, +12.0

    last_ctrl_log_t = time.monotonic()

    while con.get_altitude_ft() < target_altitude_ft:
        now = time.monotonic()
        ias = con.get_ias_kt()
        alt = con.get_altitude_ft()
        agl = alt - ground_alt
        vsi = con.get_vertical_speed_fpm()

        # Error terms
        err = (climb_kias - ias)  # + if we're slow → should command more nose-down (more + elevator)
        derr = 0.0
        if prev_ias is not None and prev_t is not None:
            dtm = max(1e-3, now - prev_t)
            derr = (err - (climb_kias - prev_ias)) / dtm  # change in speed error (knots/s)

        # Raw command
        raw_cmd = bias + (kp * err) + (kd * derr)

        # AOA protection (both sides); reduce nose-up if |AOA| too high
        aoa = _get_aoa_deg_safe()
        if aoa is not None:
            if aoa > max_aoa_deg:
                raw_cmd += +1.2 * (aoa - max_aoa_deg)  # push
            elif aoa < -max_aoa_deg:
                raw_cmd -= +1.2 * (-max_aoa_deg - aoa)  # pull a bit less negative

        # Pitch ceiling protection
        pitch = con.get_pitch_deg()
        if pitch > max_pitch_deg:
            raw_cmd += +1.5 * (pitch - max_pitch_deg)

        # Clamp and slew-limit
        raw_cmd = max(elev_min, min(elev_max, raw_cmd))
        new_cmd = _ramp_elevator_to(raw_cmd, lambda: prev_elev_cmd)

        # Apply
        con.set_elevator_percent(new_cmd)
        prev_elev_cmd = new_cmd
        prev_ias = ias
        prev_t = now

        # Logging at ~1 Hz
        t = time.monotonic()
        if t - last_ctrl_log_t >= 1.0:
            aoa_str = f"{aoa:.1f}°" if aoa is not None else "n/a"
            print(
                f"[IAS-HOLD] tgt={climb_kias:.0f} | IAS={ias:.1f} kt | ALT={alt:.0f} ft "
                f"(AGL {agl:.0f}) | VS={vsi:.0f} fpm | PITCH={pitch:.1f}° | AOA={aoa_str} "
                f"| ELEV_CMD={new_cmd:.1f}%"
            )
            last_ctrl_log_t = t

        time.sleep(dt)

    # 7) Level off: smoothly return elevator to neutral
    print(f"[LEVEL] Target altitude {target_altitude_ft:.0f} ft reached → easing elevator to 0% (level)")
    while abs(prev_elev_cmd) > 0.5:
        prev_elev_cmd = _ramp_elevator_to(0.0, lambda: prev_elev_cmd)
        con.set_elevator_percent(prev_elev_cmd)
        time.sleep(dt)

    print("[DONE] Takeoff + initial climb complete.")


def takeoff4(
    con: MSFSConnection,
    target_altitude_ft: float,
    *,
    rotate_kias: float = 160.0,
    climb_kias: float = 180.0,
    control_rate_hz: float = 10.0,
    # Pitch limits (angles are aircraft attitude, not AOA)
    target_pitch_deg: float = 10.0,     # initial rotation target pitch
    max_pitch_up_deg: float = 18.0,     # hard guard: never exceed this nose-up
    max_pitch_dn_deg: float = -10.0,    # hard guard: never exceed this nose-down
    # Controllers
    pitch_kp: float = -1.0,             # % elevator per deg error (negative → nose up for +err)
    pitch_kd: float = -0.35,            # % elevator per deg/s (damping)
    ias_to_pitch_gain: float = 0.05,    # extra pitch target per knot slow (caps apply)
    ias_to_throttle_gain: float = 0.6,  # % throttle per knot slow
    base_throttle_percent: float = 100.0,
    # Actuator rate limits
    elevator_slew_per_s: float = 35.0,  # % elevator per second
    throttle_slew_per_s: float = 40.0,  # % throttle per second
):
    """
    Smooth automated takeoff with pitch-angle stabilization and dynamic throttle.
    - Prevents large pitch excursions (±40°) via hard-envelope protection.
    - Uses pitch PID (with pitch-rate derivative) and IAS-based trim/throttle.
    - Elevator and throttle are slew-limited to avoid abrupt changes.
    """

    import time
    import math

    def clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def slew(current, target, step):
        delta = target - current
        if abs(delta) <= step:
            return target
        return current + (step if delta > 0 else -step)

    def wait_until(pred, poll_s=0.1, on_poll=None):
        while True:
            if pred():
                return
            if on_poll:
                on_poll()
            time.sleep(poll_s)

    dt = 1.0 / control_rate_hz
    elev_step = elevator_slew_per_s * dt
    thr_step = throttle_slew_per_s * dt

    # ---- Phase 0: preflight / ground snapshot --------------------------------
    ground_alt = con.get_altitude_ft()
    print(f"[INIT] Ground MSL snapshot: {ground_alt:.1f} ft")

    if not con.get_master_battery():
        print("[STEP 1] Master battery OFF → turning ON")
        con.set_master_battery(True)
    else:
        print("[STEP 1] Master battery already ON")

    if con.get_on_ground():
        print("[STEP 2] Requesting ENGINE AUTOSTART")
        con.set_engines(True)
    else:
        print("[STEP 2] Already airborne? Skipping ENGINE AUTOSTART")

    if con.get_parking_brakes():
        print("[STEP 3] Parking brake ON → releasing")
        con.set_parking_brakes(False)
    else:
        print("[STEP 3] Parking brake already released")

    print("[STEP 4] Throttle → takeoff power")
    con.set_throttle_percent(base_throttle_percent)

    # ---- Phase 1: accelerate to rotate ---------------------------------------
    last_log_t = time.monotonic()
    def _accel_log():
        nonlocal last_log_t
        t = time.monotonic()
        if t - last_log_t >= 1.0:
            ias = con.get_ias_kt()
            alt = con.get_altitude_ft()
            agl = alt - ground_alt
            vsi = con.get_vertical_speed_fpm()
            pitch = con.get_pitch_deg()
            print(f"[ACCEL] IAS={ias:.1f} kt | ALT={alt:.0f} ft | AGL={agl:.0f} ft | VS={vsi:.0f} fpm | PITCH={pitch:.1f}°")
            last_log_t = t

    print(f"[STEP 5] Waiting for rotate speed {rotate_kias:.0f} KIAS…")
    wait_until(lambda: con.get_ias_kt() >= rotate_kias, poll_s=0.1, on_poll=_accel_log)

    # ---- Phase 2: rotation (smooth ramp to target_pitch_deg) ------------------
    print(f"[ROTATE] {rotate_kias:.0f} KIAS reached → smooth rotate to ~{target_pitch_deg:.0f}° pitch")

    # Control loop state
    elev_cmd = 0.0
    thr_cmd = base_throttle_percent
    prev_pitch = con.get_pitch_deg()
    prev_t = time.monotonic()

    # Ramp rotation for a few seconds or until airborne
    rotate_duration_s = 5.0
    rotate_end = prev_t + rotate_duration_s
    while time.monotonic() < rotate_end and con.get_on_ground():
        now = time.monotonic()
        dtm = max(1e-3, now - prev_t)
        pitch = con.get_pitch_deg()
        pitch_rate = (pitch - prev_pitch) / dtm  # deg/s

        # Rotation target ramp: approach target_pitch_deg smoothly
        # Use small IAS-based bias so slow acceleration doesn't cause over-pull
        ias = con.get_ias_kt()
        target_pitch = target_pitch_deg + clamp((climb_kias - ias) * 0.02, -2.0, 2.0)
        target_pitch = clamp(target_pitch, 6.0, max_pitch_up_deg - 1.0)

        # Pitch PID (no integral): negative gains → negative elevator for positive error (nose-up)
        pitch_err = target_pitch - pitch
        raw_elev = pitch_kp * pitch_err + pitch_kd * pitch_rate

        # Envelope protection (hard): if exceeding limits, add strong counter-command
        if pitch > max_pitch_up_deg:
            raw_elev += +2.0 * (pitch - max_pitch_up_deg)  # push
        if pitch < max_pitch_dn_deg:
            raw_elev += -2.0 * (pitch - max_pitch_dn_deg)  # pull (note minus because pitch - min is negative)

        # Limit & slew elevator
        raw_elev = clamp(raw_elev, -25.0, +15.0)
        elev_cmd = slew(elev_cmd, raw_elev, elev_step)
        con.set_elevator_percent(elev_cmd)

        prev_pitch, prev_t = pitch, now
        time.sleep(dt)

    # ---- Phase 3: initial climb to 1000 AGL with angle guard -----------------
    print("[CLIMB] Climbing out; gear up at 500 AGL; transitioning to IAS hold at 1,000 AGL.")

    # Gear up at 500 AGL
    def _agl():
        return con.get_altitude_ft() - ground_alt

    wait_until(lambda: _agl() >= 500.0, poll_s=0.1, on_poll=_accel_log)
    if con.get_landing_gear_down():
        print("[GEAR] 500 ft AGL reached → Gear UP")
        con.set_landing_gear_down(False)
    else:
        print("[GEAR] Already UP at 500 ft AGL")

    # Keep a controlled pitch until 1000 AGL (still prioritizing angle safety)
    while _agl() < 1000.0:
        now = time.monotonic()
        dtm = max(1e-3, now - prev_t)
        pitch = con.get_pitch_deg()
        pitch_rate = (pitch - prev_pitch) / dtm
        ias = con.get_ias_kt()

        # IAS-trimmed pitch target (slow → slightly higher target pitch)
        target_pitch = target_pitch_deg + clamp((climb_kias - ias) * ias_to_pitch_gain, -4.0, 4.0)
        target_pitch = clamp(target_pitch, 6.0, max_pitch_up_deg - 1.0)

        # Pitch control
        pitch_err = target_pitch - pitch
        raw_elev = pitch_kp * pitch_err + pitch_kd * pitch_rate

        # Envelope protection
        if pitch > max_pitch_up_deg:
            raw_elev += +2.0 * (pitch - max_pitch_up_deg)
        if pitch < max_pitch_dn_deg:
            raw_elev += -2.0 * (pitch - max_pitch_dn_deg)

        raw_elev = clamp(raw_elev, -22.0, +12.0)
        elev_cmd = slew(elev_cmd, raw_elev, elev_step)
        con.set_elevator_percent(elev_cmd)

        # Throttle assist toward climb speed
        thr_target = base_throttle_percent + (climb_kias - ias) * ias_to_throttle_gain
        thr_target = clamp(thr_target, 70.0, 100.0)
        thr_cmd = slew(thr_cmd, thr_target, thr_step)
        con.set_throttle_percent(thr_cmd)

        # Overshoot management: if pitch rising past hard limit, also bleed a bit of power
        if pitch > (max_pitch_up_deg - 1.0):
            thr_cmd = max(70.0, thr_cmd - 2.0)
            con.set_throttle_percent(thr_cmd)

        prev_pitch, prev_t = pitch, now
        _accel_log()
        time.sleep(dt)

    print("[CLIMB] 1,000 ft AGL reached → IAS-hold climb with angle protection.")

    # ---- Phase 4: IAS-hold climb to target MSL with angle & throttle control --
    last_ctrl_log_t = time.monotonic()
    while con.get_altitude_ft() < target_altitude_ft:
        now = time.monotonic()
        dtm = max(1e-3, now - prev_t)
        pitch = con.get_pitch_deg()
        pitch_rate = (pitch - prev_pitch) / dtm
        ias = con.get_ias_kt()
        alt = con.get_altitude_ft()
        vsi = con.get_vertical_speed_fpm()
        agl = alt - ground_alt

        # Target pitch depends on IAS error (slow -> more pitch up), but bounded
        target_pitch = target_pitch_deg + clamp((climb_kias - ias) * ias_to_pitch_gain, -6.0, 6.0)
        target_pitch = clamp(target_pitch, 5.0, max_pitch_up_deg - 1.0)

        # Pitch control
        pitch_err = target_pitch - pitch
        raw_elev = pitch_kp * pitch_err + pitch_kd * pitch_rate

        # Hard envelope: keep aircraft attitude in safe bounds
        if pitch > max_pitch_up_deg:
            raw_elev += +2.5 * (pitch - max_pitch_up_deg)
        if pitch < max_pitch_dn_deg:
            raw_elev += -2.5 * (pitch - max_pitch_dn_deg)

        raw_elev = clamp(raw_elev, -22.0, +12.0)
        elev_cmd = slew(elev_cmd, raw_elev, elev_step)
        con.set_elevator_percent(elev_cmd)

        # Throttle control to chase climb speed without demanding extreme pitch
        thr_target = base_throttle_percent + (climb_kias - ias) * ias_to_throttle_gain
        thr_target = clamp(thr_target, 65.0, 100.0)
        thr_cmd = slew(thr_cmd, thr_target, thr_step)
        con.set_throttle_percent(thr_cmd)

        # Logging ~1 Hz
        t = time.monotonic()
        if t - last_ctrl_log_t >= 1.0:
            print(
                f"[IAS-HOLD] tgtIAS={climb_kias:.0f} | IAS={ias:.1f} kt | ALT={alt:.0f} ft "
                f"(AGL {agl:.0f}) | VS={vsi:.0f} fpm | PITCH={pitch:.1f}° "
                f"| ELEV={elev_cmd:.1f}% | THR={thr_cmd:.1f}%"
            )
            last_ctrl_log_t = t

        prev_pitch, prev_t = pitch, now
        time.sleep(dt)

    # ---- Phase 5: level-off ---------------------------------------------------
    print(f"[LEVEL] Target altitude {target_altitude_ft:.0f} ft reached → easing elevator to 0%")
    # Smoothly neutralize elevator
    while abs(elev_cmd) > 0.5:
        elev_cmd = slew(elev_cmd, 0.0, elev_step)
        con.set_elevator_percent(elev_cmd)
        time.sleep(dt)

    # Throttle to a conservative cruise placeholder (leave to AP/next mode)
    con.set_throttle_percent(clamp(thr_cmd, 60.0, 90.0))
    print("[DONE] Takeoff + climb complete with pitch stabilization.")



def takeoff5(
    con: MSFSConnection,
    target_altitude_ft: float,
    *,
    rotate_kias: float = 160.0,
    climb_kias: float = 180.0,
    control_rate_hz: float = 10.0,
    # Pitch limits (angles are aircraft attitude, not AOA)
    target_pitch_deg: float = 10.0,     # initial rotation target pitch
    max_pitch_up_deg: float = 18.0,     # hard guard: never exceed this nose-up
    max_pitch_dn_deg: float = -10.0,    # hard guard: never exceed this nose-down
    # Controllers
    pitch_kp: float = -1.0,             # % elevator per deg error (negative → nose up for +err)
    pitch_kd: float = -0.35,            # % elevator per deg/s (damping)
    ias_to_pitch_gain: float = 0.05,    # extra pitch target per knot slow (caps apply)
    ias_to_throttle_gain: float = 0.6,  # % throttle per knot slow
    base_throttle_percent: float = 100.0,
    # Rudder (centerline) controller
    rudder_kh: float = 0.5,             # % rudder per deg heading/track error
    rudder_kd: float = 0.35,            # % per deg/s yaw-rate (damping)
    rudder_kbeta: float = 0.6,          # % per deg sideslip (beta) to center the ball (if available)
    rudder_slew_per_s: float = 50.0,    # % per second
    rudder_limit_pct: float = 25.0,     # mechanical/comfort limit
    # Actuator rate limits
    elevator_slew_per_s: float = 35.0,  # % elevator per second
    throttle_slew_per_s: float = 40.0,  # % throttle per second
):
    """
    Smooth automated takeoff with pitch-angle stabilization, centerline tracking via rudder,
    and dynamic throttle. Uses heading/track, yaw-rate, and sideslip (if available) to keep
    the aircraft aligned and coordinated during the ground roll and early climb.
    """

    import time
    import math

    def clamp(x, lo, hi): return max(lo, min(hi, x))

    def slew(current, target, step):
        d = target - current
        if abs(d) <= step:
            return target
        return current + (step if d > 0 else -step)

    def shortest_angle_deg(a_minus_b):
        # wrap to [-180, +180]
        e = (a_minus_b + 180.0) % 360.0
        return e - 180.0

    def wait_until(pred, poll_s=0.1, on_poll=None):
        while True:
            if pred():
                return
            if on_poll:
                on_poll()
            time.sleep(poll_s)

    # --- Optional getters with graceful fallback ------------------------------
    def _get_sideslip_deg():
        # SimVar often named 'INCIDENCE BETA' (deg). If not available, return 0.
        try:
            return float(con.get_sideslip_deg())
        except Exception:
            return 0.0

    def _get_ground_track_deg():
        # Prefer GPS ground track if your wrapper exposes it; fallback to magnetic heading.
        try:
            return float(con.get_ground_track_deg())
        except Exception:
            return float(con.get_heading_magnetic_deg())

    def _get_yaw_rate_deg_s(prev_hdg, prev_t):
        # If simvar for yaw rate exists, use it; else estimate from heading derivative.
        try:
            return float(con.get_yaw_rate_deg_s())
        except Exception:
            now = time.monotonic()
            dtm = max(1e-3, now - prev_t[0])
            hdg = float(con.get_heading_magnetic_deg())
            rate = shortest_angle_deg(hdg - prev_hdg[0]) / dtm
            prev_hdg[0] = hdg
            prev_t[0] = now
            return rate

    dt = 1.0 / control_rate_hz
    elev_step = elevator_slew_per_s * dt
    thr_step  = throttle_slew_per_s * dt
    rud_step  = rudder_slew_per_s * dt

    # ---- Phase 0: preflight / ground snapshot --------------------------------
    ground_alt = con.get_altitude_ft()
    runway_hdg = float(con.get_heading_magnetic_deg())  # centerline proxy
    print(f"[INIT] Ground MSL snapshot: {ground_alt:.1f} ft | Runway heading≈{runway_hdg:.1f}°M")

    if not con.get_master_battery():
        print("[STEP 1] Master battery OFF → turning ON")
        con.set_master_battery(True)
    else:
        print("[STEP 1] Master battery already ON")

    if con.get_on_ground():
        print("[STEP 2] Requesting ENGINE AUTOSTART")
        con.set_engines(True)
    else:
        print("[STEP 2] Already airborne? Skipping ENGINE AUTOSTART")

    if con.get_parking_brakes():
        print("[STEP 3] Parking brake ON → releasing")
        con.set_parking_brakes(False)
    else:
        print("[STEP 3] Parking brake already released")

    print("[STEP 4] Throttle → takeoff power")
    con.set_throttle_percent(base_throttle_percent)

    # ---- Phase 1: ground roll to rotate (rudder centerline control) ----------
    last_log_t = time.monotonic()
    elev_cmd = 0.0
    thr_cmd  = base_throttle_percent
    rud_cmd  = 0.0

    # state for yaw-rate estimate
    _prev_hdg = [runway_hdg]
    _prev_t   = [time.monotonic()]

    print(f"[STEP 5] Accelerating to rotate speed {rotate_kias:.0f} KIAS… (centerline hold active)")
    while con.get_ias_kt() < rotate_kias and con.get_on_ground():
        now = time.monotonic()

        # --- RUDDER: track runway heading/ground track, damp yaw-rate & sideslip
        track = _get_ground_track_deg()  # deg
        # Error toward runway heading (use track if available; else heading fallback is inside getter)
        hdg_err = shortest_angle_deg(runway_hdg - track)  # + means we need to yaw right
        yaw_rate = _get_yaw_rate_deg_s(_prev_hdg, _prev_t)  # deg/s
        beta = _get_sideslip_deg()  # deg; positive = wind from right (typically needs right rudder)

        raw_rud = rudder_kh * hdg_err - rudder_kd * yaw_rate + rudder_kbeta * beta
        raw_rud = clamp(raw_rud, -rudder_limit_pct, +rudder_limit_pct)
        rud_cmd = slew(rud_cmd, raw_rud, rud_step)
        con.set_rudder_percent(rud_cmd)

        # --- LOG & pace
        if now - last_log_t >= 1.0:
            ias = con.get_ias_kt()
            alt = con.get_altitude_ft()
            agl = alt - ground_alt
            pitch = con.get_pitch_deg()
            print(
                f"[ROLL] IAS={ias:.1f} kt | AGL={agl:.0f} ft | PITCH={pitch:+.1f}° | "
                f"TRACK={track:.1f}° | HDG_ERR={hdg_err:+.1f}° | YAW_RATE={yaw_rate:+.1f}°/s | "
                f"BETA={beta:+.1f}° | RUD={rud_cmd:+.1f}%"
            )
            last_log_t = now

        time.sleep(dt)

    # ---- Phase 2: rotation (smooth ramp to target_pitch_deg) ------------------
    print(f"[ROTATE] {rotate_kias:.0f} KIAS reached → smooth rotate to ~{target_pitch_deg:.0f}° pitch")

    prev_pitch = con.get_pitch_deg()
    prev_t     = time.monotonic()

    rotate_duration_s = 5.0
    rotate_end = prev_t + rotate_duration_s
    while time.monotonic() < rotate_end and con.get_on_ground():
        now = time.monotonic()
        dtm = max(1e-3, now - prev_t)
        pitch = con.get_pitch_deg()
        pitch_rate = (pitch - prev_pitch) / dtm  # deg/s

        ias = con.get_ias_kt()
        target_pitch = target_pitch_deg + clamp((climb_kias - ias) * 0.02, -2.0, 2.0)
        target_pitch = clamp(target_pitch, 6.0, max_pitch_up_deg - 1.0)

        pitch_err = target_pitch - pitch
        raw_elev = pitch_kp * pitch_err + pitch_kd * pitch_rate

        # Attitude envelope
        if pitch > max_pitch_up_deg:
            raw_elev += +2.0 * (pitch - max_pitch_up_deg)
        if pitch < max_pitch_dn_deg:
            raw_elev += -2.0 * (pitch - max_pitch_dn_deg)

        raw_elev = clamp(raw_elev, -25.0, +15.0)
        elev_cmd = slew(elev_cmd, raw_elev, elev_step)
        con.set_elevator_percent(elev_cmd)

        # Keep using rudder a bit through rotation (crosswind/torque)
        track = _get_ground_track_deg()
        hdg_err = shortest_angle_deg(runway_hdg - track)
        yaw_rate = _get_yaw_rate_deg_s(_prev_hdg, _prev_t)
        beta = _get_sideslip_deg()
        raw_rud = rudder_kh * hdg_err - rudder_kd * yaw_rate + rudder_kbeta * beta
        raw_rud = clamp(raw_rud, -rudder_limit_pct, +rudder_limit_pct)
        rud_cmd = slew(rud_cmd, raw_rud, rud_step)
        con.set_rudder_percent(rud_cmd)

        prev_pitch, prev_t = pitch, now
        time.sleep(dt)

    # ---- Phase 3: initial climb to 1000 AGL with angle & coordination guard ---
    print("[CLIMB] Climbing out; gear up at 500 AGL; transitioning to IAS hold at 1,000 AGL.")

    def _agl(): return con.get_altitude_ft() - ground_alt

    # Gear at 500 AGL
    wait_until(lambda: _agl() >= 500.0, poll_s=0.1)
    if con.get_landing_gear_down():
        print("[GEAR] 500 ft AGL reached → Gear UP")
        con.set_landing_gear_down(False)
    else:
        print("[GEAR] Already UP at 500 ft AGL")

    # Maintain controlled pitch + keep ball centered (rudder) until 1000 AGL
    while _agl() < 1000.0:
        now = time.monotonic()
        dtm = max(1e-3, now - prev_t)
        pitch = con.get_pitch_deg()
        pitch_rate = (pitch - prev_pitch) / dtm
        ias = con.get_ias_kt()

        target_pitch = target_pitch_deg + clamp((climb_kias - ias) * ias_to_pitch_gain, -4.0, 4.0)
        target_pitch = clamp(target_pitch, 6.0, max_pitch_up_deg - 1.0)

        pitch_err = target_pitch - pitch
        raw_elev = pitch_kp * pitch_err + pitch_kd * pitch_rate

        if pitch > max_pitch_up_deg:
            raw_elev += +2.0 * (pitch - max_pitch_up_deg)
        if pitch < max_pitch_dn_deg:
            raw_elev += -2.0 * (pitch - max_pitch_dn_deg)

        raw_elev = clamp(raw_elev, -22.0, +12.0)
        elev_cmd = slew(elev_cmd, raw_elev, elev_step)
        con.set_elevator_percent(elev_cmd)

        # Rudder: now prioritize coordination (beta → 0) more than heading
        beta = _get_sideslip_deg()
        yaw_rate = _get_yaw_rate_deg_s(_prev_hdg, _prev_t)
        # Small heading term to keep initial runway alignment during climb-out
        track = _get_ground_track_deg()
        hdg_err = shortest_angle_deg(runway_hdg - track)
        raw_rud = 0.4 * rudder_kh * hdg_err - rudder_kd * yaw_rate + 1.2 * rudder_kbeta * beta
        raw_rud = clamp(raw_rud, -rudder_limit_pct, +rudder_limit_pct)
        rud_cmd = slew(rud_cmd, raw_rud, rud_step)
        con.set_rudder_percent(rud_cmd)

        # Throttle assist toward climb speed
        thr_target = base_throttle_percent + (climb_kias - ias) * ias_to_throttle_gain
        thr_target = clamp(thr_target, 70.0, 100.0)
        thr_cmd = slew(thr_cmd, thr_target, thr_step)
        con.set_throttle_percent(thr_cmd)

        # Overshoot management near pitch limit
        if pitch > (max_pitch_up_deg - 1.0):
            thr_cmd = max(70.0, thr_cmd - 2.0)
            con.set_throttle_percent(thr_cmd)

        prev_pitch, prev_t = pitch, now
        time.sleep(dt)

    print("[CLIMB] 1,000 ft AGL reached → IAS-hold climb with angle protection (rudder coordination continues).")

    # ---- Phase 4: IAS-hold climb to target MSL with angle & throttle control --
    last_ctrl_log_t = time.monotonic()
    while con.get_altitude_ft() < target_altitude_ft:
        now = time.monotonic()
        dtm = max(1e-3, now - prev_t)
        pitch = con.get_pitch_deg()
        pitch_rate = (pitch - prev_pitch) / dtm
        ias = con.get_ias_kt()
        alt = con.get_altitude_ft()
        vsi = con.get_vertical_speed_fpm()
        agl = alt - ground_alt

        target_pitch = target_pitch_deg + clamp((climb_kias - ias) * ias_to_pitch_gain, -6.0, 6.0)
        target_pitch = clamp(target_pitch, 5.0, max_pitch_up_deg - 1.0)

        pitch_err = target_pitch - pitch
        raw_elev = pitch_kp * pitch_err + pitch_kd * pitch_rate

        if pitch > max_pitch_up_deg:
            raw_elev += +2.5 * (pitch - max_pitch_up_deg)
        if pitch < max_pitch_dn_deg:
            raw_elev += -2.5 * (pitch - max_pitch_dn_deg)

        raw_elev = clamp(raw_elev, -22.0, +12.0)
        elev_cmd = slew(elev_cmd, raw_elev, elev_step)
        con.set_elevator_percent(elev_cmd)

        # Rudder: keep ball centered (primary), damp yaw-rate (secondary)
        beta = _get_sideslip_deg()
        yaw_rate = _get_yaw_rate_deg_s(_prev_hdg, _prev_t)
        raw_rud = -rudder_kd * yaw_rate + 1.2 * rudder_kbeta * beta
        raw_rud = clamp(raw_rud, -rudder_limit_pct, +rudder_limit_pct)
        rud_cmd = slew(rud_cmd, raw_rud, rud_step)
        con.set_rudder_percent(rud_cmd)

        # Throttle control to chase climb speed without demanding extreme pitch
        thr_target = base_throttle_percent + (climb_kias - ias) * ias_to_throttle_gain
        thr_target = clamp(thr_target, 65.0, 100.0)
        thr_cmd = slew(thr_cmd, thr_target, thr_step)
        con.set_throttle_percent(thr_cmd)

        if now - last_ctrl_log_t >= 1.0:
            try:
                print(
                    f"[IAS-HOLD] tgtIAS={climb_kias:.0f} | IAS={ias:.1f} kt | ALT={alt:.0f} ft "
                    f"(AGL {agl:.0f}) | VS={vsi:.0f} fpm | PITCH={pitch:.1f}° "
                    f"| ELEV={elev_cmd:.1f}% | RUD={rud_cmd:.1f}% | THR={thr_cmd:.1f}%"
                )
                last_ctrl_log_t = now
            except:
                continue

        prev_pitch, prev_t = pitch, now
        time.sleep(dt)

    # ---- Phase 5: level-off ---------------------------------------------------
    print(f"[LEVEL] Target altitude {target_altitude_ft:.0f} ft reached → easing elevator to 0%")
    while abs(elev_cmd) > 0.5:
        elev_cmd = slew(elev_cmd, 0.0, elev_step)
        con.set_elevator_percent(elev_cmd)
        time.sleep(dt)

    con.set_throttle_percent(clamp(thr_cmd, 60.0, 90.0))
    # Relax rudder toward neutral as we hand off to next mode
    while abs(rud_cmd) > 0.5:
        rud_cmd = slew(rud_cmd, 0.0, rud_step)
        con.set_rudder_percent(rud_cmd)
        time.sleep(dt)

    print("[DONE] Takeoff + climb complete with centerline and pitch stabilization.")



def stabilize_level(
    con: MSFSConnection,
    target_altitude_ft: float,
    *,
    hold_heading: bool = True,
    duration_s: float = 60.0,          # run this long; increase to keep holding
    cruise_throttle_percent: float | None = None,
    update_hz: float = 5.0
):
    """
    Hold wings level and maintain target MSL altitude using elevator + aileron.
    Optionally holds initial magnetic heading. Uses ONLY MSFSConnection getters/setters.

    Control conventions (per your setup):
      - Elevator: negative = nose up, positive = nose down
      - Aileron: positive ≈ roll right, negative ≈ roll left
    """

    # --- Helpers -----------------------------------------------------------
    def now() -> float: return time.monotonic()

    def clamp(v, lo, hi):
        return lo if v < lo else hi if v > hi else v

    def shortest_angle_deg(err):
        # Wrap to [-180, +180]
        e = fmod(err + 180.0, 360.0)
        if e < 0:
            e += 360.0
        return e - 180.0

    dt = 1.0 / max(1.0, update_hz)

    # --- Set up references & logging --------------------------------------
    start_time = now()
    start_alt = con.get_altitude_ft()
    tgt_hdg = con.get_heading_magnetic_deg() if hold_heading else None
    print(f"[STAB] Starting stabilize_level for {duration_s:.0f}s "
          f"→ target ALT={target_altitude_ft:.0f} ft"
          + (f", heading={tgt_hdg:.1f}°M" if tgt_hdg is not None else ", heading hold: OFF"))

    if cruise_throttle_percent is not None:
        con.set_throttle_percent(float(cruise_throttle_percent))
        print(f"[STAB] Throttle set to {cruise_throttle_percent:.1f}%")

    # Use current elevator position as a bias so we don't fight existing trim
    elev_bias = con.get_elevator_percent()
    print(f"[STAB] Elev bias (from current): {elev_bias:.1f}% | Start ALT={start_alt:.0f} ft")

    # --- Tunable gains (gentle) -------------------------------------------
    # Altitude hold with simple PD: elevator_cmd = bias - Ka*alt_err + Kv*VSI
    #   * alt_err = (target - current). Positive err ⇒ we need to climb ⇒ more nose-up (negative elevator)
    #     hence the minus sign before Ka.
    Ka = 0.02     # % elevator per ft of altitude error   (e.g., 500 ft → 10%)
    Kv = 0.003    # % elevator per fpm of vertical speed  (e.g., +500 fpm → +1.5% nose-down)
    elev_min, elev_max = -25.0, +25.0

    # Heading/Bank hold: compute a desired bank from heading error, then drive aileron to meet it.
    Kh = 0.5      # deg bank per deg heading error (soft; 10° err → 5° bank target)
    Kb = 2.0      # % aileron per deg bank error   (5° bank err → 10% aileron)
    bank_target_limit = 10.0   # deg
    ail_min, ail_max = -25.0, +25.0

    last_log = start_time

    # --- Control loop ------------------------------------------------------
    while now() - start_time < duration_s:
        alt = con.get_altitude_ft()
        vsi = con.get_vertical_speed_fpm()     # fpm
        ias = con.get_ias_kt()
        pitch = con.get_pitch_deg()
        bank = con.get_bank_deg()
        hdg = con.get_heading_magnetic_deg()

        # --- Elevator (altitude hold) ---
        alt_err = (target_altitude_ft - alt)   # + if below target
        elev_cmd = elev_bias - Ka * alt_err + Kv * vsi
        elev_cmd = clamp(elev_cmd, elev_min, elev_max)
        con.set_elevator_percent(elev_cmd)

        # --- Aileron (wings/heading hold) ---
        if tgt_hdg is not None:
            hdg_err = shortest_angle_deg(tgt_hdg - hdg)   # + means we need to turn right
            bank_tgt = clamp(Kh * hdg_err, -bank_target_limit, bank_target_limit)
        else:
            bank_tgt = 0.0  # wings level only

        bank_err = bank_tgt - bank
        ail_cmd = clamp(Kb * bank_err, ail_min, ail_max)
        con.set_aileron_percent(ail_cmd)

        # --- Logging (1 Hz) ------------
        t = now()
        if t - last_log >= 1.0:
            print(f"[HOLD] ALT={alt:.0f} ft (err {alt_err:+.0f}) | VSI={vsi:+.0f} fpm | IAS={ias:.0f} kt "
                  f"| PITCH={pitch:+.1f}° | BANK={bank:+.1f}° (tgt {bank_tgt:+.1f}°) "
                  f"| HDG={hdg:.1f}°M{f' (err {hdg_err:+.1f})' if tgt_hdg is not None else ''} "
                  f"| ELEV={elev_cmd:+.1f}% | AIL={ail_cmd:+.1f}%")
            last_log = t

        time.sleep(dt)

    # Neutralize controls gently when we’re done (optional)
    con.set_elevator_percent(0.0)
    con.set_aileron_percent(0.0)
    print("[STAB] Stabilize complete → controls neutralized.")


def stabilize_level2(
    con: MSFSConnection,
    target_altitude_ft: float,
    *,
    hold_heading: bool = True,
    duration_s: float = 60.0,
    cruise_throttle_percent: float | None = None,
    update_hz: float = 5.0
):
    """
    Smoothly converge to target altitude and hold wings level (and heading if enabled),
    using elevator/aileron + dynamic throttle. Once stabilized, throttle is set to the
    requested cruise_throttle_percent (if provided).
    Control conventions:
      - Elevator: negative = nose up, positive = nose down
      - Aileron : positive = roll right, negative = roll left
    """

    import time
    from math import fmod

    # ---------- helpers ----------
    def now() -> float: return time.monotonic()

    def clamp(v, lo, hi):
        return lo if v < lo else hi if v > hi else v

    def shortest_angle_deg(err):
        # Wrap to [-180, +180]
        e = fmod(err + 180.0, 360.0)
        if e < 0:
            e += 360.0
        return e - 180.0

    def slew(current, target, step):
        delta = target - current
        if abs(delta) <= step:
            return target
        return current + (step if delta > 0 else -step)

    dt = 1.0 / max(1.0, update_hz)

    # ---------- references & setup ----------
    start_t = now()
    start_alt = con.get_altitude_ft()
    ref_ias = con.get_ias_kt()          # try to keep speed near current unless told otherwise
    tgt_hdg = con.get_heading_magnetic_deg() if hold_heading else None

    print(
        f"[STAB] Target ALT={target_altitude_ft:.0f} ft | "
        f"Hold heading: {'ON' if tgt_hdg is not None else 'OFF'}"
        f"{f' (tgt {tgt_hdg:.1f}°M)' if tgt_hdg is not None else ''} | "
        f"Duration={duration_s:.0f}s | Ref IAS≈{ref_ias:.0f} kt"
    )

    # Initial throttle: if caller gave cruise, start near it; else keep current (assumed managed elsewhere)
    thr_cmd = float(clamp(con.get_throttle_percent() if cruise_throttle_percent is None
                          else cruise_throttle_percent, 50.0, 100.0))
    con.set_throttle_percent(thr_cmd)

    # Keep current elevator/aileron as bias to avoid fighting trim/autopilot residuals
    elev_cmd = float(con.get_elevator_percent())
    ail_cmd = float(con.get_aileron_percent())

    # ---------- controller gains ----------
    # Altitude hold (PD+I with anti-windup). Use VSI as derivative proxy (ft/min).
    # Sign: positive alt_err (below target) → need nose-up (negative elevator).
    Ka = 0.018      # % elevator per ft error (scaled small; 500 ft → ~9%)
    Kv = 0.0035     # % per fpm (damps with VS: +500 fpm climb → ~+1.75% nose-down)
    Ki = 0.0006     # % per (ft·s) integral (very gentle)

    elev_min, elev_max = -25.0, +20.0
    elev_slew_per_s = 35.0               # %/s

    # Heading/Bank hold (heading → bank target, bank → aileron)
    Kh = 0.5        # deg bank per deg heading error (10° err → 5° bank target)
    Kb_p = 1.8      # % aileron per deg bank error (P)
    Kb_d = 0.35     # % per (deg/s) roll-rate (D via bank change)
    bank_target_limit = 12.0             # deg
    ail_min, ail_max = -25.0, +25.0
    ail_slew_per_s = 40.0                # %/s

    # Pitch guardrails (attitude protection during transients)
    max_pitch_up_deg = 18.0
    max_pitch_dn_deg = -12.0

    # Throttle dynamics: keep IAS near reference and help vertical damping
    # Positive (ref_ias - ias) → increase throttle. Negative VSI (descending) → add a little power.
    Kt_ias = 0.8     # % throttle per knot speed error
    Kt_vsi = 0.0002  # % throttle per fpm (small; 500 fpm sink → +0.1%)
    thr_min, thr_max = 55.0, 100.0
    thr_slew_per_s = 35.0

    # Stabilization criteria to hand throttle back to requested cruise (%)
    alt_window_ft = 75.0
    vsi_window_fpm = 150.0
    stable_hold_s = 4.0
    stable_since = None
    cruise_set_done = False

    # Integral state & previous samples
    alt_int = 0.0
    prev_bank = con.get_bank_deg()
    last_log = start_t

    # ---------- loop ----------
    while now() - start_t < duration_s:
        t1 = now()

        alt = con.get_altitude_ft()
        vsi = con.get_vertical_speed_fpm()       # fpm
        ias = con.get_ias_kt()
        pitch = con.get_pitch_deg()
        bank = con.get_bank_deg()
        hdg = con.get_heading_magnetic_deg()

        # ---- elevator: altitude PD+I (derivative via VSI) ----
        alt_err = (target_altitude_ft - alt)     # + if we are below target
        # Anti-windup: only integrate when command not saturated and vertical rate small
        if elev_min < elev_cmd < elev_max and abs(vsi) < 1200:
            alt_int += alt_err * dt              # ft·s
            alt_int = clamp(alt_int, -20000.0, 20000.0)

        raw_elev = (elev_cmd  # use current as bias for smoothness
                    - Ka * alt_err
                    + Kv * vsi
                    - Ki * alt_int)

        # Pitch attitude protection
        if pitch > max_pitch_up_deg:
            raw_elev += +2.0 * (pitch - max_pitch_up_deg)   # push
        if pitch < max_pitch_dn_deg:
            raw_elev += -2.0 * (pitch - max_pitch_dn_deg)   # pull (note sign)

        # Clamp and slew-limit elevator
        raw_elev = clamp(raw_elev, elev_min, elev_max)
        elev_cmd = slew(elev_cmd, raw_elev, elev_slew_per_s * dt)
        con.set_elevator_percent(elev_cmd)

        # ---- aileron: wings/heading hold ----
        if tgt_hdg is not None:
            hdg_err = shortest_angle_deg(tgt_hdg - hdg)     # + need to turn right
            bank_tgt = clamp(Kh * hdg_err, -bank_target_limit, bank_target_limit)
        else:
            hdg_err = 0.0
            bank_tgt = 0.0

        bank_rate = (bank - prev_bank) / dt                  # deg/s (approx)
        bank_err = bank_tgt - bank
        raw_ail = Kb_p * bank_err - Kb_d * bank_rate
        raw_ail = clamp(raw_ail, ail_min, ail_max)
        ail_cmd = slew(ail_cmd, raw_ail, ail_slew_per_s * dt)
        con.set_aileron_percent(ail_cmd)
        prev_bank = bank

        # ---- throttle: dynamic until stabilized ----
        if not cruise_set_done:
            thr_target = thr_cmd + Kt_ias * (ref_ias - ias) + Kt_vsi * (-vsi)
            thr_target = clamp(thr_target, thr_min, thr_max)
            thr_cmd = slew(thr_cmd, thr_target, thr_slew_per_s * dt)
            con.set_throttle_percent(thr_cmd)

        # ---- stabilization detection & cruise handoff ----
        if abs(alt_err) <= alt_window_ft and abs(vsi) <= vsi_window_fpm:
            if stable_since is None:
                stable_since = t1
            elif not cruise_set_done and cruise_throttle_percent is not None and (t1 - stable_since) >= stable_hold_s:
                thr_cmd = float(clamp(cruise_throttle_percent, thr_min, thr_max))
                con.set_throttle_percent(thr_cmd)
                cruise_set_done = True
                print(f"[STAB] Altitude stabilized → throttle set to cruise {thr_cmd:.1f}%")
        else:
            stable_since = None  # reset dwell if we drift out

        # ---- logging ~1 Hz ----
        if t1 - last_log >= 1.0:
            try:
                print(
                    f"[HOLD] ALT={alt:.0f} ft (err {alt_err:+.0f}) | VSI={vsi:+.0f} fpm | IAS={ias:.0f} kt "
                    f"| PITCH={pitch:+.1f}° | BANK={bank:+.1f}° (tgt {bank_tgt:+.1f}°) "
                    f"| HDG={hdg:.1f}°M{(f' (err {hdg_err:+.1f})' if tgt_hdg is not None else '')} "
                    f"| ELEV={elev_cmd:+.1f}% | AIL={ail_cmd:+.1f}% | THR={thr_cmd:.1f}%"
                )
                last_log = t1
            except:
                continue

        # pace the loop
        sleep_left = dt - (now() - t1)
        if sleep_left > 0:
            time.sleep(sleep_left)

    # Optional: neutralize aileron bias a bit if heading hold was off
    if not hold_heading:
        con.set_aileron_percent(0.0)

    print("[STAB] Stabilize_level complete.")


# MAIN PROCS

def tstab(con: MSFSConnection, alt: int):
    takeoff5(con, alt)
    stabilize_level2(con, alt, cruise_throttle_percent=60)

if __name__ == "__main__":
    import pdb

    # Create connection instance
    con = MSFSConnection()

    try:
        con.connect()
        LOGGER.info("MSFSConnection CONDOR ready. Use `con` in pdb to interact with the sim.")
        LOGGER.info("By Usman Ahmad.")
        pdb.set_trace()
    except KeyboardInterrupt:
        LOGGER.info("Interrupted by user.")
    except Exception as e:
        LOGGER.error("Fatal error: %s", e, exc_info=True)
    finally:
        con.close()
        LOGGER.info("Session closed.")
