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

    # Flight metrics
    def get_altitude_ft(self) -> float:
        return float(self.get_simvar("PLANE_ALTITUDE"))

    def get_aoa_deg(self) -> float:
        return float(self.get_simvar("INCIDENCE_ALPHA"))

    def get_ias_kt(self) -> float:
        return float(self.get_simvar("AIRSPEED_INDICATED"))

    def get_tas_kt(self) -> float:
        return float(self.get_simvar("AIRSPEED_TRUE"))

    def get_ground_speed_kt(self) -> float:
        # GROUND VELOCITY in knots for convenience
        return float(self.get_simvar("GROUND_VELOCITY"))

    def get_vertical_speed_fpm(self) -> float:
        return float(self.get_simvar("VERTICAL_SPEED"))

    def get_pitch_deg(self) -> float:
        return float(self.get_simvar("PLANE_PITCH_DEGREES"))

    def get_bank_deg(self) -> float:
        return float(self.get_simvar("PLANE_BANK_DEGREES"))

    def get_heading_true_deg(self) -> float:
        return float(self.get_simvar("PLANE_HEADING_DEGREES_TRUE"))

    def get_heading_magnetic_deg(self) -> float:
        return float(self.get_simvar("PLANE_HEADING_DEGREES_MAGNETIC"))

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


# MAIN PROCS

def tstab1(con: MSFSConnection, alt: int):
    takeoff(con, alt)
    stabilize_level(con, alt, hold_heading=True, duration_s=120, cruise_throttle_percent=60.0)

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
