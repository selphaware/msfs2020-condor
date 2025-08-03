#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
msfs_control.py

PEP8-compliant helpers to control Microsoft Flight Simulator 2020
via SimConnect (MSFS SDK). Provides generic send/get functions and
convenience wrappers for common cockpit interactions and telemetry.

Requires:
    pip install SimConnect

Tested with the 'SimConnect' Python wrapper (python-simconnect).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional
import logging

try:
    # The widely used Python wrapper for MSFS SimConnect
    from SimConnect import SimConnect, AircraftEvents, AircraftRequests
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency: install with `pip install SimConnect`.\n"
        "Package sometimes published as `python-simconnect`."
    ) from exc


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
    return int(round(p * AXIS_MAX))


# ---------------------------------------------------------------------------
# Connection
# ---------------------------------------------------------------------------

@dataclass
class MSFSConnection:
    """
    Thin wrapper around SimConnect, providing generic send/get and
    convenience actions/queries.
    """

    request_timeout_ms: int = 1000

    def __post_init__(self) -> None:
        self._sm: Optional[SimConnect] = None
        self._events: Optional[AircraftEvents] = None
        self._requests: Optional[AircraftRequests] = None

    # -- lifecycle -----------------------------------------------------------

    def connect(self) -> None:
        """Open a SimConnect session and prepare helpers."""
        if self._sm is not None:
            return
        LOGGER.info("Connecting to MSFS via SimConnect...")
        self._sm = SimConnect()
        self._events = AircraftEvents(self._sm)
        self._requests = AircraftRequests(self._sm, _time=self.request_timeout_ms)
        LOGGER.info("Connected.")

    def close(self) -> None:
        """Close the SimConnect session."""
        if self._sm is not None:
            LOGGER.info("Closing SimConnect session.")
            self._sm.exit()
        self._sm = None
        self._events = None
        self._requests = None

    # -- core API ------------------------------------------------------------

    def send_event(self, event_id: str, value: Optional[int] = None) -> None:
        """
        Send a Client Event by its MSFS SDK Event ID.

        Args:
            event_id: Exact Client Event name (e.g., 'GEAR_TOGGLE',
                      'AXIS_THROTTLE_SET', 'PARKING_BRAKES_SET').
            value: Optional integer parameter (range depends on the event).
                   If omitted, the event will be "fired" with no parameter.
        """
        self._ensure_connected()
        assert self._events is not None

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
        assert self._requests is not None

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

    # -- convenience setters (write) ----------------------------------------

    # Master battery
    def set_master_battery(self, on: bool) -> None:
        """
        Set master battery on/off.
        Uses 'MASTER_BATTERY_SET' if available; otherwise toggles to reach state.
        """
        try:
            self.send_event("MASTER_BATTERY_SET", 1 if on else 0)
        except ValueError:
            # Fallback to toggle
            is_on = bool(self.get_simvar("ELECTRICAL MASTER BATTERY"))
            if is_on != on:
                self.send_event("TOGGLE_MASTER_BATTERY")

    # Avionics master
    def set_avionics_master(self, on: bool) -> None:
        """Set avionics master on/off."""
        try:
            self.send_event("AVIONICS_MASTER_SET", 1 if on else 0)
        except ValueError:
            is_on = bool(self.get_simvar("AVIONICS MASTER SWITCH"))
            if is_on != on:
                self.send_event("AVIONICS_MASTER_SET", 1 if on else 0)

    # Engines
    def set_engines_running(self, on: bool) -> None:
        """Start or stop engines using auto-start/auto-shutdown."""
        if on:
            self.send_event("ENGINE_AUTOSTART")
        else:
            self.send_event("ENGINE_AUTO_SHUTDOWN")

    # Throttle (0..100 %)
    def set_throttle_percent(self, percent: float) -> None:
        """Set throttle (all engines) as percent [0..100]."""
        value = percent_to_unipolar_axis(percent)
        # Commonly supported:
        for eid in ("AXIS_THROTTLE_SET", "THROTTLE_SET"):
            try:
                self.send_event(eid, value)
                return
            except ValueError:
                continue
        raise ValueError("No suitable throttle event found (AXIS_THROTTLE_SET/THROTTLE_SET).")

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
        value = percent_to_bipolar_axis(percent)
        # Prefer axis event; some aircraft accept ELEVATOR_TRIM_SET (0..16383) too.
        try:
            self.send_event("AXIS_ELEV_TRIM_SET", value)
        except ValueError:
            # Map to unipolar if using ELEVATOR_TRIM_SET
            self.send_event("ELEVATOR_TRIM_SET", percent_to_unipolar_axis((percent + 100.0) / 2.0))

    # Flaps (0..100 %)
    def set_flaps_percent(self, percent: float) -> None:
        """Set flaps as percent [0..100]."""
        value = percent_to_unipolar_axis(percent)
        for eid in ("AXIS_FLAPS_SET", "FLAPS_SET"):
            try:
                self.send_event(eid, value)
                return
            except ValueError:
                continue
        raise ValueError("No suitable flaps event found (AXIS_FLAPS_SET/FLAPS_SET).")

    # Parking brake
    def set_parking_brake(self, on: bool) -> None:
        """Set parking brake on/off."""
        try:
            self.send_event("PARKING_BRAKES_SET", 1 if on else 0)
        except ValueError:
            # Fallback to toggle
            is_on = bool(self.get_simvar("BRAKE PARKING POSITION"))
            if is_on != on:
                self.send_event("PARKING_BRAKES")

    # Landing gear
    def set_landing_gear(self, down: bool) -> None:
        """Set landing gear down/up."""
        try:
            self.send_event("GEAR_SET", 1 if down else 0)
        except ValueError:
            # Fallback to toggle
            handle_down = bool(self.get_simvar("GEAR HANDLE POSITION"))
            if handle_down != down:
                self.send_event("GEAR_TOGGLE")

    # -- convenience getters (read) -----------------------------------------

    # System/control states
    def get_master_battery_on(self) -> bool:
        return bool(self.get_simvar("ELECTRICAL MASTER BATTERY"))

    def get_avionics_master_on(self) -> bool:
        return bool(self.get_simvar("AVIONICS MASTER SWITCH"))

    def get_parking_brake_on(self) -> bool:
        return bool(self.get_simvar("BRAKE PARKING POSITION"))

    def get_landing_gear_down(self) -> bool:
        return bool(self.get_simvar("GEAR HANDLE POSITION"))

    # Control positions (as percents)
    def get_throttle_percent(self) -> float:
        # Average engine 1 position; adjust if you need per-engine.
        val = float(self.get_simvar("GENERAL ENG THROTTLE LEVER POSITION:1", "Percent"))
        return clamp(val, 0.0, 100.0)

    def get_aileron_percent(self) -> float:
        # AILERON POSITION returns -1..+1
        pos = float(self.get_simvar("AILERON POSITION"))
        return clamp(pos * 100.0, -100.0, 100.0)

    def get_rudder_percent(self) -> float:
        pos = float(self.get_simvar("RUDDER POSITION"))
        return clamp(pos * 100.0, -100.0, 100.0)

    def get_elevator_percent(self) -> float:
        pos = float(self.get_simvar("ELEVATOR POSITION"))
        return clamp(pos * 100.0, -100.0, 100.0)

    def get_trim_percent(self) -> float:
        # ELEVATOR TRIM POSITION returns radians or normalized per aircraft; try degrees
        # If degrees not supported, wrapper will still convert appropriately.
        try:
            # Some aircraft expose trim as -1..+1 via ELEVATOR TRIM POSITION
            pos = float(self.get_simvar("ELEVATOR TRIM POSITION"))
            if -1.1 <= pos <= 1.1:
                return clamp(pos * 100.0, -100.0, 100.0)
        except Exception:
            pass
        # Fallback to percent handle if available
        try:
            val = float(self.get_simvar("ELEVATOR TRIM PCT"))
            return clamp(val * 100.0, -100.0, 100.0)
        except Exception:
            raise

    def get_flaps_percent(self) -> float:
        # FLAPS HANDLE PERCENT returns 0..1
        pos = float(self.get_simvar("FLAPS HANDLE PERCENT"))
        return clamp(pos * 100.0, 0.0, 100.0)

    # Flight metrics
    def get_altitude_ft(self) -> float:
        return float(self.get_simvar("PLANE ALTITUDE", "Feet"))

    def get_aoa_deg(self) -> float:
        return float(self.get_simvar("INCIDENCE ALPHA", "Degrees"))

    def get_ias_kt(self) -> float:
        return float(self.get_simvar("AIRSPEED INDICATED", "Knots"))

    def get_tas_kt(self) -> float:
        return float(self.get_simvar("AIRSPEED TRUE", "Knots"))

    def get_ground_speed_kt(self) -> float:
        # GROUND VELOCITY in knots for convenience
        return float(self.get_simvar("GROUND VELOCITY", "Knots"))

    def get_vertical_speed_fpm(self) -> float:
        return float(self.get_simvar("VERTICAL SPEED", "Feet per minute"))

    def get_pitch_deg(self) -> float:
        return float(self.get_simvar("PLANE PITCH DEGREES", "Degrees"))

    def get_bank_deg(self) -> float:
        return float(self.get_simvar("PLANE BANK DEGREES", "Degrees"))

    def get_heading_true_deg(self) -> float:
        return float(self.get_simvar("PLANE HEADING DEGREES TRUE", "Degrees"))

    def get_heading_magnetic_deg(self) -> float:
        return float(self.get_simvar("PLANE HEADING DEGREES MAGNETIC", "Degrees"))

    # -- internal ------------------------------------------------------------

    def _ensure_connected(self) -> None:
        if self._sm is None:
            self.connect()


# ---------------------------------------------------------------------------
# Example usage
# ---------------------------------------------------------------------------

def main() -> None:
    """
    Minimal demonstration. Uncomment as needed.

    IMPORTANT: Ensure MSFS is running and ready (on the runway/cold-and-dark/etc.)
    before running this script, otherwise events/vars may be ignored.
    """
    conn = MSFSConnection()

    try:
        conn.connect()

        # --- PDB CONSOLE ---
        import pdb
        print("\n--- Condor v0.2b, by Usman Ahmad ---\n")
        pdb.set_trace()

        # --- Generic API examples
        # Send an arbitrary event by its ID:
        # conn.send_event("GEAR_TOGGLE")
        # Read an arbitrary SimVar (explicit units recommended):
        # altitude_ft = conn.get_simvar("PLANE ALTITUDE", "Feet")

        # --- Convenience setters
        # conn.set_master_battery(True)
        # conn.set_avionics_master(True)
        # conn.set_engines_running(True)
        # conn.set_throttle_percent(45.0)
        # conn.set_aileron_percent(-20.0)   # left
        # conn.set_rudder_percent(10.0)     # right
        # conn.set_elevator_percent(-5.0)   # push
        # conn.set_trim_percent(3.0)        # trim nose up a touch
        # conn.set_flaps_percent(50.0)
        # conn.set_parking_brake(False)
        # conn.set_landing_gear(True)

        # --- Convenience getters
        # print("Battery on:", conn.get_master_battery_on())
        # print("Avionics on:", conn.get_avionics_master_on())
        # print("Throttle %:", conn.get_throttle_percent())
        # print("Aileron %:", conn.get_aileron_percent())
        # print("Rudder %:", conn.get_rudder_percent())
        # print("Elevator %:", conn.get_elevator_percent())
        # print("Trim %:", conn.get_trim_percent())
        # print("Flaps %:", conn.get_flaps_percent())
        # print("Parking brake on:", conn.get_parking_brake_on())
        # print("Gear down:", conn.get_landing_gear_down())

        # print("Altitude ft:", conn.get_altitude_ft())
        # print("AOA deg:", conn.get_aoa_deg())
        # print("IAS kt:", conn.get_ias_kt())
        # print("TAS kt:", conn.get_tas_kt())
        # print("GS kt:", conn.get_ground_speed_kt())
        # print("VS fpm:", conn.get_vertical_speed_fpm())
        # print("Pitch deg:", conn.get_pitch_deg())
        # print("Bank deg:", conn.get_bank_deg())
        # print("HDG true deg:", conn.get_heading_true_deg())
        # print("HDG mag deg:", conn.get_heading_magnetic_deg())

    finally:
        conn.close()


if __name__ == "__main__":
    main()
