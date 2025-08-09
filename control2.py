from time import sleep
from SimConnect import SimConnect, AircraftRequests, AircraftEvents


def percent_to_axis_int(percent: float) -> int:
    """Convert percentage (0–100) to axis range 0–16383."""
    return max(0, min(16383, int(percent / 100.0 * 16383)))


def signed_percent_to_axis_int(percent: float) -> int:
    """Convert signed percentage (-100–100) to axis range -16383–16383."""
    return max(-16383, min(16383, int(percent / 100.0 * 16383)))


class SimManager:
    def __init__(self):
        self.sm = SimConnect()
        self.ar = AircraftRequests(self.sm)
        self.ae = AircraftEvents(self.sm)

    def send_event(self, event: str, value=None):
        ev = self.ae.find(event)
        ev() if value is None else ev(value)

    def get_var(self, name: str, units=None):
        return self.ar.get((name, units)) if units else self.ar.get(name)


def set_throttle(sim: SimManager, percent: float):
    """Set throttle percentage."""
    sim.send_event("THROTTLE_SET", percent_to_axis_int(percent))


def set_elevator(sim: SimManager, percent: float):
    """Set elevator control."""
    sim.send_event("ELEVATOR_SET", signed_percent_to_axis_int(percent))


def set_trim(sim: SimManager, percent: float):
    """Set elevator trim control."""
    sim.send_event("ELEVATOR_TRIM_SET", signed_percent_to_axis_int(percent))


def takeoff(sim: SimManager,
            rotate_speed_kts=75,
            lift_elevator_pct=25,
            gear_up_kts=100,
            flaps_up_kts=120):
    """Perform manual takeoff sequence."""
    print("Advancing throttle to 100%...")
    set_throttle(sim, 100)

    on_ground = True
    while on_ground:
        ias = sim.get_var("AIRSPEED INDICATED")
        on_ground = bool(sim.get_var("SIM ON GROUND"))
        if ias >= rotate_speed_kts and on_ground:
            print("Rotation speed reached, pulling up...")
            set_elevator(sim, lift_elevator_pct)
            sleep(1)
            set_elevator(sim, 0)
        sleep(0.05)

    print("Airborne — accelerating for gear/flaps retraction...")
    while sim.get_var("AIRSPEED INDICATED") < gear_up_kts:
        sleep(0.1)
    sim.send_event("GEAR_UP")

    while sim.get_var("AIRSPEED INDICATED") < flaps_up_kts:
        sleep(0.1)
    sim.send_event("FLAPS_UP")

    print("Takeoff complete.")


def climb_and_hold_altitude(sim: SimManager, target_alt_ft, tolerance_ft=20):
    """Manual climb & hold using trim and elevator in continuous loop."""
    print(f"Climbing to {target_alt_ft} ft...")
    while True:
        altitude = sim.get_var("PLANE ALTITUDE")  # feet MSL
        if altitude < target_alt_ft - tolerance_ft:
            set_trim(sim, 5)       # nose up trim
            set_elevator(sim, 10)  # gentle pull
        elif altitude > target_alt_ft + tolerance_ft:
            set_trim(sim, -5)      # nose down trim
            set_elevator(sim, -5)  # gentle push
        else:
            set_trim(sim, 0)
            set_elevator(sim, 0)

        sleep(0.1)


if __name__ == "__main__":
    sim = SimManager()
    takeoff(sim)
    climb_and_hold_altitude(sim, target_alt_ft=5000)
