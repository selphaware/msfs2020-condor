# stats.py
from simcon1 import MSFSController

__author__ = "Usman Ahmad"

import time
import pprint


def main():
    ctl = MSFSController()
    try:
        # Allow initial SimVar subscriptions to populate
        time.sleep(0.5)

        print("Brakes:", ctl.get_parking_brake_position())
        print("Altitudes:", ctl.get_altitudes())
        print("Speeds:", ctl.get_speeds())
        print("Engines running:", ctl.get_engine_combustion())
        print("Throttle %:", ctl.get_throttle_percent())
        print("Gear handle:", ctl.get_gear_handle_position())
        print("Flaps %:", ctl.get_flaps_percent())
        print("Trim %:", ctl.get_trim_percent())
        print("Surfaces %:", ctl.get_control_surface_percents())
        print("On ground:", ctl.is_on_ground())
        print("Electrical:", ctl.get_electrical())

    finally:
        ctl.close()


if __name__ == "__main__":
    main()
