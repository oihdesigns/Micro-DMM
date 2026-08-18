#!/usr/bin/env python3
"""example_headless.py — the same rig, driven with no GUI at all.

This is the point of the split: the GUI is one consumer of DeviceManager, not
the place the logic lives. Here a supervisory loop holds a blend ratio between
two controllers and logs it, which is roughly the shape of a real process
controller you would grow from this proof of concept.

    python example_headless.py                 # simulated rig
    python example_headless.py bench_rig.json  # real instruments
"""

from __future__ import annotations

import sys
import time

from flowlab import core
from flowlab.config import DEFAULT_RIG, build_manager, load_rig

TOTAL_FLOW = 200.0        # engineering units on the primary device
RATIO = 0.25              # fraction of the total carried by the second device
RUN_SECONDS = 20.0


def main() -> int:
    rig = load_rig(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_RIG
    manager = build_manager(rig)
    time.sleep(1.0)                       # let connect() and the first polls land

    names = manager.names()
    if len(names) < 2:
        print(f"need two controllers, rig has {names}")
        manager.shutdown()
        return 1
    primary, secondary = names[0], names[1]

    manager.start_log("blend_log.csv")
    manager.set_setpoint(primary, TOTAL_FLOW * (1 - RATIO))
    manager.set_setpoint(secondary, TOTAL_FLOW * RATIO)

    deadline = time.time() + RUN_SECONDS
    try:
        while time.time() < deadline:
            time.sleep(1.0)
            a = manager.latest(primary)
            b = manager.latest(secondary)
            if not (a and b and a.ok and b.ok):
                continue
            flow_a = a.get(core.MASS)
            flow_b = b.get(core.MASS)
            total = flow_a + flow_b
            actual_ratio = flow_b / total if total > 1e-6 else 0.0
            print(f"{time.strftime('%H:%M:%S')}  {primary} {flow_a:7.2f}  "
                  f"{secondary} {flow_b:7.2f}  total {total:7.2f}  "
                  f"ratio {actual_ratio:5.3f}")

            # Trim the secondary to hold the ratio against the measured total.
            error = RATIO - actual_ratio
            if abs(error) > 0.005:
                current = b.get(core.SETPOINT)
                manager.set_setpoint(secondary, max(0.0, current + error * total * 0.5))

            for name, snap in ((primary, a), (secondary, b)):
                if snap.flags:
                    print(f"    ! {name} flags: {' '.join(snap.flags)}")
    except KeyboardInterrupt:
        pass
    finally:
        for future in manager.all_off():
            try:
                future.result(timeout=2.0)
            except Exception as exc:      # noqa: BLE001
                print(f"shutdown: {exc}")
        manager.stop_log()
        manager.shutdown()
    print("done — blend_log.csv written")
    return 0


if __name__ == "__main__":
    sys.exit(main())
