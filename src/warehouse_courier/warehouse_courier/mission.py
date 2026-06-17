#!/usr/bin/env python3
"""
Warehouse Courier Mission Script
Route: PICKUP  →  DROPOFF  →  DOCK
Features:
  - Live distance + ETA telemetry on a single overwriting line
  - Graceful FAILED / CANCELED handling (logs and continues)
  - BasicNavigator lifecycle managed inside try/finally
"""

import sys
import os
import shutil
import time
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# ═══════════════════════════════════════════════════════════════
#  NAMED WAYPOINTS  —  (x, y, yaw_degrees)
#  Edit these to match your saved map coordinates
# ═══════════════════════════════════════════════════════════════
WAYPOINTS = {
    "PICKUP":  (-2.0,  3.5,   0.0),
    "DROPOFF": ( 3.5,  3.5,  90.0),
    "DOCK":    (-4.0, -4.0, 180.0),
}

MISSION_ORDER = ["PICKUP", "DROPOFF", "DOCK"]


# ═══════════════════════════════════════════════════════════════
#  HELPER — build a stamped PoseStamped in the map frame
# ═══════════════════════════════════════════════════════════════
def make_pose(navigator: BasicNavigator,
              x: float,
              y: float,
              yaw_deg: float) -> PoseStamped:
    """
    Convert (x, y, yaw_degrees) into a PoseStamped message
    stamped in the 'map' frame with the current sim time.
    """
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    # Convert yaw (degrees → radians → quaternion)
    # Only rotation around Z needed for a flat floor robot
    yaw_rad = math.radians(yaw_deg)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

    return pose


# ═══════════════════════════════════════════════════════════════
#  HELPER — print live telemetry on one overwriting line
# ═══════════════════════════════════════════════════════════════
def print_telemetry(name: str,
                    dist: float,
                    elapsed: float,
                    prev_dist: float,
                    prev_time: float) -> None:
    """
    Compute speed from the last two distance samples and
    print distance + ETA on a single line using \\r.
    Line is padded to fit the terminal width so leftover
    characters from a longer previous line are always
    overwritten, and so the line never wraps onto a second
    visual row (which would break the overwrite behavior).
    """
    dt = elapsed - prev_time
    if dt > 0.0 and prev_dist is not None:
        speed = (prev_dist - dist) / dt      # metres per second
    else:
        speed = 0.0

    if speed > 0.05:                         # only show ETA if moving
        eta_sec = dist / speed
        eta_str = f"{eta_sec:.0f}s"
    else:
        eta_str = "calculating..."

    line = (
        f"[{name}]  "
        f"dist: {dist:5.2f} m   "
        f"elapsed: {elapsed:4.0f} s   "
        f"speed: {speed:.2f} m/s   "
        f"ETA: {eta_str}"
    )

    # Pad/truncate to fit the actual terminal width (minus a small
    # safety margin) so the line never wraps onto a second visual
    # row — wrapping is what causes \r to stop overwriting cleanly
    # and instead push output downward, line after line.
    term_width = shutil.get_terminal_size(fallback=(70, 20)).columns
    safe_width = max(20, term_width - 2)
    sys.stdout.write("\r" + line[:safe_width].ljust(safe_width))
    sys.stdout.flush()


# ═══════════════════════════════════════════════════════════════
#  CORE MISSION
# ═══════════════════════════════════════════════════════════════
def run_mission(navigator: BasicNavigator) -> None:
    """
    Drive the robot through MISSION_ORDER waypoints in sequence.
    Handles FAILED and CANCELED results gracefully — logs and
    continues to the next waypoint instead of crashing.
    """

    print("\n[MISSION] Waiting for Nav2 to become active …")
    navigator.waitUntilNav2Active()
    print("[MISSION] Nav2 is active — starting mission.\n")
    print(f"[MISSION] Route: {' → '.join(MISSION_ORDER)}\n")

    results_log = {}   # store outcome of each waypoint for summary

    for name in MISSION_ORDER:

        x, y, yaw = WAYPOINTS[name]
        goal      = make_pose(navigator, x, y, yaw)

        print(f"[MISSION] ── Navigating to {name}  "
              f"(x={x:.1f}, y={y:.1f}, yaw={yaw:.0f}°) ──")

        # Send goal — non-blocking, returns immediately
        navigator.goToPose(goal)

        start_time = time.time()
        prev_dist  = None
        prev_time  = 0.0

        # ── Telemetry loop ─────────────────────────────────────
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            elapsed  = time.time() - start_time

            if feedback is not None:
                dist = feedback.distance_remaining

                print_telemetry(
                    name, dist, elapsed, prev_dist, prev_time
                )

                prev_dist = dist
                prev_time = elapsed

            time.sleep(0.5)   # poll at 2 Hz — enough for smooth display

        # newline after telemetry so next print starts clean
        print()

        # ── Result handling ────────────────────────────────────
        result = navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            elapsed = time.time() - start_time
            print(f"[MISSION] ✓  Reached {name} in {elapsed:.1f} s\n")
            results_log[name] = "SUCCEEDED"

        elif result == TaskResult.FAILED:
            print(f"[MISSION] ✗  FAILED to reach {name} — "
                  f"logging failure and continuing to next waypoint.\n")
            results_log[name] = "FAILED"

        elif result == TaskResult.CANCELED:
            print(f"[MISSION] ✗  {name} was CANCELED — "
                  f"logging and continuing to next waypoint.\n")
            results_log[name] = "CANCELED"

        else:
            print(f"[MISSION] ?  Unknown result for {name} — continuing.\n")
            results_log[name] = "UNKNOWN"

    # ── Mission summary ────────────────────────────────────────
    print("=" * 55)
    print("[MISSION]  MISSION COMPLETE — Summary:")
    for wp, outcome in results_log.items():
        icon = "✓" if outcome == "SUCCEEDED" else "✗"
        print(f"  {icon}  {wp:10s}  →  {outcome}")
    print("=" * 55)


# ═══════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════
def main() -> None:
    rclpy.init()
    navigator = BasicNavigator()

    try:
        run_mission(navigator)

    except KeyboardInterrupt:
        print("\n[MISSION] Interrupted by user — cancelling current goal.")
        navigator.cancelTask()

    finally:
        # Always shut down cleanly regardless of success or error
        navigator.lifecycleShutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()