#!/usr/bin/env python3
"""Sanity checks for a KITTI-format trajectory produced by lodestar_odom.

Without a ROS 1 reference run there is nothing to diff against, so this script
checks the properties a correct odometry output must have regardless of the
middleware it was produced with. It is a smoke test for the ROS 2 port, not an
accuracy evaluation - use `evo` against dataset ground truth for that.

Usage:
    python3 check_trajectory.py <est_dir>/01.txt [--frames N] [--rate 4.0]
    python3 check_trajectory.py <est>/01.txt --reference <ref>/01.txt

Exit code 0 = all hard checks passed, 1 = at least one failed.
"""

import argparse
import math
import sys

try:
    import numpy as np
except ImportError:
    sys.exit("numpy is required:  sudo apt install python3-numpy")


def load_kitti(path):
    """Load a KITTI trajectory: one 3x4 pose per line, row-major, 12 values."""
    rows = []
    with open(path) as fh:
        for n, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            vals = line.split()
            if len(vals) != 12:
                raise ValueError(f"{path}:{n}: expected 12 values, got {len(vals)}")
            rows.append([float(v) for v in vals])
    if not rows:
        raise ValueError(f"{path}: file is empty")
    return np.array(rows).reshape(-1, 3, 4)


class Report:
    def __init__(self):
        self.failed = False

    def ok(self, label, detail=""):
        print(f"  [ ok ] {label}{(': ' + detail) if detail else ''}")

    def warn(self, label, detail=""):
        print(f"  [warn] {label}{(': ' + detail) if detail else ''}")

    def fail(self, label, detail=""):
        print(f"  [FAIL] {label}{(': ' + detail) if detail else ''}")
        self.failed = True


def check(path, expected_frames, rate, rep):
    print(f"\nTrajectory: {path}")
    M = load_kitti(path)
    R = M[:, :, :3]
    t = M[:, :, 3]
    n = len(M)
    print(f"  poses: {n}")

    # --- hard checks -------------------------------------------------------
    if np.isfinite(M).all():
        rep.ok("all values finite (no NaN/Inf)")
    else:
        bad = np.argwhere(~np.isfinite(M))[:, 0]
        rep.fail("NaN or Inf present", f"first at pose {bad[0]}")
        print("  (skipping the remaining checks: they are meaningless on non-finite data)")
        return

    orth = max(float(np.abs(Ri @ Ri.T - np.eye(3)).max()) for Ri in R)
    if orth < 1e-6:
        rep.ok("rotations orthonormal", f"max |R R^T - I| = {orth:.2e}")
    elif orth < 1e-3:
        rep.warn("rotations slightly non-orthonormal", f"{orth:.2e}")
    else:
        rep.fail("rotations not orthonormal", f"max |R R^T - I| = {orth:.2e}")

    dets = np.array([float(np.linalg.det(Ri)) for Ri in R])
    if np.allclose(dets, 1.0, atol=1e-6):
        rep.ok("all rotations proper (det = +1)")
    else:
        rep.fail("improper rotation (det != +1)", f"min det = {dets.min():.6f}")

    if expected_frames is not None:
        if n == expected_frames:
            rep.ok("pose count matches frame count", str(n))
        else:
            rep.fail("pose count != frame count", f"{n} poses vs {expected_frames} frames")

    # LodeStar operates in the horizontal plane; z must stay identically zero
    if np.allclose(t[:, 2], 0.0, atol=1e-9):
        rep.ok("planar motion (z == 0 for all poses)")
    else:
        rep.warn("z component non-zero", f"max |z| = {np.abs(t[:, 2]).max():.3e}")

    # --- motion continuity -------------------------------------------------
    if n < 3:
        rep.warn("too few poses for continuity checks")
        return

    step = np.linalg.norm(np.diff(t[:, :2], axis=0), axis=1)
    print(f"  per-frame step: mean={step.mean():.4f} std={step.std():.4f} "
          f"min={step.min():.4f} max={step.max():.4f}")
    print(f"  total path length: {step.sum():.3f}")

    # A jump far outside the distribution usually means a failed registration
    # that was silently accepted. Use a median/MAD threshold rather than
    # mean+k*sigma: a single large outlier inflates the standard deviation
    # enough to hide itself, and one bad pose produces two bad steps.
    med = float(np.median(step))
    mad = float(np.median(np.abs(step - med)))
    scale = 1.4826 * mad  # MAD -> sigma for normally distributed data
    if scale > 0:
        thresh = med + 8 * scale
    else:
        thresh = med * 5 if med > 0 else float("inf")
    jumps = np.argwhere(step > thresh).ravel()
    if len(jumps) == 0:
        rep.ok("no discontinuous jumps between consecutive poses")
    else:
        rep.warn("possible registration outliers",
                 f"{len(jumps)} step(s) above {thresh:.3f} (median {med:.3f}), "
                 f"first between pose {jumps[0]} and {jumps[0] + 1}")

    if step.min() == 0 and step.max() == 0:
        rep.fail("trajectory is completely static", "registration produced no motion")
    elif step.max() == 0:
        rep.warn("some frames produced zero motion")

    yaw = np.array([math.atan2(Ri[1, 0], Ri[0, 0]) for Ri in R])
    dyaw = np.abs(np.diff(np.unwrap(yaw)))
    print(f"  per-frame yaw change [deg]: mean={math.degrees(dyaw.mean()):.3f} "
          f"max={math.degrees(dyaw.max()):.3f}")
    if math.degrees(dyaw.max()) > 90:
        rep.warn("large yaw jump between consecutive frames",
                 f"{math.degrees(dyaw.max()):.1f} deg")
    else:
        rep.ok("yaw evolves continuously")

    if rate:
        vel = step * rate
        print(f"  implied speed at {rate} Hz: mean={vel.mean():.3f} max={vel.max():.3f} [units/s]")

    # Straightness is informative, never a pass/fail criterion: a real vessel
    # track is not supposed to be a straight line.
    p = t[:, :2] - t[:, :2].mean(0)
    s = np.linalg.svd(p, compute_uv=False)
    ratio = s[1] / s[0] if s[0] > 0 else 0.0
    print(f"  path shape: principal spread {s[0]:.3f} / {s[1]:.3f} (ratio {ratio:.3f})")


def compare(est_path, ref_path, rep):
    """Absolute trajectory error against a reference, without alignment.

    Both files must contain the same number of poses in the same order. This is
    the check to use when an old ROS 1 run is available: a correct port should
    land within floating-point noise.
    """
    print(f"\nComparing against reference: {ref_path}")
    E = load_kitti(est_path)
    R = load_kitti(ref_path)
    if len(E) != len(R):
        rep.fail("pose count differs", f"{len(E)} vs {len(R)}")
        return
    d = np.linalg.norm(E[:, :, 3] - R[:, :, 3], axis=1)
    print(f"  translation error: mean={d.mean():.6e} max={d.max():.6e}")
    rot = np.array([float(np.abs(E[i, :, :3] - R[i, :, :3]).max()) for i in range(len(E))])
    print(f"  rotation elementwise error: max={rot.max():.6e}")
    if d.max() < 1e-3 and rot.max() < 1e-4:
        rep.ok("matches reference within floating-point tolerance")
    elif d.max() < 1e-1:
        rep.warn("small deviation from reference", f"max {d.max():.3e}")
    else:
        rep.fail("trajectory differs substantially from reference", f"max {d.max():.3e}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trajectory", help="KITTI file written by lodestar_odom (usually 01.txt)")
    ap.add_argument("--frames", type=int, default=None,
                    help="expected number of radar frames (from `ros2 bag info`)")
    ap.add_argument("--rate", type=float, default=None,
                    help="sensor rate in Hz, to report implied speed")
    ap.add_argument("--reference", default=None,
                    help="reference KITTI trajectory to diff against (e.g. a ROS 1 run)")
    args = ap.parse_args()

    rep = Report()
    try:
        check(args.trajectory, args.frames, args.rate, rep)
        if args.reference:
            compare(args.trajectory, args.reference, rep)
    except (OSError, ValueError) as exc:
        print(f"  [FAIL] {exc}")
        rep.failed = True

    print("\nRESULT:", "FAILED" if rep.failed else "all hard checks passed")
    return 1 if rep.failed else 0


if __name__ == "__main__":
    sys.exit(main())
