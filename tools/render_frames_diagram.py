"""Render the dual-NERO kinematic and TCP frame diagram used in the README.

Every position and axis is computed by forward kinematics on
models/source/nero_dual_tacclaw.urdf, so the figure stays in sync with the
model the teleoperation IK consumes.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pinocchio as pin  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
ARM_COLOR = {"arm_a": "#2f6fb0", "arm_b": "#c4572a"}
AXIS_COLOR = ("#d62728", "#2ca02c", "#1f5fd6")
AXIS_LENGTH_M = 0.07


def _pose(model: pin.Model, data: pin.Data, q: np.ndarray) -> dict[str, dict]:
    pin.framesForwardKinematics(model, data, q)
    result = {}
    for arm in ("arm_a", "arm_b"):
        base = data.oMf[model.getFrameId(f"{arm}_base_link")].translation
        joints = [
            data.oMi[model.getJointId(f"{arm}_joint{index}")].translation
            for index in range(1, 8)
        ]
        # oMf/oMi entries are views into data; copy before the next FK call.
        tcp = data.oMf[model.getFrameId(f"{arm}_tcp_link")].copy()
        result[arm] = {
            "chain": np.vstack([base, *joints, tcp.translation]),
            "tcp": tcp,
        }
    return result


def _draw_view(
    ax, pose, horizontal: int, *, label_tcp: bool, color: str | None = None
) -> None:
    for arm, item in pose.items():
        chain = item["chain"]
        ax.plot(
            chain[:, horizontal], chain[:, 2], "-o", color=color or ARM_COLOR[arm],
            lw=4, ms=5, solid_capstyle="round", zorder=2,
        )
        tcp = item["tcp"]
        origin = tcp.translation
        for column, axis_color in enumerate(AXIS_COLOR):
            direction = tcp.rotation[:, column] * AXIS_LENGTH_M
            if abs(direction[horizontal]) + abs(direction[2]) < 1e-6:
                # Axis points straight at the viewer; mark it with a dot.
                ax.plot(origin[horizontal], origin[2], "o", color=axis_color, ms=7, zorder=4)
                continue
            ax.annotate(
                "", xy=(origin[horizontal] + direction[horizontal], origin[2] + direction[2]),
                xytext=(origin[horizontal], origin[2]),
                arrowprops={"arrowstyle": "-|>", "color": axis_color, "lw": 2},
                zorder=4,
            )
        if label_tcp:
            shown = np.round(origin, 3) + 0.0  # drop negative zeros
            ax.text(
                origin[horizontal], origin[2] - 0.06,
                f"{arm} TCP\n({shown[0]:.3f}, {shown[1]:.3f}, {shown[2]:.3f}) m",
                ha="center", va="top", fontsize=8.5, color=ARM_COLOR[arm],
            )


def render(urdf: Path, output: Path) -> None:
    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()
    zero = pin.neutral(model)
    hands_up = zero.copy()
    for arm in ("arm_a", "arm_b"):
        joint = model.joints[model.getJointId(f"{arm}_joint4")]
        hands_up[joint.idx_q] = min(np.pi / 2.0, model.upperPositionLimit[joint.idx_q])
    zero_pose = _pose(model, data, zero)
    up_pose = _pose(model, data, hands_up)

    fig, (front, side) = plt.subplots(
        1, 2, figsize=(12, 7.2), gridspec_kw={"width_ratios": [1, 0.85]}
    )
    fig.patch.set_facecolor("white")

    # Front view: the operator faces the robot, so world +Y (arm_b, left)
    # appears on the viewer's right.
    _draw_view(front, zero_pose, horizontal=1, label_tcp=True)
    front.set_title("Front view (looking along -X), zero pose", fontsize=11)
    front.set_xlabel("world Y  [m]   (+Y = robot left = arm_b)")
    for arm, name in (("arm_a", "arm_a / right hand\ncan_arm_a"), ("arm_b", "arm_b / left hand\ncan_arm_b")):
        base = zero_pose[arm]["chain"][0]
        front.text(1.6 * base[1], 0.07, name, ha="center", fontsize=9, color=ARM_COLOR[arm], weight="bold")
    front.annotate(
        "", xy=(-0.13, 0.025), xytext=(0.13, 0.025),
        arrowprops={"arrowstyle": "<->", "color": "#444", "lw": 1.2},
    )
    front.text(0, 0.035, "base spacing 260 mm", ha="center", fontsize=8.5, color="#444")
    shoulder = zero_pose["arm_b"]["chain"][1]
    elbow = zero_pose["arm_b"]["chain"][4]
    wrist = zero_pose["arm_b"]["chain"][7]
    tcp = zero_pose["arm_b"]["tcp"].translation
    for start, end, text in (
        (shoulder, elbow, "310 mm"),
        (elbow, wrist, "270 mm"),
        (wrist, tcp, "TacClaw\n225.48 mm"),
    ):
        front.annotate(
            "", xy=(shoulder[1] + 0.07, end[2]), xytext=(shoulder[1] + 0.07, start[2]),
            arrowprops={"arrowstyle": "<->", "color": "#888", "lw": 1.0},
        )
        front.text(
            shoulder[1] + 0.085, (start[2] + end[2]) / 2.0, text,
            ha="left", va="center", fontsize=8.5, color="#555",
        )

    side.set_title("Side view (looking along +Y): zero pose vs Y-button hands-up", fontsize=11)
    _draw_view(side, {"arm_a": zero_pose["arm_a"]}, horizontal=0, label_tcp=False, color="#b5b5b5")
    # Both arms overlap in this view; draw arm_a only for the hands-up preset.
    _draw_view(side, {"arm_a": up_pose["arm_a"]}, horizontal=0, label_tcp=False)
    up_tcp = up_pose["arm_a"]["tcp"].translation
    side.text(
        up_tcp[0] - 0.05, up_tcp[2] + 0.11,
        f"hands-up (J4 = 90°)\nTCP x = {up_tcp[0]:.3f} m, z = {up_tcp[2]:.3f} m",
        ha="center", fontsize=8.5, color="#555",
    )
    zero_tcp = zero_pose["arm_a"]["tcp"].translation
    side.text(
        zero_tcp[0] + 0.05, zero_tcp[2], f"zero pose\nTCP z = {zero_tcp[2]:.4f} m",
        ha="left", va="center", fontsize=8.5, color="#555",
    )
    side.set_xlabel("world X  [m]   (+X = forward, into the workspace)")

    for ax in (front, side):
        ax.set_ylabel("world Z  [m]")
        ax.set_aspect("equal")
        ax.grid(True, color="#e3e3e3", lw=0.8)
        ax.axhline(0.0, color="#999", lw=0.8)
        ax.set_ylim(-1.0, 0.15)
    front.set_xlim(-0.48, 0.48)
    side.set_xlim(-0.15, 0.65)

    legend = [
        plt.Line2D([], [], color=AXIS_COLOR[0], lw=2, label="TCP x"),
        plt.Line2D([], [], color=AXIS_COLOR[1], lw=2, label="TCP y"),
        plt.Line2D([], [], color=AXIS_COLOR[2], lw=2, label="TCP z (approach)"),
        plt.Line2D([], [], color="#444", marker="o", lw=0, label="joint origin J1–J7"),
    ]
    fig.legend(handles=legend, loc="lower center", ncol=4, frameon=False, fontsize=9)
    fig.suptitle(
        "Dual NERO + TacClaw: frame lab_world, TCP = fingertip midpoint "
        "(link7 → xyz [0.22548, 0, -0.0235] m, rpy [0, π/2, 0])",
        fontsize=11.5,
    )
    fig.tight_layout(rect=(0, 0.05, 1, 0.95))
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    print(f"wrote {output}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=ROOT / "models/source/nero_dual_tacclaw.urdf")
    parser.add_argument("--output", type=Path, default=ROOT / "docs/images/nero_frames.png")
    args = parser.parse_args()
    render(args.urdf, args.output)


if __name__ == "__main__":
    main()
