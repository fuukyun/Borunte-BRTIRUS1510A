# -*- coding: utf-8 -*-
"""
reset_borunte_1510a.py
======================

Đưa robot "BORUNTE_BRTIRUS1510A" về trạng thái chuẩn sau khi lỡ kéo / di chuyển
bằng tay trong RoboDK (kéo robot, kéo khung Base, jog khớp, kéo lệch tool...).

Những gì được đặt lại (chỉ sửa mục nào đang lệch, có báo cáo trước/sau):
  1. Khung "BORUNTE_BRTIRUS1510A Base": cha = gốc trạm, Pose = Z -175 mm
     (robot trùng khít bộ CAD BRT_* gốc).
  2. Robot: cha = khung Base.
  3. Khớp: về Home [0, 0, 0, 0, 0, 0].
  4. Reference đang hoạt động = khung Base; tool đang hoạt động = "tool du an".
  5. Tool "tool du an": cha = robot, GeometryPose và TCP về giá trị đã hiệu chỉnh.

Cách dùng:
    python reset_borunte_1510a.py            # reset ngay (đặt khớp tức thời)
    python reset_borunte_1510a.py --animate  # robot chạy MoveJ về Home (mô phỏng)
    python reset_borunte_1510a.py --dry-run  # chỉ kiểm tra, không thay đổi gì
"""

import math
import os
import sys

for _p in (os.path.expanduser(r"~/.claude/skills/robodk-api/assets/robodk-api"),
           r"C:\RoboDK\Python"):
    if os.path.isdir(_p):
        sys.path.insert(0, _p)
        break
from robodk import robolink, robomath  # noqa: E402
from robodk.robomath import transl, roty, rotz  # noqa: E402

# ===========================================================================
# Trạng thái chuẩn
# ===========================================================================
ROBOT_NAME = "BORUNTE_BRTIRUS1510A"
BASE_FRAME_NAME = ROBOT_NAME + " Base"
TOOL_NAME = "tool du an"

BASE_POSE = transl(0, 0, -175.0)          # mặt đáy đế CAD nằm ở Z = -175 mm
JOINTS_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Tool đã hiệu chỉnh (giống align_tool_du_an.py)
TOOL_GEOMETRY_POSE = rotz(math.radians(-90)) * transl(9.4, 7.1, 0)
TOOL_TCP_POSE = transl(-221.0, 0.0, 59.5) * roty(math.radians(-90))

TOL_MM, TOL_DEG = 0.01, 0.01


# ===========================================================================
def pose_diff(A, B):
    """(sai lệch vị trí mm, sai lệch góc deg) giữa hai pose."""
    dp = math.dist(A.Pos(), B.Pos())
    R = robomath.invH(A) * B
    c = max(-1.0, min(1.0, (R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2))
    return dp, math.degrees(math.acos(c))


def pose_str(M):
    v = robomath.Pose_2_TxyzRxyz(M)
    return (f"[{v[0]:.1f}, {v[1]:.1f}, {v[2]:.1f} | "
            f"{math.degrees(v[3]):.1f}, {math.degrees(v[4]):.1f}, {math.degrees(v[5]):.1f}]")


def is_off(A, B):
    dp, dr = pose_diff(A, B)
    return dp > TOL_MM or dr > TOL_DEG


def same(a, b):
    return a.Valid() and b.Valid() and a.item == b.item


# ===========================================================================
def main():
    dry = "--dry-run" in sys.argv
    animate = "--animate" in sys.argv
    changes = []

    def fix(label, before, after, action):
        changes.append(label)
        print(f"  [LỆCH] {label:<26} {before}  ->  {after}")
        if not dry:
            action()

    with robolink.Robolink() as RDK:
        robot = RDK.Item(ROBOT_NAME, robolink.ITEM_TYPE_ROBOT)
        if not robot.Valid() or robot.Name() != ROBOT_NAME:
            raise RuntimeError(f"Không tìm thấy robot '{ROBOT_NAME}'.")
        station = RDK.ActiveStation()

        base = RDK.Item(BASE_FRAME_NAME, robolink.ITEM_TYPE_FRAME)
        if not base.Valid() or base.Name() != BASE_FRAME_NAME:
            raise RuntimeError(f"Không tìm thấy khung '{BASE_FRAME_NAME}'.")

        tool = RDK.Item(TOOL_NAME, robolink.ITEM_TYPE_TOOL)
        tool = tool if (tool.Valid() and tool.Name() == TOOL_NAME) else None

        print("=" * 78)
        print(f"Reset {ROBOT_NAME}" + ("   (DRY-RUN: chỉ kiểm tra)" if dry else ""))
        print("=" * 78)
        if not dry:
            RDK.Render(False)

        # 1. Khung Base
        if not same(base.Parent(), station):
            fix("Cha của khung Base", base.Parent().Name(), station.Name(),
                lambda: base.setParentStatic(station))
        if is_off(base.Pose(), BASE_POSE):
            fix("Vị trí khung Base", pose_str(base.Pose()), pose_str(BASE_POSE),
                lambda: base.setPose(BASE_POSE))

        # 2. Robot nằm dưới khung Base
        if not same(robot.Parent(), base):
            fix("Cha của robot", robot.Parent().Name(), base.Name(),
                lambda: robot.setParent(base))

        # 3. Khớp về Home
        q = robot.Joints().list()[:6]
        if max(abs(a - b) for a, b in zip(q, JOINTS_HOME)) > TOL_DEG:
            def go_home():
                if animate:
                    robot.MoveJ(JOINTS_HOME)
                else:
                    robot.setJoints(JOINTS_HOME)
            fix("Khớp robot", "[" + ", ".join(f"{v:.2f}" for v in q) + "]",
                str(JOINTS_HOME), go_home)

        # 4. Reference đang hoạt động
        if is_off(robot.PoseFrame(), robomath.eye(4)):
            fix("Reference hoạt động", pose_str(robot.PoseFrame()), "khung Base",
                lambda: robot.setPoseFrame(base))

        # 5. Tool
        if tool is None:
            print(f"  [BỎ QUA] không có tool '{TOOL_NAME}' trong trạm")
        else:
            if not same(tool.Parent(), robot):
                fix("Cha của tool", tool.Parent().Name(), robot.Name(),
                    lambda: tool.setParent(robot))
            if is_off(tool.GeometryPose(), TOOL_GEOMETRY_POSE):
                fix("Hình học tool", pose_str(tool.GeometryPose()), pose_str(TOOL_GEOMETRY_POSE),
                    lambda: tool.setGeometryPose(TOOL_GEOMETRY_POSE, False))
            if is_off(tool.PoseTool(), TOOL_TCP_POSE):
                fix("TCP tool", pose_str(tool.PoseTool()), pose_str(TOOL_TCP_POSE),
                    lambda: tool.setPoseTool(TOOL_TCP_POSE))
            if is_off(robot.PoseTool(), TOOL_TCP_POSE):
                fix("Tool hoạt động của robot", pose_str(robot.PoseTool()), TOOL_NAME,
                    lambda: robot.setPoseTool(tool))

        if not dry:
            RDK.Render(True)

        # Kết quả
        print("-" * 78)
        if not changes:
            print("Mọi thứ đã ở trạng thái chuẩn - không cần thay đổi.")
        elif dry:
            print(f"{len(changes)} mục đang lệch. Chạy lại không có --dry-run để reset.")
        else:
            print(f"Đã reset {len(changes)} mục.")

        q = robot.Joints().list()[:6]
        flange = base.PoseAbs() * robot.SolveFK(q)
        print(f"Khớp hiện tại          : [{', '.join(f'{v:.2f}' for v in q)}]")
        print(f"Khung Base (gốc trạm)  : {pose_str(base.PoseAbs())}")
        print(f"Mặt bích (gốc trạm)    : {pose_str(flange)}")
        if tool is not None:
            tcp = flange * tool.PoseTool()
            p, z = tcp.Pos(), tcp.VZ()
            print(f"TCP mũi dao (gốc trạm) : X={p[0]:.1f}  Y={p[1]:.1f}  Z={p[2]:.1f} mm, "
                  f"hướng [{z[0]:.3f}, {z[1]:.3f}, {z[2]:.3f}]")
        print("=" * 78)


if __name__ == "__main__":
    main()
