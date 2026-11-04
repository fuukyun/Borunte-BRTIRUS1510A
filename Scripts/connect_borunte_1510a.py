# -*- coding: utf-8 -*-
"""
connect_borunte_1510a.py
========================

Kết nối tới trạm RoboDK đang mở và lấy tham chiếu tới robot + tool ĐÃ DỰNG SẴN
(chỉ đọc — KHÔNG tạo mới, KHÔNG sửa pose, TCP, khớp hay cây trạm).

Trạm hiện tại (1510A_Station):
    1510A_Station
    └── BORUNTE_BRTIRUS1510A Base   (frame gốc robot)
        └── BORUNTE_BRTIRUS1510A    (robot 6 trục)
            └── tool du an          (tool, TCP = [-221, 0, 59.5] mm, Ry = -90°)

Cách dùng:
    * Chạy trực tiếp để kiểm tra kết nối và in thông tin:
          python connect_borunte_1510a.py
    * Import vào script khác:
          from connect_borunte_1510a import connect
          RDK, robot, tool, frame = connect()
          robot.MoveJ(...)
"""

import math
import os
import sys

# ---------------------------------------------------------------------------
# Import RoboDK API: ưu tiên bản đi kèm skill (>= 6.0.2), sau đó bản của RoboDK.
# ---------------------------------------------------------------------------
for _p in (os.path.expanduser(r"~/.claude/skills/robodk-api/assets/robodk-api"),
           r"C:\RoboDK\Python"):
    if os.path.isdir(_p):
        sys.path.insert(0, _p)
        break
from robodk import robolink, robomath  # noqa: E402

ROBOT_NAME = "BORUNTE_BRTIRUS1510A"
TOOL_NAME = "tool du an"
FRAME_NAME = "BORUNTE_BRTIRUS1510A Base"


def _require(item, kind, name):
    if not item.Valid():
        raise RuntimeError(f"Không tìm thấy {kind} '{name}' trong trạm đang mở.")
    return item


def connect(RDK=None):
    """Kết nối RoboDK và trả về (RDK, robot, tool, frame). Không thay đổi trạm."""
    if RDK is None:
        RDK = robolink.Robolink()

    robot = _require(RDK.Item(ROBOT_NAME, robolink.ITEM_TYPE_ROBOT), "robot", ROBOT_NAME)
    tool = _require(RDK.Item(TOOL_NAME, robolink.ITEM_TYPE_TOOL), "tool", TOOL_NAME)
    frame = _require(RDK.Item(FRAME_NAME, robolink.ITEM_TYPE_FRAME), "frame", FRAME_NAME)

    # Kiểm tra tool thực sự được gắn trên robot này (chỉ kiểm tra, không gắn lại)
    if tool.Parent() != robot:
        raise RuntimeError(f"Tool '{TOOL_NAME}' không gắn trên robot '{ROBOT_NAME}'.")

    active = robot.getLink(robolink.ITEM_TYPE_TOOL)
    if not active.Valid() or active != tool:
        print(f"[CẢNH BÁO] Tool đang active của robot không phải '{TOOL_NAME}'. "
              f"Script không tự đổi — hãy chọn tool trong RoboDK nếu cần.")

    return RDK, robot, tool, frame


def _fmt(pose):
    x, y, z, rx, ry, rz = robomath.Pose_2_TxyzRxyz(pose)
    return (f"X={x:.2f} Y={y:.2f} Z={z:.2f} mm | "
            f"Rx={math.degrees(rx):.2f} Ry={math.degrees(ry):.2f} Rz={math.degrees(rz):.2f} deg")


def print_info(robot, tool, frame):
    print(f"Robot : {robot.Name()}")
    print(f"Frame : {frame.Name()}  -> PoseAbs {_fmt(frame.PoseAbs())}")
    print(f"Tool  : {tool.Name()}   -> TCP     {_fmt(tool.PoseTool())}")
    print("Khớp  : [" + ", ".join(f"{j:.3f}" for j in robot.Joints().list()) + "] deg")
    print(f"TCP so với frame robot: {_fmt(robot.Pose())}")
    lo, hi, _ = robot.JointLimits()
    print("Giới hạn khớp min: [" + ", ".join(f"{v:.1f}" for v in lo.list()) + "]")
    print("Giới hạn khớp max: [" + ", ".join(f"{v:.1f}" for v in hi.list()) + "]")


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    RDK, robot, tool, frame = connect()
    print(f"Đã kết nối trạm: {RDK.ActiveStation().Name()}")
    print_info(robot, tool, frame)
