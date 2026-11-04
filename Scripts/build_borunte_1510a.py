# -*- coding: utf-8 -*-
"""
build_borunte_1510a.py
======================

Khởi tạo robot 6 trục "BORUNTE_BRTIRUS1510A" trong trạm RoboDK đang mở,
từ 7 khâu CAD có sẵn trong trạm:
    Base -> BRT_Base, J1 -> BRT_Link1, ..., J6 -> BRT_Link6

Cách chạy:
    1. Mở trạm RoboDK chứa các khâu BRT_* (RoboDK phải đang chạy).
    2. python build_borunte_1510a.py
       (hoặc trong RoboDK: Tools > Run Script, chọn file này)

Chạy lại nhiều lần được: robot cùng tên đã có sẽ được thay thế (không tạo trùng).

Ghi chú kỹ thuật (đo trực tiếp trên lưới CAD trong trạm):
    * Mặt đáy BRT_Base nằm ở Z = -175 mm trong hệ toạ độ CAD, còn d1 = 488 mm được
      tính từ mặt đáy. Script tạm tịnh tiến CAD để mặt đáy trùng gốc robot khi dựng,
      sau đó đặt khung Base của robot về Z = -175 để robot trùng khít vị trí CAD gốc.
    * CAD KHÔNG được xuất ở tư thế 0. Tư thế đo được (khai báo trong JOINTS_BUILD):
        J1 ≈ -1.76°  (tâm mặt bích lệch Y = -29.6 mm ở X = 963 mm)
        J2 ≈ +3.64°, J3 ≈ -3.64°  (tâm J2 (168, Z313) -> tâm J3 (209.3, Z961.6):
                                   dài 649.9 mm ≈ a3; cẳng tay nằm ngang, cao hơn J3
                                   161.9 mm ≈ a4)
        J4 ≈ +71.66° (pháp tuyến hai mặt "càng" cổ tay nghiêng 71.66° trong mặt YZ)
      Kiểm chứng: FK mặt bích tại tư thế CAD lệch ≤ ~2.6 mm so với mặt bích CAD.
"""

import json
import os
import sys

# ---------------------------------------------------------------------------
# Import RoboDK API: ưu tiên bản robodk đi kèm skill (>= 6.0.2) nếu có,
# nếu không dùng gói đã cài (pip install robodk) hoặc gói của bản cài RoboDK.
# ---------------------------------------------------------------------------
for _p in (os.path.expanduser(r"~/.claude/skills/robodk-api/assets/robodk-api"),
           r"C:\RoboDK\Python"):
    if os.path.isdir(_p):
        sys.path.insert(0, _p)
        break
from robodk import robolink, robomath  # noqa: E402

# ===========================================================================
# THÔNG SỐ ROBOT
# ===========================================================================
ROBOT_NAME = "BORUNTE_BRTIRUS1510A"

# Khâu CAD theo thứ tự [Base, J1, J2, J3, J4, J5, J6]
LINK_NAMES = ["BRT_Base", "BRT_Link1", "BRT_Link2", "BRT_Link3",
              "BRT_Link4", "BRT_Link5", "BRT_Link6"]

# Kích thước hình học cơ cấu (mm / deg) theo bản vẽ 2D
d1 = 488.0
a2 = 170.0
d2 = 0.0
a3 = 650.0
a4 = 162.0
d4 = 634.4
d5 = 0.0     # mô hình 6 trục của RoboDK không có tham số d5 -> bắt buộc = 0
a6 = 0.0     # mô hình 6 trục của RoboDK không có tham số a6 -> bắt buộc = 0
d6 = 117.0
delta_theta2 = -90.0
delta_theta6 = 0.0   # đổi thành 180.0 nếu mặt bích bị ngược hướng

# Thứ tự tham số RDK.BuildMechanism(MAKE_ROBOT_6DOF, ...) – đã kiểm chứng bằng
# cách thay đổi từng tham số và đọc FK trên RoboDK 6.2:
#   [d1, a2, d2, a3, a4, d4, d6, Δθ1, Δθ2, Δθ3, Δθ4, Δθ5, Δθ6]
assert d5 == 0.0 and a6 == 0.0, "MAKE_ROBOT_6DOF không hỗ trợ d5/a6 khác 0"
PARAMETERS = [d1, a2, d2, a3, a4, d4, d6,
              0.0, delta_theta2, 0.0, 0.0, 0.0, delta_theta6]

# Giới hạn hành trình khớp (deg)
LOWER_LIMITS = [-165.0, -95.0, -85.0, -180.0, -115.0, -360.0]
UPPER_LIMITS = [165.0, 70.0, 75.0, 180.0, 115.0, 360.0]

# Tốc độ khớp tối đa (deg/s).
# LƯU Ý: RoboDK API 6.2 không có lệnh đặt tốc độ khớp tối đa của robot
# (đã dò toàn bộ danh sách Command/setParam). Nhập tay một lần tại:
#   double-click robot > Parameters > Joint speed (deg/s)
MAX_SPEEDS = [190.0, 173.0, 223.0, 254.0, 270.0, 342.0]

PAYLOAD_KG = 10.0
WEIGHT_KG = 152.0

JOINTS_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Chiều quay khớp (+1/-1); +1 = quy ước tham chiếu của RoboDK
JOINT_SENSES = [+1, +1, +1, +1, +1, +1]

# Tư thế các khâu CAD đang nằm trong trạm (đo từ hình học, xem docstring)
JOINTS_BUILD = [-1.76, 3.64, -3.64, 71.66, 0.0, 0.0]


def main():
    # Kết nối RoboDK đang chạy (localhost:20500). RoboDK vẫn mở sau khi script kết thúc.
    with robolink.Robolink() as RDK:
        # -------------------------------------------------------------------
        # 1. Lấy các khâu CAD
        # -------------------------------------------------------------------
        links = []
        for n in LINK_NAMES:
            it = RDK.Item(n, robolink.ITEM_TYPE_OBJECT)
            if not it.Valid() or it.Name() != n:
                raise RuntimeError(f"Không tìm thấy khâu CAD '{n}' trong trạm.")
            links.append(it)

        RDK.Render(False)

        # Robot cùng tên do lần chạy trước tạo ra -> xoá (kèm khung Base của nó)
        # để không sinh robot trùng tên
        for r in RDK.ItemList(robolink.ITEM_TYPE_ROBOT):
            if r.Name() == ROBOT_NAME:
                r.Delete()

        # -------------------------------------------------------------------
        # 2. Dịch tạm hình học CAD để mặt đáy đế trùng Z = 0 (gốc robot theo bản
        #    vẽ, d1 = 488 tính từ mặt đáy), dựng robot, rồi hoàn nguyên CAD.
        #    BuildMechanism lấy toạ độ LƯỚI CỤC BỘ của từng khâu (bỏ qua Pose()
        #    của object), nên phải dịch bằng setGeometryPose(apply=False) – cách
        #    này không sửa đỉnh lưới và hoàn nguyên được hoàn toàn.
        # -------------------------------------------------------------------
        orig_geo = [it.GeometryPose() for it in links]
        bbox = links[0].setParam("BoundingBox")       # dict (hoặc chuỗi JSON ở bản API cũ)
        if isinstance(bbox, (str, bytes)):
            bbox = json.loads(bbox)
        # BoundingBox là toạ độ tuyệt đối -> quy về hệ cục bộ của BRT_Base
        floor_z = float(bbox["min"][2]) - links[0].PoseAbs().Pos()[2]   # ≈ -175 mm
        try:
            for it, g in zip(links, orig_geo):
                it.setGeometryPose(robomath.transl(0, 0, -floor_z) * g, False)
            robot = RDK.BuildMechanism(
                # truyền bản sao: BuildMechanism tự chèn thêm phần tử vào các list
                robolink.MAKE_ROBOT_6DOF, links, list(PARAMETERS),
                list(JOINTS_BUILD), list(JOINTS_HOME), list(JOINT_SENSES),
                list(LOWER_LIMITS), list(UPPER_LIMITS),
                robomath.eye(4),             # base offset
                robomath.eye(4),             # tool (TCP = mặt bích)
                ROBOT_NAME)
        finally:
            for it, g in zip(links, orig_geo):
                it.setGeometryPose(g, False)

        if not robot.Valid():
            raise RuntimeError("BuildMechanism thất bại - kiểm tra lại thông số.")

        # -------------------------------------------------------------------
        # 3. Vị trí & thuộc tính
        # -------------------------------------------------------------------
        # Khung Base của robot (gốc = mặt đáy đế): đặt sao cho robot trùng khít
        # vị trí bộ CAD gốc trong trạm
        base_frame = robot.Parent()
        if base_frame.Valid() and base_frame.Type() == robolink.ITEM_TYPE_FRAME:
            base_frame.setPoseAbs(links[0].PoseAbs() * robomath.transl(0, 0, floor_z))

        robot.setJointLimits(list(LOWER_LIMITS), list(UPPER_LIMITS))
        robot.setParam("Payload", str(PAYLOAD_KG))
        robot.setParam("Weight", str(WEIGHT_KG))
        robot.setJoints(JOINTS_HOME)

        # Ẩn các khâu CAD rời (robot đã chứa bản sao hình học)
        for it in links:
            it.setVisible(False)

        # -------------------------------------------------------------------
        # 4. Kiểm tra nhanh, render, thông báo
        # -------------------------------------------------------------------
        flange = robot.SolveFK(JOINTS_HOME).Pos()
        RDK.Render(True)

        print("=" * 64)
        print(f"Đã tạo robot      : {robot.Name()}")
        print(f"Tham số DH        : {PARAMETERS}")
        print(f"Giới hạn khớp     : {LOWER_LIMITS}")
        print(f"                    {UPPER_LIMITS}")
        print(f"Payload / Weight  : {robot.setParam('Payload')} kg / {robot.setParam('Weight')} kg")
        print(f"Home              : {robot.Joints().list()}")
        print(f"Mặt bích @Home    : X={flange[0]:.1f}  Y={flange[1]:.1f}  Z={flange[2]:.1f} mm"
              f"  (kỳ vọng {a2 + d4 + d6:.1f}, 0.0, {d1 + a3 + a4:.1f})")
        print(f"Max speed (deg/s) : {MAX_SPEEDS}  -> nhập tay trong Robot Parameters")
        print("HOÀN TẤT: robot BORUNTE_BRTIRUS1510A đã được tạo trong trạm.")
        print("=" * 64)


if __name__ == "__main__":
    main()
