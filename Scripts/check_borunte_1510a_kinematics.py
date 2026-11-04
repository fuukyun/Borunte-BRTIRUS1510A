# -*- coding: utf-8 -*-
"""
check_borunte_1510a_kinematics.py
=================================

Kiểm tra động học robot "BORUNTE_BRTIRUS1510A" đã dựng trong trạm RoboDK
(bằng build_borunte_1510a.py). Script KHÔNG thay đổi trạm: tư thế robot được
khôi phục khi kết thúc.

Các bài kiểm tra:
  T1  Tư thế Home: vị trí + hướng mặt bích so với giá trị tính tay từ bản vẽ.
  T2  FK độc lập: so robot.SolveFK() với một mô hình giải tích tự dựng
      (product-of-exponentials từ d1, a2, d2, a3, a4, d4, d6) trên N tư thế ngẫu nhiên
      trong giới hạn khớp + một bộ tư thế đặc trưng.
  T3  IK hai chiều: q -> FK -> SolveIK(pose, q) -> FK, sai số pose và sai số khớp.
  T4  IK đa nghiệm: SolveIK_All() trả về đủ nghiệm và mọi nghiệm đều cho cùng pose.
  T5  Giới hạn khớp: đọc lại từ RoboDK so với datasheet.
  T6  Tâm cổ tay: chỉ phụ thuộc J1..J3 (cổ tay cầu) – J4..J6 không làm dịch tâm cổ tay.
  T7  Hình học CAD: ở tư thế JOINTS_BUILD, hình của từng khâu robot trùng khâu CAD gốc.
  INFO Tầm với lý thuyết, payload/weight.

Chạy: python check_borunte_1510a_kinematics.py [số_tư_thế_ngẫu_nhiên]
Mã thoát 0 = tất cả PASS, 1 = có FAIL.
"""

import math
import os
import random
import sys

for _p in (os.path.expanduser(r"~/.claude/skills/robodk-api/assets/robodk-api"),
           r"C:\RoboDK\Python"):
    if os.path.isdir(_p):
        sys.path.insert(0, _p)
        break
from robodk import robolink, robomath  # noqa: E402
from robodk.robomath import transl, rotx, roty, rotz  # noqa: E402

# ===========================================================================
# Thông số kỳ vọng (phải trùng với build_borunte_1510a.py)
# ===========================================================================
ROBOT_NAME = "BORUNTE_BRTIRUS1510A"
d1, a2, d2, a3, a4, d4, d6 = 488.0, 170.0, 0.0, 650.0, 162.0, 634.4, 117.0
delta_theta6 = 0.0
LOWER = [-165.0, -95.0, -85.0, -180.0, -115.0, -360.0]
UPPER = [165.0, 70.0, 75.0, 180.0, 115.0, 360.0]
PAYLOAD_KG, WEIGHT_KG = 10.0, 152.0
JOINTS_BUILD = [-1.76, 3.64, -3.64, 71.66, 0.0, 0.0]
LINK_NAMES = ["BRT_Base"] + [f"BRT_Link{i}" for i in range(1, 7)]

# Ngưỡng chấp nhận
TOL_POS = 1e-3      # mm
TOL_ROT = 1e-4      # deg
TOL_JNT = 1e-3      # deg
TOL_CAD = 0.05      # mm (bounding box)
SINGULAR_J5 = 1.0   # deg – bỏ qua so sánh khớp IK gần kỳ dị cổ tay

N_RANDOM = int(sys.argv[1]) if len(sys.argv) > 1 else 500
random.seed(1510)


# ===========================================================================
# Mô hình giải tích độc lập (product of exponentials, hệ gốc = mặt đáy đế)
# Ở Home: cánh tay trên thẳng đứng, cẳng tay nằm ngang theo +X.
#   J1: quay quanh +Z qua gốc
#   J2: quay quanh +Y qua (a2, d2, d1)
#   J3: quay quanh +Y qua (a2, d2, d1+a3)
#   J4: quay quanh +X qua (a2, d2, d1+a3+a4)
#   J5: quay quanh +Y qua (a2+d4, d2, d1+a3+a4)        (tâm cổ tay)
#   J6: quay quanh +X qua tâm cổ tay
#   Mặt bích Home: (a2+d4+d6, d2, d1+a3+a4), trục Z dụng cụ hướng +X
# ===========================================================================
def _about(rot, angle_deg, point):
    p = transl(*point)
    return p * rot(math.radians(angle_deg)) * robomath.invH(p)


def analytic_fk(q):
    z = d1 + a3 + a4
    wc = (a2 + d4, d2, z)
    T = _about(rotz, q[0], (0, 0, 0))
    T = T * _about(roty, q[1], (a2, d2, d1))
    T = T * _about(roty, q[2], (a2, d2, d1 + a3))
    T = T * _about(rotx, q[3], (a2, d2, z))
    T = T * _about(roty, q[4], wc)
    T = T * _about(rotx, q[5], wc)
    M = transl(a2 + d4 + d6, d2, z) * roty(math.pi / 2) * rotz(math.pi) * rotz(math.radians(delta_theta6))
    return T * M


# ===========================================================================
# Tiện ích
# ===========================================================================
def pos_err(A, B):
    return math.dist(A.Pos(), B.Pos())


def rot_err_deg(A, B):
    R = robomath.invH(A) * B
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1) / 2))))


def ang_diff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def rand_q():
    return [random.uniform(lo, hi) for lo, hi in zip(LOWER, UPPER)]


def fmt(v, n=3):
    return "[" + ", ".join(f"{x:.{n}f}" for x in v) + "]"


RESULTS = []


def report(tid, name, ok, detail):
    RESULTS.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] {tid:<4} {name:<38} {detail}")


def info(name, detail):
    print(f"[INFO]      {name:<38} {detail}")


# ===========================================================================
def main():
    with robolink.Robolink() as RDK:
        robot = RDK.Item(ROBOT_NAME, robolink.ITEM_TYPE_ROBOT)
        if not robot.Valid() or robot.Name() != ROBOT_NAME:
            print(f"Không tìm thấy robot '{ROBOT_NAME}'. Hãy chạy build_borunte_1510a.py trước.")
            return 1
        q_saved = robot.Joints().list()[:6]
        print("=" * 96)
        print(f"Kiểm tra động học: {ROBOT_NAME}   (N ngẫu nhiên = {N_RANDOM})")
        print("=" * 96)

        try:
            # ---------------- T1 Home ----------------
            H = robot.SolveFK([0] * 6)
            H_exp = analytic_fk([0] * 6)
            e_p, e_r = pos_err(H, H_exp), rot_err_deg(H, H_exp)
            report("T1", "Mặt bích tại Home", e_p < TOL_POS and e_r < TOL_ROT,
                   f"RoboDK {fmt(H.Pos(), 2)} | bản vẽ {fmt(H_exp.Pos(), 2)} | dP={e_p:.2e} mm dR={e_r:.2e}°")

            # ---------------- T2 FK độc lập ----------------
            special = [
                [90, 0, 0, 0, 0, 0], [-90, 0, 0, 0, 0, 0],
                [0, 45, 0, 0, 0, 0], [0, -60, 0, 0, 0, 0],
                [0, 0, 45, 0, 0, 0], [0, 0, -60, 0, 0, 0],
                [0, 0, 0, 90, 0, 0], [0, 0, 0, 0, 90, 0], [0, 0, 0, 0, -90, 0],
                [0, 0, 0, 0, 0, 90], [0, 0, 0, 90, 45, 90],
                LOWER, UPPER, JOINTS_BUILD,
            ]
            samples = special + [rand_q() for _ in range(N_RANDOM)]
            worst_p = worst_r = 0.0
            worst_q = None
            for q in samples:
                A, B = robot.SolveFK(q), analytic_fk(q)
                p, r = pos_err(A, B), rot_err_deg(A, B)
                if p > worst_p:
                    worst_p, worst_q = p, q
                worst_r = max(worst_r, r)
            report("T2", f"FK RoboDK vs giải tích ({len(samples)} tư thế)",
                   worst_p < TOL_POS and worst_r < TOL_ROT,
                   f"max dP={worst_p:.2e} mm  max dR={worst_r:.2e}°"
                   + ("" if worst_p < TOL_POS else f"  tại q={fmt(worst_q, 1)}"))

            # ---------------- T3 IK hai chiều ----------------
            max_pe = max_re = max_je = 0.0
            n_fail = n_cmp = 0
            for q in samples[len(special):]:
                pose = robot.SolveFK(q)
                qi = robot.SolveIK(pose, q).list()
                if len(qi) < 6:
                    n_fail += 1
                    continue
                P2 = robot.SolveFK(qi[:6])
                max_pe = max(max_pe, pos_err(pose, P2))
                max_re = max(max_re, rot_err_deg(pose, P2))
                if abs(q[4]) > SINGULAR_J5:
                    n_cmp += 1
                    max_je = max(max_je, max(ang_diff(a, b) for a, b in zip(q, qi[:6])))
            ok = n_fail == 0 and max_pe < TOL_POS and max_re < TOL_ROT and max_je < TOL_JNT
            report("T3", "IK hai chiều q->pose->q", ok,
                   f"IK lỗi={n_fail}  max dP={max_pe:.2e} mm dR={max_re:.2e}°  "
                   f"max dq={max_je:.2e}° ({n_cmp} tư thế, |J5|>{SINGULAR_J5}°)")

            # ---------------- T4 IK đa nghiệm ----------------
            counts, bad = [], 0
            for q in samples[len(special):len(special) + 50]:
                pose = robot.SolveFK(q)
                sols = robot.SolveIK_All(pose)
                ncols = sols.size(1) if sols.size(0) >= 6 else 0
                counts.append(ncols)
                for c in range(ncols):
                    qs = [sols[i, c] for i in range(6)]
                    P2 = robot.SolveFK(qs)
                    if pos_err(pose, P2) > TOL_POS or rot_err_deg(pose, P2) > TOL_ROT:
                        bad += 1
            report("T4", "IK đa nghiệm (50 tư thế)", bad == 0 and min(counts) >= 1,
                   f"số nghiệm min/max = {min(counts)}/{max(counts)}  nghiệm sai pose = {bad}")

            # ---------------- T5 Giới hạn khớp ----------------
            lo, hi, _ = robot.JointLimits()
            lo, hi = lo.list()[:6], hi.list()[:6]
            ok = all(abs(a - b) < 1e-6 for a, b in zip(lo + hi, LOWER + UPPER))
            report("T5", "Giới hạn khớp", ok, f"min {fmt(lo, 0)} max {fmt(hi, 0)}")

            # ---------------- T6 Tâm cổ tay ----------------
            max_wc = 0.0
            for q in samples[len(special):len(special) + 100]:
                F = robot.SolveFK(q)
                wc = [F.Pos()[i] - d6 * F.VZ()[i] for i in range(3)]
                q0 = q[:3] + [0, 0, 0]
                F0 = robot.SolveFK(q0)
                wc0 = [F0.Pos()[i] - d6 * F0.VZ()[i] for i in range(3)]
                max_wc = max(max_wc, math.dist(wc, wc0))
            report("T6", "Cổ tay cầu (J4-J6 không dịch tâm)", max_wc < TOL_POS,
                   f"max lệch tâm cổ tay = {max_wc:.2e} mm")

            # ---------------- T7 Trùng CAD ----------------
            links = [RDK.Item(n, robolink.ITEM_TYPE_OBJECT) for n in LINK_NAMES]
            if all(it.Valid() and it.Name() == n for it, n in zip(links, LINK_NAMES)):
                robot.setJoints(JOINTS_BUILD)
                worst, worst_n = 0.0, ""
                for i, it in enumerate(links):
                    A = robot.ObjectLink(i).setParam("BoundingBox")
                    B = it.setParam("BoundingBox")
                    e = max(abs(x - y) for k in ("min", "max") for x, y in zip(A[k], B[k]))
                    if e > worst:
                        worst, worst_n = e, LINK_NAMES[i]
                report("T7", "Hình khâu robot trùng CAD @JOINTS_BUILD", worst < TOL_CAD,
                       f"max lệch bbox = {worst:.3f} mm {('(' + worst_n + ')') if worst_n else ''}")
            else:
                info("T7 bỏ qua", "không tìm thấy đủ khâu CAD BRT_* trong trạm")

            # ---------------- INFO ----------------
            reach_wc = a2 + a3 + math.hypot(a4, d4)
            info("Tầm với lý thuyết (tâm cổ tay)", f"{reach_wc:.1f} mm  (mặt bích: {reach_wc + d6:.1f} mm)")
            info("Payload / Weight", f"{robot.setParam('Payload')} kg / {robot.setParam('Weight')} kg "
                                     f"(kỳ vọng {PAYLOAD_KG:g} / {WEIGHT_KG:g})")
        finally:
            robot.setJoints(q_saved)
            RDK.Render(True)

        n_fail = RESULTS.count(False)
        print("=" * 96)
        print(f"KẾT QUẢ: {len(RESULTS) - n_fail}/{len(RESULTS)} PASS"
              + ("" if n_fail == 0 else f"  –  {n_fail} FAIL"))
        print("=" * 96)
        return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
