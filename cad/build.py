"""Build the printable parts.  Usage:  python3 cad/build.py
Edit Params in gantry.py (or pass overrides below) and re-run."""
import json, os, sys
sys.path.insert(0, os.path.dirname(__file__))
from gantry import (Params, build_joiner, build_carrier, build_pair_joint, place_pair_joint, load_original_carrier,
                    mirror_x, to_trimesh)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "stl")
SRC = os.path.join(ROOT, "cad", "source", "Hypercube300V2_M12_Sensorless_Carrier.stl")


def drop(M):
    b = M.bounding_box()
    return M.translate([-(b[0] + b[3]) / 2, -(b[1] + b[4]) / 2, -b[2]])


def rot(M, axis, deg):
    v = [0, 0, 0]; v["xyz".index(axis)] = deg
    return M.rotate(v)


def build(P=None):
    P = P or Params()
    os.makedirs(OUT, exist_ok=True)
    JL, jl = build_joiner(P, "left")
    JRl, jr = build_joiner(P, "right")
    JR = mirror_x(JRl)                       # right joiner in its own local frame (inboard = -X)
    orig = load_original_carrier(SRC)
    C, ci = build_carrier(P, orig)

    cap = ci["caps"]["left"]                 # left and right caps are the same part (flipped): print 2
    joint = build_pair_joint(P)              # built in print orientation; same part at all 4 post positions: print 4
    parts = {"xy_joiner_left": JL, "xy_joiner_right": JR, "hotend_carrier_mgn9h": C, "belt_clamp_cap_x2": cap,
             "frame_pair_joint_x4": place_pair_joint(P, joint)}          # installed positions
    # print orientation:
    #  - joiners: on their OUTBOARD face (the carriage-mounting face), so that face is dead flat and every
    #    slice along the print lies inside the one below it
    #  - carrier: upright, as installed
    #  - clamp cap: back face on the bed, toothed face up
    #  - pair joint: plate on the bed, collar up (as built)
    printed = {"xy_joiner_left": drop(rot(JL, "y", -90)),
               "xy_joiner_right": drop(rot(JR, "y", 90)),
               "hotend_carrier_mgn9h": drop(C),
               "belt_clamp_cap_x2": drop(cap.rotate([-90, 0, 0])),
               "frame_pair_joint_x4": drop(joint)}
    info = {}
    for k, M in parts.items():
        t = to_trimesh(M)
        t.export(os.path.join(OUT, f"{k}_installed.stl"))
        tp = to_trimesh(printed[k])
        tp.export(os.path.join(OUT, f"{k}.stl"))
        info[k] = dict(volume_cm3=round(M.volume() / 1000, 1), watertight=bool(tp.is_watertight),
                       genus=M.genus(), extents_installed=[round(v, 1) for v in t.extents],
                       extents_print=[round(v, 1) for v in tp.extents])
    info["params"] = {k: getattr(P, k) for k in P.__dataclass_fields__}
    info["belt_levels"] = dict(lower=P.zL, upper=P.zU)
    info["left_idlers"] = jl["idlers"]; info["right_idlers_leftframe"] = jr["idlers"]
    info["carrier_belts"] = ci["belts"]
    with open(os.path.join(OUT, "build_info.json"), "w") as f:
        json.dump(info, f, indent=1, default=float)
    return P, parts, info


if __name__ == "__main__":
    P, parts, info = build()
    for k in parts:
        print(k, info[k])
