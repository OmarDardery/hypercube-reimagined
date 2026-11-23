"""Build the three printable parts.  Usage:  python3 cad/build.py
Edit Params in gantry.py (or pass overrides below) and re-run."""
import json, os, sys
sys.path.insert(0, os.path.dirname(__file__))
from gantry import Params, build_joiner, build_carrier, build_corner_joint, load_original_carrier, mirror_x, to_trimesh

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "stl")
SRC = os.path.join(ROOT, "cad", "source", "Hypercube300V2_M12_Sensorless_Carrier.stl")


def drop(M):
    b = M.bounding_box()
    return M.translate([-(b[0] + b[3]) / 2, -(b[1] + b[4]) / 2, -b[2]])


def build(P=None):
    P = P or Params()
    os.makedirs(OUT, exist_ok=True)
    JL, jl = build_joiner(P, "left")
    JRl, jr = build_joiner(P, "right")
    JR = mirror_x(JRl)                       # right joiner in its own local frame (inboard = -X)
    orig = load_original_carrier(SRC)
    C, ci = build_carrier(P, orig)

    cap = ci["caps"]["left"]                 # left and right caps are the same part (flipped): print 2
    joint = build_corner_joint(P)            # same part on all 4 frame corners: print 4
    parts = {"xy_joiner_left": JL, "xy_joiner_right": JR, "hotend_carrier_mgn9h": C, "belt_clamp_cap_x2": cap,
             "frame_corner_joint_x4": joint}
    # print orientation = installed orientation for all three parts:
    #  - belt tension and idler loads then run along the layers, never across them
    #  - the joiners' MGN12H face and the carrier's tube block print flat on the bed
    printed = {k: drop(M) for k, M in parts.items()}
    printed["belt_clamp_cap_x2"] = drop(cap.rotate([-90, 0, 0]))   # back face on the bed, toothed face up
    printed["frame_corner_joint_x4"] = drop(joint.rotate([90, 0, 0]))  # one plate flat on the bed, keys up
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
