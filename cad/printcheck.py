"""Slicer-style printability check.

For every layer (0.2 mm) the part is rasterised (0.25 mm pixels).  Material in a
layer with no material under it (previous layer, grown by one pixel ~ 50 deg
overhang) is "unsupported".  Each unsupported pixel is then tested for a straight
bridge: some direction where, going both ways and staying inside the current
layer, it reaches supported material within MAX_BRIDGE mm.  Pixels that fail are
reported as NEEDS SUPPORT, split by whether a support could stand on the bed.

Usage: python3 cad/printcheck.py stl/part.stl [...]
"""
import sys
import numpy as np
import trimesh
from PIL import Image, ImageDraw
from scipy import ndimage

LAYER = 0.2
PIX = 0.25
MAX_BRIDGE = 25.0
DIRS = np.deg2rad(np.arange(0, 180, 15))


def layer_masks(t):
    lo, hi = t.bounds
    W = int(np.ceil((hi[0] - lo[0]) / PIX)) + 4
    H = int(np.ceil((hi[1] - lo[1]) / PIX)) + 4
    zs = np.arange(lo[2] + LAYER / 2, hi[2], LAYER)
    secs = t.section_multiplane(plane_origin=[0, 0, 0], plane_normal=[0, 0, 1], heights=list(zs))
    masks = []
    for s in secs:
        img = Image.new("1", (W, H), 0)
        if s is not None:
            d = ImageDraw.Draw(img)
            for poly in s.polygons_full:
                def px(c):
                    c = np.asarray(c)
                    return [((x - lo[0]) / PIX + 2, (y - lo[1]) / PIX + 2) for x, y in c]
                d.polygon(px(poly.exterior.coords), fill=1)
                for ring in poly.interiors:
                    d.polygon(px(ring.coords), fill=0)
        masks.append(np.array(img, dtype=bool))
    return zs, masks, lo


def bridgeable(cur, sup, ys, xs):
    """For unsupported pixels (ys, xs): True if some direction bridges to support both ways."""
    n = int(MAX_BRIDGE / PIX)
    ok = np.zeros(len(ys), bool)
    H, W = cur.shape
    for th in DIRS:
        dx, dy = np.cos(th), np.sin(th)
        good = np.ones(len(ys), bool)
        span = np.zeros(len(ys))
        for sgn in (1, -1):
            hit = np.zeros(len(ys), bool)
            alive = np.ones(len(ys), bool)
            dist = np.full(len(ys), np.inf)
            for k in range(1, n + 1):
                yy = np.rint(ys + sgn * dy * k).astype(int)
                xx = np.rint(xs + sgn * dx * k).astype(int)
                inside = (yy >= 0) & (yy < H) & (xx >= 0) & (xx < W)
                yy = np.clip(yy, 0, H - 1); xx = np.clip(xx, 0, W - 1)
                s_ = inside & sup[yy, xx] & alive & ~hit
                dist[s_] = k
                hit |= s_
                alive &= inside & cur[yy, xx]
                if not (alive & ~hit).any():
                    break
            good &= hit
            span += np.where(hit, dist, np.inf)
        ok |= good & (span * PIX <= MAX_BRIDGE)
        if ok.all():
            break
    return ok


def check(path, verbose=True):
    t = trimesh.load(path)
    zs, masks, lo = layer_masks(t)
    need_bed, need_model, bridged, regions = 0.0, 0.0, 0.0, []
    below_any = np.zeros_like(masks[0])
    for i in range(1, len(masks)):
        cur, prev = masks[i], masks[i - 1]
        below_any |= prev
        sup = ndimage.binary_dilation(prev, iterations=1)
        uns = cur & ~sup
        if not uns.any():
            continue
        ys, xs = np.nonzero(uns)
        ok = bridgeable(cur, sup, ys.astype(float), xs.astype(float))
        bridged += ok.sum() * PIX * PIX
        bad = np.zeros_like(uns); bad[ys[~ok], xs[~ok]] = True
        if bad.any():
            lab, nlab = ndimage.label(bad)
            for k in range(1, nlab + 1):
                m = lab == k
                area = m.sum() * PIX * PIX
                if area < 0.5:
                    continue
                on_model = (m & below_any).any()
                if on_model: need_model += area
                else: need_bed += area
                cy, cx = ndimage.center_of_mass(m)
                regions.append((round(zs[i], 1), round(cx * PIX + lo[0] - 2 * PIX, 1),
                                round(cy * PIX + lo[1] - 2 * PIX, 1), round(area, 1), on_model))
    res = dict(part=path.split("/")[-1], height=round(t.extents[2], 1),
               needs_support_from_bed_mm2=round(need_bed, 1), needs_support_on_model_mm2=round(need_model, 1),
               bridged_mm2=round(bridged, 1),
               bed_contact_mm2=round(masks[0].sum() * PIX * PIX, 1))
    if verbose:
        print(res)
        # merge regions on nearby xy for a short listing
        agg = {}
        for z, x, y, a, om in regions:
            k = (round(x / 6) * 6, round(y / 6) * 6, om)
            v = agg.setdefault(k, [0.0, z, z]); v[0] += a; v[1] = min(v[1], z); v[2] = max(v[2], z)
        for (x, y, om), (a, z0, z1) in sorted(agg.items(), key=lambda kv: -kv[1][0])[:10]:
            print(f"   needs support near x={x:6.1f} y={y:6.1f} z={z0:5.1f}..{z1:5.1f}  {a:7.1f} mm2  "
                  f"{'(on part)' if om else '(from bed)'}")
    return res


if __name__ == "__main__":
    for p in sys.argv[1:]:
        check(p)
