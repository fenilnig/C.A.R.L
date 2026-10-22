"""C.A.R.L. parametric CAD (CadQuery). Run: python3 carl_cad.py
Vehicle frame: X = left/right, Y = forward, Z = up. Ground is Z = 0. All mm.
Outputs STEP + STL for every printed part (print orientation) and a full assembly.
"""
import json, math, os
import cadquery as cq

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
os.makedirs(os.path.join(OUT, "parts"), exist_ok=True)

# ---- parameters you can change --------------------------------------------
M3_CLR = 3.4          # M3 clearance hole
M3_TAP = 2.7          # M3 self-tapping / grub hole in PETG
M3_NUT_AF, M3_NUT_H = 5.8, 2.7   # nut trap (5.5 AF nut + clearance)
M6_CLR = 6.5
SHAFT_D = 6.7         # motor shaft 6.6 measured + 0.1  (CHANGE after fit test)
WHEEL_BORE = 12.0     # bore you drill in the Robocraze wheel (CHANGE after measuring)
TRACK_HALF_WHEEL = 104.0
KP_X, KP_Y = 76.0, 118.0        # kingpin position (x for the right side)
AXLE_Y, AXLE_Z = 130.0, 50.0    # front axle
REAR_Y = -130.0
FIT = 0.2             # clearance on printed sliding fits

def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))

def cyl(axis, r, length, c):
    """Cylinder along axis ('X','Y','Z') centred on point c."""
    d = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[axis]
    base = tuple(c[i] - d[i] * length / 2 for i in range(3))
    return cq.Workplane(cq.Plane(origin=base, xDir=(0, 0, 1) if axis != "Z" else (1, 0, 0), normal=d)).circle(r).extrude(length)

def hole_z(solid, pts, d, z0, z1):
    for x, y in pts:
        solid = solid.cut(cyl("Z", d / 2, z1 - z0, (x, y, (z0 + z1) / 2)))
    return solid

def hexpocket_z(solid, pts, z0, z1):
    for x, y in pts:
        p = cq.Workplane("XY").workplane(offset=z0).center(x, y).polygon(6, M3_NUT_AF / math.cos(math.pi / 6)).extrude(z1 - z0)
        solid = solid.cut(p)
    return solid

def mirror_x(s):
    return s.mirror("YZ")

parts = {}      # name -> (solid in assembly pose, print-transform fn, note)

# ---- Lower deck, main (160 x 215 x 4) -------------------------------------
deck = box(-80, 80, -165, 50, 25, 29)
h = [(sx * 75, y) for sx in (-1, 1) for y in (-95, 30)]                     # alu standoffs
h += [(sx * 75, y) for sx in (-1, 1) for y in (-8, 8)]                      # outriggers
h += [(sx * 48, REAR_Y + sy * 25.7) for sx in (-1, 1) for sy in (-1, 1)]    # motor can straps
h += [(sx * 67, REAR_Y + sy * 25.7) for sx in (-1, 1) for sy in (-1, 1)]    # motor gearbox straps
h += [(sx * 76, y) for sx in (-1, 1) for y in (-55, 0)]                     # side skirt tabs
h += [(x, 44) for x in (-30, 0, 30)]                                        # nose splice
deck = hole_z(deck, h, M3_CLR, 24, 30)
parts["01_lower_deck_main"] = (deck, lambda s: s.translate((0, 0, -25)), "4 mm plate, M3 clearance holes. Print flat, 3 walls, 25% infill.")

# ---- Lower deck, nose, with kingpin tabs -----------------------------------
nose = box(-50, 50, 50, 150, 25, 29)
for sx in (-1, 1):
    nose = nose.union(box(min(sx * 45, sx * 80), max(sx * 45, sx * 80), KP_Y - 6, KP_Y + 6, 25, 29))
nose = hole_z(nose, [(x, 56) for x in (-30, 0, 30)] + [(sx * KP_X, KP_Y) for sx in (-1, 1)] + [(h, 88 + d) for h in (-29.0, 41.0) for d in (-14, 14)] + [(-40, 144), (40, 144)], M3_CLR, 24, 30)
for _x, _y in [(h, 88 + d) for h in (-29.0, 41.0) for d in (-14, 14)] + [(-40, 144), (40, 144)]:
    nose = nose.cut(cq.Workplane('XY').workplane(offset=25).center(_x, _y).polygon(6, M3_NUT_AF / math.cos(math.pi / 6)).extrude(M3_NUT_H))
parts["02_lower_deck_nose"] = (nose, lambda s: s.translate((0, 0, -25)), "Nose with integral kingpin tabs (M3 pivot bolt each side).")

splice = hole_z(box(-40, 40, 40, 60, 29, 32), [(x, y) for x in (-30, 0, 30) for y in (44, 56)], M3_CLR, 28, 33)
parts["03_deck_splice_plate"] = (splice, lambda s: s.translate((0, 0, -29)), "Joins main deck to nose with 6 M3 screws, sits on top.")

# ---- Upper deck (damped platform) ------------------------------------------
up = box(-80, 80, -115, 45, 83.5, 87.5)
up = hole_z(up, [(sx * 75, y) for sx in (-1, 1) for y in (-95, 30)], M3_CLR, 83, 88)
XS = 10.15   # MG996R output shaft sits 10.15 mm off the middle of its case; shaft on the pan axis (x=0, y=15)
up = up.cut(box(XS - 20.6, XS + 20.6, 15 - 10.3, 15 + 10.3, 83, 88))        # pan servo slot (case 40.7 x 19.7)
for _dx in (-24.75, 24.75):
    for _dy in (-5, 5):
        up = up.cut(cyl("Z", M3_TAP / 2, 6, (XS + _dx, 15 + _dy, 85.5)))     # ear screws from below
# Arduino UNO footprint holes (verify against the real UNO Q before printing)
# 8 mm posts lift the board clear of the tall connectors and memory chip on its underside
for u, v in [(14.0, 2.5), (15.3, 50.7), (66.1, 7.6), (66.1, 35.5)]:
    px, py = v - 26.67, -110 + u
    up = up.union(cyl("Z", 3.5, 8, (px, py, 87.5 + 4)))
    up = up.cut(cyl("Z", M3_TAP / 2, 9, (px, py, 87.5 + 4.5)))
parts["04_upper_deck"] = (up, lambda s: s.translate((0, 0, -83.5)), "Upper deck on 4 aluminium standoffs + silicone dampers. UNO Q posts (8 mm) use the UNO R3 hole pattern, confirmed from a photo of the board.")

# ---- Outriggers -------------------------------------------------------------
for name, sx in (("L", -1), ("R", 1)):
    o = box(min(sx * 70, sx * 144), max(sx * 70, sx * 144), -12, 12, 17, 25)
    pts = [(sx * 75, -8), (sx * 75, 8)]
    o = hole_z(o, pts, M3_CLR, 16, 26)
    o = hexpocket_z(o, pts, 17, 17 + M3_NUT_H)
    # 4 mm rounded outer end
    o = o.edges("|Z").edges(cq.selectors.BoxSelector((sx * 140, -20, 10), (sx * 150, 20, 30))).fillet(5)
    parts["05_outrigger_" + name] = (o, lambda s: s.translate((0, 0, -17)), "Under deck; screw from top, nut in pocket. Print flat.")

# ---- Motor saddle (lower cradle) and two straps per side -------------------
for name, sx in (("L", -1), ("R", 1)):
    xa, xb = sorted((sx * 38, sx * 77))
    CLAMP_GAP = 0.6   # Rev G3: saddle top sits 0.6 mm below the motor centre line, so tightening the strap bolts really clamps the motor
    blk = box(xa, xb, REAR_Y - 29, REAR_Y + 29, 29, 50 - CLAMP_GAP)
    can_x0, can_x1 = sorted((sx * 39, sx * 57))
    gb_x0, gb_x1 = sorted((sx * 57, sx * 77))
    blk = blk.cut(cyl("X", 15.9 + FIT / 2, abs(can_x1 - can_x0), ((can_x0 + can_x1) / 2, REAR_Y, 50)))
    blk = blk.cut(cyl("X", 19.2 + FIT / 2, abs(gb_x1 - gb_x0), ((gb_x0 + gb_x1) / 2, REAR_Y, 50)))
    blk = blk.cut(cyl("X", 19.3 + FIT / 2, 2, (sx * 57, REAR_Y, 50)))     # blend step
    pts = [(sx * x, REAR_Y + sy * 25.7) for x in (48, 67) for sy in (-1, 1)]
    blk = hole_z(blk, pts, M3_CLR, 28, 51)
    parts["06_motor_saddle_" + name] = (blk, lambda s: s.translate((0, 0, -29)), "Cradle for 31.8 mm can + 38.4 mm gearbox. Bolts through deck. 0.6 mm clamp gap to the strap: tighten the strap bolts until snug (the strap squeezes the motor).")
    for nm, (x0, x1, r, cx) in {"can": (39, 57, 15.9, 48), "gearbox": (58, 76, 19.2, 67)}.items():
        xs = sorted((sx * x0 if nm == "gearbox" else sx * 40, sx * x1 if nm == "gearbox" else sx * 56))
        ro = r + FIT / 2 + 4
        st = box(xs[0], xs[1], REAR_Y - 29, REAR_Y + 29, 50, 54)
        arc = cyl("X", ro, xs[1] - xs[0], ((xs[0] + xs[1]) / 2, REAR_Y, 50)).intersect(box(xs[0] - 1, xs[1] + 1, REAR_Y - 40, REAR_Y + 40, 50, 80))
        st = st.union(arc).cut(cyl("X", r + FIT / 2, xs[1] - xs[0] + 2, ((xs[0] + xs[1]) / 2, REAR_Y, 50)))
        st = hole_z(st, [(sx * cx, REAR_Y + sy * 25.7) for sy in (-1, 1)], M3_CLR, 49, 55)
        parts["07_motor_strap_%s_%s" % (nm, name)] = (st, lambda s: s.mirror("XY").translate((0, 0, 54 + 0)), "Clamp strap. Print arch-up as exported if supports are off.")

# ---- Rear hub adapter (motor shaft -> wheel) --------------------------------
for name, sx in (("L", -1), ("R", 1)):
    xc = sx * TRACK_HALF_WHEEL
    ad = cyl("X", WHEEL_BORE / 2, 24, (xc, REAR_Y, 50))
    ad = ad.cut(cyl("X", SHAFT_D / 2, 26, (xc, REAR_Y, 50)))
    # two radial M3 grub-screw holes, 90 deg apart, 8 mm and 16 mm from the inner end
    for off, ang in ((6, 90), (16, 0)):
        x = sx * (92 + off) if sx > 0 else sx * (92 + off)
        d = (0, math.cos(math.radians(ang)), math.sin(math.radians(ang)))
        # cylinder from the axis out through the wall along direction d
        pl = cq.Plane(origin=(x, REAR_Y, 50), xDir=(1, 0, 0), normal=d)
        ad = ad.cut(cq.Workplane(pl).circle(M3_TAP / 2).extrude(WHEEL_BORE))
    parts["08_rear_hub_adapter_" + name] = (ad, lambda s, sx=sx: s.translate((-sx * TRACK_HALF_WHEEL, -REAR_Y, 0)).rotate((0, 0, 0), (0, 1, 0), 90), "Glue into reamed wheel bore, shaft through, lock with 2 M3 grub screws.")

# ---- Front: knuckle, tie rod, front wheel bush ----------------------------
def ack_tip(sx):
    dx, dy = -sx * KP_X, REAR_Y - KP_Y          # from kingpin to rear axle centre
    n = math.hypot(dx, dy)
    L = 14.5
    return (sx * KP_X + dx / n * L, KP_Y + dy / n * L)

for name, sx in (("L", -1), ("R", 1)):
    x0, x1 = sorted((sx * 70, sx * 82))
    k = box(x0, x1, 110, 140, 29, 75)
    tip = ack_tip(sx)
    arm = (cq.Workplane("XY").workplane(offset=71)
           .polyline([(sx * 70, 110), (sx * 82, 110), (tip[0] + sx * 6, tip[1]), (tip[0], tip[1] - 6), (tip[0] - sx * 6, tip[1])]).close().extrude(4))
    k = k.union(arm)
    k = k.cut(cyl("Z", M3_CLR / 2, 50, (sx * KP_X, KP_Y, 52)))                     # kingpin hole
    k = k.cut(cyl("Z", M3_CLR / 2, 10, (tip[0], tip[1], 73)))                      # tie rod pivot
    k = k.cut(cyl("X", M6_CLR / 2, 14, (sx * 76, AXLE_Y, AXLE_Z)))                 # M6 axle bolt
    parts["09_steering_knuckle_" + name] = (k, lambda s, sx=sx: s.rotate((sx * 76, 0, 0), (sx * 76, 1, 0), -sx * 90 if sx > 0 else 90).translate((-sx * 76, 0, 0)), "Kingpin M3x65 (or cut M3 rod) through hole, M6x65 axle bolt, arm angled for Ackermann.")
    bush = cyl("X", WHEEL_BORE / 2, 38, (sx * 104, AXLE_Y, AXLE_Z)).cut(cyl("X", 6.6 / 2, 40, (sx * 104, AXLE_Y, AXLE_Z)))
    parts["10_front_wheel_bush_" + name] = (bush, lambda s, sx=sx: s.translate((-sx * 104, -AXLE_Y, 0)).rotate((0, 0, 0), (0, 1, 0), 90), "Free-spinning bush on the fixed M6 axle bolt. Glue into reamed wheel bore.")

tipR = ack_tip(1)
# Rev G3: the servo-horn pin sits in a SLOT (long in Y), because the horn pin moves on a circle while the
# 4-bar tie rod moves on a slightly different path (up to 1.8 mm apart at full lock) - a round hole would bind.
tie = box(-tipR[0] - 6, tipR[0] + 6, tipR[1] - 6, tipR[1] + 6, 75, 79)
tie = hole_z(tie, [(-tipR[0], tipR[1]), (tipR[0], tipR[1])], M3_CLR, 74, 80)
tie = tie.cut(box(-M3_CLR / 2, M3_CLR / 2, tipR[1] - 3.7, tipR[1] + 3.7, 74, 80))   # slot 3.4 x 7.4 mm
parts["11_tie_rod"] = (tie, lambda s: s.translate((0, -tipR[1], -75)), "Centre SLOT (long in Y) takes the servo-horn pin so the linkage cannot bind; ends bolt to knuckle arms (M3, loose).")

# ---- Side skirts with embossed name ---------------------------------------
for name, sx in (("L", -1), ("R", 1)):
    x0, x1 = sorted((sx * 80, sx * 83))
    sk = box(x0, x1, -75, 40, 29, 47)
    for y in (-55, 0):
        tx0, tx1 = sorted((sx * 72, sx * 83))
        tab = box(tx0, tx1, y - 5, y + 5, 29, 32)
        tab = hole_z(tab, [(sx * 76, y)], M3_CLR, 28, 33)
        sk = sk.union(tab)
    face_x = sx * 83
    plane = cq.Plane(origin=(face_x, -17.5, 38), xDir=(0, sx * 1 if sx > 0 else -1, 0), normal=(sx, 0, 0))
    try:
        txt = cq.Workplane(plane).text("C.A.R.L.", 9, 0.6, combine=False, halign="center", valign="center", kind="bold")
        sk = sk.union(txt)
    except Exception as e:
        print("text emboss skipped:", e)
    parts["12_side_skirt_" + name] = (sk, lambda s, sx=sx: s.translate((-sx * 83, 0, -29)).rotate((0, 0, 0), (0, 1, 0), -sx * 90), "Raised C.A.R.L. lettering 0.6 mm. Print lying on its inner face.")

# ---- Rev G additions: steering-servo pillars, front bumper (ToF), pan/tilt gimbal ----
def ontobed(s):
    bb = s.val().BoundingBox()
    return s.translate((-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))

# Steering servo (MG996R, upright).  Output shaft must sit at x=0 (tie-rod pin), so the case is offset by XS_STEER.
SV_Y = 88.0
XS_STEER = 10.15
STEER_FOOT_HOLES = []
for name, (xa, xb, xe, holes) in {"L": (-23.2, XS_STEER - 20.6, XS_STEER - 24.75, [-29.0]), "R": (XS_STEER + 20.6, 41.0, XS_STEER + 24.75, [41.0])}.items():
    pil = box(xa, xb, SV_Y - 9, SV_Y + 9, 29, 56)                      # pillar, ear rest at z=56
    fx0, fx1 = (-34.0, xb) if name == "L" else (xa, 45.0)
    pil = pil.union(box(fx0, fx1, SV_Y - 18, SV_Y + 18, 29, 33))        # foot bolted to nose
    for dy in (-5, 5):
        pil = pil.cut(cyl("Z", M3_TAP / 2, 14, (xe, SV_Y + dy, 56 - 6.5)))   # tapped from the top, 14 deep
    pil = hole_z(pil, [(h, SV_Y - 14) for h in holes] + [(h, SV_Y + 14) for h in holes], M3_CLR, 28, 34)
    STEER_FOOT_HOLES += [(h, SV_Y + d) for h in holes for d in (-14, 14)]
    parts["13_steer_servo_pillar_" + name] = (pil, lambda s: s.translate((0, 0, -29)), "Holds MG996R upright with its output shaft at x=0. Ears screw into the top (M3x10 self-tapping). Foot bolts to the nose.")

# Front bumper with VL53L0X window
bump = box(-50, 50, 150, 154, 29, 49.5)
bump = bump.union(box(-45, 45, 138, 154, 29, 32))
bump = bump.cut(box(-5, 5, 149, 155, 40.5, 46.5))                        # sensor window 10x6
bump = bump.cut(box(-12.9, 12.9, 152.5, 155, 38, 49))                    # board pocket on front face (25.8 x 11, 1.5 deep)
bump = hole_z(bump, [(-40, 144), (40, 144)], M3_CLR, 28, 33)
for x in (-9.5, 9.5):
    bump = bump.cut(cyl("Y", 1.1, 8, (x, 151, 43.5)))                    # M2 board screws (ASSUMED spacing, check board)
bump = bump.cut(box(-12, 12, 138, 150, 29, 32)) if False else bump
parts["14_front_bumper_tof"] = (bump, lambda s: ontobed(s.translate((0, 0, -29))), "Front bumper; VL53L0X board sits in the front pocket, window 10x6 mm. Check the M2 hole spacing on your board.")
BUMP_HOLES = [(-40, 144), (40, 144)]

# ---- Pan / tilt gimbal, centred on pan axis (0, GY) ------------------------
GY = 15.0
PAN_Z0, PAN_T = 107.0, 3.0                      # pan plate underside, thickness
TILT_Z = 136.5                                  # tilt axis height
# pan plate (flat, 110 x 110) with horn hub underneath
pan = box(-55, 55, GY - 55, GY + 55, PAN_Z0, PAN_Z0 + PAN_T)
pan = pan.union(cyl("Z", 12, 4, (0, GY, PAN_Z0 - 2)))                       # hub spacer onto the servo horn
horn_pts = [(8 * math.cos(a), GY + 8 * math.sin(a)) for a in (0, math.pi / 2, math.pi, 3 * math.pi / 2)]
pan = hole_z(pan, horn_pts, M3_CLR, PAN_Z0 - 5, PAN_Z0 + PAN_T + 1)
pan = pan.cut(cyl("Z", 2.6, 10, (0, GY, PAN_Z0)))                            # centre horn screw
up_pts = [(sx * 48.75, GY + d) for sx in (-1, 1) for d in (-15, 15)]
pan = hole_z(pan, up_pts, M3_CLR, PAN_Z0 - 1, PAN_Z0 + PAN_T + 1)
parts["15_pan_plate"] = (pan, lambda s: s.translate((0, -GY, -(PAN_Z0 - 4))), "Bolts to the pan servo horn (4 x M3 on a 16 mm circle). Print flat, hub side up.")

# tilt uprights (L carries the servo, R carries the pivot bolt)
SV_WIN_Y = GY + 10.15      # window centre: the servo spline is 10.15 mm off the case centre
for name, sx in (("L", -1), ("R", 1)):
    xo0, xo1 = sorted((sx * 50.5, sx * 53.5))
    u = box(xo0, xo1, GY - 35, GY + 41, PAN_Z0 + PAN_T, TILT_Z + 15)
    xf0, xf1 = sorted((sx * 44, sx * 53.5))
    u = u.union(box(xf0, xf1, GY - 20, GY + 50, PAN_Z0 + PAN_T, PAN_Z0 + PAN_T + 6))   # foot
    u = u.cut(cyl("Z", M3_TAP / 2, 7, (sx * 48.75, GY - 15, PAN_Z0 + PAN_T + 3)))
    u = u.cut(cyl("Z", M3_TAP / 2, 7, (sx * 48.75, GY + 15, PAN_Z0 + PAN_T + 3)))
    if sx < 0:
        u = u.cut(box(xo0 - 1, xo1 + 1, SV_WIN_Y - 20.7, SV_WIN_Y + 20.7, TILT_Z - 10.2, TILT_Z + 10.2))
        for dy in (-24.75, 24.75):
            for dz in (-5, 5):
                u = u.cut(cyl("X", M3_TAP / 2, 5, (sx * 52, SV_WIN_Y + dy, TILT_Z + dz)))
        u = u.cut(cyl("X", 1.0, 5, (sx * 52, GY, TILT_Z))) if False else u
    else:
        u = u.cut(cyl("X", M3_CLR / 2, 5, (sx * 52, GY, TILT_Z)))
    parts["16_tilt_upright_" + name] = (u, lambda s, sx=sx: ontobed(s.rotate((0, 0, 0), (0, 1, 0), 90 if sx > 0 else -90)), "Bolts to the pan plate (2 x M3x10 from below). Printed lying on its outer face, 4 walls.")

# tilt cradle: platform + two end flanges
cr = box(-39, 39, GY - 35, GY + 35, TILT_Z - 1.5, TILT_Z + 1.5)
for sx in (-1, 1):
    x0, x1 = sorted((sx * 36, sx * 39))
    fl = box(x0, x1, GY - 18, GY + 18, TILT_Z - 14, TILT_Z + 14)
    cr = cr.union(fl)
cr = cr.cut(cyl("Z", 3.3, 6, (0, GY, TILT_Z)))                                      # 1/4"-20 ball-head screw (6.6 mm)
cr = hole_z(cr, [(x, GY + y) for x in (-25, 25) for y in (-25, 25)], M3_CLR, TILT_Z - 2, TILT_Z + 2)  # spare accessory holes
ho = [(0, GY + 8 * math.cos(a), TILT_Z + 8 * math.sin(a)) for a in (0, math.pi / 2, math.pi, 3 * math.pi / 2)]
for x_, y_, z_ in ho:
    cr = cr.cut(cyl("X", M3_CLR / 2, 8, (-37.5, y_, z_)))                            # servo horn screws (left flange)
cr = cr.cut(cyl("X", 2.6, 8, (-37.5, GY, TILT_Z)))
cr = cr.cut(cyl("X", M3_CLR / 2, 8, (37.5, GY, TILT_Z)))                              # pivot bolt (right flange)
parts["17_tilt_cradle"] = (cr, lambda s: s.translate((0, -GY, -(TILT_Z - 14))), "Camera platform. Left flange screws to the tilt servo horn, right flange turns on an M3 pivot bolt. Ball head screws to the centre hole.")

# pivot bush between right flange and right upright (11.5 mm)
bush = cq.Workplane("YZ").circle(4).circle(M3_CLR / 2).extrude(11.5)
parts["18_tilt_pivot_bush"] = (bush.translate((39, GY, TILT_Z)), lambda s: ontobed(s.rotate((0, 0, 0), (0, 1, 0), 90)), "Spacer on the M3 pivot bolt. Print 2 spares.")

# ---- Simplified solids for everything else ----------------------------------
P = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "parts.json")))
REPLACED = ("Lower deck", "Wheel hub adapter", "Motor saddle", "Steering knuckle", "Anti-tip outrigger", "Upper deck", "Styling: side skirt", "Tie rod", "Wheel hub axle bolt", "Kingpin bracket", "Steering arm", "Front bumper", "Pan plate", "Tilt cradle upright", "Cradle plate", "MG996R tilt servo", "Pan bracket", "Tilt platform", "MG90S", "Tilt upright", "Tilt cradle", "Tilt pivot bush", "Steering servo pillar", "Motor strap", "Deck splice plate", "UNO Q post")
simple = []
for q in P:
    if q["n"].startswith(REPLACED):
        continue
    lo, hi = q["lo"], q["hi"]
    s = box(lo[0], hi[0], lo[2], hi[2], lo[1], hi[1])
    if q["k"] == "c":
        ax = {"x": "X", "y": "Z", "z": "Y"}[q["ax"]]
        s = cyl(ax, q["r"], q["len"], (q["p"][0], q["p"][2], q["p"][1]))
    simple.append((q["n"], s, q["st"]))

# real solids for the tilt servo and ToF check (not printed)
simple.append(("MG996R tilt servo (Rev G, outside left upright)", box(-80.5, -44, SV_WIN_Y - 20.35, SV_WIN_Y + 20.35, TILT_Z - 9.85, TILT_Z + 9.85), "f"))
simple.append(("MG996R tilt servo ears", box(-53.5, -50.5 - 0.0, SV_WIN_Y - 26.75, SV_WIN_Y + 26.75, TILT_Z - 5, TILT_Z + 5), "f"))

def export_all():
    sc = cq.Assembly(name="CARL")
    for n, (s, tf, note) in parts.items():
        solid = s.val()
        printed = tf(s)
        _bb = printed.val().BoundingBox()
        printed = printed.translate((0, 0, -_bb.zmin))      # every STL sits exactly on the bed (z = 0)
        cq.exporters.export(printed, os.path.join(OUT, "parts", n + ".step"))
        cq.exporters.export(printed, os.path.join(OUT, "parts", n + ".stl"), tolerance=0.02, angularTolerance=0.1)
        sc.add(s, name=n, color=cq.Color(0.15, 0.15, 0.17))
    for i, (n, s, st) in enumerate(simple):
        sc.add(s, name="%02d_%s" % (i, "".join(c if c.isalnum() else "_" for c in n)[:40]), color=cq.Color(0.6, 0.6, 0.65))
    sc.save(os.path.join(OUT, "CARL_assembly.step"))
    allp = cq.Workplane("XY").newObject([cq.Compound.makeCompound([p[0].val() for p in parts.values()])])
    cq.exporters.export(allp, os.path.join(OUT, "CARL_printed_parts_assembled.stl"), tolerance=0.05, angularTolerance=0.2)
    return sc

if __name__ == "__main__":
    export_all()
    print("parts:", len(parts), "simplified:", len(simple))
    for n, (s, tf, note) in parts.items():
        v = s.val()
        bb = tf(s).val().BoundingBox()
        print(f"{n:34s} vol {v.Volume()/1000:7.1f} cm3  valid={v.isValid()}  print bbox {bb.xlen:.0f}x{bb.ylen:.0f}x{bb.zlen:.0f}")
