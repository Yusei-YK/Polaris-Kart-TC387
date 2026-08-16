"""GOTO staging primitive - offline geometry/sign verification (scratch script).

No TriCore toolchain on this machine, and a single sign error sends the car the
wrong way. This replays the kart_motion.c GOTO state machine on a bicycle model
using the same field-calibrated numbers, to check:
  1. signs form negative feedback under the odom convention (dx=-sin*ds, dy=+cos*ds)
  2. the phases converge from arbitrary start poses
  3. final pos/heading error fits the +-0.5m / +-10deg handover budget

Field calibration (from kart_motion.h comments):
  R*|delta| ~ 1480 mm*count  =>  curvature c = (delta - OFS)/1480  (1/m)
  steering rate ~ 1800 count/s, stiction deadband ~ 63 count
  duty -> speed  v ~ 0.00046*duty - 0.10 (m/s)
Output is ASCII only (Windows console is GBK, the source is UTF-8).
"""
import math

DEG = math.pi / 180.0

OFS        = -25.0
KP         = 25.0
STEER_MAX  = 950.0
LEAD       = 2.5
LD         = 1.20
TURN_TOL   = 25.0
ARRIVE     = 0.50
AXIS_STOP  = 0.05
REV_BEAR   = 120.0
REV_EXIT   = 90.0
REV_DIST   = 1.50
MAX_DIST   = 25.0
MAX_TICKS  = 6000

REV_YAW_SIGN = -1.0
DUTY_GOTO     = 2200
DUTY_GOTO_REV = -2000

STEER_RATE = 1800.0
DEADBAND   = 63.0
DT         = 0.010


def duty_to_v(duty):
    return 0.00046 * abs(duty) - 0.10


def get_angle(nx, ny, ax, ay):
    """kart_calc.c get_angle: -atan2(dx, dy) in degrees."""
    return -math.degrees(math.atan2(ax - nx, ay - ny))


def rel(now, aim):
    d = aim - now
    while d < -180.0:
        d += 360.0
    while d > 180.0:
        d -= 360.0
    return d


class Car(object):
    def __init__(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw
        self.delta = 0.0
        self.dist = 0.0

    def step(self, cmd, duty):
        err = cmd - self.delta
        if abs(err) > DEADBAND:
            mx = STEER_RATE * DT
            self.delta += max(-mx, min(mx, err))
        v = duty_to_v(duty)
        ds = v * DT * (1.0 if duty > 0 else -1.0)
        curv = (self.delta - OFS) / 1480.0
        self.yaw = rel(0.0, self.yaw + math.degrees(curv * ds))
        rad = self.yaw * DEG
        self.x += -math.sin(rad) * ds
        self.y += math.cos(rad) * ds
        self.dist += abs(ds)


def steer(bear, reverse, sign=None):
    d = KP * bear
    if reverse:
        d *= (REV_YAW_SIGN if sign is None else sign)
    return max(-STEER_MAX, min(STEER_MAX, d))


def run(x0, y0, yaw0, tx, ty, tyaw, variable_lead='now', rev_en=True,
        rev_sign=None, lead0=LEAD, ld=LD, corridor=None, replan=False,
        clamp_aim=True, yaw_gate=None, arrive=ARRIVE, ld_gain=0.0,
        lat_gain=3.0, gap_min=3.0,
        axis_law='pursuit', lat_k=1.0, max_ang=60.0, v_ref=0.91,
        trace=False):
    """variable_lead: push the staging point back at start when s0 > -lead0
    replan:          also push it back mid-run when the car crosses the staging
                     plane misaligned (orbit repair instead of committing)
    corridor:        lateral tolerance required to hand over to AXIS
                     (None = current C code: no lateral/heading test at all)"""
    car = Car(x0, y0, yaw0)
    rad = tyaw * DEG
    fx, fy = -math.sin(rad), math.cos(rad)          # axis forward unit vector
    rx, ry = math.cos(rad), math.sin(rad)           # axis right unit vector

    def axis_s():
        return fx * (car.x - tx) + fy * (car.y - ty)

    def axis_lat():
        return rx * (car.x - tx) + ry * (car.y - ty)

    st = {}

    def set_lead(lead):
        st['lead'] = lead
        st['lx'] = tx - fx * lead
        st['ly'] = ty - fy * lead
        st['lead_s'] = -lead
        st['behind'] = False

    lead = lead0
    s0, lat0 = axis_s(), axis_lat()
    if variable_lead == 'off':
        # Fixed lead-in, no push-back at all. Scored best with pure pursuit.
        pass
    elif variable_lead == 'past':
        # Push back only when the car has actually overshot the target (s0 > 0).
        if s0 > 0.0:
            lead += s0
    elif variable_lead == 'now':
        # Current C code: push the staging point back whenever the car is not
        # already at least lead0 behind it.
        if s0 > -lead0:
            lead += (s0 + lead0)
    elif variable_lead == 'auto':
        # The lead point exists ONLY to guarantee runway for the AXIS phase.
        # Every other mode places it at a FIXED s=-lead0, which for a car that
        # is already behind the target (say s0=-4, lead0=6) sits 2m BEHIND the
        # car - so the car loops backwards to fetch runway it already has, and
        # burns the 25m budget. That single mistake is the entire residual
        # failure set (all of them are s=-4/-6, lat~0, i.e. the EASY poses).
        #
        # Correct rule: work out the runway needed, then
        #   enough runway already (-s0 >= need)  -> put the lead point AT the car
        #                                          => DRIVE ends at once, AXIS
        #                                             starts from here;
        #   not enough (car close to / past it)  -> lead point at s=-need, the
        #                                          car does have to loop back.
        need = max(lead0, lat_gain * abs(lat0))
        lead = -s0 if (-s0 >= need) else need
    elif variable_lead == 'runway':
        # Runway needed to merge |lat| sideways at max curvature:
        #   lat ~ L^2/(2R)  =>  L ~ sqrt(2*R*lat);  use a linear over-estimate.
        need = max(lead0, lat_gain * abs(lat0))
        # The pursuit target must also be far enough AHEAD of the car along the
        # axis, else it sits beside the car and the car just orbits it (that is
        # the s=-6/lat=2 failure cluster).
        if s0 > -need - gap_min:
            need = -s0 + gap_min
        lead = need
    set_lead(lead)

    if (math.hypot(tx - car.x, ty - car.y) <= ARRIVE and
            abs(rel(car.yaw, tyaw)) <= TURN_TOL):
        return ('DONE_STILL', 0.0, abs(rel(car.yaw, tyaw)), 0.0, 0.0)

    bear = rel(car.yaw, get_angle(car.x, car.y, st['lx'], st['ly']))
    if rev_en and abs(bear) >= REV_BEAR:
        phase, rev_d0 = 'REV', car.dist
    elif abs(bear) <= TURN_TOL:
        phase, rev_d0 = 'DRIVE', 0.0
    else:
        phase, rev_d0 = 'TURN', 0.0

    d0 = car.dist
    axis_entry_yaw_err = 0.0
    nreplan = 0
    for tick in range(MAX_TICKS):
        if car.dist - d0 >= MAX_DIST:
            return ('FAULT_DIST', math.hypot(tx - car.x, ty - car.y),
                    abs(rel(car.yaw, tyaw)), car.dist - d0, axis_entry_yaw_err)
        lx, ly, lead_s = st['lx'], st['ly'], st['lead_s']

        if phase == 'REV':
            bear = rel(car.yaw, get_angle(car.x, car.y, lx, ly))
            car.step(steer(bear, 1, rev_sign) + OFS, DUTY_GOTO_REV)
            if abs(bear) <= REV_EXIT or (car.dist - rev_d0) >= REV_DIST:
                phase = 'TURN'
        elif phase == 'TURN':
            bear = rel(car.yaw, get_angle(car.x, car.y, lx, ly))
            car.step(steer(bear, 0) + OFS, DUTY_GOTO)
            if abs(bear) <= TURN_TOL:
                phase = 'DRIVE'
        elif phase == 'DRIVE':
            bear = rel(car.yaw, get_angle(car.x, car.y, lx, ly))
            car.step(steer(bear, 0) + OFS, DUTY_GOTO)
            s, lat = axis_s(), axis_lat()
            if s <= lead_s:
                st['behind'] = True
            if (math.hypot(lx - car.x, ly - car.y) <= arrive or
                    (st['behind'] and s >= lead_s)):
                aligned = ((corridor is None or abs(lat) <= corridor) and
                           (yaw_gate is None or
                            abs(rel(car.yaw, tyaw)) <= yaw_gate))
                if aligned:
                    axis_entry_yaw_err = abs(rel(car.yaw, tyaw))
                    phase = 'AXIS'
                elif replan and nreplan < 4:
                    nreplan += 1
                    set_lead(lead0 + abs(lat))
        elif phase == 'AXIS' and axis_law == 'stanley':
            # Cross-track -> heading cascade (Stanley-like), NOT pure pursuit.
            #
            # Why change the law at all: pure pursuit steers at a POINT. Once
            # the aim point falls inside the 1.32m minimum turn circle the car
            # can only circle it, forever - no gain choice fixes that, it is
            # geometry. That is the whole residual failure set.
            #
            # This law never aims at a point. It asks for a heading:
            #   desired heading = tyaw + atan(lat_k * lat / v)   (clamped)
            # and feeds the heading ERROR to the steering. Cross-track error
            # cannot produce a command larger than max_ang of correction, so the
            # commanded curvature stays inside what the car can hold, and the
            # error decays monotonically instead of orbiting.
            s, lat = axis_s(), axis_lat()
            # lat > 0 = car is to the RIGHT of the axis -> must aim LEFT of tyaw.
            corr = math.degrees(math.atan(lat_k * lat / max(v_ref, 0.2)))
            corr = max(-max_ang, min(max_ang, corr))
            aim_yaw = tyaw + corr
            yerr = rel(car.yaw, aim_yaw)
            car.step(steer(yerr, 0) + OFS, DUTY_GOTO)
            if axis_s() >= -AXIS_STOP:
                return ('DONE', math.hypot(tx - car.x, ty - car.y),
                        abs(rel(car.yaw, tyaw)), car.dist - d0, axis_entry_yaw_err)
        else:
            s, lat = axis_s(), axis_lat()
            # Adaptive lookahead. Pure pursuit is only stable while the
            # lookahead distance exceeds the lateral error; with a fixed
            # ld=1.20 and lat=2.0 the aim point sits inside the 1.56m minimum
            # turn circle and the car orbits it forever (that is the FAULT_DIST
            # cluster). ld_gain=0 reproduces the fixed-ld C code.
            lde = max(ld, ld_gain * abs(lat))
            # clamp_aim=True is the current C code: the aim point is clamped to
            # the target, so as s->0 the lookahead collapses to 0, the <0.02m
            # guard in motion_goto_bearing_err returns 0, and the steering
            # freezes exactly where alignment matters most.
            sa = min(0.0, s + lde) if clamp_aim else (s + lde)
            ax, ay = tx + fx * sa, ty + fy * sa
            bear = rel(car.yaw, get_angle(car.x, car.y, ax, ay))
            car.step(steer(bear, 0) + OFS, DUTY_GOTO)
            if axis_s() >= -AXIS_STOP:
                return ('DONE', math.hypot(tx - car.x, ty - car.y),
                        abs(rel(car.yaw, tyaw)), car.dist - d0, axis_entry_yaw_err)

        if trace and tick % 25 == 0:
            print('    t=%5.2fs %-5s x=%6.2f y=%6.2f yaw=%7.1f s=%6.2f '
                  'lat=%6.2f d=%5.2f'
                  % (tick * DT, phase, car.x, car.y, car.yaw, axis_s(),
                     axis_lat(), car.dist - d0))

    return ('FAULT_TICKS', math.hypot(tx - car.x, ty - car.y),
            abs(rel(car.yaw, tyaw)), car.dist - d0, axis_entry_yaw_err)


TX, TY, TYAW = 2.0, -3.0, 30.0
_RAD = TYAW * DEG
FX, FY = -math.sin(_RAD), math.cos(_RAD)        # axis forward
RX, RY = math.cos(_RAD), math.sin(_RAD)         # axis right


def from_axis(s, lat, yaw_rel):
    """Build a world start pose from axis coords (s along tyaw, lat to the right)."""
    return (TX + FX * s + RX * lat, TY + FY * s + RY * lat, TYAW + yaw_rel)


YAWS = [0, 45, 90, 135, 180, -135, -90, -45]


def make_cases(s_list, lat_list):
    out = []
    for s in s_list:
        for lat in lat_list:
            for yr in YAWS:
                out.append(from_axis(s, lat, yr))
    return out


# Normal band: the staging point sits between the task area and the gate row, and
# tyaw points at the gate. So the car doing random actions in the task area is
# BEHIND the staging point (s < 0), laterally within the task area width.
NORMAL = make_cases([-4.0, -6.0, -10.0, -15.0], [0.0, 2.0, -2.0, 5.0, -5.0])
# Pathological band: car ended up level with or past the staging point (s >= 0),
# i.e. it drifted toward the gate row. Should not happen if the staging line is
# respected, but must not blow up.
PAST = make_cases([0.0, 1.0, 3.0], [0.0, 2.0, -2.0])
# Close-in band: car nearly on top of the target.
CLOSE = make_cases([-1.0, -2.0, -3.0], [0.0, 1.0, -1.0])


def verdict(res, ep, ey):
    if not res.startswith('DONE'):
        return 'X'                      # never reached the target axis
    if ep <= 0.5 and ey <= 10.0:
        return '.'                      # inside the handover budget
    if ey > 10.0:
        return 'y'                      # arrived, heading off
    return 'p'                          # arrived, position off


def grid(label, s_list, lat_list, **kw):
    """One row per (s,lat) station, one column per relative start heading."""
    print('%s   [cols = start yaw rel to tyaw: %s]'
          % (label, ' '.join('%d' % y for y in YAWS)))
    npass = ntot = 0
    for s in s_list:
        for lat in lat_list:
            row, worst = [], (0.0, 0.0, 0.0)
            for yr in YAWS:
                x0, y0, yaw0 = from_axis(s, lat, yr)
                res, ep, ey, dd, ae = run(x0, y0, yaw0, TX, TY, TYAW, **kw)
                v = verdict(res, ep, ey)
                row.append(v)
                ntot += 1
                if v == '.':
                    npass += 1
                worst = max(worst, (ep, ey, dd))
            print('    s=%6.1f lat=%5.1f  %s   worst pos %5.2f yaw %5.1f dist %4.1f'
                  % (s, lat, ' '.join(row), worst[0], worst[1], worst[2]))
    print('    -> pass %d/%d' % (npass, ntot))
    return npass, ntot


def sweep(label, cases, **kw):
    npass, wp, wy, wd = 0, 0.0, 0.0, 0.0
    fails = []
    for (x0, y0, yaw0) in cases:
        res, ep, ey, dd, ae = run(x0, y0, yaw0, TX, TY, TYAW, **kw)
        if verdict(res, ep, ey) == '.':
            npass += 1
        else:
            fails.append((x0, y0, yaw0, res, ep, ey, dd))
        wp, wy, wd = max(wp, ep), max(wy, ey), max(wd, dd)
    print('  %-34s pass %3d/%3d  worst pos %5.2fm yaw %6.1fdeg dist %5.1fm'
          % (label, npass, len(cases), wp, wy, wd))
    return npass, fails


ALL = NORMAL + CLOSE + PAST

if __name__ == '__main__':
    print('=== GOTO tuning: target (%.1f,%.1f,%.0fdeg), %d start poses ==='
          % (TX, TY, TYAW, len(ALL)))
    sweep('BASELINE = current C code', ALL)
    print('-- pure pursuit, best known --')
    sweep('pursuit lead6 noclamp vlead=off', ALL, lead0=6.0,
          clamp_aim=False, variable_lead='off')
    sweep('pursuit lead6 noclamp vlead=past', ALL, lead0=6.0,
          clamp_aim=False, variable_lead='past')

    print('-- focused: law x vlead x lead0 --')
    best = None
    for law in ('pursuit', 'stanley'):
        for vl in ('off', 'now', 'past'):
            for ld0 in (5.0, 6.0, 8.0, 10.0):
                kw = dict(variable_lead=vl, lead0=ld0, clamp_aim=False)
                if law == 'stanley':
                    kw.update(axis_law='stanley', lat_k=1.0, max_ang=50.0)
                n, f = sweep('%s vlead=%-4s lead=%.0f' % (law, vl, ld0), ALL, **kw)
                if best is None or n > best[0]:
                    best = (n, law, vl, ld0, f)
    print()
    print('BEST: pass %d/%d  law=%s vlead=%s lead=%.0f'
          % (best[0], len(ALL), best[1], best[2], best[3]))
    print('  first 12 failures (x, y, yaw, result, pos, yaw, dist):')
    for f in best[4][:12]:
        print('    (%6.2f,%6.2f,%7.1f) %-11s pos %5.2f yaw %6.1f dist %5.1f'
              % f)
