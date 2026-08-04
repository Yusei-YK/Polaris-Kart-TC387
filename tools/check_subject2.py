"""科目二固定动作 —— 结构化自检(无需 TriCore 工具链)

这台机器上没有编译器,编译烧写在 AURIX Studio 里做。本脚本做文本层面的
一致性检查,相当于自动化 code review,防止改漏/改错/前后不一致。
重点盯三件事:
  1. 宏定义与引用是否对得上(宏定义了没人用,编译器不报错)
  2. 开环支路是否排在 !enable 检查之后(顺序反了遥控急停就失效)
  3. 那三处后轮 duty 清零仲裁有没有被手滑改动

用法: python tools/check_subject2.py      (退出码 0=全过, 1=有失败项)
"""
import io, os, re, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
U = os.path.join(ROOT, 'Kart_TC387', 'user')

def rd(n):
    return io.open(os.path.join(U, n), encoding='utf-8', errors='surrogateescape').read()

F = {}
for n in ['kart_motion.h','kart_motion.c','kart_control.h','kart_control.c',
          'kart_steer_ctrl.h','kart_steer_ctrl.c','kart_odom.c','isr.c','cpu0_main.c']:
    F[n] = rd(n)

fail = []
def ck(cond, msg):
    print(('  OK   ' if cond else '  FAIL ') + msg)
    if not cond:
        fail.append(msg)

def strip_c(s):
    s = re.sub(r'/\*.*?\*/', '', s, flags=re.S)
    return re.sub(r'//[^\n]*', '', s)

def mval(f, n):
    m = re.search(r'#define\s+' + n + r'\s+\(([-+\d.]+)[fuFU]?\)', F[f])
    return float(m.group(1).lstrip('+')) if m else None

mc  = strip_c(F['kart_motion.c'])
cc  = F['kart_control.c']
seg = mc[mc.find('switch(voice_cmd)'):mc.find('void kart_motion_update')]

print('[1] kart_motion.h macro values')
for name, want in [('KART_MOTION_FWD_DIST',10.0),('KART_MOTION_BACK_DIST',10.0),
                   ('KART_MOTION_SNAKE_DIST',10.0),('KART_MOTION_CIRCLE_DELTA',950.0),
                   ('KART_MOTION_TURN_DELTA',950.0),('KART_MOTION_OPENLOOP_REAR',1.0)]:
    v = mval('kart_motion.h', name)
    ck(v == want, '%s = %s (want %s)' % (name, v, want))

print('[2] duty macros defined, BACK negative')
DUTY = ['KART_MOTION_DUTY_FWD','KART_MOTION_DUTY_BACK','KART_MOTION_DUTY_SNAKE',
        'KART_MOTION_DUTY_CIRCLE','KART_MOTION_DUTY_TURN']
for d in DUTY:
    ck(mval('kart_motion.h', d) is not None, '%s = %s' % (d, mval('kart_motion.h', d)))
ck((mval('kart_motion.h','KART_MOTION_DUTY_BACK') or 0) < 0, 'DUTY_BACK negative (reverse)')

print('[3] every duty macro referenced in kart_motion.c')
for d in DUTY:
    ck(d in mc, '%s referenced' % d)

print('[4] all 8 start cases routed through motion_set_rear')
ck(seg.count('motion_set_rear(') == 8, 'motion_set_rear x8 (got %d)' % seg.count('motion_set_rear('))
ck('kart_control_set_target' not in seg, 'no direct set_target left in cases')
ck('kart_control_set_enable(1)' not in seg, 'set_enable(1) folded into helper')

print('[5] new funcs: declared / defined / called')
for fn, h, c in [('kart_control_set_open_duty','kart_control.h','kart_control.c'),
                 ('kart_control_clear_open_loop','kart_control.h','kart_control.c'),
                 ('kart_steer_use_back_gains','kart_steer_ctrl.h','kart_steer_ctrl.c'),
                 ('kart_steer_use_fwd_gains','kart_steer_ctrl.h','kart_steer_ctrl.c')]:
    ck(re.search(r'void\s+' + fn + r'\s*\(', F[h]) is not None, '%s declared' % fn)
    ck(re.search(r'^void\s+' + fn + r'\s*\(', F[c], re.M) is not None, '%s defined' % fn)
    ck(fn in mc, '%s called from kart_motion.c' % fn)

print('[6] struct fields + init')
ck(re.search(r'uint8\s+open_loop', F['kart_control.h']) is not None, 'open_loop in struct')
ck(re.search(r'int16\s+open_duty', F['kart_control.h']) is not None, 'open_duty in struct')
ck(re.search(r'kart_speed\.open_loop\s*=\s*0;', cc) is not None, 'open_loop init in _init()')

print('[7] SAFETY: open-loop branch after the !enable check')
i_en, i_ol = cc.find('if(!kart_speed.enable)'), cc.find('if(kart_speed.open_loop)')
ck(i_en != -1 and i_ol != -1 and i_en < i_ol,
   '!enable at %d precedes open_loop at %d' % (i_en, i_ol))

print('[8] set_open_duty writes duty before flag (ISR race)')
m = re.search(r'void kart_control_set_open_duty[^{]*\{(.*?)\n\}', cc, re.S)
b = m.group(1) if m else ''
ck(b and b.find('open_duty') < b.find('open_loop'), 'duty assigned before flag')

print('[9] set_enable(0) clears open_loop (e-stop convergence point)')
m = re.search(r'void kart_control_set_enable[^{]*\{(.*?)\n\}', cc, re.S)
ck(m is not None and 'open_loop = 0' in m.group(1), 'set_enable(0) clears open_loop')

print('[10] kart_motion_stop full teardown')
m = re.search(r'void kart_motion_stop[^{]*\{(.*?)\n\}', F['kart_motion.c'], re.S)
sb = m.group(1) if m else ''
for t in ['kart_steer_use_fwd_gains','kart_control_clear_open_loop','kart_control_set_enable(0)']:
    ck(t in sb, 'stop() calls %s' % t)

print('[11] back gains only on reverse cases')
def case_body(tag):
    i = seg.find('case ' + tag); return seg[i:seg.find('break;', i)]
for t in ['KART_VOICE_CMD_BACK_10M','KART_VOICE_CMD_SNAKE_BACK_10M']:
    ck('kart_steer_use_back_gains' in case_body(t), '%s -> back gains' % t)
for t in ['KART_VOICE_CMD_FWD_10M','KART_VOICE_CMD_SNAKE_FWD_10M','KART_VOICE_CMD_CCW_CIRCLE',
          'KART_VOICE_CMD_CW_CIRCLE','KART_VOICE_CMD_TURN_LEFT','KART_VOICE_CMD_TURN_RIGHT']:
    ck('kart_steer_use_back_gains' not in case_body(t), '%s -> fwd gains' % t)

print('[12] back gain softer than forward')
kpf, kpb = mval('kart_steer_ctrl.h','KART_STEER_KP_DEFAULT'), mval('kart_steer_ctrl.h','KART_STEER_KP_BACK')
kdf, kdb = mval('kart_steer_ctrl.h','KART_STEER_KD_DEFAULT'), mval('kart_steer_ctrl.h','KART_STEER_KD_BACK')
ck(kpb < kpf, 'Kp_back %s < Kp_fwd %s' % (kpb, kpf))
ck(kdb > kdf, 'Kd_back %s > Kd_fwd %s' % (kdb, kdf))

print('[13] steer targets inside soft limits')
lim_l = int(re.search(r'KART_STEER_DELTA_LIMIT_L\s+\(\+?(\d+)\)', F['kart_steer_ctrl.h']).group(1))
lim_r = abs(int(re.search(r'KART_STEER_DELTA_LIMIT_R\s+\((-?\d+)\)', F['kart_steer_ctrl.h']).group(1)))
for n in ['KART_MOTION_CIRCLE_DELTA','KART_MOTION_TURN_DELTA','KART_MOTION_SNAKE_DELTA']:
    v = mval('kart_motion.h', n)
    ck(v <= lim_l and v <= lim_r, '%s=%s <= min(L=%d,R=%d)' % (n, v, lim_l, lim_r))

print('[14] odom dist monotonic (reverse predicate valid)')
ck('dist_sum += fabsf(ds)' in F['kart_odom.c'], 'dist_sum accumulates |ds|')

print('[15] three arbitration points untouched')
ck(F['isr.c'].count('power_force_rear_pwm_zero();') == 1, 'isr.c intact')
ck(F['cpu0_main.c'].count('power_set_rear_duty(0, 0);') == 2, 'cpu0_main.c intact (x2)')
ck('if(!kart_speed.enable)' in cc, 'kart_control.c intact')

print('[16] deadman still first in update()')
ub = re.search(r'void kart_motion_update[^{]*\{(.*)', F['kart_motion.c'], re.S).group(1)
ck(ub.find('kart_remote_is_online') < ub.find('switch(motion_phase)'), 'deadman before switch')

print('[18] center offset: single funnel, no direct set_target_delta left')
# motion_set_delta 必须是唯一叠偏置的出口;直调只允许 stop() 里那一处(交还转向留干净 0)
ck('KART_MOTION_DELTA_CENTER_OFS' in F['kart_motion.h'], 'OFS macro defined')
m = re.search(r'static void motion_set_delta[^{]*\{(.*?)\n\}', mc, re.S)
ck(m is not None and 'KART_MOTION_DELTA_CENTER_OFS' in m.group(1),
   'motion_set_delta applies OFS')
# OFS 只许出现两次:下发出口叠偏置 + 回正段判到位(下发的是 OFS,所以比的是 meas-OFS)
ck(mc.count('KART_MOTION_DELTA_CENTER_OFS') == 2,
   'OFS referenced twice (funnel + center-done test), got %d' % mc.count('KART_MOTION_DELTA_CENTER_OFS'))
direct = mc.count('kart_steer_set_target_delta(')
ck(direct == 2, 'direct set_target_delta x2 only (funnel + stop), got %d' % direct)
ck('kart_steer_set_target_delta(0.0f);' in sb, 'the one direct call is in stop()')
ck('kart_steer_set_target_delta' not in seg, 'no direct call left in any start case')

print('[19] snake: flip on heading band, not on distance')
# 按路程翻转干不过转向机的转速(900->-900 要 1800counts / 1800cps = 1s),
# 摆幅会越摆越小;改成"摆过 SNAKE_YAW 度才翻",并留路程保底
ck('KART_MOTION_SNAKE_HALF_DIST' not in F['kart_motion.h'] + mc, 'distance-modulo macro gone')
ck('fmodf(' not in strip_c(mc), 'fmodf predicate gone from code')
ck(mval('kart_motion.h','KART_MOTION_SNAKE_DELTA') == 900.0, 'SNAKE_DELTA=900')
sy = mval('kart_motion.h','KART_MOTION_SNAKE_YAW')
ck(sy is not None and 5.0 <= sy <= 30.0, 'SNAKE_YAW=%s in sane band' % sy)
ck(mval('kart_motion.h','KART_MOTION_SNAKE_MAX_HALF') is not None, 'SNAKE_MAX_HALF bail-out defined')
sk = re.search(r'static void motion_snake_step[^{]*\{(.*?)\n\}', mc, re.S)
ck(sk is not None, 'motion_snake_step exists')
sk = sk.group(1) if sk else ''
ck('KART_MOTION_SNAKE_YAW' in sk and 'KART_MOTION_SNAKE_MAX_HALF' in sk,
   'both flip conditions present (yaw band OR distance bail-out)')
ck('KART_MOTION_REV_YAW_SIGN' in sk, 'flip predicate normalises the reverse sign')
ck(re.search(r'motion_snake_sign\s*=\s*-\s*motion_snake_sign', sk) is not None,
   'sign toggles on flip')
ck(re.search(r'motion_snake_half_d0\s*=\s*kart_odom_get_dist\(\)', sk) is not None,
   'per-swing distance datum re-latched on flip')
ck('motion_set_delta(motion_snake_sign' in sk, 'swing delta goes through the OFS funnel')
ck(mc.count('motion_snake_step(motion_reverse)') == 1, 'both snake arms share one snake_step call')

print('[20] snake end-of-run straighten phase')
ck('MOTION_SNAKE_END' in mc, 'MOTION_SNAKE_END state exists')
ck(re.search(r'motion_phase\s*=\s*MOTION_SNAKE_END', mc) is not None, 'snake transitions into it')
ck('KART_MOTION_SNAKE_END_TOL' in mc and 'KART_MOTION_SNAKE_END_MAX_DIST' in mc,
   'both exits (tolerance + bail-out distance) used')
eb = mc[mc.find('case MOTION_SNAKE_END'):]
eb = eb[:eb.find('break;')]
ck('motion_set_delta' in eb, 'END phase drives steering (no swinging)')
ck('KART_MOTION_SNAKE_DELTA' not in eb, 'END phase stops swinging')

print('[21] local heading corrector shared by forward and reverse')
ck('KART_MOTION_BACK_YAW_SIGN' not in F['kart_motion.h'] + mc, 'old BACK_YAW_SIGN name gone')
ck('KART_MOTION_BACK_YAW_KP' not in F['kart_motion.h'] + mc, 'old BACK_YAW_KP name gone')
ck('KART_MOTION_BACK_YAW_LIMIT' not in F['kart_motion.h'] + mc, 'old BACK_YAW_LIMIT name gone')
ck(mval('kart_motion.h','KART_MOTION_REV_YAW_SIGN') == -1.0, 'REV_YAW_SIGN = -1')
kp = mval('kart_motion.h','KART_MOTION_YAW_KP')
lim = mval('kart_motion.h','KART_MOTION_YAW_LIMIT')
ck(kp is not None and lim is not None, 'YAW_KP / YAW_LIMIT defined')
ck(lim + abs(mval('kart_motion.h','KART_MOTION_DELTA_CENTER_OFS')) <= min(lim_l, lim_r),
   'corr limit %s + |OFS| within soft limits' % lim)
# KP 必须足够大:实测转向机静摩擦死区 ~950duty,除以内环 Kp15 = 63counts 的角度误差
# 才会让轮子动。要求 3 度以内的偏航就能顶穿,否则就是"修的很少"
ck(kp * 3.0 >= 63.0, 'YAW_KP=%s breaks the 63-count stiction band within 3deg' % kp)
bb = mc[mc.find('case MOTION_BACK'):]
bb = bb[:bb.find('break;')]
ck('motion_yaw_corr_delta(1)' in bb, 'MOTION_BACK applies reverse heading correction')
ck('kart_steer_set_target_delta(0.0f)' not in bb, 'old hardcoded delta=0 gone')
fb = mc[mc.find('case MOTION_FWD'):]
fb = fb[:fb.find('break;')]
ck('motion_yaw_corr_delta(0)' in fb, 'MOTION_FWD uses the local corrector too')
# head_pid 绕开 motion_set_delta 的偏置、纯 P 有常驻误差,常驻误差又落在静摩擦死区里
# => 实车"前行十米修的很少"。科目二整条链路都不许再开航向环,head_pid 留给科目一/四/遥控
ck('kart_steer_set_head_enable(1)' not in mc, 'subject-2 never enables the shared heading loop')
ck('kart_steer_set_target_yaw' not in mc, 'no absolute-yaw target left (drift immunity)')
ap = mc[mc.find('case MOTION_TURN_APPROACH'):]
ap = ap[:ap.find('break;')]
ck('motion_yaw_corr_delta(0)' in ap, 'TURN_APPROACH straightens with the local corrector')

print('[23] finish -> centre the wheels before releasing the steering')
# angle_enable=0 会直接把转向电机断电,方向盘没有回中力矩,会停在打死位;
# 所以正常做完要先原地拉回中位,再交还转向。急停/切模式仍走 kart_motion_stop() 硬停
ck('MOTION_CENTER' in mc, 'MOTION_CENTER state exists')
fn = re.search(r'static void motion_finish[^{]*\{(.*?)\n\}', mc, re.S)
ck(fn is not None, 'motion_finish exists')
fn = fn.group(1) if fn else ''
ck('kart_control_set_enable(0)' in fn, 'finish stops the rear first')
ck('kart_steer_set_angle_enable(1)' in fn, 'finish KEEPS the inner loop energised')
ck('kart_steer_set_head_enable(0)' in fn, 'finish leaves the shared heading loop off')
ck('motion_set_delta(0.0f)' in fn, 'finish commands centre through the funnel')
ck(re.search(r'motion_phase\s*=\s*MOTION_CENTER', fn) is not None, 'finish enters MOTION_CENTER')
ck('motion_center_ticks = 0' in fn, 'finish resets the bail-out counter')
cb = mc[mc.find('case MOTION_CENTER'):]
cb = cb[:cb.find('break;')]
ck('kart_steer_get_meas_delta' in cb, 'CENTER watches the measured angle')
ck('KART_MOTION_CENTER_TOL' in cb and 'KART_MOTION_CENTER_TICKS' in cb,
   'both exits (tolerance + tick bail-out) used')
ck('kart_motion_stop()' in cb, 'CENTER hands off to the hard stop when done')
ck(mval('kart_motion.h','KART_MOTION_CENTER_TOL') is not None, 'CENTER_TOL defined')
ct = mval('kart_motion.h','KART_MOTION_CENTER_TICKS')
ck(ct is not None and ct >= 100, 'CENTER_TICKS=%s covers 5ms and 10ms beats' % ct)
# 收车路径分工:六条终态走 motion_finish(),看门狗/default 走 kart_motion_stop() 硬停。
# 蛇形两条不算终态(先转 SNAKE_END);GOTO 前三段也不算(只在段间切换)。
# 2026-07-29: 5 → 6,新增摆位原语的终态 MOTION_GOTO_AXIS。
ck(mc.count('motion_finish();') == 6, 'six terminal phases go through finish, got %d'
   % mc.count('motion_finish();'))
for cs in ['MOTION_FWD', 'MOTION_BACK', 'MOTION_SNAKE_END', 'MOTION_CIRCLE',
           'MOTION_TURN_ROTATE', 'MOTION_GOTO_AXIS']:
    blk = mc[mc.find('case %s:' % cs):]
    blk = blk[:blk.find('break;')]
    ck('motion_finish();' in blk, '%s completes via finish' % cs)
ck('kart_motion_stop();' in ub[:ub.find('switch(motion_phase)')], 'deadman still hard-stops')
ck('motion_reverse' in mc, 'motion_reverse flag present')
ck('motion_snake_reverse' not in mc, 'old motion_snake_reverse name gone')
ck('motion_snake_delta' not in mc, 'old motion_snake_delta static gone')

print('[22] heading refs are all relative (yaw drift immunity)')
# 全模块只许通过 motion_yaw_err / motion_accum_yaw 用航向,不许直接比绝对 yaw
m = re.search(r'static float motion_yaw_err[^{]*\{(.*?)\n\}', mc, re.S)
ck(m is not None and 'motion_yaw0' in m.group(1), 'motion_yaw_err compares against latched yaw0')
ck('motion_yaw0      = kart_imu_get_yaw();' in mc or
   re.search(r'motion_yaw0\s*=\s*kart_imu_get_yaw\(\)', mc) is not None,
   'yaw0 latched at action start')

print('[17] brace balance')
for n in ['kart_motion.h','kart_motion.c','kart_control.h','kart_control.c',
          'kart_steer_ctrl.h','kart_steer_ctrl.c']:
    s = strip_c(F[n])
    ck(s.count('{') == s.count('}'), '%s %d/%d' % (n, s.count('{'), s.count('}')))

print('')
print('RESULT: %s' % ('%d FAILED' % len(fail) if fail else 'ALL PASS'))
for f in fail:
    print('   - ' + f)
sys.exit(1 if fail else 0)
