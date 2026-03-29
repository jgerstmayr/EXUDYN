import pandas as pd
import numpy as np

df = pd.read_csv('contact_debug.csv')
print('总行数:', len(df))
print()

# 物理分析：检查力矩符号
print("="*80)
print("物理分析：摩擦力矩符号检查")
print("="*80)

row = df.iloc[0]
print(f"\n初始状态 (counter=0):")
print(f"  omegaZ_curve (上圆) = {row['omegaZ_curve']:.4f} rad/s")
print(f"  omegaZ_circle (下圆) = {row['omegaZ_circle']:.4f} rad/s")
print(f"  接触点: ({row['contactPt_x']:.4f}, {row['contactPt_y']:.4f})")
print(f"  r_circle: ({row['r_circle_x']:.4f}, {row['r_circle_y']:.4f})")
print(f"  vCircle: ({row['vCircle_x']:.4f}, {row['vCircle_y']:.4f})")
print(f"  vCurve: ({row['vCurve_x']:.4f}, {row['vCurve_y']:.4f})")
print(f"  fNormal = {row['fNormal']:.4f} N")
print(f"  frictionMag = {row['frictionMag']:.4f} N")
print(f"  torqueOnCurve = {row['torqueOnCurve']:.6f} Nm")
print(f"  torqueOnCircle = {row['torqueOnCircle']:.6f} Nm")

# 验证力矩计算
r_circle = np.array([row['r_circle_x'], row['r_circle_y']])
force_x = row['forceLocal_x']
force_y = row['forceLocal_y']
forceOnCircle = np.array([force_x, force_y])
forceOnCurve = -forceOnCircle

print(f"\n力验证:")
print(f"  contactForceLocal = ({force_x:.4f}, {force_y:.4f})")
print(f"  forceOnCircle = ({forceOnCircle[0]:.4f}, {forceOnCircle[1]:.4f})")
print(f"  forceOnCurve = ({forceOnCurve[0]:.4f}, {forceOnCurve[1]:.4f})")

# 正确的物理力矩: τ = r × F (2D: τ = r_x*F_y - r_y*F_x)
contactPt = np.array([row['contactPt_x'], row['contactPt_y']])
correct_torque_curve = contactPt[0] * forceOnCurve[1] - contactPt[1] * forceOnCurve[0]
correct_torque_circle = r_circle[0] * forceOnCircle[1] - r_circle[1] * forceOnCircle[0]

print(f"\n力矩计算验证:")
print(f"  r_curve (接触点) = ({contactPt[0]:.4f}, {contactPt[1]:.4f})")
print(f"  r_circle = ({r_circle[0]:.4f}, {r_circle[1]:.4f})")
print(f"  正确的 torque_curve = r_curve × forceOnCurve = {correct_torque_curve:.6f} Nm")
print(f"  正确的 torque_circle = r_circle × forceOnCircle = {correct_torque_circle:.6f} Nm")
print(f"  代码输出 torqueOnCurve = {row['torqueOnCurve']:.6f} Nm")
print(f"  代码输出 torqueOnCircle = {row['torqueOnCircle']:.6f} Nm")

# 检查符号
print(f"\n符号分析:")
if np.sign(correct_torque_curve) != np.sign(row['torqueOnCurve']) and row['torqueOnCurve'] != 0:
    print(f"  ❌ torqueOnCurve 符号错误! 正确应为 {correct_torque_curve:.6f}, 代码给出 {row['torqueOnCurve']:.6f}")
else:
    print(f"  ✓ torqueOnCurve 符号正确")
    
if np.sign(correct_torque_circle) != np.sign(row['torqueOnCircle']) and row['torqueOnCircle'] != 0:
    print(f"  ❌ torqueOnCircle 符号错误! 正确应为 {correct_torque_circle:.6f}, 代码给出 {row['torqueOnCircle']:.6f}")
else:
    print(f"  ✓ torqueOnCircle 符号正确")

# 物理意义分析
print(f"\n物理意义:")
print(f"  下圆(circle)角速度={row['omegaZ_circle']:.2f} rad/s (逆时针)")
print(f"  接触点处下圆切向速度向上 (Y正方向)")
print(f"  摩擦力应该阻止滑动，即作用在下圆上的摩擦力应该向下 (-Y方向)")
print(f"  这个摩擦力产生的力矩应该是顺时针的 (负值)，减慢下圆")
print(f"  但代码给出的 torqueOnCircle = {row['torqueOnCircle']:.6f}，这是{'加速' if row['torqueOnCircle'] > 0 else '减速'}下圆!")

print("\n" + "="*80)
print("能量分析：检查角速度变化")
print("="*80)

samples = [0, 100, 1000, 5000, 10000, 50000, len(df)-1]
print("\ncounter | omega_curve | omega_circle | torque_curve | torque_circle | 物理预期")
print("-"*90)
for i in samples:
    if i < len(df):
        row = df.iloc[i]
        # 根据力矩符号判断物理预期
        expected_curve = "加速" if row['torqueOnCurve'] > 0 else "减速"
        expected_circle = "加速" if row['torqueOnCircle'] > 0 else "减速"
        print(f"{row['counter']:6.0f} | {row['omegaZ_curve']:11.2f} | {row['omegaZ_circle']:12.2f} | {row['torqueOnCurve']:12.4f} | {row['torqueOnCircle']:13.4f} | curve:{expected_curve}, circle:{expected_circle}")

print()
print("=== 分析 counter 40000-70000 附近（转折点）===")
samples2 = list(range(40000, 70000, 5000)) + [67679, 67680, 67681]
for i in samples2:
    if i < len(df):
        row = df.iloc[i]
        print(f"c={row['counter']:5.0f}: omega_circle={row['omegaZ_circle']:8.2f}, fNormal={row['fNormal']:10.2f}, gap={row['gap']:.8f}, torque_circle={row['torqueOnCircle']:8.2f}")

print()
print("=== 检查法向力符号变化 ===")
# 找法向力从负变正的点
df['fNormal_sign'] = df['fNormal'].apply(lambda x: 1 if x > 0 else -1)
df['sign_change'] = df['fNormal_sign'].diff().abs() > 0
sign_changes = df[df['sign_change']]
print(f"法向力符号变化次数: {len(sign_changes)}")
if len(sign_changes) > 0:
    print("前5次符号变化:")
    print(sign_changes.head(5)[['counter', 'fNormal', 'gap', 'omegaZ_circle']])

print()
print("=== 检查gap变化 ===")
print(f"gap最小值: {df['gap'].min():.10f}")
print(f"gap最大值: {df['gap'].max():.10f}")
# 找gap接近0的点
near_zero = df[df['gap'] > -0.000005]
if len(near_zero) > 0:
    print(f"gap接近0开始于 counter={near_zero.iloc[0]['counter']:.0f}")
    print(near_zero.head(5)[['counter', 'gap', 'fNormal', 'omegaZ_circle']])
