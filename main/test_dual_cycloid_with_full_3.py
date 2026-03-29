import sys, os
import numpy as np

# 统一两圆接触（带符号半径）：r>0 凸圆，r<0 凹内壁
def circle_circle_geom(c1, r1, c2, r2_signed, eps=1e-9, n_prev=None):
    d_vec = c2 - c1
    d = float(np.linalg.norm(d_vec))
    if d < eps:
        n = np.array([1.0, 0.0]) if n_prev is None else n_prev
    else:
        n = d_vec / d
    gap = d - (r1 + r2_signed)
    p1 = c1 + r1 * n
    p2 = c2 - r2_signed * n
    return dict(contact=(gap <= 0.0), gap=gap, n=n, p1=p1, p2=p2, pMid=0.5 * (p1 + p2))

def circle_circle_penalty(c1, r1, v1, c2, r2_signed, v2,
                          kn, cn, mu=0.0, ct=0.0, eps=1e-9, n_prev=None):
    g = circle_circle_geom(c1, r1, c2, r2_signed, eps, n_prev)
    if not g['contact']:
        return {**g, **dict(Fn=0.0, Ft=0.0, F1=np.zeros(2), F2=np.zeros(2))}
    n, gap = g['n'], g['gap']
    delta = -gap
    vrel = (v2 - v1)
    vn = float(np.dot(vrel, n))
    Fn = kn * delta + cn * delta * max(-vn, 0.0)
    if Fn < 0.0:
        Fn = 0.0
    vt = vrel - vn * n
    vt_norm = float(np.linalg.norm(vt))
    Ft_vec = np.zeros(2)
    if mu > 0.0 and vt_norm > 1e-12 and Fn > 0.0:
        t = vt / vt_norm
        Ft_visc = -ct * vt_norm
        Ft_mag = min(abs(Ft_visc), mu * Fn)
        Ft_vec = -Ft_mag * t if Ft_visc < 0 else Ft_mag * (-t)
    F_n = Fn * n
    F1 = -F_n - (-Ft_vec)
    F2 = +F_n + (+Ft_vec)
    return {**g, **dict(Fn=Fn, Ft=float(np.linalg.norm(Ft_vec)), F1=F1, F2=F2)}

# 圆在圆孔内的接触（内壁）：阈值 S = r_hole - r_pin，d > S 才接触
def circle_in_hole_penalty(c_pin, r_pin, v_pin, c_hole, r_hole, v_hole,
                           kn, cn, eps=1e-9, n_prev=None):
    d_vec = c_pin - c_hole
    d = float(np.linalg.norm(d_vec))
    if d < eps:
        n = np.array([1.0, 0.0]) if n_prev is None else n_prev  # 从孔中心指向针齿中心
    else:
        n = d_vec / d
    S = float(r_hole - r_pin)
    if d <= S:
        return dict(contact=False, gap=S - d, n=n, Fn=0.0, F1=np.zeros(2), F2=np.zeros(2))
    # 穿透量
    delta = d - S
    vrel = (v_pin - v_hole)
    vn = float(np.dot(vrel, n))  # 外向速度（增大d）
    Fn = kn * delta + cn * max(vn, 0.0)
    if Fn < 0.0:
        Fn = 0.0
    F_pin = -Fn * n  # 作用在针齿（向内推回）
    F_hole = +Fn * n  # 作用在孔
    return dict(contact=True, gap=-delta, n=n, Fn=Fn, F1=F_pin, F2=F_hole)

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
from exudyn.machines import GetBallBearingData, CreateBallBearing

useGraphics = True
SC = exu.SystemContainer()
mbs = SC.AddSystem()

# =================== 参数（与原脚本一致） ===================
z_b = 40
z_g = z_b - 1
e = 1.3
delta_e = 0
R_z = 64
r_z = 3

second_delta_rp = 0
second_delta_r_rp = 0
second_delta = 0
second_zp = z_b
second_a = e + delta_e
second_rp = R_z
second_r_rp = r_z

r_crank_main = 6
r_crank_eccentric = 10
L_crank_main1 = 20
L_crank_eccentric1 = 15
L_crank_middle = 5
L_crank_eccentric2 = 15
L_crank_main2 = 20
eccentric_offset = e

z_eccentric1 = L_crank_eccentric1 / 2
z_eccentric2 = L_crank_eccentric1 + L_crank_middle + L_crank_eccentric2 / 2

# 滚针轴承参数
r_needle = 1.5  # 滚针半径
n_needles = 12  # 每个轴承的滚针数量
bearing_clearance = 0.02  # 轴承径向间隙（很小但避免初始穿透）
r_needle_pitch = r_crank_eccentric + r_needle + bearing_clearance  # 滚针节圆半径
r_hole = r_needle_pitch + r_needle + bearing_clearance  # 摆线轮孔半径

r_pin_shell = R_z
r_pin_hole = r_z + 0.01
thickness_shell = 8

R_shell_outer = R_z + 12
R_shell_inner = R_z - 12

# 三曲柄轴配置参数
n_cranks = 3  # 曲柄轴数量
crank_distribution_radius = 25.0  # 曲柄轴分布圆半径（增大以更往外扩）

print("="*60)
print("三曲柄轴双片摆线针轮减速器系统（带滚针轴承）")
print("="*60)
print(f"曲柄轴数量: {n_cranks}")
print(f"曲柄轴分布圆半径: {crank_distribution_radius} mm")
print(f"偏心轴外圆半径: {r_crank_eccentric} mm")
print(f"摆线轮孔半径: {r_hole:.3f} mm")
print(f"滚针半径: {r_needle} mm")
print(f"每个轴承滚针数: {n_needles}")
print(f"滚针节圆半径: {r_needle_pitch:.3f} mm")
print(f"轴承径向间隙: {bearing_clearance} mm (每侧)")
print(f"偏心轴-滚针名义间隙: {r_needle_pitch - r_crank_eccentric - r_needle:.3f} mm")
print(f"滚针-孔壁名义间隙: {r_hole - r_needle_pitch - r_needle:.3f} mm")
print(f"针齿半径: {r_z} mm")
print(f"针齿壳孔半径: {r_pin_hole} mm")
print(f"针齿-孔间隙: {r_pin_hole - r_z} mm")
print(f"针齿壳孔分布圆半径: {r_pin_shell} mm")
print(f"第一片摆线轮Z位置: {z_eccentric1} mm")
print(f"第二片摆线轮Z位置: {z_eccentric2} mm")
print(f"两片摆线轮相位差: 180°")
print(f"每个摆线轮孔数: {n_cranks} (与曲柄轴对应)")

contactStiffness_tooth = 1e8
contactDamping_tooth = 1e2
contactStiffness_hole = 1e9
contactDamping_hole = 1e2
contactStiffness_pin_hole = 5e8
contactDamping_pin_hole = 5e1

# 法兰轴承参数
r_flange_shaft = r_crank_main  # 法兰孔与曲柄主轴接触（主轴半径）
r_flange_needle = 1.0  # 法兰轴承滚针半径（比摆线轮轴承小）
n_flange_needles = 10  # 每个法兰轴承的滚针数量
flange_bearing_clearance = 0.02
r_flange_needle_pitch = r_flange_shaft + r_flange_needle + flange_bearing_clearance
r_flange_hole = r_flange_needle_pitch + r_flange_needle + flange_bearing_clearance  # 法兰孔半径

oGround = mbs.CreateGround()

def GenerateCycloidProfile(phi_h=0, n_points=4000):
    x_profile = []
    y_profile = []
    for j in range(n_points):
        phi_b = 2 * (second_zp - 1) * np.pi / n_points * j
        second_k1_pie = second_a * second_zp / (second_rp - second_delta_rp)
        second_s_pie = 1 + second_k1_pie**2 - 2 * second_k1_pie * np.cos(phi_b)
        second_i_H = second_zp / (second_zp - 1)
        x_2j = -((second_rp - second_delta_rp) - (second_r_rp + second_delta_r_rp) * second_s_pie**(-1/2)) * np.sin((1 - second_i_H) * phi_b - second_delta) - \
               second_a / (second_rp - second_delta_rp) * ((second_rp - second_delta_rp) - second_zp * (second_r_rp + second_delta_r_rp) * second_s_pie**(-1/2)) * np.sin(second_i_H * phi_b + second_delta)
        y_2j = ((second_rp - second_delta_rp) - (second_r_rp + second_delta_r_rp) * second_s_pie**(-1/2)) * np.cos((1 - second_i_H) * phi_b - second_delta) - \
               second_a / (second_rp - second_delta_rp) * ((second_rp - second_delta_rp) - second_zp * (second_r_rp + second_delta_r_rp) * second_s_pie**(-1/2)) * np.cos(second_i_H * phi_b + second_delta)
        phi_g = phi_h / z_g
        x_profile.append(x_2j * np.cos(phi_g) - y_2j * np.sin(phi_g))
        y_profile.append(x_2j * np.sin(phi_g) + y_2j * np.cos(phi_g))
    return x_profile, y_profile

x_cycloid1, y_cycloid1 = GenerateCycloidProfile(phi_h=0, n_points=1000)
x_cycloid2, y_cycloid2 = GenerateCycloidProfile(phi_h=np.pi, n_points=1000)

def CreateToothSegments(x_profile, y_profile):
    nSeg = len(x_profile)
    segmentsData = np.zeros((nSeg, 4))
    for i in range(nSeg):
        segmentsData[i, 0:2] = [x_profile[i], y_profile[i]]
        segmentsData[i, 2:4] = [x_profile[(i + 1) % nSeg], y_profile[(i + 1) % nSeg]]
    return segmentsData, nSeg

segmentsData_tooth1, nSeg_tooth1 = CreateToothSegments(x_cycloid1, y_cycloid1)
segmentsData_tooth2, nSeg_tooth2 = CreateToothSegments(x_cycloid2, y_cycloid2)

def CreateCircleSegments(radius, center, nPoints=120):
    pList = []
    for i in range(nPoints):
        phi = i * 2 * np.pi / nPoints
        x = center[0] + radius * np.cos(phi)
        y = center[1] + radius * np.sin(phi)
        pList.append([x, y])
    nSeg = len(pList)
    segmentsData = np.zeros((nSeg, 4))
    for i in range(nSeg):
        segmentsData[i, 0:2] = pList[i]
        segmentsData[i, 2:4] = pList[(i + 1) % nSeg]
    return segmentsData, nSeg

segmentsData_cycloid_hole, nSeg_cycloid_hole = CreateCircleSegments(r_hole, [0, 0], nPoints=120)

# =================== 三曲柄轴系统 ===================
# 曲柄轴参数
mass_crank = 8.0
I_crank = RigidBodyInertia(
    mass=mass_crank,
    inertiaTensor=np.array([[150, 0, 0],[0, 150, 0],[0, 0, 300]]),
    com=[0, 0, 0]
)

omega_target = 2.0  # 目标转速（rad/s）
print(f"曲柄轴目标转速: {omega_target} rad/s = {omega_target*60/(2*np.pi):.2f} RPM")

# 创建三个曲柄轴
crankshaft_bodies = []
crankshaft_markers_ecc1 = []  # 第一片摆线轮对应的偏心段标记
crankshaft_markers_ecc2 = []  # 第二片摆线轮对应的偏心段标记

for i_crank in range(n_cranks):
    angle_crank = 2 * np.pi * i_crank / n_cranks  # 120度均布
    x_crank = crank_distribution_radius * np.cos(angle_crank)
    y_crank = crank_distribution_radius * np.sin(angle_crank)
    
    # 每个曲柄轴的偏心方向（第一片正向，第二片反向）
    crankshaft_graphics = [
        graphics.Cylinder(pAxis=[0, 0, -L_crank_main1], vAxis=[0, 0, L_crank_main1], 
                         radius=r_crank_main, color=graphics.color.darkgrey, nTiles=32),
        graphics.Cylinder(pAxis=[eccentric_offset, 0, 0], vAxis=[0, 0, L_crank_eccentric1], 
                         radius=r_crank_eccentric, color=graphics.color.red, nTiles=32),
        graphics.Cylinder(pAxis=[0, 0, L_crank_eccentric1], vAxis=[0, 0, L_crank_middle], 
                         radius=r_crank_main, color=graphics.color.darkgrey, nTiles=32),
        graphics.Cylinder(pAxis=[-eccentric_offset, 0, L_crank_eccentric1 + L_crank_middle], 
                         vAxis=[0, 0, L_crank_eccentric2], 
                         radius=r_crank_eccentric, color=graphics.color.orange, nTiles=32),
        graphics.Cylinder(pAxis=[0, 0, L_crank_eccentric1 + L_crank_middle + L_crank_eccentric2], 
                         vAxis=[0, 0, L_crank_main2], 
                         radius=r_crank_main, color=graphics.color.darkgrey, nTiles=32),
    ]
    
    oCrank = mbs.CreateRigidBody(referencePosition=[x_crank, y_crank, 0], 
                                  inertia=I_crank, gravity=[0, 0, 0], 
                                  graphicsDataList=crankshaft_graphics)
    crankshaft_bodies.append(oCrank)
    
    # 添加旋转副
    mGroundCrank = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[x_crank, y_crank, 0]))
    mCrank = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank, localPosition=[0, 0, 0]))
    mbs.AddObject(RevoluteJointZ(markerNumbers=[mGroundCrank, mCrank], 
                                 rotationMarker0=np.eye(3), rotationMarker1=np.eye(3), 
                                 visualization=VObjectJointRevoluteZ(axisRadius=0.5, axisLength=1)))
    
    # 添加扭矩控制（只对第一个曲柄轴施加主动扭矩）
    if i_crank == 0:
        def CrankTorqueControl(mbs, t, load, crankBody=oCrank):
            nodeNumber = mbs.GetObject(crankBody)['nodeNumber']
            angVel = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.AngularVelocity)
            omega_current = angVel[2]
            Kp = 1000.0
            torque = Kp * (omega_target - omega_current)
            return [0, 0, torque]
        
        mCrankLoad = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank, localPosition=[0, 0, 0]))
        mbs.AddLoad(LoadTorqueVector(markerNumber=mCrankLoad, loadVector=[0, 0, 0], 
                                     loadVectorUserFunction=CrankTorqueControl))
    
    # 创建偏心段标记
    mEcc1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank, 
                                          localPosition=[eccentric_offset, 0, z_eccentric1]))
    mEcc2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCrank, 
                                          localPosition=[-eccentric_offset, 0, z_eccentric2]))
    crankshaft_markers_ecc1.append(mEcc1)
    crankshaft_markers_ecc2.append(mEcc2)

# 简化约束：只约束曲柄轴的Z轴旋转同步，允许其他自由度
for i_crank in range(1, n_cranks):
    mCrank0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=crankshaft_bodies[0], localPosition=[0, 0, 0]))
    mCrankI = mbs.AddMarker(MarkerBodyRigid(bodyNumber=crankshaft_bodies[i_crank], localPosition=[0, 0, 0]))
    # 只约束Z轴旋转同步，允许X、Y平移和旋转
    mbs.AddObject(GenericJoint(markerNumbers=[mCrank0, mCrankI], 
                               constrainedAxes=[0,0,0,0,0,1], 
                               visualization=VObjectJointGeneric(axesRadius=0.3, axesLength=0.6, show=False)))

print(f"创建了 {n_cranks} 个曲柄轴，分布圆半径: {crank_distribution_radius} mm")

# =================== 输入输出法兰系统 ===================
# 法兰位置
z_input_flange = -L_crank_main1 / 2  # 输入法兰在曲柄轴下方
z_output_flange = L_crank_eccentric1 + L_crank_middle + L_crank_eccentric2 + L_crank_main2 / 2  # 输出法兰在曲柄轴上方

# 法兰参数
mass_flange = 5.0
r_flange_outer = crank_distribution_radius + 15  # 法兰外半径
r_flange_inner = crank_distribution_radius - 10  # 法兰内半径
thickness_flange = 5.0

I_flange = RigidBodyInertia(
    mass=mass_flange,
    inertiaTensor=np.array([[300, 0, 0], [0, 300, 0], [0, 0, 600]]),
    com=[0, 0, 0]
)

# 输入法兰图形
input_flange_graphics = []
n_flange_circle_points = 60
# 外圆
for i in range(n_flange_circle_points):
    angle1 = i * 2 * np.pi / n_flange_circle_points
    angle2 = (i + 1) * 2 * np.pi / n_flange_circle_points
    x1 = r_flange_outer * np.cos(angle1)
    y1 = r_flange_outer * np.sin(angle1)
    x2 = r_flange_outer * np.cos(angle2)
    y2 = r_flange_outer * np.sin(angle2)
    input_flange_graphics.append({'type': 'Line', 'color': [0.3, 0.3, 0.8, 1], 
                                  'data': [x1, y1, 0, x2, y2, 0]})
# 内圆
for i in range(n_flange_circle_points):
    angle1 = i * 2 * np.pi / n_flange_circle_points
    angle2 = (i + 1) * 2 * np.pi / n_flange_circle_points
    x1 = r_flange_inner * np.cos(angle1)
    y1 = r_flange_inner * np.sin(angle1)
    x2 = r_flange_inner * np.cos(angle2)
    y2 = r_flange_inner * np.sin(angle2)
    input_flange_graphics.append({'type': 'Line', 'color': [0.3, 0.3, 0.8, 1], 
                                  'data': [x1, y1, 0, x2, y2, 0]})
# 3个孔
for i_hole in range(n_cranks):
    angle_hole = 2 * np.pi * i_hole / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    for i in range(40):
        angle1 = i * 2 * np.pi / 40
        angle2 = (i + 1) * 2 * np.pi / 40
        x1 = x_hole_center + r_flange_hole * np.cos(angle1)
        y1 = y_hole_center + r_flange_hole * np.sin(angle1)
        x2 = x_hole_center + r_flange_hole * np.cos(angle2)
        y2 = y_hole_center + r_flange_hole * np.sin(angle2)
        input_flange_graphics.append({'type': 'Line', 'color': [0.8, 0.8, 0.2, 1], 
                                      'data': [x1, y1, 0, x2, y2, 0]})

# 输出法兰图形（类似但颜色不同）
output_flange_graphics = []
# 外圆
for i in range(n_flange_circle_points):
    angle1 = i * 2 * np.pi / n_flange_circle_points
    angle2 = (i + 1) * 2 * np.pi / n_flange_circle_points
    x1 = r_flange_outer * np.cos(angle1)
    y1 = r_flange_outer * np.sin(angle1)
    x2 = r_flange_outer * np.cos(angle2)
    y2 = r_flange_outer * np.sin(angle2)
    output_flange_graphics.append({'type': 'Line', 'color': [0.8, 0.3, 0.3, 1], 
                                   'data': [x1, y1, 0, x2, y2, 0]})
# 内圆
for i in range(n_flange_circle_points):
    angle1 = i * 2 * np.pi / n_flange_circle_points
    angle2 = (i + 1) * 2 * np.pi / n_flange_circle_points
    x1 = r_flange_inner * np.cos(angle1)
    y1 = r_flange_inner * np.sin(angle1)
    x2 = r_flange_inner * np.cos(angle2)
    y2 = r_flange_inner * np.sin(angle2)
    output_flange_graphics.append({'type': 'Line', 'color': [0.8, 0.3, 0.3, 1], 
                                   'data': [x1, y1, 0, x2, y2, 0]})
# 3个孔
for i_hole in range(n_cranks):
    angle_hole = 2 * np.pi * i_hole / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    for i in range(40):
        angle1 = i * 2 * np.pi / 40
        angle2 = (i + 1) * 2 * np.pi / 40
        x1 = x_hole_center + r_flange_hole * np.cos(angle1)
        y1 = y_hole_center + r_flange_hole * np.sin(angle1)
        x2 = x_hole_center + r_flange_hole * np.cos(angle2)
        y2 = y_hole_center + r_flange_hole * np.sin(angle2)
        output_flange_graphics.append({'type': 'Line', 'color': [0.8, 0.8, 0.2, 1], 
                                       'data': [x1, y1, 0, x2, y2, 0]})

# 创建输入法兰（不固定在地面，可自由运动）
oInputFlange = mbs.CreateRigidBody(
    referencePosition=[0, 0, z_input_flange],
    inertia=I_flange,
    gravity=[0, 0, 0],
    graphicsDataList=input_flange_graphics
)
# 输入法兰只能绕Z轴旋转（相对地面），但XY平移和Z位置固定
mGroundInputFlange = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, z_input_flange]))
mInputFlange = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oInputFlange, localPosition=[0, 0, 0]))
mbs.AddObject(GenericJoint(markerNumbers=[mGroundInputFlange, mInputFlange],
                           constrainedAxes=[0, 0, 1, 1, 1, 0],  # 只允许Z轴旋转
                           visualization=VObjectJointGeneric(axesRadius=0.5, axesLength=1)))

# 创建输出法兰（不固定在地面）
oOutputFlange = mbs.CreateRigidBody(
    referencePosition=[0, 0, z_output_flange],
    inertia=I_flange,
    gravity=[0, 0, 0],
    graphicsDataList=output_flange_graphics
)

# 输入法兰和输出法兰之间刚性固定（形成一个整体）
mInputFlangeConnect = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oInputFlange, localPosition=[0, 0, 0]))
mOutputFlangeConnect = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oOutputFlange, localPosition=[0, 0, 0]))
mbs.AddObject(GenericJoint(markerNumbers=[mInputFlangeConnect, mOutputFlangeConnect],
                           constrainedAxes=[1, 1, 1, 1, 1, 1],  # 完全固定
                           visualization=VObjectJointGeneric(axesRadius=0.8, axesLength=1.5, show=True)))

# 创建法兰孔的标记点
input_flange_hole_markers = []
output_flange_hole_markers = []
crank_shaft_input_markers = []  # 曲柄轴在输入法兰处的标记
crank_shaft_output_markers = []  # 曲柄轴在输出法兰处的标记

for i_crank in range(n_cranks):
    angle_hole = 2 * np.pi * i_crank / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    
    # 输入法兰孔标记
    mInputHole = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oInputFlange, 
                                                localPosition=[x_hole_center, y_hole_center, 0]))
    input_flange_hole_markers.append(mInputHole)
    
    # 输出法兰孔标记
    mOutputHole = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oOutputFlange, 
                                                 localPosition=[x_hole_center, y_hole_center, 0]))
    output_flange_hole_markers.append(mOutputHole)
    
    # 曲柄轴在输入法兰处的标记（主轴段）
    mCrankInput = mbs.AddMarker(MarkerBodyRigid(bodyNumber=crankshaft_bodies[i_crank], 
                                                 localPosition=[0, 0, z_input_flange]))
    crank_shaft_input_markers.append(mCrankInput)
    
    # 曲柄轴在输出法兰处的标记（主轴段）
    mCrankOutput = mbs.AddMarker(MarkerBodyRigid(bodyNumber=crankshaft_bodies[i_crank], 
                                                  localPosition=[0, 0, z_output_flange]))
    crank_shaft_output_markers.append(mCrankOutput)

print(f"创建了输入输出法兰系统")
print(f"  输入法兰位置: Z = {z_input_flange:.2f} mm")
print(f"  输出法兰位置: Z = {z_output_flange:.2f} mm")
print(f"  两法兰之间: 刚性固定（形成一体）")
print(f"  法兰系统相对地面: 可绕Z轴旋转")
print(f"  法兰孔半径: {r_flange_hole:.3f} mm")
print(f"  法兰孔分布圆半径: {crank_distribution_radius} mm")
print(f"  法兰与曲柄轴通过滚针轴承连接")

# =================== 摆线轮（带3个均布孔） ===================
mass_cycloid = 8.0
I_cycloid = RigidBodyInertia(mass=mass_cycloid, inertiaTensor=np.array([[500,0,0],[0,500,0],[0,0,1000]]), com=[0,0,0])

# 第一片摆线轮
cycloid1_graphics = []
# 外齿廓
for i in range(len(x_cycloid1)):
    i_next = (i + 1) % len(x_cycloid1)
    cycloid1_graphics.append({'type':'Line','color':[0.9,0.1,0.1,1],'data':[x_cycloid1[i],y_cycloid1[i],0,x_cycloid1[i_next],y_cycloid1[i_next],0]})

# 3个均布的孔
n_hole_points = 40
for i_hole in range(n_cranks):
    angle_hole = 2 * np.pi * i_hole / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    for i in range(n_hole_points):
        angle1 = i * 2 * np.pi / n_hole_points
        angle2 = (i + 1) * 2 * np.pi / n_hole_points
        x1 = x_hole_center + r_hole * np.cos(angle1)
        y1 = y_hole_center + r_hole * np.sin(angle1)
        x2 = x_hole_center + r_hole * np.cos(angle2)
        y2 = y_hole_center + r_hole * np.sin(angle2)
        cycloid1_graphics.append({'type':'Line','color':[0.1,0.7,0.1,1],'data':[x1,y1,0,x2,y2,0]})

# 第一片摆线轮初始位置：与第一个曲柄轴的偏心段对齐
oCycloid1 = mbs.CreateRigidBody(referencePosition=[eccentric_offset, 0, z_eccentric1], inertia=I_cycloid, gravity=[0,0,0], graphicsDataList=cycloid1_graphics)
mCycloid1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid1, localPosition=[0,0,0]))
mGroundCycloid1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[eccentric_offset, 0, z_eccentric1]))
mbs.AddObject(GenericJoint(markerNumbers=[mGroundCycloid1, mCycloid1], constrainedAxes=[0,0,1,1,1,0], visualization=VObjectJointGeneric(axesRadius=0.5, axesLength=1)))

# 第二片摆线轮
cycloid2_graphics = []
# 外齿廓
for i in range(len(x_cycloid2)):
    i_next = (i + 1) % len(x_cycloid2)
    cycloid2_graphics.append({'type':'Line','color':[0.6,0.1,0.9,1],'data':[x_cycloid2[i],y_cycloid2[i],0,x_cycloid2[i_next],y_cycloid2[i_next],0]})

# 3个均布的孔
for i_hole in range(n_cranks):
    angle_hole = 2 * np.pi * i_hole / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    for i in range(n_hole_points):
        angle1 = i * 2 * np.pi / n_hole_points
        angle2 = (i + 1) * 2 * np.pi / n_hole_points
        x1 = x_hole_center + r_hole * np.cos(angle1)
        y1 = y_hole_center + r_hole * np.sin(angle1)
        x2 = x_hole_center + r_hole * np.cos(angle2)
        y2 = y_hole_center + r_hole * np.sin(angle2)
        cycloid2_graphics.append({'type':'Line','color':[0.1,0.7,0.9,1],'data':[x1,y1,0,x2,y2,0]})

# 第二片摆线轮初始位置：与第二个曲柄轴的偏心段对齐
oCycloid2 = mbs.CreateRigidBody(referencePosition=[-eccentric_offset, 0, z_eccentric2], inertia=I_cycloid, gravity=[0,0,0], graphicsDataList=cycloid2_graphics)
mCycloid2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid2, localPosition=[0,0,0]))
mGroundCycloid2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[-eccentric_offset, 0, z_eccentric2]))
mbs.AddObject(GenericJoint(markerNumbers=[mGroundCycloid2, mCycloid2], constrainedAxes=[0,0,1,1,1,0], visualization=VObjectJointGeneric(axesRadius=0.5, axesLength=1)))

# 创建摆线轮孔的标记（用于轴承连接）
cycloid1_hole_markers = []
cycloid2_hole_markers = []
for i_hole in range(n_cranks):
    angle_hole = 2 * np.pi * i_hole / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    
    mHole1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid1, localPosition=[x_hole_center, y_hole_center, 0]))
    mHole2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oCycloid2, localPosition=[x_hole_center, y_hole_center, 0]))
    cycloid1_hole_markers.append(mHole1)
    cycloid2_hole_markers.append(mHole2)

# 注意：输出法兰不与摆线轮直接连接，仅通过法兰轴承与曲柄轴连接

# =================== 创建滚针轴承 ===================
# 为6个轴承（3个曲柄 × 2个摆线轮）创建滚针
contactStiffness_bearing = 1e7  # 轴承接触刚度（降低以提高稳定性）
contactDamping_bearing = 5e2    # 轴承接触阻尼

mass_needle = 0.05
inertia_needle = InertiaCylinder(density=mass_needle*100, length=0.1, outerRadius=r_needle, axis=2)

# 存储所有滚针信息
all_needle_bodies = []  # 所有滚针刚体列表
all_needle_markers = []  # 所有滚针标记列表
bearing_info = []  # 存储每个轴承的信息：(曲柄标记, 摆线轮孔标记, 滚针标记列表, z位置)

# 为每个曲柄-摆线轮配对创建一圈滚针
for i_crank in range(n_cranks):
    # 第一片摆线轮的轴承
    angle_hole = 2 * np.pi * i_crank / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    z_bearing1 = z_eccentric1
    
    needle_bodies_1 = []
    needle_markers_1 = []
    
    for i_needle in range(n_needles):
        angle_needle = 2 * np.pi * i_needle / n_needles
        # 滚针位置相对于轴承中心
        x_needle = x_hole_center + eccentric_offset + r_needle_pitch * np.cos(angle_needle)
        y_needle = y_hole_center + r_needle_pitch * np.sin(angle_needle)
        
        oNeedle = mbs.CreateRigidBody(
            referencePosition=[x_needle, y_needle, z_bearing1],
            inertia=inertia_needle,
            gravity=[0, 0, 0],
            graphicsDataList=[graphics.Cylinder(
                pAxis=[0, 0, -2], vAxis=[0, 0, 4],
                radius=r_needle, color=graphics.color.darkgrey, nTiles=16)]
        )
        
        # 约束滚针只能在XY平面内平移（禁止旋转和Z方向移动）
        mGroundNeedle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                                       localPosition=[x_needle, y_needle, z_bearing1]))
        mNeedle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle, localPosition=[0, 0, 0]))
        mbs.AddObject(GenericJoint(markerNumbers=[mGroundNeedle, mNeedle],
                                   constrainedAxes=[0, 0, 1, 1, 1, 1],  # 只允许XY平移
                                   visualization=VObjectJointGeneric(axesRadius=0.1, axesLength=0.2, show=False)))
        
        mNeedleMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle, localPosition=[0, 0, 0]))
        needle_bodies_1.append(oNeedle)
        needle_markers_1.append(mNeedleMarker)
        all_needle_bodies.append(oNeedle)
        all_needle_markers.append(mNeedleMarker)
    
    # 固定滚针之间的相对位置（形成保持架效果）
    for i_needle in range(n_needles):
        i_next = (i_needle + 1) % n_needles
        mNeedle1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=needle_bodies_1[i_needle], 
                                                  localPosition=[0, 0, 0]))
        mNeedle2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=needle_bodies_1[i_next], 
                                                  localPosition=[0, 0, 0]))
        # 使用距离约束固定相邻滚针间距
        angle_diff = 2 * np.pi / n_needles
        expected_distance = 2 * r_needle_pitch * np.sin(angle_diff / 2)
        mbs.AddObject(DistanceConstraint(markerNumbers=[mNeedle1, mNeedle2],
                                         distance=expected_distance,
                                         visualization=VObjectConnectorDistance(show=False)))
    
    bearing_info.append((crankshaft_markers_ecc1[i_crank], cycloid1_hole_markers[i_crank], 
                        needle_markers_1, z_bearing1))
    
    # 第二片摆线轮的轴承
    z_bearing2 = z_eccentric2
    needle_bodies_2 = []
    needle_markers_2 = []
    
    for i_needle in range(n_needles):
        angle_needle = 2 * np.pi * i_needle / n_needles
        # 滚针位置相对于轴承中心
        x_needle = x_hole_center - eccentric_offset + r_needle_pitch * np.cos(angle_needle)
        y_needle = y_hole_center + r_needle_pitch * np.sin(angle_needle)
        
        oNeedle = mbs.CreateRigidBody(
            referencePosition=[x_needle, y_needle, z_bearing2],
            inertia=inertia_needle,
            gravity=[0, 0, 0],
            graphicsDataList=[graphics.Cylinder(
                pAxis=[0, 0, -2], vAxis=[0, 0, 4],
                radius=r_needle, color=graphics.color.darkgrey, nTiles=16)]
        )
        
        # 约束滚针只能在XY平面内平移
        mGroundNeedle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                                       localPosition=[x_needle, y_needle, z_bearing2]))
        mNeedle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle, localPosition=[0, 0, 0]))
        mbs.AddObject(GenericJoint(markerNumbers=[mGroundNeedle, mNeedle],
                                   constrainedAxes=[0, 0, 1, 1, 1, 1],  # 只允许XY平移
                                   visualization=VObjectJointGeneric(axesRadius=0.1, axesLength=0.2, show=False)))
        
        mNeedleMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle, localPosition=[0, 0, 0]))
        needle_bodies_2.append(oNeedle)
        needle_markers_2.append(mNeedleMarker)
        all_needle_bodies.append(oNeedle)
        all_needle_markers.append(mNeedleMarker)
    
    # 固定滚针之间的相对位置
    for i_needle in range(n_needles):
        i_next = (i_needle + 1) % n_needles
        mNeedle1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=needle_bodies_2[i_needle], 
                                                  localPosition=[0, 0, 0]))
        mNeedle2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=needle_bodies_2[i_next], 
                                                  localPosition=[0, 0, 0]))
        angle_diff = 2 * np.pi / n_needles
        expected_distance = 2 * r_needle_pitch * np.sin(angle_diff / 2)
        mbs.AddObject(DistanceConstraint(markerNumbers=[mNeedle1, mNeedle2],
                                         distance=expected_distance,
                                         visualization=VObjectConnectorDistance(show=False)))
    
    bearing_info.append((crankshaft_markers_ecc2[i_crank], cycloid2_hole_markers[i_crank], 
                        needle_markers_2, z_bearing2))

print(f"创建了 {len(bearing_info)} 个摆线轮滚针轴承，共 {len(all_needle_bodies)} 个滚针")

# =================== 创建法兰轴承（法兰孔-曲柄主轴） ===================
contactStiffness_flange_bearing = 1e7
contactDamping_flange_bearing = 5e2

mass_flange_needle = 0.03
inertia_flange_needle = InertiaCylinder(density=mass_flange_needle*100, length=0.1, 
                                        outerRadius=r_flange_needle, axis=2)

all_flange_needle_bodies = []
all_flange_needle_markers = []
flange_bearing_info = []  # (法兰孔标记, 曲柄轴标记, 滚针标记列表, z位置, 是否输入法兰)

# 为每个曲柄轴创建输入法兰轴承
for i_crank in range(n_cranks):
    angle_hole = 2 * np.pi * i_crank / n_cranks
    x_hole_center = crank_distribution_radius * np.cos(angle_hole)
    y_hole_center = crank_distribution_radius * np.sin(angle_hole)
    
    # 输入法兰轴承滚针
    needle_bodies_input = []
    needle_markers_input = []
    
    for i_needle in range(n_flange_needles):
        angle_needle = 2 * np.pi * i_needle / n_flange_needles
        x_needle = x_hole_center + r_flange_needle_pitch * np.cos(angle_needle)
        y_needle = y_hole_center + r_flange_needle_pitch * np.sin(angle_needle)
        
        oNeedle = mbs.CreateRigidBody(
            referencePosition=[x_needle, y_needle, z_input_flange],
            inertia=inertia_flange_needle,
            gravity=[0, 0, 0],
            graphicsDataList=[graphics.Cylinder(
                pAxis=[0, 0, -1.5], vAxis=[0, 0, 3],
                radius=r_flange_needle, color=graphics.color.grey, nTiles=16)]
        )
        
        # 约束滚针只能在XY平面内平移
        mGroundNeedle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                                       localPosition=[x_needle, y_needle, z_input_flange]))
        mNeedle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle, localPosition=[0, 0, 0]))
        mbs.AddObject(GenericJoint(markerNumbers=[mGroundNeedle, mNeedle],
                                   constrainedAxes=[0, 0, 1, 1, 1, 1],
                                   visualization=VObjectJointGeneric(axesRadius=0.1, axesLength=0.2, show=False)))
        
        mNeedleMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle, localPosition=[0, 0, 0]))
        needle_bodies_input.append(oNeedle)
        needle_markers_input.append(mNeedleMarker)
        all_flange_needle_bodies.append(oNeedle)
        all_flange_needle_markers.append(mNeedleMarker)
    
    # 固定滚针之间的相对位置
    for i_needle in range(n_flange_needles):
        i_next = (i_needle + 1) % n_flange_needles
        mNeedle1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=needle_bodies_input[i_needle], 
                                                  localPosition=[0, 0, 0]))
        mNeedle2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=needle_bodies_input[i_next], 
                                                  localPosition=[0, 0, 0]))
        angle_diff = 2 * np.pi / n_flange_needles
        expected_distance = 2 * r_flange_needle_pitch * np.sin(angle_diff / 2)
        mbs.AddObject(DistanceConstraint(markerNumbers=[mNeedle1, mNeedle2],
                                         distance=expected_distance,
                                         visualization=VObjectConnectorDistance(show=False)))
    
    flange_bearing_info.append((input_flange_hole_markers[i_crank], 
                                crank_shaft_input_markers[i_crank], 
                                needle_markers_input, z_input_flange, True))
    
    # 输出法兰轴承滚针
    needle_bodies_output = []
    needle_markers_output = []
    
    for i_needle in range(n_flange_needles):
        angle_needle = 2 * np.pi * i_needle / n_flange_needles
        x_needle = x_hole_center + r_flange_needle_pitch * np.cos(angle_needle)
        y_needle = y_hole_center + r_flange_needle_pitch * np.sin(angle_needle)
        
        oNeedle = mbs.CreateRigidBody(
            referencePosition=[x_needle, y_needle, z_output_flange],
            inertia=inertia_flange_needle,
            gravity=[0, 0, 0],
            graphicsDataList=[graphics.Cylinder(
                pAxis=[0, 0, -1.5], vAxis=[0, 0, 3],
                radius=r_flange_needle, color=graphics.color.grey, nTiles=16)]
        )
        
        # 约束滚针只能在XY平面内平移
        mGroundNeedle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, 
                                                       localPosition=[x_needle, y_needle, z_output_flange]))
        mNeedle = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle, localPosition=[0, 0, 0]))
        mbs.AddObject(GenericJoint(markerNumbers=[mGroundNeedle, mNeedle],
                                   constrainedAxes=[0, 0, 1, 1, 1, 1],
                                   visualization=VObjectJointGeneric(axesRadius=0.1, axesLength=0.2, show=False)))
        
        mNeedleMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle, localPosition=[0, 0, 0]))
        needle_bodies_output.append(oNeedle)
        needle_markers_output.append(mNeedleMarker)
        all_flange_needle_bodies.append(oNeedle)
        all_flange_needle_markers.append(mNeedleMarker)
    
    # 固定滚针之间的相对位置
    for i_needle in range(n_flange_needles):
        i_next = (i_needle + 1) % n_flange_needles
        mNeedle1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=needle_bodies_output[i_needle], 
                                                  localPosition=[0, 0, 0]))
        mNeedle2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=needle_bodies_output[i_next], 
                                                  localPosition=[0, 0, 0]))
        angle_diff = 2 * np.pi / n_flange_needles
        expected_distance = 2 * r_flange_needle_pitch * np.sin(angle_diff / 2)
        mbs.AddObject(DistanceConstraint(markerNumbers=[mNeedle1, mNeedle2],
                                         distance=expected_distance,
                                         visualization=VObjectConnectorDistance(show=False)))
    
    flange_bearing_info.append((output_flange_hole_markers[i_crank], 
                                crank_shaft_output_markers[i_crank], 
                                needle_markers_output, z_output_flange, False))

print(f"创建了 {len(flange_bearing_info)} 个法兰轴承，共 {len(all_flange_needle_bodies)} 个法兰滚针")

# 针齿壳（可视部分）
z_shell = (z_eccentric1 + z_eccentric2) / 2
shell_graphics_list = []
n_shell_outer_points = 120
for i in range(n_shell_outer_points):
    angle1 = i * 2 * np.pi / n_shell_outer_points
    angle2 = (i + 1) * 2 * np.pi / n_shell_outer_points
    x1 = R_shell_outer * np.cos(angle1); y1 = R_shell_outer * np.sin(angle1)
    x2 = R_shell_outer * np.cos(angle2); y2 = R_shell_outer * np.sin(angle2)
    shell_graphics_list.append({'type':'Line','color':[0.5,0.5,0.5,1],'data':[x1,y1,z_shell,x2,y2,z_shell]})
for i in range(n_shell_outer_points):
    angle1 = i * 2 * np.pi / n_shell_outer_points
    angle2 = (i + 1) * 2 * np.pi / n_shell_outer_points
    x1 = R_shell_inner * np.cos(angle1); y1 = R_shell_inner * np.sin(angle1)
    x2 = R_shell_inner * np.cos(angle2); y2 = R_shell_inner * np.sin(angle2)
    shell_graphics_list.append({'type':'Line','color':[0.5,0.5,0.5,1],'data':[x1,y1,z_shell,x2,y2,z_shell]})

n_pins_show = 40
n_hole_circle_points = 48
for i in range(n_pins_show):
    angle_pin = 2 * np.pi * (i + 0.25) / n_pins_show
    x_hole_center = r_pin_shell * np.sin(angle_pin)
    y_hole_center = r_pin_shell * np.cos(angle_pin)
    for j in range(n_hole_circle_points):
        angle1 = j * 2 * np.pi / n_hole_circle_points
        angle2 = (j + 1) * 2 * np.pi / n_hole_circle_points
        x1 = x_hole_center + r_pin_hole * np.cos(angle1)
        y1 = y_hole_center + r_pin_hole * np.sin(angle1)
        x2 = x_hole_center + r_pin_hole * np.cos(angle2)
        y2 = y_hole_center + r_pin_hole * np.sin(angle2)
        shell_graphics_list.append({'type':'Line','color':[0.9,0.5,0.1,1],'data':[x1,y1,z_shell,x2,y2,z_shell]})

mbs.CreateGround(graphicsDataList=shell_graphics_list)

def CreatePinHoleSegments(hole_radius, hole_center, nPoints=40):
    pList = []
    for i in range(nPoints):
        phi = i * 2 * np.pi / nPoints
        x = hole_center[0] + hole_radius * np.cos(phi)
        y = hole_center[1] + hole_radius * np.sin(phi)
        pList.append([x, y])
    nSeg = len(pList)
    segmentsData = np.zeros((nSeg, 4))
    for i in range(nSeg):
        segmentsData[i, 0:2] = pList[i]
        segmentsData[i, 2:4] = pList[(i + 1) % nSeg]
    return segmentsData, nSeg

# 针齿（可运动）
pin_bodies = []
pin_markers = []
pin_radii = []
pin_hole_centers = []
pin_load_ids = []
pin_load_sensor_ids = []

mass_pin = 0.3
inertia_pin = InertiaCylinder(density=mass_pin*100, length=0.1, outerRadius=r_z, axis=2)

for i in range(n_pins_show):
    angle = 2 * np.pi * (i + 0.25) / n_pins_show
    x_pin = r_pin_shell * np.sin(angle)
    y_pin = r_pin_shell * np.cos(angle)
    z_pin = z_shell
    oPin = mbs.CreateRigidBody(referencePosition=[x_pin, y_pin, z_pin], inertia=inertia_pin, gravity=[0,0,0], graphicsDataList=[graphics.Cylinder(pAxis=[0,0,-10], vAxis=[0,0,20], radius=r_z, color=graphics.color.steelblue, nTiles=32)])
    mGroundPinPlane = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[x_pin, y_pin, z_pin]))
    mPinPlane = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oPin, localPosition=[0, 0, 0]))
    mbs.AddObject(GenericJoint(markerNumbers=[mGroundPinPlane, mPinPlane], constrainedAxes=[0,0,1,1,1,0], visualization=VObjectJointGeneric(axesRadius=0.15, axesLength=0.4, show=False)))
    mPin = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oPin, localPosition=[0,0,0]))
    pin_markers.append(mPin)
    pin_radii.append(r_z)
    pin_bodies.append(oPin)
    pin_hole_centers.append((x_pin, y_pin))

# 外齿廓-针齿接触（仍用内置 ContactCurveCircles）
initialCoords_tooth1 = []
for _ in range(nSeg_tooth1):
    initialCoords_tooth1.extend([-1.0, 0.0, 0.0])
nGenericData_tooth1 = mbs.AddNode(NodeGenericData(initialCoordinates=initialCoords_tooth1, numberOfDataCoordinates=3 * nSeg_tooth1))
mbs.AddObject(ObjectContactCurveCircles(markerNumbers=[mCycloid1] + pin_markers, nodeNumber=nGenericData_tooth1, circlesRadii=pin_radii, segmentsData=exu.MatrixContainer(segmentsData_tooth1), contactStiffness=contactStiffness_tooth, contactDamping=contactDamping_tooth, visualization=VObjectContactCurveCircles(show=True, color=graphics.color.blue)))

initialCoords_tooth2 = []
for _ in range(nSeg_tooth2):
    initialCoords_tooth2.extend([-1.0, 0.0, 0.0])
nGenericData_tooth2 = mbs.AddNode(NodeGenericData(initialCoordinates=initialCoords_tooth2, numberOfDataCoordinates=3 * nSeg_tooth2))
mbs.AddObject(ObjectContactCurveCircles(markerNumbers=[mCycloid2] + pin_markers, nodeNumber=nGenericData_tooth2, circlesRadii=pin_radii, segmentsData=exu.MatrixContainer(segmentsData_tooth2), contactStiffness=contactStiffness_tooth, contactDamping=contactDamping_tooth, visualization=VObjectContactCurveCircles(show=True, color=graphics.color.magenta)))

""" 滚针轴承接触：使用圆-圆接触模型"""
# 创建偏心轴-滚针、滚针-摆线轮孔的接触负载

# 1. 偏心轴外圆 - 滚针接触（凸圆-凸圆）
def CrankNeedleContactLoadUF(mbs, t, loadVector, crankMarker, needleBody, r_crank, r_needle, kn, cn):
    # 获取偏心轴位置和速度
    pos_crank = mbs.GetMarkerOutput(crankMarker, exu.OutputVariableType.Position)
    vel_crank = mbs.GetMarkerOutput(crankMarker, exu.OutputVariableType.Velocity)
    c1 = np.array([pos_crank[0], pos_crank[1]])
    v1 = np.array([vel_crank[0], vel_crank[1]])
    
    # 获取滚针位置和速度
    nodeNumber = mbs.GetObject(needleBody)['nodeNumber']
    pos_needle = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Position)
    vel_needle = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Velocity)
    c2 = np.array([pos_needle[0], pos_needle[1]])
    v2 = np.array([vel_needle[0], vel_needle[1]])
    
    # 使用圆-圆接触（两个凸圆）
    out = circle_circle_penalty(c1, r_crank, v1, c2, r_needle, v2, kn=kn, cn=cn)
    
    return [out['F1'][0], out['F1'][1], 0.0]

# 2. 滚针 - 摆线轮孔接触（凸圆-凹内壁）
def NeedleHoleContactLoadUF(mbs, t, loadVector, needleBody, cycloidMarker, r_needle, r_hole, kn, cn):
    # 获取滚针位置和速度
    nodeNumber = mbs.GetObject(needleBody)['nodeNumber']
    pos_needle = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Position)
    vel_needle = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Velocity)
    c_pin = np.array([pos_needle[0], pos_needle[1]])
    v_pin = np.array([vel_needle[0], vel_needle[1]])
    
    # 获取摆线轮孔位置和速度
    pos_cycloid = mbs.GetMarkerOutput(cycloidMarker, exu.OutputVariableType.Position)
    vel_cycloid = mbs.GetMarkerOutput(cycloidMarker, exu.OutputVariableType.Velocity)
    c_hole = np.array([pos_cycloid[0], pos_cycloid[1]])
    v_hole = np.array([vel_cycloid[0], vel_cycloid[1]])
    
    # 使用圆-圆孔接触（滚针在摆线轮孔内）
    out = circle_in_hole_penalty(c_pin, r_needle, v_pin, c_hole, r_hole, v_hole, kn=kn, cn=cn)
    
    return [out['F1'][0], out['F1'][1], 0.0]

# 创建所有滚针轴承的接触负载
bearing_load_ids = []
bearing_contact_count = 0

for (crankMarker, cycloidMarker, needle_markers, z_bearing) in bearing_info:
    for needle_marker in needle_markers:
        needleBody = mbs.GetMarker(needle_marker)['bodyNumber']
        
        # 偏心轴-滚针接触
        lid1 = mbs.AddLoad(LoadForceVector(
            markerNumber=crankMarker,
            loadVector=[0, 0, 0],
            loadVectorUserFunction=lambda mbs,t,load, crank=crankMarker, needle=needleBody: 
                CrankNeedleContactLoadUF(mbs, t, load, crank, needle, r_crank_eccentric, r_needle, 
                                        contactStiffness_bearing, contactDamping_bearing)
        ))
        bearing_load_ids.append(lid1)
        bearing_contact_count += 1
        
        # 滚针-摆线轮孔接触
        lid2 = mbs.AddLoad(LoadForceVector(
            markerNumber=needle_marker,
            loadVector=[0, 0, 0],
            loadVectorUserFunction=lambda mbs,t,load, needle=needleBody, cycloid=cycloidMarker: 
                NeedleHoleContactLoadUF(mbs, t, load, needle, cycloid, r_needle, r_hole, 
                                       contactStiffness_bearing, contactDamping_bearing)
        ))
        bearing_load_ids.append(lid2)
        bearing_contact_count += 1

print(f"创建了 {bearing_contact_count} 个摆线轮轴承接触负载（{len(bearing_info)} 个轴承 × {n_needles} 个滚针 × 2种接触）")

# =================== 法兰轴承接触负载 ===================
flange_bearing_load_ids = []
flange_bearing_contact_count = 0

# 法兰轴承接触函数（曲柄主轴-滚针）
def FlangeCrankNeedleContactLoadUF(mbs, t, loadVector, crankMarker, needleBody, r_shaft, r_needle, kn, cn):
    pos_crank = mbs.GetMarkerOutput(crankMarker, exu.OutputVariableType.Position)
    vel_crank = mbs.GetMarkerOutput(crankMarker, exu.OutputVariableType.Velocity)
    c1 = np.array([pos_crank[0], pos_crank[1]])
    v1 = np.array([vel_crank[0], vel_crank[1]])
    
    nodeNumber = mbs.GetObject(needleBody)['nodeNumber']
    pos_needle = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Position)
    vel_needle = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Velocity)
    c2 = np.array([pos_needle[0], pos_needle[1]])
    v2 = np.array([vel_needle[0], vel_needle[1]])
    
    out = circle_circle_penalty(c1, r_shaft, v1, c2, r_needle, v2, kn=kn, cn=cn)
    return [out['F1'][0], out['F1'][1], 0.0]

# 法兰孔-滚针接触函数
def FlangeHoleNeedleContactLoadUF(mbs, t, loadVector, needleBody, flangeMarker, r_needle, r_hole, kn, cn):
    nodeNumber = mbs.GetObject(needleBody)['nodeNumber']
    pos_needle = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Position)
    vel_needle = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Velocity)
    c_pin = np.array([pos_needle[0], pos_needle[1]])
    v_pin = np.array([vel_needle[0], vel_needle[1]])
    
    pos_flange = mbs.GetMarkerOutput(flangeMarker, exu.OutputVariableType.Position)
    vel_flange = mbs.GetMarkerOutput(flangeMarker, exu.OutputVariableType.Velocity)
    c_hole = np.array([pos_flange[0], pos_flange[1]])
    v_hole = np.array([vel_flange[0], vel_flange[1]])
    
    out = circle_in_hole_penalty(c_pin, r_needle, v_pin, c_hole, r_hole, v_hole, kn=kn, cn=cn)
    return [out['F1'][0], out['F1'][1], 0.0]

# 创建所有法兰轴承的接触负载
for (flangeMarker, crankMarker, needle_markers, z_pos, is_input) in flange_bearing_info:
    for needle_marker in needle_markers:
        needleBody = mbs.GetMarker(needle_marker)['bodyNumber']
        
        # 曲柄主轴-滚针接触
        lid1 = mbs.AddLoad(LoadForceVector(
            markerNumber=crankMarker,
            loadVector=[0, 0, 0],
            loadVectorUserFunction=lambda mbs,t,load, crank=crankMarker, needle=needleBody: 
                FlangeCrankNeedleContactLoadUF(mbs, t, load, crank, needle, r_flange_shaft, r_flange_needle, 
                                               contactStiffness_flange_bearing, contactDamping_flange_bearing)
        ))
        flange_bearing_load_ids.append(lid1)
        flange_bearing_contact_count += 1
        
        # 滚针-法兰孔接触
        lid2 = mbs.AddLoad(LoadForceVector(
            markerNumber=needle_marker,
            loadVector=[0, 0, 0],
            loadVectorUserFunction=lambda mbs,t,load, needle=needleBody, flange=flangeMarker: 
                FlangeHoleNeedleContactLoadUF(mbs, t, load, needle, flange, r_flange_needle, r_flange_hole, 
                                              contactStiffness_flange_bearing, contactDamping_flange_bearing)
        ))
        flange_bearing_load_ids.append(lid2)
        flange_bearing_contact_count += 1

print(f"创建了 {flange_bearing_contact_count} 个法兰轴承接触负载（{len(flange_bearing_info)} 个轴承 × {n_flange_needles} 个滚针 × 2种接触）")

# ============ 替换的接触5：针齿-针齿壳孔（用带符号半径的圆-圆接触负载） ============
def PinHoleLoadUF(mbs, t, loadVector, pinBody, holeCenter, r_pin, r_hole, kn, cn):
    node = mbs.GetObject(pinBody)['nodeNumber']
    pos = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)
    vel = mbs.GetNodeOutput(node, exu.OutputVariableType.Velocity)
    c1 = np.array([pos[0], pos[1]])
    v1 = np.array([vel[0], vel[1]])
    c2 = np.array([holeCenter[0], holeCenter[1]])
    v2 = np.array([0.0, 0.0])
    out = circle_in_hole_penalty(c1, r_pin, v1, c2, r_hole, v2, kn=kn, cn=cn)
    return [out['F1'][0], out['F1'][1], 0.0]

for (mPin, oPin, holeCenter) in zip(pin_markers, pin_bodies, pin_hole_centers):
    lid = mbs.AddLoad(LoadForceVector(
        markerNumber=mPin,
        loadVector=[0, 0, 0],
        loadVectorUserFunction=lambda mbs,t,load, pinBody=oPin, hole=holeCenter: PinHoleLoadUF(
            mbs, t, load, pinBody, hole, r_z, r_pin_hole, contactStiffness_pin_hole, contactDamping_pin_hole)
    ))
    pin_load_ids.append(lid)
    pin_load_sensor_ids.append(mbs.AddSensor(SensorLoad(loadNumber=lid, storeInternal=True)))

# =================== 传感器/组装/仿真 ===================
mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[0, 0, -10], size=200)])

# 传感器（需在仿真前创建并 storeInternal 以记录数据）
sCycloid1Pos = mbs.AddSensor(SensorBody(bodyNumber=oCycloid1, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
sCycloid2Pos = mbs.AddSensor(SensorBody(bodyNumber=oCycloid2, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
sCrankRot    = mbs.AddSensor(SensorBody(bodyNumber=crankshaft_bodies[0], storeInternal=True, outputVariableType=exu.OutputVariableType.Rotation))
sPin0Pos     = mbs.AddSensor(SensorBody(bodyNumber=pin_bodies[0], storeInternal=True, outputVariableType=exu.OutputVariableType.Position))

mbs.Assemble()

simulationSettings = exu.SimulationSettings()
stepSize = 0.001  # 减小步长以提高稳定性
tEnd = 2.0  # 增加仿真时长以获得更多数据点
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.sensorsWritePeriod = 0.01
# 指定并创建解文件输出目录，避免默认文件缺失或被其他会话覆盖
solDir = os.path.join(os.path.dirname(__file__), 'solution')
try:
    os.makedirs(solDir, exist_ok=True)
except Exception:
    pass
solutionFile = os.path.join(solDir, 'cycloid_bearing_solution.txt')
simulationSettings.solutionSettings.coordinatesSolutionFileName = solutionFile
simulationSettings.timeIntegration.numberOfSteps = int(tEnd / stepSize)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.simulateInRealtime = False

# 使用稀疏求解器以处理大规模系统
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

# Newton求解器设置
simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 1e-6
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-6

# Generalized Alpha设置
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.7  # 增加数值阻尼
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True

# 自适应步长
simulationSettings.timeIntegration.adaptiveStep = True
simulationSettings.timeIntegration.adaptiveStepIncrease = 1.5
simulationSettings.timeIntegration.adaptiveStepDecrease = 0.5
simulationSettings.timeIntegration.minimumStepSize = 1e-6

simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.displayComputationTime = True
simulationSettings.displayStatistics = True

SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
SC.visualizationSettings.window.renderWindowSize = [1600, 1200]
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.openGL.shadow = 0.3
SC.visualizationSettings.loads.show = True
SC.visualizationSettings.connectors.showContact = True
SC.visualizationSettings.loads.loadSizeFactor = 1e-4  # 载荷箭头长度
SC.visualizationSettings.loads.defaultRadius = 0.6     # 箭头半径
SC.visualizationSettings.loads.defaultSize = 4.0       # 基础尺寸更大
SC.visualizationSettings.loads.fixedLoadSize = False   # 随力大小缩放（方案B：同步loadVector可见）
SC.visualizationSettings.loads.drawWithUserFunction = True  # 绘制用户函数载荷
SC.visualizationSettings.loads.showNumbers = False

# 内置接触对象（齿廓-针齿）的接触力可视化
SC.visualizationSettings.contact.showContactForces = True
SC.visualizationSettings.contact.contactForcesFactor = 1.0

print("\n" + "="*60)
print("系统组装完成，开始仿真...")
print("="*60)

if useGraphics:
    SC.renderer.Start(); SC.renderer.DoIdleTasks()

import traceback, sys

# 调试计数器
debug_print_counter = [0]

def UFupdateVisLoads(mbs, t):
    try:
        # 收集针齿-孔接触力数据
        pin_hole_forces = []
        for i, lid in enumerate(pin_load_ids):
            F = mbs.GetLoadValues(lid)
            mbs.SetLoadParameter(lid, 'loadVector', [float(F[0]), float(F[1]), float(F[2])])
            F_mag = np.sqrt(F[0]**2 + F[1]**2 + F[2]**2)
            pin_hole_forces.append((i, F_mag))
        
        # 每个solver step打印一次详细信息
        debug_print_counter[0] += 1
        
        # 只在前20步或有接触时打印
        pin_hole_active = [(pid, f) for pid, f in pin_hole_forces if f > 0.01]
        has_contact = len(pin_hole_active) > 0
        
        if has_contact or debug_print_counter[0] <= 20:
            print(f"\n{'='*80}")
            print(f"⏱️  [test_dual_cycloid_with_full_3.py] Step #{debug_print_counter[0]}, 时间: {t:.6f}s")
            print(f"{'='*80}")
            
            # 打印前3个针齿的位置和距离孔中心的距离
            print(f"\n📍 前3个针齿的位置信息:")
            for i in range(min(3, len(pin_bodies))):
                pos_pin = mbs.GetNodeOutput(mbs.GetObject(pin_bodies[i])['nodeNumber'], exu.OutputVariableType.Position)
                hole_center = pin_hole_centers[i]
                d = np.sqrt((pos_pin[0] - hole_center[0])**2 + (pos_pin[1] - hole_center[1])**2)
                S = r_pin_hole - r_z
                print(f"   针齿#{i}: 位置=({pos_pin[0]:.6f}, {pos_pin[1]:.6f}), d={d:.6f}mm, S={S:.6f}mm, 接触={d > S}")
            
            # 打印针齿-孔接触力
            if pin_hole_active:
                print(f"\n🟡 针齿-针齿孔接触力 (共{len(pin_hole_active)}个接触):")
                pin_sorted = sorted(pin_hole_active, key=lambda x: x[1], reverse=True)[:5]
                for pid, f_mag in pin_sorted:
                    # 获取针齿位置和孔中心的距离
                    pos_pin = mbs.GetNodeOutput(mbs.GetObject(pin_bodies[pid])['nodeNumber'], exu.OutputVariableType.Position)
                    hole_center = pin_hole_centers[pid]
                    d = np.sqrt((pos_pin[0] - hole_center[0])**2 + (pos_pin[1] - hole_center[1])**2)
                    S = r_pin_hole - r_z
                    print(f"   针齿#{pid:2d}: F={f_mag:10.4f} N, d={d:.6f}mm, S={S:.6f}mm")
                max_pin = max(f for _, f in pin_hole_active)
                print(f"   ➤ 最大力: {max_pin:.4f} N")
            else:
                print(f"\n🟡 针齿-针齿孔接触力: 无接触 (力<0.01N)")
    except Exception as e:
        print(f"调试输出错误: {e}")
        pass
    return True

mbs.SetPreStepUserFunction(UFupdateVisLoads)
try:
    mbs.SolveDynamic(simulationSettings)
except Exception:
    print("\n仿真出错，打印完整 traceback：")
    traceback.print_exc()
    sys.exit(1)

if useGraphics:
    try:
        SC.renderer.Stop()
    except:
        pass

print("\n" + "="*60)
print("仿真成功完成！")
print("="*60)

import matplotlib.pyplot as plt

# 读取数据并绘图（仿真后）
print("正在读取传感器数据...")
try:
    data_rot = mbs.GetSensorStoredData(sCrankRot)
    print(f"曲柄轴旋转数据形状: {data_rot.shape}")
except Exception as e:
    print(f"读取曲柄轴旋转数据失败: {e}")
    data_rot = None

try:
    data_cyc1 = mbs.GetSensorStoredData(sCycloid1Pos)
    print(f"第一片摆线轮位置数据形状: {data_cyc1.shape}")
except Exception as e:
    print(f"读取第一片摆线轮位置数据失败: {e}")
    data_cyc1 = None

try:
    data_cyc2 = mbs.GetSensorStoredData(sCycloid2Pos)
    print(f"第二片摆线轮位置数据形状: {data_cyc2.shape}")
except Exception as e:
    print(f"读取第二片摆线轮位置数据失败: {e}")
    data_cyc2 = None

fig = plt.figure(figsize=(16, 12))
ax1 = plt.subplot(3, 2, 1)
ax2 = plt.subplot(3, 2, 2)
ax3 = plt.subplot(3, 2, 3)
ax4 = plt.subplot(3, 2, 4)
ax5 = plt.subplot(3, 2, 5)
ax6 = plt.subplot(3, 2, 6)
ax5b = ax5.twinx()  # 在第五幅叠加接触力

if data_rot is not None and len(data_rot) > 0:
    ax1.plot(data_rot[:,0], data_rot[:,3], 'b-', linewidth=2)
    ax1.set_xlabel('Time (s)'); ax1.set_ylabel('Rotation (rad)'); ax1.set_title('Crankshaft Rotation Angle'); ax1.grid(True)
else:
    ax1.text(0.5, 0.5, 'No rotation data', ha='center', va='center', transform=ax1.transAxes)
    ax1.set_title('Crankshaft Rotation Angle (No Data)')

if data_cyc1 is not None and len(data_cyc1) > 0:
    ax2.plot(data_cyc1[:,0], data_cyc1[:,1], 'r-', label='X1', linewidth=2)
    ax2.plot(data_cyc1[:,0], data_cyc1[:,2], 'g-', label='Y1', linewidth=2)
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Position (mm)'); ax2.set_title('Cycloid Wheel 1 Position'); ax2.legend(); ax2.grid(True)
else:
    ax2.text(0.5, 0.5, 'No cycloid 1 data', ha='center', va='center', transform=ax2.transAxes)
    ax2.set_title('Cycloid Wheel 1 Position (No Data)')

if data_cyc2 is not None and len(data_cyc2) > 0:
    ax3.plot(data_cyc2[:,0], data_cyc2[:,1], 'r--', label='X2', linewidth=2)
    ax3.plot(data_cyc2[:,0], data_cyc2[:,2], 'g--', label='Y2', linewidth=2)
    ax3.set_xlabel('Time (s)'); ax3.set_ylabel('Position (mm)'); ax3.set_title('Cycloid Wheel 2 Position'); ax3.legend(); ax3.grid(True)
else:
    ax3.text(0.5, 0.5, 'No cycloid 2 data', ha='center', va='center', transform=ax3.transAxes)
    ax3.set_title('Cycloid Wheel 2 Position (No Data)')

if data_cyc1 is not None and len(data_cyc1) > 0:
    ax4.plot(data_cyc1[:,1], data_cyc1[:,2], 'r-', linewidth=2, label='Cycloid 1')
    ax4.plot(data_cyc1[0,1], data_cyc1[0,2], 'go', markersize=10, label='Start 1')
    ax4.plot(data_cyc1[-1,1], data_cyc1[-1,2], 'ro', markersize=10, label='End 1')
    ax4.set_xlabel('X (mm)'); ax4.set_ylabel('Y (mm)'); ax4.set_title('Cycloid Wheel 1 Trajectory'); ax4.axis('equal'); ax4.legend(); ax4.grid(True)
else:
    ax4.text(0.5, 0.5, 'No cycloid 1 data', ha='center', va='center', transform=ax4.transAxes)
    ax4.set_title('Cycloid Wheel 1 Trajectory (No Data)')

if data_cyc2 is not None and len(data_cyc2) > 0:
    ax5.plot(data_cyc2[:,1], data_cyc2[:,2], 'b-', linewidth=2, label='Cycloid 2')
    ax5.plot(data_cyc2[0,1], data_cyc2[0,2], 'mo', markersize=10, label='Start 2')
    ax5.plot(data_cyc2[-1,1], data_cyc2[-1,2], 'co', markersize=10, label='End 2')
    ax5.set_xlabel('X (mm)'); ax5.set_ylabel('Y (mm)'); ax5.set_title('Cycloid Wheel 2 Trajectory'); ax5.axis('equal'); ax5.legend(); ax5.grid(True)
else:
    ax5.text(0.5, 0.5, 'No cycloid 2 data', ha='center', va='center', transform=ax5.transAxes)
    ax5.set_title('Cycloid Wheel 2 Trajectory (No Data)')

# 取部分载荷的力曲线（示例：第一个针齿）
if len(pin_load_sensor_ids) > 0:
    dFpin0 = mbs.GetSensorStoredData(pin_load_sensor_ids[0])
    ax5b.plot(dFpin0[:,0], np.hypot(dFpin0[:,1], dFpin0[:,2]), 'k-', alpha=0.6, label='|F_pin0|')
ax5b.set_ylabel('Force (N)')
ax5b.legend(loc='lower right')

if data_cyc1 is not None and len(data_cyc1) > 0:
    ax6.plot(data_cyc1[:,1], data_cyc1[:,2], 'r-', linewidth=2, label='Cycloid 1', alpha=0.7)
    ax6.plot(data_cyc1[0,1], data_cyc1[0,2], 'ro', markersize=10)
if data_cyc2 is not None and len(data_cyc2) > 0:
    ax6.plot(data_cyc2[:,1], data_cyc2[:,2], 'b-', linewidth=2, label='Cycloid 2', alpha=0.7)
    ax6.plot(data_cyc2[0,1], data_cyc2[0,2], 'bo', markersize=10)

for i in range(40):
    angle = 2 * np.pi * (i+0.25) / 40
    x_pin = r_pin_shell * np.sin(angle); y_pin = r_pin_shell * np.cos(angle)
    circle = plt.Circle((x_pin, y_pin), r_z, color='gray', fill=False, alpha=0.3)
    ax6.add_patch(circle)
ax6.set_xlabel('X (mm)'); ax6.set_ylabel('Y (mm)'); ax6.set_title('Both Cycloid Wheels Trajectories'); ax6.axis('equal'); ax6.legend(); ax6.grid(True)

plt.tight_layout(); plt.show()

print("\n启动运动回放器...")
print("="*60)
print("可视化说明（三曲柄轴双片摆线轮 + 滚针轴承）：")
print("  【三曲柄轴系统】")
print("    - 3根曲柄轴，120°均布在分布圆上")
print("    - 每根曲柄轴包含：")
print("      * 灰色圆柱：主轴段")
print("      * 红色圆柱：第一个偏心段（正向）")
print("      * 橙色圆柱：第二个偏心段（反向）")
print("  【滚针轴承】")
print(f"    - 6个滚针轴承（3曲柄 × 2摆线轮）")
print(f"    - 每个轴承有{n_needles}个深灰色小圆柱滚针")
print(f"    - 滚针通过距离约束保持相对位置固定")
print(f"    - 滚针半径: {r_needle} mm，节圆半径: {r_needle_pitch:.2f} mm")
print("  【第一片摆线轮】（下方）")
print("    - 红色轮廓线：第一片摆线轮齿廓")
print(f"    - 3个绿色内孔圆：孔半径 {r_hole:.2f} mm")
print("    - 通过滚针轴承与3个红色偏心段连接")
print("  【第二片摆线轮】（上方）")
print("    - 紫色轮廓线：第二片摆线轮齿廓（相位差180°）")
print(f"    - 3个青色内孔圆：孔半径 {r_hole:.2f} mm")
print("    - 通过滚针轴承与3个橙色偏心段连接")
print("  【针齿壳}")
print("    - 灰色双层圆：针齿壳内外圈")
print("    - 40个橙色小圆：针齿孔位置标记")
print("  【针齿}")
print("    - 蓝色圆柱：40个针齿（同时与两片摆线轮接触）")
print("  【接触}")
print("    - 蓝色接触线：第一片摆线轮 <-> 针齿")
print("    - 紫色接触线：第二片摆线轮 <-> 针齿")
print("    - 圆-圆接触：偏心轴外圆 <-> 滚针")
print("    - 圆-圆孔接触：滚针 <-> 摆线轮孔内壁")
print("="*60)

def _is_valid_solution_file(path):
    try:
        if not os.path.isfile(path):
            return False
        with open(path, 'r', encoding='utf-8', errors='ignore') as f:
            # 读前几行，寻找包含多个数值的行
            linesChecked = 0
            for line in f:
                s = line.strip()
                if not s or s.startswith('#'):
                    continue
                tokens = s.split(',')  # 解文件用逗号分隔
                # 至少包含时间+一个坐标列
                if len(tokens) >= 2:
                    return True
                linesChecked += 1
                if linesChecked > 10:  # 减少检查行数
                    break
        return False
    except Exception:
        return False

# 现在使用简化的圆-圆接触模型，可以正常使用SolutionViewer回放器
print("使用简化的圆-圆接触模型，可以正常回放")

# 启动SolutionViewer回放器
if useGraphics:
    try:
        # 尝试简单的回放器启动
        mbs.SolutionViewer()
    except Exception as e:
        print(f"启动回放器时出错: {e}")
        print("尝试使用基础回放器...")
        try:
            # 备用方案：使用基础回放器
            from exudyn.interactive import SolutionViewer
            SolutionViewer(mbs)
        except Exception as e2:
            print(f"基础回放器也失败: {e2}")
            print("请检查SolutionViewer是否可用")

print("\n程序结束")

visForceScale = 0.02  # 可视化力箭头缩放（单位依场景调）
# ========== 基于 graphicsDataUserFunction 的力箭头可视化（仅针齿） ==========

def UFgraphicsForces(mbs, itemNumber):
    g = []
    # 针齿力（最多绘制8个以免太密）
    maxPinsToDraw = min(8, len(pin_load_ids))
    for i in range(maxPinsToDraw):
        lid = pin_load_ids[i]
        F = mbs.GetLoadValues(lid)  # [Fx, Fy, Fz]
        P = mbs.GetMarkerOutput(pin_markers[i], exu.OutputVariableType.Position)
        g.append(graphics.Arrow(pAxis=[P[0], P[1], P[2]],
                                vAxis=[F[0]*visForceScale, F[1]*visForceScale, F[2]*visForceScale],
                                radius=0.6, color=graphics.color.yellow, nTiles=50))
    return g

# 添加一个仅用于绘制的 ground 对象以调用图形回调
mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsDataUserFunction=UFgraphicsForces)))


