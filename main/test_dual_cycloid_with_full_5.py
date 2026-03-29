import sys, os
import numpy as np
from scipy.optimize import minimize_scalar

# =================== 接触力调试输出功能 ===================
# 创建调试输出文件
contact_debug_file = None
contact_debug_enabled = True
contact_debug_interval = 0.0  # 输出间隔时间（秒）- 修改为0表示每步都输出
last_debug_time = 0.0
contact_debug_buffer = []  # 缓冲区，用于批量写入以提高性能
contact_debug_buffer_size = 10  # 缓冲区大小，已减少到10以便更快看到输出
contact_debug_every_step = True  # 控制是否每步都输出
contact_debug_call_count = 0  # 添加调用计数器
contact_debug_write_count = 0  # 添加写入计数器

# 新增控制选项
contact_debug_console_output = True  # 控制是否在控制台显示详细的接触力信息
contact_debug_console_interval = 100  # 控制台输出间隔（每N步输出一次）
contact_debug_console_max_forces = 5  # 控制台显示的最大接触力数量
contact_debug_console_force_threshold = 1.0  # 接触力阈值，只显示大于此值的力（N）
contact_debug_step_counter = 0  # 步数计数器，用于控制输出频率

def init_contact_debug_output():
    """初始化接触力调试输出文件"""
    global contact_debug_file
    if contact_debug_enabled:
        try:
            debug_path = os.path.join(os.path.dirname(__file__), 'ContactForces_debug.txt')
            contact_debug_file = open(debug_path, 'w', encoding='utf-8')
            # 写入表头
            contact_debug_file.write("# 接触力调试输出文件（每步输出）\n")
            contact_debug_file.write("# 格式: 时间, 接触类型, 力大小(X,Y,Z), 力矩大小(X,Y,Z), 作用点坐标(X,Y,Z)\n")
            contact_debug_file.write("# 接触类型: CycloidTooth(摆线轮-针齿), CrankNeedle(偏心轴-滚针), NeedleHole(滚针-孔), FlangeCrank(法兰-曲柄), FlangeHole(法兰-孔), PinHole(针齿-孔)\n")
            contact_debug_file.write("# 时间(s), 接触类型, Fx(N), Fy(N), Fz(N), Mx(N·mm), My(N·mm), Mz(N·mm), Px(mm), Py(mm), Pz(mm)\n")
            contact_debug_file.flush()
            print(f"接触力调试输出文件已创建: {debug_path}")
            print("注意: 现在每步仿真都会输出接触力信息，可能会生成大量数据")
            print(f"缓冲区大小: {contact_debug_buffer_size} 行，每10步刷新一次缓冲区")
            print("如需修改输出频率，可调整 contact_debug_every_step 变量")
            
            # 新增控制选项说明
            print("\n接触力控制台输出控制:")
            print(f"  控制台详细输出: {'启用' if contact_debug_console_output else '禁用'}")
            print(f"  控制台输出间隔: 每 {contact_debug_console_interval} 步输出一次")
            print(f"  最大显示力数量: {contact_debug_console_max_forces} 个")
            print(f"  力阈值: {contact_debug_console_force_threshold} N (只显示大于此值的力)")
            if contact_debug_console_output:
                print("  启用后，控制台将显示具体的接触力值而不是计数信息")
            else:
                print("  禁用时，控制台只显示计数信息")
        except Exception as e:
            print(f"创建接触力调试输出文件失败: {e}")
            contact_debug_file = None

def flush_contact_debug_buffer():
    """将缓冲区中的数据写入文件"""
    global contact_debug_file, contact_debug_buffer, contact_debug_write_count
    global contact_debug_step_counter, contact_debug_console_output, contact_debug_console_interval
    global contact_debug_console_max_forces, contact_debug_console_force_threshold
    
    if contact_debug_file is not None and len(contact_debug_buffer) > 0:
        try:
            # 先写入文件
            for item in contact_debug_buffer:
                line = item[0]  # 获取原始行数据
                contact_debug_file.write(line)
            contact_debug_file.flush()
            contact_debug_write_count += 1
            
            # 控制台输出详细接触力信息
            if contact_debug_console_output and contact_debug_step_counter % contact_debug_console_interval == 0:
                print(f"\n[接触力详情] 时间: {contact_debug_buffer[0][3]:.6f}s")
                print("="*60)
                
                # 按力大小排序，只显示最大的几个力
                sorted_forces = sorted(contact_debug_buffer, key=lambda x: x[1], reverse=True)
                displayed_count = 0
                
                for item in sorted_forces:
                    line, force_magnitude, contact_type, t, force, point = item
                    
                    # 只显示大于阈值的力
                    if force_magnitude >= contact_debug_console_force_threshold and displayed_count < contact_debug_console_max_forces:
                        print(f"  {contact_type}:")
                        print(f"    力大小: {force_magnitude:.3f} N")
                        print(f"    力分量: Fx={force[0]:.3f} N, Fy={force[1]:.3f} N, Fz={force[2]:.3f} N")
                        print(f"    作用点: ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}) mm")
                        displayed_count += 1
                
                if displayed_count == 0:
                    print(f"  无大于阈值 {contact_debug_console_force_threshold} N 的接触力")
                else:
                    print(f"  显示了 {displayed_count} 个最大接触力（共 {len(contact_debug_buffer)} 个）")
                print("="*60)
            else:
                # 简单的计数信息
                print(f"[调试] 接触力写入第{contact_debug_write_count}次，调用次数: {contact_debug_call_count}")
            
            contact_debug_buffer.clear()
        except Exception as e:
            print(f"刷新接触力调试缓冲区失败: {e}")

def write_contact_debug(t, contact_type, force, moment, point):
    """写入接触力调试信息"""
    global contact_debug_file, last_debug_time, contact_debug_buffer, contact_debug_buffer_size
    global contact_debug_call_count, contact_debug_write_count, contact_debug_step_counter
    
    # 增加调用计数器和步数计数器
    contact_debug_call_count += 1
    contact_debug_step_counter += 1
    
    # 如果禁用每步输出，则使用原来的间隔控制
    if not contact_debug_every_step and (t - last_debug_time) < contact_debug_interval:
        return
    
    if contact_debug_file is not None:
        try:
            # 确保force和moment是3D向量，如果是2D则补0
            if len(force) == 2:
                force = np.array([force[0], force[1], 0.0])
            if len(moment) == 2:
                moment = np.array([moment[0], moment[1], 0.0])
            if len(point) == 2:
                point = np.array([point[0], point[1], 0.0])
                
            # 计算力的总大小
            force_magnitude = np.sqrt(force[0]**2 + force[1]**2 + force[2]**2)
                
            # 格式: 时间, 接触类型, 力大小(X,Y,Z), 力矩大小(X,Y,Z), 作用点坐标
            line = f"{t:.6f}, {contact_type}, {force[0]:.6f}, {force[1]:.6f}, {force[2]:.6f}, "
            line += f"{moment[0]:.6f}, {moment[1]:.6f}, {moment[2]:.6f}, "
            line += f"{point[0]:.6f}, {point[1]:.6f}, {point[2]:.6f}\n"
            
            # 添加到缓冲区
            contact_debug_buffer.append((line, force_magnitude, contact_type, t, force, point))
            
            # 如果缓冲区满了，写入文件
            if len(contact_debug_buffer) >= contact_debug_buffer_size:
                flush_contact_debug_buffer()
                
            last_debug_time = t
        except Exception as e:
            print(f"写入接触力调试信息失败: {e}")

def close_contact_debug_output():
    """关闭接触力调试输出文件"""
    global contact_debug_file
    if contact_debug_file is not None:
        try:
            # 在关闭前刷新缓冲区
            flush_contact_debug_buffer()
            contact_debug_file.close()
            contact_debug_file = None
            print("接触力调试输出文件已关闭")
        except Exception as e:
            print(f"关闭接触力调试输出文件失败: {e}")

# =================== 接触力调试输出功能结束 ===================

# =================== 约束设置修改说明 ===================
#
# 本文件已修改二维体的约束设置，解决自由度约束问题：
#
# 1. 摆线轮约束修改：
#    - 允许XY平面内的平移和Z轴旋转 (constrainedAxes=[0,0,1,1,1,0])
#    - 这样摆线轮可以进行必要的偏心运动，同时绕Z轴旋转
#    - 约束Z轴平移和XY轴旋转，确保摆线轮保持在正确的工作平面
#
# 2. 法兰系统约束修改：
#    - 输入法兰：允许XY平面内平移和Z轴旋转 (constrainedAxes=[0,0,1,1,1,0])
#    - 输入输出法兰连接：允许相对Z轴旋转 (constrainedAxes=[1,1,1,1,1,0])
#    - 这样法兰系统可以支撑整个系统，同时允许必要的相对运动
#
# 3. 滚针轴承约束修改：
#    - 所有滚针：允许XY平面内平移，约束Z平移和所有旋转 (constrainedAxes=[0,0,1,1,1,0])
#    - 这样滚针可以在XY平面内自由移动以适应轴承接触
#
# 4. 针齿约束修改：
#    - 允许XY平面内平移和Z轴旋转 (constrainedAxes=[0,0,1,1,1,0])
#    - 这样针齿可以适应与摆线轮的接触，同时可以绕Z轴旋转
#
# 5. 曲柄轴同步约束：
#    - 只约束Z轴旋转同步，允许其他自由度 (constrainedAxes=[0,0,0,0,0,1])
#    - 确保三个曲柄轴同步旋转，但允许独立平移
#
# 这些修改确保系统能够正确模拟摆线针轮减速器的工作原理，
# 允许必要的偏心运动，同时保持系统的稳定性。
# =====================================================

# =================== 齿廓理论函数类 ===================
class CycloidProfileFunction:
    """摆线轮齿廓理论函数 - 可计算任意参数对应的点"""
    
    def __init__(self, z_b, e, R_z, r_z, phi_h, delta_e=0):
        self.z_b = z_b
        self.z_g = z_b - 1
        self.e = e
        self.R_z = R_z
        self.r_z = r_z
        self.phi_h = phi_h
        self.delta_e = delta_e
        
        # 计算参数
        self.second_delta_rp = 0
        self.second_delta_r_rp = 0
        self.second_delta = 0
        self.second_zp = z_b
        self.second_a = e + delta_e
        self.second_rp = R_z
        self.second_r_rp = r_z
        self.second_i_H = self.second_zp / (self.second_zp - 1)
        self.phi_b_max = 2 * (self.second_zp - 1) * np.pi
    
    def compute_point(self, phi_b):
        """计算参数phi_b对应的齿廓点坐标"""
        second_k1_pie = self.second_a * self.second_zp / (self.second_rp - self.second_delta_rp)
        second_s_pie = 1 + second_k1_pie**2 - 2 * second_k1_pie * np.cos(phi_b)
        
        x_2j = -((self.second_rp - self.second_delta_rp) - (self.second_r_rp + self.second_delta_r_rp) * second_s_pie**(-1/2)) * \
               np.sin((1 - self.second_i_H) * phi_b - self.second_delta) - \
               self.second_a / (self.second_rp - self.second_delta_rp) * \
               ((self.second_rp - self.second_delta_rp) - self.second_zp * (self.second_r_rp + self.second_delta_r_rp) * second_s_pie**(-1/2)) * \
               np.sin(self.second_i_H * phi_b + self.second_delta)
        
        y_2j = ((self.second_rp - self.second_delta_rp) - (self.second_r_rp + self.second_delta_r_rp) * second_s_pie**(-1/2)) * \
               np.cos((1 - self.second_i_H) * phi_b - self.second_delta) - \
               self.second_a / (self.second_rp - self.second_delta_rp) * \
               ((self.second_rp - self.second_delta_rp) - self.second_zp * (self.second_r_rp + self.second_delta_r_rp) * second_s_pie**(-1/2)) * \
               np.cos(self.second_i_H * phi_b + self.second_delta)
        
        phi_g = self.phi_h / self.z_g
        x = x_2j * np.cos(phi_g) - y_2j * np.sin(phi_g)
        y = x_2j * np.sin(phi_g) + y_2j * np.cos(phi_g)
        
        return x, y

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

# =================== 自定义摆线轮齿廓-针齿接触函数 ===================
class CycloidToothPinContact:
    """优化的摆线轮齿廓-针齿接触计算类（支持理论函数插值）"""
    
    def __init__(self, x_profile, y_profile, r_pin, kn, cn, search_range=50, max_iter=10, 
                 profile_function=None, use_optimization=True):
        """
        x_profile, y_profile: 齿廓离散点坐标（局部坐标系）
        r_pin: 针齿半径
        kn, cn: 接触刚度和阻尼
        search_range: 初始搜索范围（齿廓点数）
        max_iter: 最近点搜索的最大迭代次数（离散点迭代用）
        profile_function: CycloidProfileFunction对象（可选，用于理论函数优化）
        use_optimization: 是否启用理论函数优化（需要提供profile_function）
        """
        self.x_profile = np.array(x_profile)
        self.y_profile = np.array(y_profile)
        self.r_pin = r_pin
        self.kn = kn
        self.cn = cn
        self.search_range = search_range
        self.max_iter = max_iter
        self.n_points = len(x_profile)
        
        # 理论函数优化相关
        self.profile_func = profile_function
        self.use_optimization = use_optimization and (profile_function is not None)
        
        # 如果启用优化，预计算离散点对应的参数phi_b
        if self.use_optimization:
            self.phi_b_discrete = np.linspace(0, self.profile_func.phi_b_max, self.n_points)
        
        # 预计算切向量（用于快速查找最近点）
        self.tangents = np.zeros((self.n_points, 2))
        for i in range(self.n_points):
            i_next = (i + 1) % self.n_points
            dx = self.x_profile[i_next] - self.x_profile[i]
            dy = self.y_profile[i_next] - self.y_profile[i]
            length = np.sqrt(dx**2 + dy**2)
            if length > 1e-9:
                self.tangents[i] = np.array([dx/length, dy/length])
            else:
                self.tangents[i] = np.array([1.0, 0.0])
        
        # 缓存：记录每个针齿上次接触的齿廓索引（用于加速搜索）
        self.last_contact_idx = {}
        
        # 统计信息
        self.optimization_count = 0
        self.optimization_improvement_sum = 0.0
    
    def transform_profile_to_global(self, pos_cycloid, rot_cycloid):
        """将齿廓从局部坐标系转换到全局坐标系"""
        # rot_cycloid 是旋转角度（绕Z轴）
        cos_theta = np.cos(rot_cycloid)
        sin_theta = np.sin(rot_cycloid)
        
        x_global = pos_cycloid[0] + cos_theta * self.x_profile - sin_theta * self.y_profile
        y_global = pos_cycloid[1] + sin_theta * self.x_profile + cos_theta * self.y_profile
        
        # 转换切向量
        tangents_global = np.zeros_like(self.tangents)
        tangents_global[:, 0] = cos_theta * self.tangents[:, 0] - sin_theta * self.tangents[:, 1]
        tangents_global[:, 1] = sin_theta * self.tangents[:, 0] + cos_theta * self.tangents[:, 1]
        
        return x_global, y_global, tangents_global
    
    def find_closest_point_on_profile(self, pin_center, x_global, y_global, tangents_global, pin_id):
        """
        找到齿廓上离针齿中心最近的点
        使用优化算法：先粗略搜索，再精细迭代
        """
        # Step 1: 粗略搜索 - 找到离针齿最近的齿廓段
        if pin_id in self.last_contact_idx:
            # 如果有缓存，从上次接触点附近开始搜索
            start_idx = max(0, self.last_contact_idx[pin_id] - self.search_range // 2)
            end_idx = min(self.n_points, self.last_contact_idx[pin_id] + self.search_range // 2)
            search_indices = list(range(start_idx, end_idx))
        else:
            # 第一次接触，搜索整个齿廓
            search_indices = list(range(self.n_points))
        
        # 计算所有搜索点到针齿中心的距离
        distances = np.sqrt((x_global[search_indices] - pin_center[0])**2 + 
                           (y_global[search_indices] - pin_center[1])**2)
        
        # 找到最近的点
        min_idx_in_search = np.argmin(distances)
        closest_idx = search_indices[min_idx_in_search]
        
        # Step 2: 精细迭代 - 在最近点附近找到真正的最近点
        # 条件：从针齿中心到齿廓点的向量与齿廓切线垂直
        best_idx = closest_idx
        best_dist = distances[min_idx_in_search]
        
        for iteration in range(self.max_iter):
            # 当前点
            p_current = np.array([x_global[best_idx], y_global[best_idx]])
            
            # 从针齿中心指向当前点的向量
            vec_to_point = p_current - pin_center
            dist_current = np.linalg.norm(vec_to_point)
            
            if dist_current < 1e-9:
                break
            
            vec_to_point_normalized = vec_to_point / dist_current
            
            # 当前点的切向量
            tangent = tangents_global[best_idx]
            
            # 计算投影：看向量是否垂直于切线
            # 如果垂直，dot product应该接近0
            dot_product = np.dot(vec_to_point_normalized, tangent)
            
            # 如果已经足够垂直，停止迭代
            if abs(dot_product) < 0.0001:
                break
            
            # 否则，根据dot_product的符号决定向哪个方向移动
            if dot_product > 0:
                # 需要向后移动（索引减小）
                next_idx = (best_idx - 1) % self.n_points
            else:
                # 需要向前移动（索引增加）
                next_idx = (best_idx + 1) % self.n_points
            
            # 检查新点是否更近
            p_next = np.array([x_global[next_idx], y_global[next_idx]])
            dist_next = np.linalg.norm(p_next - pin_center)
            
            if dist_next < dist_current:
                best_idx = next_idx
                best_dist = dist_next
            else:
                # 如果不更近，说明已经找到局部最小值
                break
        
        # 更新缓存
        self.last_contact_idx[pin_id] = best_idx
        
        # 返回最近点的坐标、索引、距离
        closest_point = np.array([x_global[best_idx], y_global[best_idx]])
        return closest_point, best_idx, best_dist
    
    def optimize_closest_point_with_theory(self, pin_center, pos_cycloid, rot_cycloid, phi_b_init):
        """
        使用齿廓理论函数在参数空间优化，找到真正的最近点
        
        pin_center: 针齿中心（全局坐标）
        pos_cycloid: 摆线轮位置（全局坐标）
        rot_cycloid: 摆线轮旋转角度
        phi_b_init: 初始参数（来自离散搜索）
        
        返回：最优phi_b, 最小距离, 最近点坐标
        """
        def distance_squared(phi_b):
            """目标函数：针齿中心到齿廓点的距离平方"""
            # 使用理论函数计算齿廓点（局部坐标）
            x_local, y_local = self.profile_func.compute_point(phi_b)
            
            # 转换到全局坐标
            cos_theta = np.cos(rot_cycloid)
            sin_theta = np.sin(rot_cycloid)
            x_global = pos_cycloid[0] + cos_theta * x_local - sin_theta * y_local
            y_global = pos_cycloid[1] + sin_theta * x_local + cos_theta * y_local
            
            # 计算距离平方
            dist_sq = (x_global - pin_center[0])**2 + (y_global - pin_center[1])**2
            return dist_sq
        
        # 在phi_b_init附近进行局部优化
        # 搜索范围：±5%的参数空间
        search_range = 0.05 * self.profile_func.phi_b_max
        bounds = (max(0, phi_b_init - search_range), 
                 min(self.profile_func.phi_b_max, phi_b_init + search_range))
        
        # 使用Brent方法进行一维优化（无需导数）
        result = minimize_scalar(distance_squared, bounds=bounds, method='bounded')
        
        optimal_phi_b = result.x
        min_dist = np.sqrt(result.fun)
        
        # 计算最优点坐标（全局）
        x_local, y_local = self.profile_func.compute_point(optimal_phi_b)
        cos_theta = np.cos(rot_cycloid)
        sin_theta = np.sin(rot_cycloid)
        x_global = pos_cycloid[0] + cos_theta * x_local - sin_theta * y_local
        y_global = pos_cycloid[1] + sin_theta * x_local + cos_theta * y_local
        
        return optimal_phi_b, min_dist, np.array([x_global, y_global])
    
    def compute_contact_force(self, pin_center, pin_velocity, cycloid_pos, cycloid_rot, 
                             cycloid_velocity, cycloid_angular_velocity, pin_id):
        """
        计算针齿与摆线轮齿廓的接触力
        返回：作用在针齿上的力，作用在摆线轮上的力
        """
        # 步骤1：将齿廓转换到全局坐标系，进行离散点搜索
        x_global, y_global, tangents_global = self.transform_profile_to_global(cycloid_pos, cycloid_rot)
        
        # 找到最近的离散点
        closest_point_discrete, idx, dist_discrete = self.find_closest_point_on_profile(
            pin_center, x_global, y_global, tangents_global, pin_id)
        
        # 步骤2：如果启用理论函数优化，则进一步精确搜索
        if self.use_optimization:
            # 获取离散点对应的参数phi_b
            phi_b_init = self.phi_b_discrete[idx]
            
            # 使用理论函数优化
            phi_b_optimal, dist_optimal, closest_point_optimal = self.optimize_closest_point_with_theory(
                pin_center, cycloid_pos[:2], cycloid_rot, phi_b_init)
            
            # 使用优化后的结果
            closest_point = closest_point_optimal
            dist = dist_optimal
            
            # 统计优化效果
            improvement = dist_discrete - dist_optimal
            if improvement > 0:
                self.optimization_count += 1
                self.optimization_improvement_sum += improvement
        else:
            # 不使用优化，直接用离散点结果
            closest_point = closest_point_discrete
            dist = dist_discrete
        
        # 计算间隙（gap）
        gap = dist - self.r_pin
        
        # 如果没有接触（gap > 0），返回零力
        if gap > 0:
            return np.zeros(2), np.zeros(2), gap, closest_point
        
        # 有接触，计算法向量（从齿廓点指向针齿中心）
        vec_to_pin = pin_center - closest_point
        if dist < 1e-9:
            # 避免除零
            normal = np.array([1.0, 0.0])
        else:
            normal = vec_to_pin / dist
        
        # 计算穿透量
        delta = -gap  # 穿透量为正
        
        # 计算相对速度
        # 摆线轮上接触点的速度 = 平移速度 + 旋转速度
        # v_point = v_cycloid + omega × r (omega是绕Z轴，r是从摆线轮中心到接触点)
        r_vec = closest_point - cycloid_pos[:2]
        v_cycloid_at_point = cycloid_velocity[:2] + cycloid_angular_velocity[2] * np.array([-r_vec[1], r_vec[0]])
        
        v_rel = pin_velocity[:2] - v_cycloid_at_point
        v_n = float(np.dot(v_rel, normal))  # 法向相对速度
        
        # 计算法向力（惩罚力 + 阻尼）
        F_n = self.kn * delta + self.cn * delta * max(-v_n, 0.0)
        
        if F_n < 0.0:
            F_n = 0.0
        
        # 法向力向量
        F_normal = F_n * normal
        
        # 作用在针齿上的力（指向齿廓，即负法向）
        F_pin = F_normal
        
        # 作用在摆线轮上的力（反作用力）
        F_cycloid = -F_normal
        
        return F_pin, F_cycloid, gap, closest_point

# 添加带调试输出的接触力计算函数
def compute_contact_force_with_debug(contact_obj, pin_center, pin_velocity, cycloid_pos, cycloid_rot,
                                   cycloid_velocity, cycloid_angular_velocity, pin_id, contact_type, t):
    """计算接触力并输出调试信息"""
    F_pin, F_cycloid, gap, contact_point = contact_obj.compute_contact_force(
        pin_center, pin_velocity, cycloid_pos, cycloid_rot, cycloid_velocity, cycloid_angular_velocity, pin_id)
    
    # 输出调试信息（现在每步都会输出）
    if contact_debug_enabled and hasattr(contact_obj, 'compute_contact_force'):
        # 计算力矩（简化为0，因为点接触）
        moment = np.zeros(3)
        # 输出针齿上的力
        write_contact_debug(t, f"{contact_type}_Pin", F_pin, moment, contact_point)
        # 输出摆线轮上的反作用力
        cycloid_point = np.array([cycloid_pos[0], cycloid_pos[1], cycloid_pos[2] if len(cycloid_pos) > 2 else 0])
        write_contact_debug(t, f"{contact_type}_Cycloid", F_cycloid, moment, cycloid_point)
    
    return F_pin, F_cycloid, gap, contact_point

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
bearing_clearance = 0.002  # 轴承径向间隙（很小但避免初始穿透）
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

contactStiffness_tooth = 1e8  # 降低齿廓接触刚度以提高收敛性（从2e8改为1e8）
contactDamping_tooth = 5e2    # 增加齿廓接触阻尼以提高稳定性（从2e2改为5e2）
contactStiffness_hole = 5e8   # 降低孔接触刚度（从1e9改为5e8）
contactDamping_hole = 2e2     # 增加孔接触阻尼（从1e2改为2e2）
contactStiffness_pin_hole = 2e8  # 降低针齿-孔接触刚度（从5e8改为2e8）
contactDamping_pin_hole = 1e2    # 增加针齿-孔接触阻尼（从5e1改为1e2）

# 法兰轴承参数
r_flange_shaft = r_crank_main  # 法兰孔与曲柄主轴接触（主轴半径）
r_flange_needle = 1.0  # 法兰轴承滚针半径（比摆线轮轴承小）
n_flange_needles = 10  # 每个法兰轴承的滚针数量
flange_bearing_clearance = 0.002
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

x_cycloid1, y_cycloid1 = GenerateCycloidProfile(phi_h=0, n_points=8000)
x_cycloid2, y_cycloid2 = GenerateCycloidProfile(phi_h=np.pi, n_points=8000)

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
    
    # 曲柄轴通过法兰轴承约束XY轴转动和Z轴平动，无需额外GenericJoint
    # 法兰轴承的滚针接触已提供必要的约束
    
    # 为所有曲柄轴添加扭矩控制（均匀分布力矩）
    def CrankTorqueControl(mbs, t, load, crankBody=oCrank):
        nodeNumber = mbs.GetObject(crankBody)['nodeNumber']
        angVel = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.AngularVelocity)
        omega_current = angVel[2]
        
        # 增大控制增益以提高输入力矩
        Kp = 5000.0  # 从1000.0增加到5000.0，提高5倍
        
        # 计算误差
        error = omega_target - omega_current
        
        # 简单的比例控制
        torque = Kp * error
        
        # 限制最大扭矩，避免过大导致不稳定
        max_torque = 5000.0  # 最大扭矩限制为5000 N·mm
        torque = max(min(torque, max_torque), -max_torque)
        
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

# 曲柄轴同步约束：约束Z轴旋转同步，允许其他自由度
# 这样三个曲柄轴可以独立平移，但旋转保持同步
for i_crank in range(1, n_cranks):
    mCrank0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=crankshaft_bodies[0], localPosition=[0, 0, 0]))
    mCrankI = mbs.AddMarker(MarkerBodyRigid(bodyNumber=crankshaft_bodies[i_crank], localPosition=[0, 0, 0]))
    # 只约束Z轴旋转同步，允许X、Y、Z平移和X、Y旋转
    # constrainedAxes=[0,0,0,0,0,1] 表示：[X平移, Y平移, Z平移, X旋转, Y旋转, Z旋转]
    # 这样确保三个曲柄轴同步旋转，但允许它们在空间中有独立的平移以适应轴承接触
    mbs.AddObject(GenericJoint(markerNumbers=[mCrank0, mCrankI],
                               constrainedAxes=[0,0,0,0,0,1],  # 只约束Z轴旋转
                               visualization=VObjectJointGeneric(axesRadius=0.3, axesLength=0.6, show=False)))

print(f"创建了 {n_cranks} 个曲柄轴，分布圆半径: {crank_distribution_radius} mm")
print(f"曲柄轴约束方式: 通过法兰轴承约束，Z轴旋转同步")

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
# 修改法兰约束：允许Z轴旋转和有限的XY平移，约束其他自由度
# constrainedAxes=[0,0,1,1,1,0] 表示：[X平移, Y平移, Z平移, X旋转, Y旋转, Z旋转]
# 允许法兰在XY平面内有微小移动（由轴承接触决定），同时可以绕Z轴旋转
mbs.AddObject(GenericJoint(markerNumbers=[mGroundInputFlange, mInputFlange],
                           constrainedAxes=[0, 0, 0, 1, 1, 0],  # 允许XY平移和Z轴旋转
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
# 修改法兰间连接：允许相对Z轴旋转，约束其他自由度
# constrainedAxes=[1,1,1,1,1,0] 表示：[X平移, Y平移, Z平移, X旋转, Y旋转, Z旋转]
# 这样允许输入输出法兰之间有相对Z轴旋转，更符合实际减速器的工作原理
mbs.AddObject(GenericJoint(markerNumbers=[mInputFlangeConnect, mOutputFlangeConnect],
                           constrainedAxes=[1, 1, 0, 1, 1, 0],  # 允许相对Z轴旋转
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
# 修改摆线轮约束：允许XY平面内平移和Z轴旋转，约束其他自由度
# constrainedAxes=[0,0,1,1,1,0] 表示：[X平移, Y平移, Z平移, X旋转, Y旋转, Z旋转]
# 0=不约束(自由), 1=约束(固定)
# 这样允许摆线轮在XY平面内做偏心运动，同时可以绕Z轴旋转
# 修改摆线轮约束：稍微限制XY平移以提高稳定性，但仍允许必要的偏心运动
# constrainedAxes=[0,0,0,1,1,0] 表示：[X平移, Y平移, Z平移, X旋转, Y旋转, Z旋转]
# 使用虚约束代替完全自由，提高数值稳定性
mbs.AddObject(GenericJoint(markerNumbers=[mGroundCycloid1, mCycloid1],
                          constrainedAxes=[0,0,1,1,1,0],  # 约束Z平移，保持XY平移和Z旋转自由
                          visualization=VObjectJointGeneric(axesRadius=0.5, axesLength=1)))

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
# 修改摆线轮约束：允许XY平面内平移和Z轴旋转，约束其他自由度
# 与第一片摆线轮相同的约束设置，确保两片摆线轮都能进行必要的偏心运动
# 修改摆线轮约束：与第一片摆线轮相同的约束设置
mbs.AddObject(GenericJoint(markerNumbers=[mGroundCycloid2, mCycloid2],
                          constrainedAxes=[0,0,1,1,1,0],  # 约束Z平移，保持XY平移和Z旋转自由
                          visualization=VObjectJointGeneric(axesRadius=0.5, axesLength=1)))

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
contactStiffness_bearing = 1e7  # 降低轴承接触刚度以提高收敛性（从2e7改为1e7）
contactDamping_bearing = 2e3    # 增加轴承接触阻尼以提高稳定性（从1e3改为2e3）

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
        # 滚针约束：允许XY平面内平移，约束Z平移和所有旋转
        # constrainedAxes=[0,0,1,1,1,0] 表示：[X平移, Y平移, Z平移, X旋转, Y旋转, Z旋转]
        # 这样滚针可以在XY平面内自由移动以适应轴承接触，但不会旋转或沿Z轴移动
        mbs.AddObject(GenericJoint(markerNumbers=[mGroundNeedle, mNeedle],
                                   constrainedAxes=[0, 0, 0, 1, 1, 0],  # 只允许XY平移
                                   visualization=VObjectJointGeneric(axesRadius=0.1, axesLength=0.2, show=False)))
        
        mNeedleMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle, localPosition=[0, 0, 0]))
        needle_bodies_1.append(oNeedle)
        needle_markers_1.append(mNeedleMarker)
        all_needle_bodies.append(oNeedle)
        all_needle_markers.append(mNeedleMarker)
    
    # 注释掉固定滚针之间的相对位置（去除保持架效果）
    # for i_needle in range(n_needles):
    #     i_next = (i_needle + 1) % n_needles
    #     mNeedle1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=needle_bodies_1[i_needle],
    #                                               localPosition=[0, 0, 0]))
    #     mNeedle2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=needle_bodies_1[i_next],
    #                                               localPosition=[0, 0, 0]))
    #     # 使用距离约束固定相邻滚针间距
    #     angle_diff = 2 * np.pi / n_needles
    #     expected_distance = 2 * r_needle_pitch * np.sin(angle_diff / 2)
    #     mbs.AddObject(DistanceConstraint(markerNumbers=[mNeedle1, mNeedle2],
    #                                      distance=expected_distance,
    #                                      visualization=VObjectConnectorDistance(show=False)))
    
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
        # 滚针约束：允许XY平面内平移，约束Z平移和所有旋转
        # 与第一片摆线轮的滚针约束相同，确保一致性
        mbs.AddObject(GenericJoint(markerNumbers=[mGroundNeedle, mNeedle],
                                   constrainedAxes=[0, 0, 0, 1, 1, 0],  # 只允许XY平移
                                   visualization=VObjectJointGeneric(axesRadius=0.1, axesLength=0.2, show=False)))
        
        mNeedleMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle, localPosition=[0, 0, 0]))
        needle_bodies_2.append(oNeedle)
        needle_markers_2.append(mNeedleMarker)
        all_needle_bodies.append(oNeedle)
        all_needle_markers.append(mNeedleMarker)
    
    # 注释掉固定滚针之间的相对位置
    # for i_needle in range(n_needles):
    #     i_next = (i_needle + 1) % n_needles
    #     mNeedle1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=needle_bodies_2[i_needle],
    #                                               localPosition=[0, 0, 0]))
    #     mNeedle2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=needle_bodies_2[i_next],
    #                                               localPosition=[0, 0, 0]))
    #     angle_diff = 2 * np.pi / n_needles
    #     expected_distance = 2 * r_needle_pitch * np.sin(angle_diff / 2)
    #     mbs.AddObject(DistanceConstraint(markerNumbers=[mNeedle1, mNeedle2],
    #                                      distance=expected_distance,
    #                                      visualization=VObjectConnectorDistance(show=False)))
    
    bearing_info.append((crankshaft_markers_ecc2[i_crank], cycloid2_hole_markers[i_crank], 
                        needle_markers_2, z_bearing2))

print(f"创建了 {len(bearing_info)} 个摆线轮滚针轴承，共 {len(all_needle_bodies)} 个滚针")

# =================== 创建法兰轴承（法兰孔-曲柄主轴） ===================
contactStiffness_flange_bearing = 1e7  # 降低法兰轴承接触刚度
contactDamping_flange_bearing = 5e2    # 增加法兰轴承接触阻尼

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
        # 法兰滚针约束：允许XY平面内平移，约束Z平移和所有旋转
        # 与摆线轮滚针约束相同，确保所有滚针行为一致
        mbs.AddObject(GenericJoint(markerNumbers=[mGroundNeedle, mNeedle],
                                   constrainedAxes=[0, 0, 0, 1, 1, 0],
                                   visualization=VObjectJointGeneric(axesRadius=0.1, axesLength=0.2, show=False)))
        
        mNeedleMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle, localPosition=[0, 0, 0]))
        needle_bodies_input.append(oNeedle)
        needle_markers_input.append(mNeedleMarker)
        all_flange_needle_bodies.append(oNeedle)
        all_flange_needle_markers.append(mNeedleMarker)
    
    # 注释掉固定滚针之间的相对位置
    # for i_needle in range(n_flange_needles):
    #     i_next = (i_needle + 1) % n_flange_needles
    #     mNeedle1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=needle_bodies_input[i_needle],
    #                                               localPosition=[0, 0, 0]))
    #     mNeedle2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=needle_bodies_input[i_next],
    #                                               localPosition=[0, 0, 0]))
    #     angle_diff = 2 * np.pi / n_flange_needles
    #     expected_distance = 2 * r_flange_needle_pitch * np.sin(angle_diff / 2)
    #     mbs.AddObject(DistanceConstraint(markerNumbers=[mNeedle1, mNeedle2],
    #                                      distance=expected_distance,
    #                                      visualization=VObjectConnectorDistance(show=False)))
    
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
        # 法兰滚针约束：允许XY平面内平移，约束Z平移和所有旋转
        # 与输入法兰滚针约束相同，确保所有法兰滚针行为一致
        mbs.AddObject(GenericJoint(markerNumbers=[mGroundNeedle, mNeedle],
                                   constrainedAxes=[0, 0, 0, 1, 1, 0],
                                   visualization=VObjectJointGeneric(axesRadius=0.1, axesLength=0.2, show=False)))
        
        mNeedleMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oNeedle, localPosition=[0, 0, 0]))
        needle_bodies_output.append(oNeedle)
        needle_markers_output.append(mNeedleMarker)
        all_flange_needle_bodies.append(oNeedle)
        all_flange_needle_markers.append(mNeedleMarker)
    
    # 注释掉固定滚针之间的相对位置
    # for i_needle in range(n_flange_needles):
    #     i_next = (i_needle + 1) % n_flange_needles
    #     mNeedle1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=needle_bodies_output[i_needle],
    #                                               localPosition=[0, 0, 0]))
    #     mNeedle2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=needle_bodies_output[i_next],
    #                                               localPosition=[0, 0, 0]))
    #     angle_diff = 2 * np.pi / n_flange_needles
    #     expected_distance = 2 * r_flange_needle_pitch * np.sin(angle_diff / 2)
    #     mbs.AddObject(DistanceConstraint(markerNumbers=[mNeedle1, mNeedle2],
    #                                      distance=expected_distance,
    #                                      visualization=VObjectConnectorDistance(show=False)))
    
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
    # 针齿约束：允许XY平面内平移和Z轴旋转，约束Z平移和XY轴旋转
    # constrainedAxes=[0,0,1,1,1,0] 表示：[X平移, Y平移, Z平移, X旋转, Y旋转, Z旋转]
    # 这样针齿可以在XY平面内移动以适应与摆线轮的接触，同时可以绕Z轴旋转
    mbs.AddObject(GenericJoint(markerNumbers=[mGroundPinPlane, mPinPlane], constrainedAxes=[0,0,1,1,1,0], visualization=VObjectJointGeneric(axesRadius=0.15, axesLength=0.4, show=False)))
    mPin = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oPin, localPosition=[0,0,0]))
    pin_markers.append(mPin)
    pin_radii.append(r_z)
    pin_bodies.append(oPin)
    pin_hole_centers.append((x_pin, y_pin))

# =================== 外齿廓-针齿接触（使用自定义接触函数） ===================
# 创建齿廓理论函数对象（用于精确优化）
profile_func_1 = CycloidProfileFunction(z_b=z_b, e=e, R_z=R_z, r_z=r_z, phi_h=0, delta_e=delta_e)
profile_func_2 = CycloidProfileFunction(z_b=z_b, e=e, R_z=R_z, r_z=r_z, phi_h=np.pi, delta_e=delta_e)

# 创建两个摆线轮的接触计算对象
# use_optimization=True: 使用理论函数优化（更精确，稍慢）
# use_optimization=False: 仅用离散点（更快，稍不精确）
USE_THEORY_OPTIMIZATION = True  # ← 这里可以切换是否使用理论函数优化

contact_cycloid1 = CycloidToothPinContact(
    x_cycloid1, y_cycloid1, r_z, 
    contactStiffness_tooth, contactDamping_tooth, 
    search_range=50, max_iter=10,
    profile_function=profile_func_1,
    use_optimization=USE_THEORY_OPTIMIZATION)

contact_cycloid2 = CycloidToothPinContact(
    x_cycloid2, y_cycloid2, r_z, 
    contactStiffness_tooth, contactDamping_tooth, 
    search_range=25, max_iter=5,
    profile_function=profile_func_2,
    use_optimization=USE_THEORY_OPTIMIZATION)

print(f"\n创建自定义齿廓-针齿接触")
print(f"  摆线轮1齿廓点数: {len(x_cycloid1)}")
print(f"  摆线轮2齿廓点数: {len(x_cycloid2)}")
print(f"  针齿数量: {len(pin_bodies)}")
print(f"  接触刚度: {contactStiffness_tooth:.2e} N/mm")
print(f"  接触阻尼: {contactDamping_tooth:.2e} N·s/mm")
print(f"  最近点搜索范围: {contact_cycloid1.search_range} 个点")
print(f"  最大迭代次数: {contact_cycloid1.max_iter}")
print(f"  ★ 理论函数优化: {'启用 ✓' if USE_THEORY_OPTIMIZATION else '禁用 ✗'}")
if USE_THEORY_OPTIMIZATION:
    print(f"     - 方法: 离散点搜索 + Brent一维优化")
    print(f"     - 精度: 连续参数空间（优于离散点）")
    print(f"     - 计算量: 每次接触约10次额外函数评估")

# 创建齿廓接触负载列表
tooth_contact_load_ids = []
tooth_contact_sensor_ids = []

# 全局字典：存储每次计算的接触力，用于同时施加到摆线轮和针齿
contact_forces_cache = {}

# 定义摆线轮1与针齿的接触用户函数（作用在针齿上）
def CycloidToothContactUF_1_Pin(mbs, t, loadVector, pinBody, pin_id, contact_obj=contact_cycloid1, cycloid_body=oCycloid1):
    # 获取针齿位置和速度
    nodeNumber = mbs.GetObject(pinBody)['nodeNumber']
    pos_pin = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Position)
    vel_pin = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Velocity)
    pin_center = np.array([pos_pin[0], pos_pin[1]])
    pin_velocity = np.array([vel_pin[0], vel_pin[1], 0.0])
    
    # 获取摆线轮位置、旋转、速度和角速度
    cycloid_nodeNumber = mbs.GetObject(cycloid_body)['nodeNumber']
    pos_cycloid = mbs.GetNodeOutput(cycloid_nodeNumber, exu.OutputVariableType.Position)
    rot_cycloid = mbs.GetNodeOutput(cycloid_nodeNumber, exu.OutputVariableType.Rotation)
    vel_cycloid = mbs.GetNodeOutput(cycloid_nodeNumber, exu.OutputVariableType.Velocity)
    angvel_cycloid = mbs.GetNodeOutput(cycloid_nodeNumber, exu.OutputVariableType.AngularVelocity)
    
    # 计算接触力（带调试输出）
    F_pin, F_cycloid, gap, contact_point = compute_contact_force_with_debug(
        contact_obj, pin_center, pin_velocity, pos_cycloid, rot_cycloid[2], vel_cycloid, angvel_cycloid,
        pin_id, "CycloidTooth1", t)
    
    # 缓存接触力（供摆线轮负载使用）
    cache_key = ('cyc1', pin_id)
    contact_forces_cache[cache_key] = (F_pin, F_cycloid, contact_point)
    
    return [F_pin[0], F_pin[1], 0.0]

# 定义摆线轮1的反作用力函数（作用在摆线轮上）
def CycloidToothContactUF_1_Cycloid(mbs, t, loadVector, pin_id):
    cache_key = ('cyc1', pin_id)
    if cache_key in contact_forces_cache:
        F_pin, F_cycloid, contact_point = contact_forces_cache[cache_key]
        return [F_cycloid[0], F_cycloid[1], 0.0]
    return [0.0, 0.0, 0.0]

# 定义摆线轮2与针齿的接触用户函数（作用在针齿上）
def CycloidToothContactUF_2_Pin(mbs, t, loadVector, pinBody, pin_id, contact_obj=contact_cycloid2, cycloid_body=oCycloid2):
    # 获取针齿位置和速度
    nodeNumber = mbs.GetObject(pinBody)['nodeNumber']
    pos_pin = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Position)
    vel_pin = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Velocity)
    pin_center = np.array([pos_pin[0], pos_pin[1]])
    pin_velocity = np.array([vel_pin[0], vel_pin[1], 0.0])
    
    # 获取摆线轮位置、旋转、速度和角速度
    cycloid_nodeNumber = mbs.GetObject(cycloid_body)['nodeNumber']
    pos_cycloid = mbs.GetNodeOutput(cycloid_nodeNumber, exu.OutputVariableType.Position)
    rot_cycloid = mbs.GetNodeOutput(cycloid_nodeNumber, exu.OutputVariableType.Rotation)
    vel_cycloid = mbs.GetNodeOutput(cycloid_nodeNumber, exu.OutputVariableType.Velocity)
    angvel_cycloid = mbs.GetNodeOutput(cycloid_nodeNumber, exu.OutputVariableType.AngularVelocity)
    
    # 计算接触力（带调试输出）
    F_pin, F_cycloid, gap, contact_point = compute_contact_force_with_debug(
        contact_obj, pin_center, pin_velocity, pos_cycloid, rot_cycloid[2], vel_cycloid, angvel_cycloid,
        pin_id, "CycloidTooth2", t)
    
    # 缓存接触力（供摆线轮负载使用）
    cache_key = ('cyc2', pin_id)
    contact_forces_cache[cache_key] = (F_pin, F_cycloid, contact_point)
    
    return [F_pin[0], F_pin[1], 0.0]

# 定义摆线轮2的反作用力函数（作用在摆线轮上）
def CycloidToothContactUF_2_Cycloid(mbs, t, loadVector, pin_id):
    cache_key = ('cyc2', pin_id)
    if cache_key in contact_forces_cache:
        F_pin, F_cycloid, contact_point = contact_forces_cache[cache_key]
        return [F_cycloid[0], F_cycloid[1], 0.0]
    return [0.0, 0.0, 0.0]

# 为每个针齿创建与两片摆线轮的接触负载（针齿+摆线轮双向力）
cycloid1_contact_loads = []  # 作用在摆线轮1上的所有负载
cycloid2_contact_loads = []  # 作用在摆线轮2上的所有负载

for i, (mPin, oPin) in enumerate(zip(pin_markers, pin_bodies)):
    # ========== 针齿与摆线轮1的接触 ==========
    # 1) 作用在针齿上的力
    lid1_pin = mbs.AddLoad(LoadForceVector(
        markerNumber=mPin,
        loadVector=[0, 0, 0],
        bodyFixed=False,
        loadVectorUserFunction=lambda mbs,t,load, pinBody=oPin, pin_id=i: 
            CycloidToothContactUF_1_Pin(mbs, t, load, pinBody, pin_id),
        visualization=VLoadForceVector(show=True)
    ))
    tooth_contact_load_ids.append(lid1_pin)
    tooth_contact_sensor_ids.append(mbs.AddSensor(SensorLoad(loadNumber=lid1_pin, storeInternal=True)))
    
    # 2) 作用在摆线轮1上的反作用力
    lid1_cyc = mbs.AddLoad(LoadForceVector(
        markerNumber=mCycloid1,
        loadVector=[0, 0, 0],
        bodyFixed=False,
        loadVectorUserFunction=lambda mbs,t,load, pin_id=i: 
            CycloidToothContactUF_1_Cycloid(mbs, t, load, pin_id),
        visualization=VLoadForceVector(show=False)  # 不显示（避免重复）
    ))
    cycloid1_contact_loads.append(lid1_cyc)
    
    # ========== 针齿与摆线轮2的接触 ==========
    # 1) 作用在针齿上的力
    lid2_pin = mbs.AddLoad(LoadForceVector(
        markerNumber=mPin,
        loadVector=[0, 0, 0],
        bodyFixed=False,
        loadVectorUserFunction=lambda mbs,t,load, pinBody=oPin, pin_id=i: 
            CycloidToothContactUF_2_Pin(mbs, t, load, pinBody, pin_id),
        visualization=VLoadForceVector(show=True)
    ))
    tooth_contact_load_ids.append(lid2_pin)
    tooth_contact_sensor_ids.append(mbs.AddSensor(SensorLoad(loadNumber=lid2_pin, storeInternal=True)))
    
    # 2) 作用在摆线轮2上的反作用力
    lid2_cyc = mbs.AddLoad(LoadForceVector(
        markerNumber=mCycloid2,
        loadVector=[0, 0, 0],
        bodyFixed=False,
        loadVectorUserFunction=lambda mbs,t,load, pin_id=i: 
            CycloidToothContactUF_2_Cycloid(mbs, t, load, pin_id),
        visualization=VLoadForceVector(show=False)  # 不显示（避免重复）
    ))
    cycloid2_contact_loads.append(lid2_cyc)

print(f"创建了 {len(tooth_contact_load_ids)} 个齿廓-针齿接触负载（作用在针齿上）")
print(f"创建了 {len(cycloid1_contact_loads)} 个摆线轮1反作用力负载")
print(f"创建了 {len(cycloid2_contact_loads)} 个摆线轮2反作用力负载")
print(f"总计: {len(tooth_contact_load_ids) + len(cycloid1_contact_loads) + len(cycloid2_contact_loads)} 个齿廓接触负载")

""" 滚针轴承接触：使用圆-圆接触模型"""
# 创建偏心轴-滚针、滚针-摆线轮孔的接触负载

# 1. 偏心轴外圆 - 滚针接触（凸圆-凸圆）
def _CrankNeedleContactResult(mbs, t, crankMarker, needleBody, r_crank, r_needle, kn, cn):
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
    result = circle_circle_penalty(c1, r_crank, v1, c2, r_needle, v2, kn=kn, cn=cn)
    
    # 输出调试信息（现在每步都会输出）
    if contact_debug_enabled:
        moment = np.zeros(3)
        contact_point = result['pMid']
        write_contact_debug(t, "CrankNeedle", result['F2'], moment,
                          np.array([contact_point[0], contact_point[1], 0]))
        write_contact_debug(t, "NeedleCrank", result['F1'], moment,
                          np.array([c1[0], c1[1], 0]))
    
    return result

def CrankNeedleLoadOnCrank(mbs, t, loadVector, crankMarker, needleBody, r_crank, r_needle, kn, cn):
    out = _CrankNeedleContactResult(mbs, t, crankMarker, needleBody, r_crank, r_needle, kn, cn)
    return [out['F1'][0], out['F1'][1], 0.0]

def CrankNeedleLoadOnNeedle(mbs, t, loadVector, crankMarker, needleBody, r_crank, r_needle, kn, cn):
    out = _CrankNeedleContactResult(mbs, t, crankMarker, needleBody, r_crank, r_needle, kn, cn)
    return [out['F2'][0], out['F2'][1], 0.0]

def _NeedleHoleContactResult(mbs, t, needleBody, cycloidMarker, r_needle, r_hole, kn, cn):
    nodeNumber = mbs.GetObject(needleBody)['nodeNumber']
    pos_needle = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Position)
    vel_needle = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Velocity)
    c_pin = np.array([pos_needle[0], pos_needle[1]])
    v_pin = np.array([vel_needle[0], vel_needle[1]])

    pos_cycloid = mbs.GetMarkerOutput(cycloidMarker, exu.OutputVariableType.Position)
    vel_cycloid = mbs.GetMarkerOutput(cycloidMarker, exu.OutputVariableType.Velocity)
    c_hole = np.array([pos_cycloid[0], pos_cycloid[1]])
    v_hole = np.array([vel_cycloid[0], vel_cycloid[1]])

    result = circle_in_hole_penalty(c_pin, r_needle, v_pin, c_hole, r_hole, v_hole, kn=kn, cn=cn)
    
    # 输出调试信息（现在每步都会输出）
    if contact_debug_enabled:
        moment = np.zeros(3)
        write_contact_debug(t, "NeedleHole", result['F1'], moment,
                          np.array([c_pin[0], c_pin[1], 0]))
        write_contact_debug(t, "HoleNeedle", result['F2'], moment,
                          np.array([c_hole[0], c_hole[1], 0]))
    
    return result

def NeedleHoleLoadOnNeedle(mbs, t, loadVector, needleBody, cycloidMarker, r_needle, r_hole, kn, cn):
    out = _NeedleHoleContactResult(mbs, t, needleBody, cycloidMarker, r_needle, r_hole, kn, cn)
    return [out['F1'][0], out['F1'][1], 0.0]

def NeedleHoleLoadOnCycloid(mbs, t, loadVector, needleBody, cycloidMarker, r_needle, r_hole, kn, cn):
    out = _NeedleHoleContactResult(mbs, t, needleBody, cycloidMarker, r_needle, r_hole, kn, cn)
    return [out['F2'][0], out['F2'][1], 0.0]

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
                CrankNeedleLoadOnCrank(mbs, t, load, crank, needle, r_crank_eccentric, r_needle, 
                                       contactStiffness_bearing, contactDamping_bearing)
        ))
        bearing_load_ids.append(lid1)
        bearing_contact_count += 1

        lid1n = mbs.AddLoad(LoadForceVector(
            markerNumber=needle_marker,
            loadVector=[0, 0, 0],
            loadVectorUserFunction=lambda mbs,t,load, crank=crankMarker, needle=needleBody: 
                CrankNeedleLoadOnNeedle(mbs, t, load, crank, needle, r_crank_eccentric, r_needle, 
                                        contactStiffness_bearing, contactDamping_bearing)
        ))
        bearing_load_ids.append(lid1n)
        bearing_contact_count += 1
        
        # 滚针-摆线轮孔接触
        lid2 = mbs.AddLoad(LoadForceVector(
            markerNumber=needle_marker,
            loadVector=[0, 0, 0],
            loadVectorUserFunction=lambda mbs,t,load, needle=needleBody, cycloid=cycloidMarker: 
                NeedleHoleLoadOnNeedle(mbs, t, load, needle, cycloid, r_needle, r_hole, 
                                       contactStiffness_bearing, contactDamping_bearing)
        ))
        bearing_load_ids.append(lid2)
        bearing_contact_count += 1

        lid2c = mbs.AddLoad(LoadForceVector(
            markerNumber=cycloidMarker,
            loadVector=[0, 0, 0],
            loadVectorUserFunction=lambda mbs,t,load, needle=needleBody, cycloid=cycloidMarker: 
                NeedleHoleLoadOnCycloid(mbs, t, load, needle, cycloid, r_needle, r_hole, 
                                        contactStiffness_bearing, contactDamping_bearing)
        ))
        bearing_load_ids.append(lid2c)
        bearing_contact_count += 1

print(f"创建了 {bearing_contact_count} 个摆线轮轴承接触负载（{len(bearing_info)} 个轴承 × {n_needles} 个滚针 × 4个载荷）")

# =================== 法兰轴承接触负载 ===================
flange_bearing_load_ids = []
flange_bearing_contact_count = 0

# 法兰轴承接触函数（曲柄主轴-滚针）
def _FlangeCrankNeedleContactResult(mbs, t, crankMarker, needleBody, r_shaft, r_needle, kn, cn):
    pos_crank = mbs.GetMarkerOutput(crankMarker, exu.OutputVariableType.Position)
    vel_crank = mbs.GetMarkerOutput(crankMarker, exu.OutputVariableType.Velocity)
    c1 = np.array([pos_crank[0], pos_crank[1]])
    v1 = np.array([vel_crank[0], vel_crank[1]])
    
    nodeNumber = mbs.GetObject(needleBody)['nodeNumber']
    pos_needle = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Position)
    vel_needle = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Velocity)
    c2 = np.array([pos_needle[0], pos_needle[1]])
    v2 = np.array([vel_needle[0], vel_needle[1]])
    
    result = circle_circle_penalty(c1, r_shaft, v1, c2, r_needle, v2, kn=kn, cn=cn)
    
    # 输出调试信息（现在每步都会输出）
    if contact_debug_enabled:
        moment = np.zeros(3)
        contact_point = result['pMid']
        write_contact_debug(t, "FlangeCrank", result['F2'], moment,
                          np.array([contact_point[0], contact_point[1], 0]))
        write_contact_debug(t, "CrankFlange", result['F1'], moment,
                          np.array([c1[0], c1[1], 0]))
    
    return result

def FlangeCrankNeedleLoadOnCrank(mbs, t, loadVector, crankMarker, needleBody, r_shaft, r_needle, kn, cn):
    out = _FlangeCrankNeedleContactResult(mbs, t, crankMarker, needleBody, r_shaft, r_needle, kn, cn)
    return [out['F1'][0], out['F1'][1], 0.0]

def FlangeCrankNeedleLoadOnNeedle(mbs, t, loadVector, crankMarker, needleBody, r_shaft, r_needle, kn, cn):
    out = _FlangeCrankNeedleContactResult(mbs, t, crankMarker, needleBody, r_shaft, r_needle, kn, cn)
    return [out['F2'][0], out['F2'][1], 0.0]

# 法兰孔-滚针接触函数
def _FlangeHoleNeedleContactResult(mbs, t, needleBody, flangeMarker, r_needle, r_hole, kn, cn):
    nodeNumber = mbs.GetObject(needleBody)['nodeNumber']
    pos_needle = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Position)
    vel_needle = mbs.GetNodeOutput(nodeNumber, exu.OutputVariableType.Velocity)
    c_pin = np.array([pos_needle[0], pos_needle[1]])
    v_pin = np.array([vel_needle[0], vel_needle[1]])
    
    pos_flange = mbs.GetMarkerOutput(flangeMarker, exu.OutputVariableType.Position)
    vel_flange = mbs.GetMarkerOutput(flangeMarker, exu.OutputVariableType.Velocity)
    c_hole = np.array([pos_flange[0], pos_flange[1]])
    v_hole = np.array([vel_flange[0], vel_flange[1]])
    
    result = circle_in_hole_penalty(c_pin, r_needle, v_pin, c_hole, r_hole, v_hole, kn=kn, cn=cn)
    
    # 输出调试信息（现在每步都会输出）
    if contact_debug_enabled:
        moment = np.zeros(3)
        write_contact_debug(t, "FlangeNeedle", result['F1'], moment,
                          np.array([c_pin[0], c_pin[1], 0]))
        write_contact_debug(t, "NeedleFlange", result['F2'], moment,
                          np.array([c_hole[0], c_hole[1], 0]))
    
    return result

def FlangeHoleNeedleLoadOnNeedle(mbs, t, loadVector, needleBody, flangeMarker, r_needle, r_hole, kn, cn):
    out = _FlangeHoleNeedleContactResult(mbs, t, needleBody, flangeMarker, r_needle, r_hole, kn, cn)
    return [out['F1'][0], out['F1'][1], 0.0]

def FlangeHoleNeedleLoadOnFlange(mbs, t, loadVector, needleBody, flangeMarker, r_needle, r_hole, kn, cn):
    out = _FlangeHoleNeedleContactResult(mbs, t, needleBody, flangeMarker, r_needle, r_hole, kn, cn)
    return [out['F2'][0], out['F2'][1], 0.0]

# 创建所有法兰轴承的接触负载
for (flangeMarker, crankMarker, needle_markers, z_pos, is_input) in flange_bearing_info:
    for needle_marker in needle_markers:
        needleBody = mbs.GetMarker(needle_marker)['bodyNumber']
        
        # 曲柄主轴-滚针接触
        lid1 = mbs.AddLoad(LoadForceVector(
            markerNumber=crankMarker,
            loadVector=[0, 0, 0],
            loadVectorUserFunction=lambda mbs,t,load, crank=crankMarker, needle=needleBody: 
                FlangeCrankNeedleLoadOnCrank(mbs, t, load, crank, needle, r_flange_shaft, r_flange_needle, 
                                             contactStiffness_flange_bearing, contactDamping_flange_bearing)
        ))
        flange_bearing_load_ids.append(lid1)
        flange_bearing_contact_count += 1

        lid1n = mbs.AddLoad(LoadForceVector(
            markerNumber=needle_marker,
            loadVector=[0, 0, 0],
            loadVectorUserFunction=lambda mbs,t,load, crank=crankMarker, needle=needleBody: 
                FlangeCrankNeedleLoadOnNeedle(mbs, t, load, crank, needle, r_flange_shaft, r_flange_needle, 
                                              contactStiffness_flange_bearing, contactDamping_flange_bearing)
        ))
        flange_bearing_load_ids.append(lid1n)
        flange_bearing_contact_count += 1
        
        # 滚针-法兰孔接触
        lid2 = mbs.AddLoad(LoadForceVector(
            markerNumber=needle_marker,
            loadVector=[0, 0, 0],
            loadVectorUserFunction=lambda mbs,t,load, needle=needleBody, flange=flangeMarker: 
                FlangeHoleNeedleLoadOnNeedle(mbs, t, load, needle, flange, r_flange_needle, r_flange_hole, 
                                              contactStiffness_flange_bearing, contactDamping_flange_bearing)
        ))
        flange_bearing_load_ids.append(lid2)
        flange_bearing_contact_count += 1

        lid2f = mbs.AddLoad(LoadForceVector(
            markerNumber=flangeMarker,
            loadVector=[0, 0, 0],
            loadVectorUserFunction=lambda mbs,t,load, needle=needleBody, flange=flangeMarker: 
                FlangeHoleNeedleLoadOnFlange(mbs, t, load, needle, flange, r_flange_needle, r_flange_hole, 
                                              contactStiffness_flange_bearing, contactDamping_flange_bearing)
        ))
        flange_bearing_load_ids.append(lid2f)
        flange_bearing_contact_count += 1

print(f"创建了 {flange_bearing_contact_count} 个法兰轴承接触负载（{len(flange_bearing_info)} 个轴承 × {n_flange_needles} 个滚针 × 4个载荷）")

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
    
    # 输出调试信息（现在每步都会输出）
    if contact_debug_enabled:
        moment = np.zeros(3)
        write_contact_debug(t, "PinHole", out['F1'], moment,
                          np.array([c1[0], c1[1], 0]))
        write_contact_debug(t, "HolePin", out['F2'], moment,
                          np.array([c2[0], c2[1], 0]))
    
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
stepSize = 0.001  # 进一步减小步长以提高稳定性（从0.005改为0.001）
tEnd = 0.5  # 减小仿真时长以加快验证（从2.0改为0.5）
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.sensorsWritePeriod = 0.001  # 减小传感器记录周期（从0.002改为0.001）
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

# Newton求解器设置（优化以提高收敛性）
simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 1e-5  # 放宽容差（从1e-6改为1e-5）
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-4  # 进一步放宽容差（从1e-5改为1e-4）
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-4  # 进一步放宽容差（从1e-5改为1e-4）
simulationSettings.timeIntegration.newton.maxIterations = 25  # 增加最大迭代次数（从15改为25）
# 注释掉不支持的residualTolerance设置
# simulationSettings.timeIntegration.newton.residualTolerance = 1e-6  # 添加残差容差

# Generalized Alpha设置
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5  # 增加数值阻尼（从0.7改为0.5）
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True

# 自适应步长（优化以提高稳定性）
simulationSettings.timeIntegration.adaptiveStep = True
simulationSettings.timeIntegration.adaptiveStepIncrease = 1.3  # 减小步长增加因子（从1.5改为1.3）
simulationSettings.timeIntegration.adaptiveStepDecrease = 0.7  # 减小步长减小因子（从0.5改为0.7）
simulationSettings.timeIntegration.minimumStepSize = 1e-7  # 减小最小步长（从1e-6改为1e-7）
# 注释掉不支持的adaptiveStepVerbose设置
# simulationSettings.timeIntegration.adaptiveStepVerbose = 1  # 添加自适应步长详细输出

simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.displayComputationTime = True
simulationSettings.displayStatistics = True

SC.visualizationSettings.general.graphicsUpdateInterval = 0.01  # 增加实时显示更新频率（从0.02改为0.01）
SC.visualizationSettings.window.renderWindowSize = [1600, 1200]
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.openGL.shadow = 0.3

# 载荷可视化设置（关键！）
SC.visualizationSettings.loads.show = True
SC.visualizationSettings.connectors.showContact = True
SC.visualizationSettings.loads.loadSizeFactor = 0.001  # 载荷箭头缩放因子（增大以显示更明显）
SC.visualizationSettings.loads.defaultRadius = 1.0     # 箭头半径
SC.visualizationSettings.loads.defaultSize = 5.0       # 基础尺寸
SC.visualizationSettings.loads.fixedLoadSize = False   # 随力大小缩放
SC.visualizationSettings.loads.drawSimplified = False  # 不简化绘制
SC.visualizationSettings.loads.showNumbers = False     # 不显示编号（避免太乱）

print("\n" + "="*60)
print("载荷可视化设置:")
print(f"  显示载荷: {SC.visualizationSettings.loads.show}")
print(f"  载荷缩放因子: {SC.visualizationSettings.loads.loadSizeFactor}")
print(f"  箭头半径: {SC.visualizationSettings.loads.defaultRadius}")
print(f"  基础尺寸: {SC.visualizationSettings.loads.defaultSize}")
print(f"  显示更新间隔: {SC.visualizationSettings.general.graphicsUpdateInterval} s")
print("  提示: 如果看不到力箭头，可能力太小或缩放因子需要调整")
print("="*60)

print("\n" + "="*60)
print("仿真设置:")
print(f"  仿真时长: {tEnd} s")
print(f"  时间步长: {stepSize} s")
print(f"  总步数: {simulationSettings.timeIntegration.numberOfSteps}")
print(f"  传感器记录周期: {simulationSettings.solutionSettings.sensorsWritePeriod} s")
print(f"  预期数据点数: ~{int(tEnd / simulationSettings.solutionSettings.sensorsWritePeriod)}")
print(f"  显示更新频率: {SC.visualizationSettings.general.graphicsUpdateInterval} s")
print("="*60)

if useGraphics:
    SC.renderer.Start(); SC.renderer.DoIdleTasks()

import traceback, sys

# 调试计数器
debug_print_counter = [0]
max_force_debug = [0.0, 0.0]  # [齿廓接触力最大值, 针齿-孔接触力最大值]
newton_iterations_history = []  # 记录Newton迭代次数
step_size_history = []  # 记录步长变化
convergence_failures = 0  # 收敛失败计数器

def UFupdateVisLoads(mbs, t):
    global newton_iterations_history, step_size_history, convergence_failures
    
    try:
        # 获取当前Newton迭代次数和步长（用于调试收敛问题）
        try:
            current_step_info = mbs.GetSimulationStepInfo()
            if hasattr(current_step_info, 'newtonIterations'):
                newton_iterations_history.append(current_step_info.newtonIterations)
                # 检测Newton迭代次数过多的情况
                if current_step_info.newtonIterations > 20:
                    print(f"[调试] Newton迭代次数过多: {current_step_info.newtonIterations}，可能存在收敛问题")
            if hasattr(current_step_info, 'currentStepSize'):
                step_size_history.append(current_step_info.currentStepSize)
                # 检测步长是否过小，可能是收敛困难的迹象
                if current_step_info.currentStepSize < 1e-6:
                    print(f"[调试] 步长过小: {current_step_info.currentStepSize:.2e}，可能存在收敛问题")
                    convergence_failures += 1
        except:
            pass  # 如果无法获取步长信息，继续执行
        
        # 更新针齿-孔接触力的可视化
        for lid in pin_load_ids:
            F = mbs.GetLoadValues(lid)
            mbs.SetLoadParameter(lid, 'loadVector', [float(F[0]), float(F[1]), float(F[2])])
            F_mag = np.sqrt(F[0]**2 + F[1]**2 + F[2]**2)
            if F_mag > max_force_debug[1]:
                max_force_debug[1] = F_mag
        
        # 更新齿廓-针齿接触力的可视化（针齿上的力）
        for lid in tooth_contact_load_ids:
            F = mbs.GetLoadValues(lid)
            mbs.SetLoadParameter(lid, 'loadVector', [float(F[0]), float(F[1]), float(F[2])])
            F_mag = np.sqrt(F[0]**2 + F[1]**2 + F[2]**2)
            if F_mag > max_force_debug[0]:
                max_force_debug[0] = F_mag
        
        # 更新摆线轮反作用力的可视化
        for lid in cycloid1_contact_loads:
            F = mbs.GetLoadValues(lid)
            mbs.SetLoadParameter(lid, 'loadVector', [float(F[0]), float(F[1]), float(F[2])])
        
        for lid in cycloid2_contact_loads:
            F = mbs.GetLoadValues(lid)
            mbs.SetLoadParameter(lid, 'loadVector', [float(F[0]), float(F[1]), float(F[2])])
        
        # 每隔一段时间打印一次最大接触力信息
        debug_print_counter[0] += 1
        if debug_print_counter[0] % 100 == 0:  # 减少到每100步打印一次
            avg_newton = np.mean(newton_iterations_history[-10:]) if len(newton_iterations_history) > 0 else 0
            current_step = step_size_history[-1] if len(step_size_history) > 0 else 0
            print(f"\r[t={t:.3f}s] 最大齿廓接触力: {max_force_debug[0]:.2f}N, 最大针齿-孔接触力: {max_force_debug[1]:.2f}N, 平均Newton迭代: {avg_newton:.1f}, 当前步长: {current_step:.2e}, 收敛失败次数: {convergence_failures}", end='')
        
        # 每隔一定步数刷新接触力调试缓冲区，确保数据及时写入
        if contact_debug_enabled and debug_print_counter[0] % 10 == 0:  # 每10步刷新一次
            flush_contact_debug_buffer()
            
    except Exception as e:
        pass
    return True

mbs.SetPreStepUserFunction(UFupdateVisLoads)

print("\n" + "="*60)
print("接触力可视化说明（通过箭头粗细区分两片摆线轮）:")
print("="*60)
print("1. 力箭头显示方式：")
print("   - 针齿上的力：内置LoadForceVector可视化（默认颜色）")
print("   - 摆线轮受力：自定义图形函数绘制（区分粗细）")
print("2. 摆线轮受力箭头区分：")
print("   ★ 摆线轮1（第一片）- 粗箭头（红色，radius=0.8）")
print("   ★ 摆线轮2（第二片）- 细箭头（蓝色，radius=0.3）")
print("   - 针齿-孔接触：黄色箭头")
print("3. 如果看不到力箭头，可能原因：")
print("   - 接触力为零（没有接触）")
print("   - 力太小（阈值1.0N），箭头太短看不见")
print("   - 缩放因子需要调整")
print("4. 调整方法：")
print("   - 修改 visForceScale (当前=0.02) 调整箭头长度")
print("   - 修改 UFgraphicsForces 中的 radius 参数调整箭头粗细")
print("   - 修改力阈值 (当前=1.0N) 控制显示哪些力")
print("5. 仿真过程中会打印最大接触力值")
print("="*60 + "\n")

# 初始化接触力调试输出文件
init_contact_debug_output()

print("\n开始仿真，已应用以下修复措施：")
print("1. 降低接触刚度以提高收敛性")
print("2. 增加接触阻尼以提高稳定性")
print("3. 调整约束设置，限制Z轴平移")
print("4. 放宽Newton求解器容差")
print("5. 调整自适应步长参数")
print("6. 添加调试日志监控收敛过程")
print("7. 优化接触力输出缓冲区")
print("="*60)

try:
    mbs.SolveDynamic(simulationSettings, showHints=True)
except Exception as e:
    print("\n仿真出错，打印完整 traceback：")
    traceback.print_exc()
    print(f"\n错误类型: {type(e).__name__}")
    print(f"错误信息: {str(e)}")
    
    # 打印调试统计信息
    print("\n调试统计信息：")
    print(f"接触力调试调用次数: {contact_debug_call_count}")
    print(f"接触力调试写入次数: {contact_debug_write_count}")
    if len(newton_iterations_history) > 0:
        print(f"平均Newton迭代次数: {np.mean(newton_iterations_history):.2f}")
        print(f"最大Newton迭代次数: {np.max(newton_iterations_history)}")
    if len(step_size_history) > 0:
        print(f"最终步长: {step_size_history[-1]:.2e}")
        print(f"最小步长: {np.min(step_size_history):.2e}")
    
    sys.exit(1)
finally:
    # 确保在仿真结束后关闭调试输出文件
    close_contact_debug_output()
    
    # 打印调试统计信息
    print("\n仿真完成，调试统计信息：")
    print(f"接触力调试调用次数: {contact_debug_call_count}")
    print(f"接触力调试写入次数: {contact_debug_write_count}")
    if len(newton_iterations_history) > 0:
        print(f"平均Newton迭代次数: {np.mean(newton_iterations_history):.2f}")
        print(f"最大Newton迭代次数: {np.max(newton_iterations_history)}")
    if len(step_size_history) > 0:
        print(f"最终步长: {step_size_history[-1]:.2e}")
        print(f"最小步长: {np.min(step_size_history):.2e}")

if useGraphics:
    try:
        SC.renderer.Stop()
    except:
        pass

print("\n" + "="*60)
print("仿真成功完成！")
print("="*60)
print(f"最大齿廓接触力: {max_force_debug[0]:.2f} N")
print(f"最大针齿-孔接触力: {max_force_debug[1]:.2f} N")
print("="*60)

# 输出理论函数优化统计
if USE_THEORY_OPTIMIZATION:
    print("\n" + "="*60)
    print("理论函数优化效果统计：")
    print("="*60)
    opt_count_1 = contact_cycloid1.optimization_count
    opt_improvement_1 = contact_cycloid1.optimization_improvement_sum
    opt_count_2 = contact_cycloid2.optimization_count
    opt_improvement_2 = contact_cycloid2.optimization_improvement_sum
    
    total_count = opt_count_1 + opt_count_2
    total_improvement = opt_improvement_1 + opt_improvement_2
    
    if total_count > 0:
        avg_improvement = total_improvement / total_count
        print(f"  摆线轮1优化次数: {opt_count_1}")
        print(f"  摆线轮2优化次数: {opt_count_2}")
        print(f"  总优化次数: {total_count}")
        print(f"  总距离改进: {total_improvement:.6f} mm")
        print(f"  平均改进: {avg_improvement * 1000:.3f} μm/次")
        print(f"  精度提升: {avg_improvement / (R_z * 2 / 1000) * 100:.4f}% (相对于齿廓尺寸)")
        print("\n  结论: 理论函数优化有效地提升了接触点精度！")
    else:
        print("  未检测到有效优化（可能接触较少）")
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
    ax5b.plot(dFpin0[:,0], np.hypot(dFpin0[:,1], dFpin0[:,2]), 'k-', alpha=0.6, label='Pin-Hole Force')

# 绘制齿廓接触力（第一个针齿与两片摆线轮的接触力）
if len(tooth_contact_sensor_ids) >= 2:
    try:
        dFtooth1 = mbs.GetSensorStoredData(tooth_contact_sensor_ids[0])  # 针齿0与摆线轮1
        dFtooth2 = mbs.GetSensorStoredData(tooth_contact_sensor_ids[1])  # 针齿0与摆线轮2
        ax5b.plot(dFtooth1[:,0], np.hypot(dFtooth1[:,1], dFtooth1[:,2]), 'r-', alpha=0.5, linewidth=1.5, label='Tooth Force (Cyc1)')
        ax5b.plot(dFtooth2[:,0], np.hypot(dFtooth2[:,1], dFtooth2[:,2]), 'b-', alpha=0.5, linewidth=1.5, label='Tooth Force (Cyc2)')
    except Exception as e:
        print(f"绘制齿廓接触力失败: {e}")

ax5b.set_ylabel('Force (N)')
ax5b.legend(loc='upper right', fontsize=8)

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
print(f"    - 滚针之间无约束（已去除保持架效果）")
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
print("  【接触力可视化 - 通过粗细区分】")
print("    ★ 粗箭头（红色）：摆线轮1受到的反作用力")
print("    ★ 细箭头（蓝色）：摆线轮2受到的反作用力")
print("    - 黄色箭头：针齿与针齿壳孔的接触力")
print("    - 其他箭头：针齿受力（默认颜色）")
print("  【接触算法}")
print("    - 自定义齿廓-针齿接触：第一片摆线轮 <-> 针齿")
print("    - 自定义齿廓-针齿接触：第二片摆线轮 <-> 针齿")
print("    - 使用优化的最近点搜索算法（切线平行法）")
print("    - 圆-圆接触：偏心轴外圆 <-> 滚针")
print("    - 圆-圆孔接触：滚针 <-> 摆线轮孔内壁")
print("    - 圆-圆孔接触：针齿 <-> 针齿壳孔")
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
# ========== 基于 graphicsDataUserFunction 的力箭头可视化（区分两片摆线轮） ==========

def UFgraphicsForces(mbs, itemNumber):
    g = []
    # 针齿-孔接触力（最多绘制8个以免太密）
    maxPinsToDraw = min(8, len(pin_load_ids))
    for i in range(maxPinsToDraw):
        lid = pin_load_ids[i]
        F = mbs.GetLoadValues(lid)  # [Fx, Fy, Fz]
        F_mag = np.linalg.norm([F[0], F[1]])
        if F_mag > 0.1:  # 只显示有接触的
            P = mbs.GetMarkerOutput(pin_markers[i], exu.OutputVariableType.Position)
            g.append(graphics.Arrow(pAxis=[P[0], P[1], P[2]],
                                    vAxis=[F[0]*visForceScale, F[1]*visForceScale, F[2]*visForceScale],
                                    radius=0.6, color=graphics.color.yellow, nTiles=50))
    
    # 齿廓-针齿接触力（显示前几个有接触的针齿）
    tooth_force_count = 0
    max_tooth_force_display = 6
    for i in range(min(len(tooth_contact_load_ids) // 2, len(pin_markers))):
        if tooth_force_count >= max_tooth_force_display:
            break
        # 摆线轮1的接触力
        lid1 = tooth_contact_load_ids[2*i]
        F1 = mbs.GetLoadValues(lid1)
        F1_mag = np.linalg.norm([F1[0], F1[1]])
        if F1_mag > 1.0:  # 只显示有明显接触力的
            P = mbs.GetMarkerOutput(pin_markers[i], exu.OutputVariableType.Position)
            g.append(graphics.Arrow(pAxis=[P[0], P[1], P[2]],
                                    vAxis=[F1[0]*visForceScale, F1[1]*visForceScale, F1[2]*visForceScale],
                                    radius=0.5, color=graphics.color.red, nTiles=50))
            tooth_force_count += 1
    
    # ★★★ 摆线轮1受到的反作用力 - 粗箭头（在摆线轮位置显示）★★★
    cyc1_reaction_count = 0
    max_cyc1_reaction_display = 8
    try:
        nodeNumber1 = mbs.GetObject(oCycloid1)['nodeNumber']
        pos_cycloid1 = mbs.GetNodeOutput(nodeNumber1, exu.OutputVariableType.Position)
        for i, lid in enumerate(cycloid1_contact_loads):
            if cyc1_reaction_count >= max_cyc1_reaction_display:
                break
            F = mbs.GetLoadValues(lid)
            F_mag = np.linalg.norm([F[0], F[1]])
            if F_mag > 1.0:
                # 在摆线轮位置显示反作用力 - 粗箭头（radius=0.8）
                g.append(graphics.Arrow(pAxis=[pos_cycloid1[0], pos_cycloid1[1], pos_cycloid1[2]],
                                        vAxis=[F[0]*visForceScale, F[1]*visForceScale, F[2]*visForceScale],
                                        radius=0.8, color=graphics.color.red, nTiles=50))
                cyc1_reaction_count += 1
    except:
        pass
    
    # ★★★ 摆线轮2受到的反作用力 - 细箭头（在摆线轮位置显示）★★★
    cyc2_reaction_count = 0
    max_cyc2_reaction_display = 8
    try:
        nodeNumber2 = mbs.GetObject(oCycloid2)['nodeNumber']
        pos_cycloid2 = mbs.GetNodeOutput(nodeNumber2, exu.OutputVariableType.Position)
        for i, lid in enumerate(cycloid2_contact_loads):
            if cyc2_reaction_count >= max_cyc2_reaction_display:
                break
            F = mbs.GetLoadValues(lid)
            F_mag = np.linalg.norm([F[0], F[1]])
            if F_mag > 1.0:
                # 在摆线轮位置显示反作用力 - 细箭头（radius=0.3）
                g.append(graphics.Arrow(pAxis=[pos_cycloid2[0], pos_cycloid2[1], pos_cycloid2[2]],
                                        vAxis=[F[0]*visForceScale, F[1]*visForceScale, F[2]*visForceScale],
                                        radius=0.3, color=graphics.color.blue, nTiles=50))
                cyc2_reaction_count += 1
    except:
        pass
    
    return g

# 添加一个仅用于绘制的 ground 对象以调用图形回调
mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsDataUserFunction=UFgraphicsForces)))

print("\n" + "="*60)
print("自定义接触算法说明：")
print("="*60)
print("【摆线轮齿廓-针齿接触】")
print("  算法：优化的最近点搜索（Closest Point Search）")
print("  原理：")
print("    1. 粗略搜索：利用缓存的上次接触位置，在附近区域快速定位")
print("    2. 精细迭代：寻找切线与法向垂直的点（切线平行法）")
if USE_THEORY_OPTIMIZATION:
    print("    3. 理论函数优化：使用齿廓方程在连续参数空间精确求解最近点")
print("    条件：从针齿中心到齿廓点的向量 ⊥ 齿廓切线")
print("  力的施加：")
print("    - 针齿受力：F_pin（阻止针齿穿入摆线轮）")
print("    - 摆线轮受力：F_cycloid = -F_pin（牛顿第三定律）")
print("    - 关键修复：必须同时施加两个力才能正确传递扭矩！")
print("  优势：")
print("    - 避免遍历所有齿廓点，计算效率高")
print("    - 利用时间连续性，缓存上次接触位置")
print("    - 迭代收敛快，通常3-5次即可找到最近点")
print("    - 精度高，考虑了齿廓曲率和接触几何")
if USE_THEORY_OPTIMIZATION:
    print("    - 理论函数优化：可以找到离散点之间的真正最近点")
print(f"  参数：搜索范围={contact_cycloid1.search_range}点，最大迭代={contact_cycloid1.max_iter}次")
print("="*60 + "\n")


