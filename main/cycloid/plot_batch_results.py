"""
批量结果绘图脚本
================
读取批量仿真结果，绘制滞回曲线和传动误差曲线。

用法：
    python plot_batch_results.py

输出：
    - hysteresis_curves.png: 所有工况的滞回曲线
    - transmission_error_curves.png: 所有工况的传动误差曲线
    - comparison_subplots.png: 按误差类型分组的对比图
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams

# 设置中文字体
rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
rcParams['axes.unicode_minus'] = False

# 批量结果目录
BATCH_DIR = os.path.join(os.path.dirname(__file__), 'batch_results_crankshaft')
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), 'batch_plots')

# 创建输出目录
os.makedirs(OUTPUT_DIR, exist_ok=True)


def correct_hysteresis_offset(torque, angle):
    """
    校正滞回曲线的y轴偏移
    
    通过查找扭矩穿越零点时的角度值，计算上下零点的中点，
    并将整条曲线平移使该中点位于y=0。
    
    Args:
        torque: 扭矩数组 (x轴)
        angle: 角度数组 (y轴)
    
    Returns:
        angle_corrected: 校正后的角度数组
    """
    if len(torque) < 10:
        return angle
    
    # 查找扭矩穿越零点的位置（排除初始阶段）
    y_at_zero = []
    skip_initial = max(10, int(len(torque) * 0.05))  # 跳过前5%或至少10个点
    
    for i in range(skip_initial, len(torque) - 1):
        # 检测符号变化（穿越零点）
        if torque[i] * torque[i+1] < 0:
            # 线性插值找 torque=0 时的 angle
            t1, t2 = torque[i], torque[i+1]
            a1, a2 = angle[i], angle[i+1]
            if t2 != t1:
                y_interp = a1 + (a2 - a1) * (0 - t1) / (t2 - t1)
                y_at_zero.append(y_interp)
        elif abs(torque[i]) < 1e-6:  # 扭矩接近零
            y_at_zero.append(angle[i])
    
    # 计算上下零点的中点作为偏移量
    if len(y_at_zero) >= 2:
        y_upper = max(y_at_zero)
        y_lower = min(y_at_zero)
        offset = (y_upper + y_lower) / 2
    elif len(y_at_zero) == 1:
        offset = y_at_zero[0]
    else:
        offset = 0
    
    # 平移整条曲线
    angle_corrected = angle - offset
    
    return angle_corrected


def load_hysteresis_data(case_dir):
    """加载滞回曲线数据（刚度模式），并自动校正y轴偏移"""
    filepath = os.path.join(case_dir, 'stiffness', 'torque_angle_curve.txt')
    if not os.path.exists(filepath):
        return None
    data = np.loadtxt(filepath, skiprows=1)
    
    torque = data[:, 1]
    angle = data[:, 2]
    
    # 应用偏移校正
    angle_corrected = correct_hysteresis_offset(torque, angle)
    
    return {
        'time': data[:, 0],
        'torque': torque,
        'angle': angle_corrected
    }


def separate_loading_unloading(torque, angle):
    """
    分离加载和卸载曲线
    
    滞回曲线三个阶段：
    - 阶段1: 0→+T (初始加载，跳过)
    - 阶段2: +T→-T (正向卸载 + 负向加载)
    - 阶段3: -T→+T (负向卸载 + 正向加载)
    
    Returns:
        dict: 包含正向加载/卸载、负向加载/卸载四条曲线
    """
    curves = {
        'pos_loading': ([], []),      # 正向加载: 阶段3中 T>0 且 T递增
        'pos_unloading': ([], []),    # 正向卸载: 阶段2中 T>0 且 T递减
        'neg_loading': ([], []),      # 负向加载: 阶段2中 T<0 且 T递减
        'neg_unloading': ([], [])     # 负向卸载: 阶段3中 T<0 且 T递增
    }
    
    # 找到方向变化点（扭矩从增变减，或从减变增）
    direction_changes = [0]  # 开始位置
    for i in range(1, len(torque) - 1):
        prev_dir = torque[i] - torque[i-1]
        next_dir = torque[i+1] - torque[i]
        if prev_dir * next_dir < 0:  # 方向改变
            direction_changes.append(i)
    direction_changes.append(len(torque) - 1)  # 结束位置
    
    # 确定阶段：阶段1=第一段增加，阶段2=减少段，阶段3=第二段增加
    stage = 1  # 从阶段1开始
    
    for i in range(len(torque) - 1):
        t = torque[i]
        t_next = torque[i + 1]
        a = angle[i]
        
        increasing = t_next > t
        
        # 检测阶段变化
        if stage == 1 and not increasing:
            stage = 2  # 从增变减，进入阶段2
        elif stage == 2 and increasing:
            stage = 3  # 从减变增，进入阶段3
        
        # 根据阶段和扭矩值分类
        if stage == 1:
            # 阶段1: 初始加载 - 跳过
            pass
        elif stage == 2:
            # 阶段2: +T→-T
            if t > 0:  # 正向卸载
                curves['pos_unloading'][0].append(t)
                curves['pos_unloading'][1].append(a)
            else:  # 负向加载
                curves['neg_loading'][0].append(t)
                curves['neg_loading'][1].append(a)
        elif stage == 3:
            # 阶段3: -T→+T
            if t < 0:  # 负向卸载
                curves['neg_unloading'][0].append(t)
                curves['neg_unloading'][1].append(a)
            else:  # 正向加载
                curves['pos_loading'][0].append(t)
                curves['pos_loading'][1].append(a)
    
    # 转换为numpy数组
    for key in curves:
        curves[key] = (np.array(curves[key][0]), np.array(curves[key][1]))
    
    return curves


def interpolate_at_torque(torque_arr, angle_arr, target_torque):
    """在指定扭矩处插值得到角度"""
    if len(torque_arr) < 2:
        return None
    try:
        # np.interp 需要单调递增的x，所以按扭矩排序
        sort_idx = np.argsort(torque_arr)
        torque_sorted = torque_arr[sort_idx]
        angle_sorted = angle_arr[sort_idx]
        
        # 检查目标值是否在范围内
        if target_torque < torque_sorted.min() or target_torque > torque_sorted.max():
            return None
        
        return float(np.interp(target_torque, torque_sorted, angle_sorted))
    except:
        return None


def calculate_stiffness_and_lost_motion(torque, angle, T_rated=3200.0):
    """
    计算扭转刚度和空程
    
    Args:
        torque: 扭矩数组
        angle: 角度数组 (arcmin)
        T_rated: 额定扭矩 (N·m)
    
    Returns:
        dict: 包含 K_pos, K_neg, K_avg, lost_motion 等指标
    """
    curves = separate_loading_unloading(torque, angle)
    
    metrics = {}
    
    # ========== 正向扭转刚度 ==========
    # 使用实际数据的最大扭矩（略小于峰值以确保可插值）
    if len(curves['pos_loading'][0]) > 0 and len(curves['pos_unloading'][0]) > 0:
        T_max_pos = min(curves['pos_loading'][0].max(), curves['pos_unloading'][0].max()) * 0.98
        T2_pos = T_max_pos
        T1_pos = (2.0/3.0) * T2_pos
        
        # 在T1处求中点
        theta_load_1 = interpolate_at_torque(curves['pos_loading'][0], curves['pos_loading'][1], T1_pos)
        theta_unload_1 = interpolate_at_torque(curves['pos_unloading'][0], curves['pos_unloading'][1], T1_pos)
        
        # 在T2处求中点
        theta_load_2 = interpolate_at_torque(curves['pos_loading'][0], curves['pos_loading'][1], T2_pos)
        theta_unload_2 = interpolate_at_torque(curves['pos_unloading'][0], curves['pos_unloading'][1], T2_pos)
        
        if all(v is not None for v in [theta_load_1, theta_unload_1, theta_load_2, theta_unload_2]):
            theta_mid_1 = (theta_load_1 + theta_unload_1) / 2
            theta_mid_2 = (theta_load_2 + theta_unload_2) / 2
            delta_theta = theta_mid_2 - theta_mid_1
            delta_T = T2_pos - T1_pos
            if abs(delta_theta) > 1e-10:
                metrics['K_pos'] = delta_T / delta_theta  # N·m/arcmin
            else:
                metrics['K_pos'] = None
        else:
            metrics['K_pos'] = None
    else:
        metrics['K_pos'] = None
    
    # ========== 负向扭转刚度 ==========
    if len(curves['neg_loading'][0]) > 0 and len(curves['neg_unloading'][0]) > 0:
        T_min_neg = max(curves['neg_loading'][0].min(), curves['neg_unloading'][0].min()) * 0.98
        T2_neg = T_min_neg
        T1_neg = (2.0/3.0) * T2_neg
        
        theta_load_1_neg = interpolate_at_torque(curves['neg_loading'][0], curves['neg_loading'][1], T1_neg)
        theta_unload_1_neg = interpolate_at_torque(curves['neg_unloading'][0], curves['neg_unloading'][1], T1_neg)
        theta_load_2_neg = interpolate_at_torque(curves['neg_loading'][0], curves['neg_loading'][1], T2_neg)
        theta_unload_2_neg = interpolate_at_torque(curves['neg_unloading'][0], curves['neg_unloading'][1], T2_neg)
        
        if all(v is not None for v in [theta_load_1_neg, theta_unload_1_neg, theta_load_2_neg, theta_unload_2_neg]):
            theta_mid_1_neg = (theta_load_1_neg + theta_unload_1_neg) / 2
            theta_mid_2_neg = (theta_load_2_neg + theta_unload_2_neg) / 2
            delta_theta_neg = theta_mid_2_neg - theta_mid_1_neg
            delta_T_neg = T2_neg - T1_neg
            if abs(delta_theta_neg) > 1e-10:
                metrics['K_neg'] = abs(delta_T_neg / delta_theta_neg)  # 取绝对值
        else:
            metrics['K_neg'] = None
    else:
        metrics['K_neg'] = None
    
    # 平均刚度
    if metrics.get('K_pos') and metrics.get('K_neg'):
        metrics['K_avg'] = (metrics['K_pos'] + metrics['K_neg']) / 2
    elif metrics.get('K_pos'):
        metrics['K_avg'] = metrics['K_pos']
    elif metrics.get('K_neg'):
        metrics['K_avg'] = metrics['K_neg']
    else:
        metrics['K_avg'] = None
    
    # ========== 空程 (Lost Motion) ==========
    T_eval_pos = 0.03 * T_rated  # +3%
    T_eval_neg = -0.03 * T_rated  # -3%
    
    # 在+3%处求中点
    theta_load_pos = interpolate_at_torque(curves['pos_loading'][0], curves['pos_loading'][1], T_eval_pos)
    theta_unload_pos = interpolate_at_torque(curves['pos_unloading'][0], curves['pos_unloading'][1], T_eval_pos)
    
    # 在-3%处求中点
    theta_load_neg = interpolate_at_torque(curves['neg_loading'][0], curves['neg_loading'][1], T_eval_neg)
    theta_unload_neg = interpolate_at_torque(curves['neg_unloading'][0], curves['neg_unloading'][1], T_eval_neg)
    
    if all(v is not None for v in [theta_load_pos, theta_unload_pos, theta_load_neg, theta_unload_neg]):
        theta_mid_pos = (theta_load_pos + theta_unload_pos) / 2
        theta_mid_neg = (theta_load_neg + theta_unload_neg) / 2
        metrics['lost_motion'] = abs(theta_mid_pos - theta_mid_neg)  # arcmin
    else:
        metrics['lost_motion'] = None
        
    # ========== 背隙 (Backlash) ==========
    # 扭矩为0时的上下曲线角度差
    # 上曲线（卸载方向）：pos_unloading (T>0) 和 neg_loading (T<0) 的组合
    # 下曲线（加载方向）：pos_loading (T>0) 和 neg_unloading (T<0) 的组合
    
    # 构建上曲线数据用于插值
    upper_torque = np.concatenate([curves['neg_loading'][0], curves['pos_unloading'][0]])
    upper_angle = np.concatenate([curves['neg_loading'][1], curves['pos_unloading'][1]])
    
    # 构建下曲线数据用于插值
    lower_torque = np.concatenate([curves['neg_unloading'][0], curves['pos_loading'][0]])
    lower_angle = np.concatenate([curves['neg_unloading'][1], curves['pos_loading'][1]])
    
    theta_upper_0 = interpolate_at_torque(upper_torque, upper_angle, 0.0)
    theta_lower_0 = interpolate_at_torque(lower_torque, lower_angle, 0.0)
    
    if theta_upper_0 is not None and theta_lower_0 is not None:
        metrics['backlash'] = abs(theta_upper_0 - theta_lower_0)
    else:
        metrics['backlash'] = None

    return metrics


def load_transmission_error_data(case_dir):
    """加载传动误差数据（精度模式）"""
    # 尝试从原始旋转数据计算（更准确，可修正传动比）
    input_file = os.path.join(case_dir, 'accuracy', 'input_rotation.txt')
    output_file = os.path.join(case_dir, 'accuracy', 'output_rotation.txt')
    
    if os.path.exists(input_file) and os.path.exists(output_file):
        try:
            # 读取数据
            data_in = np.loadtxt(input_file, skiprows=1)
            data_out = np.loadtxt(output_file, skiprows=1)
            
            # 提取时间
            time_in = data_in[:, 0]
            time_out = data_out[:, 0]
            
            # 提取角度 (atan2(R21, R11))
            # 列索引: time=0, R11=1, R12=2, R13=3, R21=4, ...
            angle_in = np.unwrap(np.arctan2(data_in[:, 4], data_in[:, 1]))
            angle_out = np.unwrap(np.arctan2(data_out[:, 4], data_out[:, 1]))
            
            # 插值到同一时间轴
            time_common = np.linspace(max(time_in[0], time_out[0]),
                                      min(time_in[-1], time_out[-1]),
                                      min(len(time_in), len(time_out)))
            
            angle_in_interp = np.interp(time_common, time_in, angle_in)
            angle_out_interp = np.interp(time_common, time_out, angle_out)
            
            # 计算传动误差 (使用正确的总传动比 117.0)
            # 117.0 = 3.0 (Input->Crank) * 39.0 (Crank->Output)
            TOTAL_RATIO = 117.0
            theoretical_out = angle_in_interp / TOTAL_RATIO
            error_rad = angle_out_interp - theoretical_out
            
            # 转换为 arcsec
            error_arcsec = error_rad * (180.0 / np.pi) * 3600.0
            
            print(f"  [Info] Re-calculated error for {os.path.basename(case_dir)} using ratio {TOTAL_RATIO}")
            return {
                'time': time_common,
                'error': error_arcsec
            }
        except Exception as e:
            print(f"  [Warn] Failed to recalculate error for {os.path.basename(case_dir)}: {e}")

    # 回退到读取预计算的文件
    filepath = os.path.join(case_dir, 'accuracy', 'transmission_error_arcsec.txt')
    if not os.path.exists(filepath):
        return None
    data = np.loadtxt(filepath, skiprows=1)
    return {
        'time': data[:, 0],
        'error': data[:, 1]
    }


def parse_case_name(case_name):
    """解析工况名称，提取误差类型和数值"""
    if 'err_radius' in case_name:
        parts = case_name.split('err_radius_')
        value = parts[1].replace('um', '').replace('arcsec', '')
        return 'eccentric_radius', float(value), f"偏心圆半径误差 {value} μm"
    elif 'err_eccentricity' in case_name:
        parts = case_name.split('err_eccentricity_')
        value = parts[1].replace('um', '').replace('arcsec', '')
        return 'eccentricity', float(value), f"偏心距误差 {value} μm"
    elif 'err_angle' in case_name:
        parts = case_name.split('err_angle_')
        value = parts[1].replace('arcsec', '')
        return 'angle', float(value), f"偏心角误差 {value} arcsec"
    elif 'bearing_clearance' in case_name:
        parts = case_name.split('bearing_clearance_')
        value = parts[1].replace('um', '')
        return 'bearing_clearance', float(value), f"轴承间隙调整 {value} μm"
    return 'unknown', 0, case_name


def get_all_cases():
    """获取所有工况目录"""
    cases = []
    if not os.path.exists(BATCH_DIR):
        print(f"批量结果目录不存在: {BATCH_DIR}")
        return cases
    
    for name in sorted(os.listdir(BATCH_DIR)):
        case_path = os.path.join(BATCH_DIR, name)
        if os.path.isdir(case_path):
            error_type, error_value, label = parse_case_name(name)
            cases.append({
                'name': name,
                'path': case_path,
                'error_type': error_type,
                'error_value': error_value,
                'label': label
            })
    return cases


def plot_hysteresis_curves(cases, output_dir):
    """
    绘制所有工况的滞回曲线，并计算刚度和空程
    每种误差类型输出单独的图片
    """
    error_types = ['eccentric_radius', 'eccentricity', 'angle', 'bearing_clearance']
    titles = ['偏心圆半径误差', '偏心距误差', '偏心角误差', '轴承间隙调整']
    
    # 存储所有计算结果
    all_metrics = []
    
    for etype, title in zip(error_types, titles):
        # 创建单独的图形
        fig, ax = plt.subplots(figsize=(8, 6))
        
        ax.set_title(title, fontsize=14)
        ax.set_xlabel('扭矩 (N·m)', fontsize=12)
        ax.set_ylabel('输出角度 (arcmin)', fontsize=12)
        ax.grid(True, alpha=0.3)
        
        type_cases = [c for c in cases if c['error_type'] == etype]
        # 按误差值排序，确保从小到大显示
        type_cases = sorted(type_cases, key=lambda x: x['error_value'])
        colors = plt.cm.viridis(np.linspace(0, 1, len(type_cases)))
        
        # 用于存储本类型的所有文本标注
        text_lines = []
        
        for case, color in zip(type_cases, colors):
            data = load_hysteresis_data(case['path'])
            if data is not None:
                # 根据误差类型确定单位
                if etype == 'angle':
                    unit = '"'  # arcsec
                else:
                    unit = 'μm'
                
                ax.plot(data['torque'], data['angle'], 
                       color=color, linewidth=1.5, alpha=0.8,
                       label=f"{case['error_value']:.1f}{unit}")
                
                # 计算刚度和空程
                metrics = calculate_stiffness_and_lost_motion(data['torque'], data['angle'])
                
                # 保存结果
                result = {
                    'case_name': case['name'],
                    'error_type': etype,
                    'error_value': case['error_value'],
                    'K_pos': metrics.get('K_pos'),
                    'K_neg': metrics.get('K_neg'),
                    'K_avg': metrics.get('K_avg'),
                    'lost_motion': metrics.get('lost_motion'),
                    'backlash': metrics.get('backlash')
                }
                all_metrics.append(result)
                
                # 准备文本标注（带单位）
                k_str = f"{metrics.get('K_avg', 0):.0f}" if metrics.get('K_avg') else "N/A"
                lm_str = f"{metrics.get('lost_motion', 0):.3f}" if metrics.get('lost_motion') else "N/A"
                bl_str = f"{metrics.get('backlash', 0):.3f}" if metrics.get('backlash') else "N/A"
                text_lines.append(f"{case['error_value']:.1f}{unit}: K={k_str}N·m/', LM={lm_str}', BL={bl_str}'")
        
        if type_cases:
            # 根据误差类型设置图例标题
            if etype == 'angle':
                legend_title = '误差值 (arcsec)'
            else:
                legend_title = '误差值 (μm)'
            ax.legend(title=legend_title, fontsize=9, loc='upper left')
            
            # 在图上添加计算结果文本（显示所有）
            text_content = '\n'.join(text_lines)
            ax.text(0.98, 0.02, text_content, transform=ax.transAxes,
                   fontsize=8, verticalalignment='bottom', horizontalalignment='right',
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                   family='monospace')
        
        plt.tight_layout()
        
        # 保存单个图片
        filename = f'hysteresis_curves_{etype}.png'
        save_path = os.path.join(output_dir, filename)
        plt.savefig(save_path, dpi=200, bbox_inches='tight')
        print(f"滞回曲线已保存: {save_path}")
        plt.close()
    
    # 保存计算结果到txt文件（汇总所有类型）
    metrics_file = os.path.join(output_dir, 'hysteresis_curves_metrics.txt')
    with open(metrics_file, 'w', encoding='utf-8') as f:
        f.write("滞回曲线刚度和空程计算结果\n")
        f.write("=" * 100 + "\n")
        f.write(f"{'工况名称':<50} {'K+ (N·m/arcmin)':<18} {'K- (N·m/arcmin)':<18} {'K_avg':<15} {'空程 (arcmin)':<15} {'背隙 (arcmin)':<15}\n")
        f.write("-" * 100 + "\n")
        for m in all_metrics:
            k_pos = f"{m['K_pos']:.2f}" if m['K_pos'] else "N/A"
            k_neg = f"{m['K_neg']:.2f}" if m['K_neg'] else "N/A"
            k_avg = f"{m['K_avg']:.2f}" if m['K_avg'] else "N/A"
            lm = f"{m['lost_motion']:.4f}" if m['lost_motion'] else "N/A"
            bl = f"{m['backlash']:.4f}" if m['backlash'] else "N/A"
            f.write(f"{m['case_name']:<50} {k_pos:<18} {k_neg:<18} {k_avg:<15} {lm:<15} {bl:<15}\n")
        f.write("=" * 100 + "\n")
        f.write("\n说明:\n")
        f.write("  K+ = 正向扭转刚度 (在 2/3*T_rated 到 T_rated 区间计算)\n")
        f.write("  K- = 负向扭转刚度 (在 -2/3*T_rated 到 -T_rated 区间计算)\n")
        f.write("  K_avg = 平均扭转刚度 = (K+ + K-) / 2\n")
        f.write("  空程 (Lost Motion) = 在 ±3%*T_rated 处滞回曲线中点的角度差\n")
        f.write("  背隙 (Backlash) = 在 0 N·m 扭矩处，卸载曲线与加载曲线的角度差\n")
    
    print(f"计算结果已保存: {metrics_file}")
    
    return all_metrics


def plot_transmission_error_curves(cases, output_dir):
    """绘制所有工况的传动误差曲线（每种误差类型单独图片）"""
    error_types = ['eccentric_radius', 'eccentricity', 'angle', 'bearing_clearance']
    titles = ['偏心圆半径误差', '偏心距误差', '偏心角误差', '轴承间隙调整']
    
    for etype, title in zip(error_types, titles):
        fig, ax = plt.subplots(figsize=(10, 5))
        
        ax.set_title(f"{title} - 传动误差", fontsize=14)
        ax.set_xlabel('时间 (s)', fontsize=12)
        ax.set_ylabel('传动误差 (arcsec)', fontsize=12)
        ax.grid(True, alpha=0.3)
        
        type_cases = [c for c in cases if c['error_type'] == etype]
        # 按误差值排序
        type_cases = sorted(type_cases, key=lambda x: x['error_value'])
        colors = plt.cm.plasma(np.linspace(0, 1, len(type_cases)))
        
        for case, color in zip(type_cases, colors):
            data = load_transmission_error_data(case['path'])
            if data is not None:
                # 确定单位
                if etype == 'angle':
                    unit = '"'
                else:
                    unit = 'μm'
                    
                ax.plot(data['time'], data['error'], 
                       color=color, linewidth=1.0, alpha=0.8,
                       label=f"{case['error_value']:.1f}{unit}")
        
        if type_cases:
            if etype == 'angle':
                legend_title = '误差值 (arcsec)'
            else:
                legend_title = '误差值 (μm)'
            ax.legend(title=legend_title, fontsize=9, loc='upper right')
    
        plt.tight_layout()
        
        filename = f'transmission_error_{etype}.png'
        save_path = os.path.join(output_dir, filename)
        plt.savefig(save_path, dpi=200, bbox_inches='tight')
        print(f"传动误差曲线已保存: {save_path}")
        plt.close()


def plot_comparison_summary(cases, output_path):
    """绘制对比汇总图（每种误差类型单独一行）"""
    fig, axes = plt.subplots(4, 2, figsize=(14, 16))
    error_types = ['eccentric_radius', 'eccentricity', 'angle', 'bearing_clearance']
    titles = ['偏心圆半径误差', '偏心距误差', '偏心角误差', '轴承间隙调整']
    
    for row, (etype, title) in enumerate(zip(error_types, titles)):
        type_cases = [c for c in cases if c['error_type'] == etype]
        # 排序
        type_cases = sorted(type_cases, key=lambda x: x['error_value'])
        colors = plt.cm.tab10(np.linspace(0, 1, len(type_cases)))
        
        # 左列：滞回曲线
        ax_hyst = axes[row, 0]
        ax_hyst.set_title(f'{title} - 滞回曲线', fontsize=11)
        ax_hyst.set_xlabel('扭矩 (N·m)')
        ax_hyst.set_ylabel('输出角度 (arcmin)')
        ax_hyst.grid(True, alpha=0.3)
        
        # 右列：传动误差
        ax_te = axes[row, 1]
        ax_te.set_title(f'{title} - 传动误差', fontsize=11)
        ax_te.set_xlabel('时间 (s)')
        ax_te.set_ylabel('传动误差 (arcsec)')
        ax_te.grid(True, alpha=0.3)
        
        text_lines = []
        
        for case, color in zip(type_cases, colors):
            # 滞回曲线
            hyst_data = load_hysteresis_data(case['path'])
            if hyst_data is not None:
                # 确定单位
                if etype == 'angle':
                    unit = '"'
                else:
                    unit = 'μm'
                    
                ax_hyst.plot(hyst_data['torque'], hyst_data['angle'], 
                            color=color, linewidth=1.0, alpha=0.9,
                            label=f"{case['error_value']:.1f}{unit}")
                
                # 计算指标用于标注
                metrics = calculate_stiffness_and_lost_motion(hyst_data['torque'], hyst_data['angle'])
                k_str = f"{metrics.get('K_avg', 0):.0f}" if metrics.get('K_avg') else "N/A"
                lm_str = f"{metrics.get('lost_motion', 0):.3f}" if metrics.get('lost_motion') else "N/A"
                bl_str = f"{metrics.get('backlash', 0):.3f}" if metrics.get('backlash') else "N/A"
                text_lines.append(f"{case['error_value']:.1f}{unit}: K={k_str}, LM={lm_str}, BL={bl_str}")
            
            # 传动误差
            te_data = load_transmission_error_data(case['path'])
            if te_data is not None:
                ax_te.plot(te_data['time'], te_data['error'], 
                          color=color, linewidth=1.0, alpha=0.9,
                          label=f"{case['error_value']:.1f}")
        
        if type_cases:
            if etype == 'angle':
                legend_title = '误差值 (arcsec)'
            else:
                legend_title = '误差值 (μm)'
                
            ax_hyst.legend(title=legend_title, fontsize=6, loc='upper left', ncol=2)
            ax_te.legend(title=legend_title, fontsize=6, loc='upper right', ncol=2)
            
            # 添加指标文本
            text_content = '\n'.join(text_lines)
            ax_hyst.text(0.98, 0.02, text_content, transform=ax_hyst.transAxes,
                   fontsize=5, verticalalignment='bottom', horizontalalignment='right',
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                   family='monospace')
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=200, bbox_inches='tight')
    print(f"对比汇总图已保存: {output_path}")
    plt.close()


def main():
    print("=" * 60)
    print("批量结果绘图")
    print("=" * 60)
    
    # 获取所有工况
    cases = get_all_cases()
    print(f"\n找到 {len(cases)} 个工况")
    
    if not cases:
        print("未找到批量结果，请先运行批量仿真脚本")
        return
    
    # 统计各类型工况数量
    from collections import Counter
    type_counts = Counter(c['error_type'] for c in cases)
    print("\n工况统计:")
    for etype, count in type_counts.items():
        print(f"  {etype}: {count} 组")
    
    # 绘制滞回曲线
    print("\n绘制滞回曲线...")
    plot_hysteresis_curves(cases, OUTPUT_DIR)
    
    # 绘制传动误差曲线
    print("绘制传动误差曲线...")
    plot_transmission_error_curves(cases, OUTPUT_DIR)
    
    # 绘制对比汇总图
    print("绘制对比汇总图...")
    plot_comparison_summary(cases, os.path.join(OUTPUT_DIR, 'comparison_summary.png'))
    
    print("\n" + "=" * 60)
    print(f"所有图片已保存至: {OUTPUT_DIR}")
    print("=" * 60)


if __name__ == '__main__':
    main()
