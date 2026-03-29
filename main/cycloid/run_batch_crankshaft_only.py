"""
曲柄轴精度批量分析脚本（只包含曲柄轴精度参数）
================================================

研究目标：只分析曲柄轴的3个制造精度参数对减速器性能的影响
- 偏心圆半径误差（eccentric_error）
- 偏心距误差（eccentricity_error）
- 偏心角误差（angle_error）

固定工况参数：load_torque=100 N·m, input_speed=60 rpm

使用方法：
    python run_batch_crankshaft_only.py

输出：
    - batch_results_crankshaft.xlsx: 曲柄轴精度影响数据
"""

import sys
import os
import numpy as np
import pandas as pd
import json
import subprocess
import tempfile

# 确保当前目录在Python路径中
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ============ 参数定义（只包含曲柄轴精度参数） ============

print("="*70)
print("曲柄轴精度批量分析")
print("="*70)

# 固定工况参数（根据文献2的测试条件）
FIXED_LOAD_TORQUE = 100  # N·m（文献2使用的测试扭矩）
FIXED_INPUT_SPEED = 60    # rpm（典型工作转速）

print(f"\n固定工况参数:")
print(f"  负载扭矩: {FIXED_LOAD_TORQUE} N·m")
print(f"  输入转速: {FIXED_INPUT_SPEED} rpm")

# ============ 曲柄轴精度参数（根据文献扩展范围） ============

# 1. 偏心圆半径误差 (eccentric_error)
# 文献范围：高精度(-3~0), 中精度(-6~0), 低精度(-9~0)
print("\n1. 偏心圆半径误差 (eccentric_error):")
eccentric_radius_errors = np.linspace(-10e-6, 0, 6)  # -10~0 μm
print(f"   范围: {eccentric_radius_errors[0]*1e6:.1f} ~ {eccentric_radius_errors[-1]*1e6:.1f} μm")
print(f"   水平数: {len(eccentric_radius_errors)}")

# 2. 偏心距误差 (eccentricity_error)
# 文献范围：高精度(±2), 中精度(±4), 低精度(±6)
print("\n2. 偏心距误差 (eccentricity_error):")
eccentricity_errors = np.linspace(-6e-6, 6e-6, 7)  # ±6 μm
print(f"   范围: {eccentricity_errors[0]*1e6:.1f} ~ {eccentricity_errors[-1]*1e6:.1f} μm")
print(f"   水平数: {len(eccentricity_errors)}")

# 3. 偏心角误差 (angle_error)
# 文献范围：高精度(±60), 中精度(±90), 低精度(±120)
print("\n3. 偏心角误差 (angle_error):")
angle_errors = np.linspace(-120, 120, 5)  # ±120 arcsec
angle_errors_rad = angle_errors * (np.pi / (180 * 3600))
print(f"   范围: {angle_errors[0]:.1f} ~ {angle_errors[-1]:.1f} arcsec")
print(f"   水平数: {len(angle_errors)}")

# 4. 轴承间隙调整 (bearing_clearance_adjust) - 新增
# 文献指出转臂轴承径向间隙对回程间隙影响显著
print("\n4. 轴承间隙调整 (bearing_clearance_adjust):")
bearing_clearance_adjusts = np.linspace(0, 20e-6, 5)  # 0~20 μm (增加间隙)
print(f"   范围: {bearing_clearance_adjusts[0]*1e6:.1f} ~ {bearing_clearance_adjusts[-1]*1e6:.1f} μm")
print(f"   水平数: {len(bearing_clearance_adjusts)}")

# ============ 生成参数组合（单因素扫描）============
print("\n" + "="*70)
print("生成参数组合...")
print("="*70)

param_sets = []

# 扫描1：偏心圆半径误差（最重要）
print(f"\n扫描1: 偏心圆半径误差 ({len(eccentric_radius_errors)}组)")
for err in eccentric_radius_errors:
    param_sets.append({
        'case_name': f'err_radius_{err*1e6:.1f}um',
        'eccentric_error': err,
        'eccentricity_error': 0,
        'angle_error': 0,
        'bearing_clearance_adjust': 0,
        'load_torque': FIXED_LOAD_TORQUE,
        'input_speed': FIXED_INPUT_SPEED,
        'simulation_time': 1.0,
        'description': '偏心圆半径误差'
    })

# 扫描2：偏心距误差
print(f"\n扫描2: 偏心距误差 ({len(eccentricity_errors)}组)")
for err in eccentricity_errors:
    param_sets.append({
        'case_name': f'err_eccentricity_{err*1e6:.1f}um',
        'eccentric_error': 0,
        'eccentricity_error': err,
        'angle_error': 0,
        'bearing_clearance_adjust': 0,
        'load_torque': FIXED_LOAD_TORQUE,
        'input_speed': FIXED_INPUT_SPEED,
        'simulation_time': 1.0,
        'description': '偏心距误差'
    })

# 扫描3：偏心角误差
print(f"\n扫描3: 偏心角误差 ({len(angle_errors)}组)")
for i, err in enumerate(angle_errors_rad):
    param_sets.append({
        'case_name': f'err_angle_{angle_errors[i]:.1f}arcsec',
        'eccentric_error': 0,
        'eccentricity_error': 0,
        'angle_error': err,
        'bearing_clearance_adjust': 0,
        'load_torque': FIXED_LOAD_TORQUE,
        'input_speed': FIXED_INPUT_SPEED,
        'simulation_time': 1.0,
        'description': '偏心角误差'
    })

# 扫描4：轴承间隙调整（新增）
print(f"\n扫描4: 轴承间隙调整 ({len(bearing_clearance_adjusts)}组)")
for err in bearing_clearance_adjusts:
    param_sets.append({
        'case_name': f'bearing_clearance_{err*1e6:.1f}um',
        'eccentric_error': 0,
        'eccentricity_error': 0,
        'angle_error': 0,
        'bearing_clearance_adjust': err,
        'load_torque': FIXED_LOAD_TORQUE,
        'input_speed': FIXED_INPUT_SPEED,
        'simulation_time': 1.0,
        'description': '轴承间隙调整'
    })

print(f"\n总计 {len(param_sets)} 组参数")
print(f"每组运行1次（精度）= {len(param_sets)} 次仿真")
print("="*70)

# ============ 定义运行单次仿真的函数 ============
def run_single_simulation(params, mode, output_dir):
    """
    运行单次仿真

    Args:
        params: 参数字典
        mode: 'stiffness' 或 'accuracy'
        output_dir: 输出目录

    Returns:
        (status, metrics): 状态和指标字典
    """
    # 添加模式参数
    params_mode = params.copy()
    params_mode['simulation_mode'] = mode
    params_mode['output_dir'] = output_dir
    params_mode['show_graphics'] = False  # 批量模式禁用图形显示，避免阻塞

    # 创建临时参数文件
    param_file = tempfile.mktemp(suffix='.json')
    with open(param_file, 'w', encoding='utf-8') as f:
        json.dump(params_mode, f)

    try:
        # 运行cycloidDyn.py（精度分析）
        result = subprocess.run(
            [sys.executable, 'cycloidDyn.py', param_file],
            capture_output=False,  # 改为False，输出直接显示到终端以查看进度
            text=True,
            cwd=os.path.dirname(os.path.abspath(__file__))
        )

        # 清理参数文件
        try:
            os.unlink(param_file)
        except:
            pass

        # 读取结果（使用绝对路径）
        script_dir = os.path.dirname(os.path.abspath(__file__))
        abs_output_dir = os.path.normpath(os.path.join(script_dir, output_dir))
        if mode == 'stiffness':
            metrics_file = os.path.join(abs_output_dir, 'stiffness_metrics.txt')
        else:
            metrics_file = os.path.join(abs_output_dir, 'accuracy_metrics.txt')

        # 等待文件系统同步（防止缓存问题）
        import time
        time.sleep(0.5)

        if os.path.exists(metrics_file):
            # 解析结果
            metrics = {}
            with open(metrics_file, 'r', encoding='utf-8') as f:
                lines = f.readlines()
                for line in lines:
                    if ':' in line:
                        key, value = line.split(':', 1)
                        key = key.strip()
                        value = value.strip()
                        try:
                            value_float = float(''.join(c for c in value if (c.isdigit() or c == '.' or c == '-' or c == 'e' or c == '+')))
                            metrics[key] = value_float
                        except:
                            pass
            return 'success', metrics
        else:
            print(f"    [DEBUG] 文件不存在: {metrics_file}")
            return 'failed', {}

    except subprocess.TimeoutExpired:
        return 'timeout', {}
    except Exception as e:
        return 'error', {}

# ============ 批量运行（刚度+精度）============
print("\n开始批量仿真...")
print("="*70)

results = []

for case_id, params in enumerate(param_sets):
    print(f"\n{'='*70}")
    print(f"Case {case_id + 1}/{len(param_sets)}: {params['description']}")
    print(f"  参数名称: {params['case_name']}")
    print(f"{'='*70}")

    # 打印曲柄轴精度参数
    if params['eccentric_error'] != 0:
        print(f"  偏心圆半径误差: {params['eccentric_error']*1e6:.2f} μm")
    if params['eccentricity_error'] != 0:
        print(f"  偏心距误差: {params['eccentricity_error']*1e6:.2f} μm")
    if params['angle_error'] != 0:
        print(f"  偏心角误差: {np.rad2deg(params['angle_error'])*3600:.1f} arcsec")

    # ======== 步骤1：跳过刚度计算 ========
    print(f"\n  [1/2] 跳过刚度计算...")
    output_dir_stiffness = f'./batch_results_crankshaft/case_{case_id:03d}_{params["case_name"]}/stiffness'
    status_stiff = 'skipped'
    metrics_stiff = {}
    print(f"    - 刚度计算已跳过")

    # ======== 步骤2：运行精度计算 ========
    print(f"  [2/2] 运行精度计算...")
    output_dir_accuracy = f'./batch_results_crankshaft/case_{case_id:03d}_{params["case_name"]}/accuracy'
    status_acc, metrics_acc = run_single_simulation(params, 'accuracy', output_dir_accuracy)

    if status_acc == 'success':
        print(f"    ✓ 精度计算完成")
        print(f"      平均传动误差: {metrics_acc.get('平均传动误差', 'N/A')} arcsec")
        print(f"      传动误差峰峰值: {metrics_acc.get('传动误差峰峰值', 'N/A')} arcsec")
    else:
        print(f"    ✗ 精度计算失败: {status_acc}")

    # ======== 合并结果 ========
    result_record = {
        'case_id': case_id,
        'description': params['description'],
        'case_name': params['case_name'],
        # 曲柄轴精度参数
        'eccentric_error_um': params['eccentric_error'] * 1e6,
        'eccentricity_error_um': params['eccentricity_error'] * 1e6,
        'angle_error_arcsec': np.rad2deg(params['angle_error']) * 3600 if params['angle_error'] != 0 else 0,
        # 固定工况参数
        'load_torque_Nm': params['load_torque'],
        'input_speed_rpm': params['input_speed'],
    }

    # 刚度指标
    result_record['stiffness_status'] = status_stiff
    if status_stiff == 'success':
        result_record['torsional_stiffness_Nm_per_arcmin'] = metrics_stiff.get('扭转刚度', np.nan)
        result_record['stiffness_r_squared'] = metrics_stiff.get('stiffness_r_squared', np.nan)
        result_record['lost_motion_stiffness_arcmin'] = metrics_stiff.get('回程间隙', np.nan)
        result_record['hysteresis_area'] = metrics_stiff.get('迟滞面积', np.nan)
        result_record['stiffness_output_dir'] = output_dir_stiffness
    else:
        result_record['torsional_stiffness_Nm_per_arcmin'] = np.nan
        result_record['stiffness_r_squared'] = np.nan
        result_record['lost_motion_stiffness_arcmin'] = np.nan
        result_record['hysteresis_area'] = np.nan
        result_record['stiffness_output_dir'] = output_dir_stiffness

    # 精度指标
    result_record['accuracy_status'] = status_acc
    if status_acc == 'success':
        result_record['mean_transmission_error_arcsec'] = metrics_acc.get('平均传动误差', np.nan)
        result_record['transmission_error_std_arcsec'] = metrics_acc.get('传动误差标准差', np.nan)
        result_record['transmission_error_peak_to_peak_arcsec'] = metrics_acc.get('传动误差峰峰值', np.nan)
        result_record['max_instantaneous_error_arcsec'] = metrics_acc.get('瞬时最大误差', np.nan)
        result_record['lost_motion_accuracy_arcmin'] = metrics_acc.get('回程间隙', np.nan)
        result_record['actual_reduction_ratio'] = metrics_acc.get('实际减速比', np.nan)
        result_record['accuracy_output_dir'] = output_dir_accuracy
    else:
        result_record['mean_transmission_error_arcsec'] = np.nan
        result_record['transmission_error_std_arcsec'] = np.nan
        result_record['transmission_error_peak_to_peak_arcsec'] = np.nan
        result_record['max_instantaneous_error_arcsec'] = np.nan
        result_record['lost_motion_accuracy_arcmin'] = np.nan
        result_record['actual_reduction_ratio'] = np.nan
        result_record['accuracy_output_dir'] = output_dir_accuracy

    results.append(result_record)

    # 实时保存进度
    df = pd.DataFrame(results)
    df.to_excel('batch_results_crankshaft.xlsx', index=False)
    df.to_csv('batch_results_crankshaft.csv', index=False)

    print(f"\n  ✓ Case {case_id + 1} 完成")

# ============ 完成 ============
print("\n" + "="*70)
print("曲柄轴精度批量分析完成！")
print("="*70)

# 统计结果
df = pd.DataFrame(results)
n_stiff_success = (df['stiffness_status'] == 'success').sum()
n_acc_success = (df['accuracy_status'] == 'success').sum()

print(f"\n结果统计:")
print(f"  刚度计算成功: {n_stiff_success}/{len(df)}")
print(f"  精度计算成功: {n_acc_success}/{len(df)}")

if n_acc_success > 0:
    df_acc = df[df['accuracy_status'] == 'success']
    print(f"\n精度指标统计（成功样本）:")
    print(f"  平均传动误差: {df_acc['mean_transmission_error_arcsec'].mean():.2f} ± {df_acc['mean_transmission_error_arcsec'].std():.2f} arcsec")
    print(f"  回程间隙（精度）: {df_acc['lost_motion_accuracy_arcmin'].mean():.4f} ± {df_acc['lost_motion_accuracy_arcmin'].std():.4f} arcmin")

if n_stiff_success > 0:
    df_stiff = df[df['stiffness_status'] == 'success']
    print(f"\n刚度指标统计（成功样本）:")
    print(f"  扭转刚度: {df_stiff['torsional_stiffness_Nm_per_arcmin'].mean():.2f} ± {df_stiff['torsional_stiffness_Nm_per_arcmin'].std():.2f} N·m/arcmin")
    print(f"  回程间隙（刚度）: {df_stiff['lost_motion_stiffness_arcmin'].mean():.4f} ± {df_stiff['lost_motion_stiffness_arcmin'].std():.4f} arcmin")

print(f"\n结果已保存:")
print(f"  Excel: batch_results_crankshaft.xlsx")
print(f"  CSV: batch_results_crankshaft.csv")
print(f"  详细数据: batch_results_crankshaft/")
print("="*70)
