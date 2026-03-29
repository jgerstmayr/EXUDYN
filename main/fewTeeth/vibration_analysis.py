"""
振动响应频谱分析脚本
读取 fewTeethDyn.py 生成的 vibration_response.csv，进行FFT频谱分析
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import fft
from scipy.signal import welch, spectrogram
from scipy.signal.windows import hann
import os
import argparse

# 设置中文字体
plt.rcParams['font.family'] = ['Microsoft YaHei', 'SimHei', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['mathtext.fontset'] = 'cm'


def load_vibration_data(csv_path: str) -> dict:
    """
    加载振动响应CSV数据
    
    Returns:
        dict: 包含时间和各通道数据的字典
    """
    data = np.loadtxt(csv_path, delimiter=',', skiprows=1)
    
    return {
        'time': data[:, 0],
        'flange_omega': data[:, 1:4],      # 法兰角速度 [rad/s]
        'flange_alpha': data[:, 4:7],      # 法兰角加速度 [rad/s²]
        'cycloid_acc': data[:, 7:10],      # 主齿轮加速度 [m/s²]
        'input_omega': data[:, 10:13],     # 输入轴角速度 [rad/s]
        'crank_omega_z': data[:, 13],      # 曲柄轴Z角速度 [rad/s]
    }


def compute_fft(signal: np.ndarray, dt: float) -> tuple:
    """
    计算FFT频谱
    
    Args:
        signal: 时域信号
        dt: 采样间隔
        
    Returns:
        (freq, amplitude): 频率数组和幅值数组
    """
    n = len(signal)
    # 去直流分量
    signal_ac = signal - np.mean(signal)
    # 加窗减少频谱泄漏
    window = hann(n)
    signal_windowed = signal_ac * window
    
    # FFT
    spectrum = fft.rfft(signal_windowed)
    freq = fft.rfftfreq(n, dt)
    amplitude = np.abs(spectrum) * 2 / n
    
    return freq, amplitude


def compute_psd(signal: np.ndarray, fs: float, nperseg: int = None) -> tuple:
    """
    计算功率谱密度 (Welch方法)
    
    Args:
        signal: 时域信号
        fs: 采样频率
        nperseg: 每段长度
        
    Returns:
        (freq, psd): 频率数组和功率谱密度数组
    """
    if nperseg is None:
        nperseg = min(256, len(signal) // 4)
    
    signal_ac = signal - np.mean(signal)
    freq, psd = welch(signal_ac, fs=fs, nperseg=nperseg)
    
    return freq, psd


def compute_spectrogram(signal: np.ndarray, fs: float, nperseg: int = None, noverlap: int = None) -> tuple:
    """
    计算频谱图 (Spectrogram) 用于时频分析
    
    Args:
        signal: 时域信号
        fs: 采样频率
        nperseg: 每段长度（越大频率分辨率越高，但时间分辨率降低）
        noverlap: 重叠点数（越大时间分辨率越高）
        
    Returns:
        (times, freq, Sxx): 时间数组, 频率数组, 功率谱矩阵
    """
    n = len(signal)
    if nperseg is None:
        # 根据数据长度自适应选择窗口大小
        # 尽量用大窗口获得好的频率分辨率
        nperseg = min(256, n // 2)
        nperseg = max(nperseg, 32)  # 最小32点
    if noverlap is None:
        noverlap = nperseg * 3 // 4  # 75%重叠，提高时间分辨率
    
    signal_ac = signal - np.mean(signal)
    freq, times, Sxx = spectrogram(signal_ac, fs=fs, nperseg=nperseg, noverlap=noverlap)
    
    return times, freq, Sxx


def analyze_and_plot(data: dict, output_dir: str, input_speed_rpm: float = 1000, start_time: float = 0.15, freq_max: float = 5000):
    """
    分析振动数据并生成图表
    
    Args:
        data: 振动数据字典
        output_dir: 输出目录
        input_speed_rpm: 输入轴转速 (rpm)
        start_time: 分析起始时间 (s)，忽略之前的瞬态数据
        freq_max: 频谱显示的最大频率 (Hz)，默认500
    """
    os.makedirs(output_dir, exist_ok=True)
    
    # 截取数据：从 start_time 开始
    time = data['time']
    start_idx = np.searchsorted(time, start_time)
    
    time = time[start_idx:]
    flange_omega = data['flange_omega'][start_idx:, :]
    flange_alpha = data['flange_alpha'][start_idx:, :]
    cycloid_acc = data['cycloid_acc'][start_idx:, :]
    input_omega = data['input_omega'][start_idx:, :]
    crank_omega_z = data['crank_omega_z'][start_idx:]
    
    dt = (time[-1] - time[0]) / (len(time) - 1)  # 使用平均采样间隔
    fs = 1.0 / dt
    
    print(f"数据信息:")
    print(f"  采样点数: {len(time)}")
    print(f"  时间范围: {time[0]:.4f} ~ {time[-1]:.4f} s")
    print(f"  采样频率: {fs:.1f} Hz")
    print(f"  频率分辨率: {1/(time[-1]-time[0]):.2f} Hz")
    
    # 计算特征频率
    f_input = input_speed_rpm / 60  # 输入轴频率 [Hz]
    f_crank = f_input / 3  # 曲柄轴频率 (输入轴通过3:1传动)
    gear_ratio = 49  # 减速比 (假设)
    f_output = f_input / gear_ratio  # 输出轴频率
    
    print(f"\n特征频率 (输入转速 {input_speed_rpm} rpm):")
    print(f"  输入轴频率: {f_input:.2f} Hz")
    print(f"  曲柄轴频率: {f_crank:.2f} Hz")
    print(f"  输出轴频率: {f_output:.4f} Hz")
    
    # ============ 图1: 时域信号 ============
    fig1, axes1 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    
    # 法兰Z轴角速度
    ax = axes1[0]
    omega_z = flange_omega[:, 2]
    ax.plot(time * 1000, omega_z, 'b-', linewidth=0.5)
    ax.set_ylabel('法兰角速度 Z\n[rad/s]')
    ax.grid(True, alpha=0.3)
    ax.set_title(f'时域振动响应 (采样频率: {fs:.0f} Hz, 起始时间: {start_time}s)')
    
    # 法兰Z轴角加速度
    ax = axes1[1]
    alpha_z = flange_alpha[:, 2]
    ax.plot(time * 1000, alpha_z, 'r-', linewidth=0.5)
    ax.set_ylabel('法兰角加速度 Z\n[rad/s²]')
    ax.grid(True, alpha=0.3)
    
    # 主齿轮XY加速度幅值
    ax = axes1[2]
    acc_xy = np.sqrt(cycloid_acc[:, 0]**2 + cycloid_acc[:, 1]**2)
    ax.plot(time * 1000, acc_xy, 'g-', linewidth=0.5)
    ax.set_ylabel('主齿轮加速度 XY\n[m/s²]')
    ax.set_xlabel('时间 [ms]')
    ax.grid(True, alpha=0.3)
    
    fig1.tight_layout()
    fig1.savefig(os.path.join(output_dir, 'time_domain.png'), dpi=150)
    print(f"\n已保存: time_domain.png")
    
    # ============ 图2: FFT频谱 ============
    fig2, axes2 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    
    # 法兰角速度Z FFT
    ax = axes2[0]
    freq, amp = compute_fft(omega_z, dt)
    ax.semilogy(freq, amp, 'b-', linewidth=0.8)
    ax.axvline(f_crank, color='g', linestyle='-.', alpha=0.5, label=f'曲柄轴1× {f_crank:.1f}Hz')
    ax.axvline(f_crank*3, color='r', linestyle='--', alpha=0.5, label=f'曲柄轴3× {f_crank*3:.1f}Hz')
    ax.axvline(f_crank*6, color='m', linestyle=':', alpha=0.5, label=f'曲柄轴6× {f_crank*6:.1f}Hz')
    ax.set_ylabel('法兰ωz 幅值\n[rad/s]')
    ax.set_xlim([0, min(freq_max, fs/2)])
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    ax.set_title('FFT频谱分析')
    
    # 法兰角加速度Z FFT
    ax = axes2[1]
    freq, amp = compute_fft(alpha_z, dt)
    ax.semilogy(freq, amp, 'r-', linewidth=0.8)
    ax.axvline(f_crank, color='g', linestyle='-.', alpha=0.5)
    ax.axvline(f_crank*3, color='r', linestyle='--', alpha=0.5)
    ax.axvline(f_crank*6, color='m', linestyle=':', alpha=0.5)
    ax.set_ylabel('法兰αz 幅值\n[rad/s²]')
    ax.set_xlim([0, min(freq_max, fs/2)])
    ax.grid(True, alpha=0.3)
    
    # 主齿轮加速度XY FFT
    ax = axes2[2]
    freq, amp = compute_fft(acc_xy, dt)
    ax.semilogy(freq, amp, 'g-', linewidth=0.8)
    ax.axvline(f_crank, color='g', linestyle='-.', alpha=0.5)
    ax.axvline(f_crank*3, color='r', linestyle='--', alpha=0.5)
    ax.axvline(f_crank*6, color='m', linestyle=':', alpha=0.5)
    ax.set_ylabel('主齿轮acc_xy 幅值\n[m/s²]')
    ax.set_xlabel('频率 [Hz]')
    ax.set_xlim([0, min(freq_max, fs/2)])
    ax.grid(True, alpha=0.3)
    
    fig2.tight_layout()
    fig2.savefig(os.path.join(output_dir, 'fft_spectrum.png'), dpi=150)
    print(f"已保存: fft_spectrum.png")
    
    # ============ 图3: 功率谱密度 (PSD) ============
    fig3, axes3 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    
    # PSD窗口：增大以提高频率分辨率
    nperseg = min(4096, len(time) // 4)  # 增大窗口
    nperseg = max(nperseg, 256)  # 最小256点
    
    ax = axes3[0]
    freq, psd = compute_psd(omega_z, fs, nperseg)
    ax.semilogy(freq, psd, 'b-', linewidth=0.8)
    ax.set_ylabel('法兰ωz PSD\n[(rad/s)²/Hz]')
    ax.set_xlim([0, min(freq_max, fs/2)])
    ax.grid(True, alpha=0.3)
    ax.set_title('功率谱密度 (Welch方法)')
    
    ax = axes3[1]
    freq, psd = compute_psd(alpha_z, fs, nperseg)
    ax.semilogy(freq, psd, 'r-', linewidth=0.8)
    ax.set_ylabel('法兰αz PSD\n[(rad/s²)²/Hz]')
    ax.set_xlim([0, min(freq_max, fs/2)])
    ax.grid(True, alpha=0.3)
    
    ax = axes3[2]
    freq, psd = compute_psd(acc_xy, fs, nperseg)
    ax.semilogy(freq, psd, 'g-', linewidth=0.8)
    ax.set_ylabel('主齿轮acc PSD\n[(m/s²)²/Hz]')
    ax.set_xlabel('频率 [Hz]')
    ax.set_xlim([0, min(freq_max, fs/2)])
    ax.grid(True, alpha=0.3)
    
    fig3.tight_layout()
    fig3.savefig(os.path.join(output_dir, 'psd_spectrum.png'), dpi=150)
    print(f"已保存: psd_spectrum.png")
    
    # ============ 图4: 时频谱图 (Spectrogram) ============
    fig4, axes4 = plt.subplots(3, 1, figsize=(14, 12))
    
    # Spectrogram参数 - 尽量用大窗口提高频率分辨率
    spec_nperseg = min(4096, len(time) // 2)
    spec_nperseg = max(spec_nperseg, 64)
    spec_noverlap = spec_nperseg * 3 // 4  # 75%重叠
    
    print(f"  时频分析参数: nperseg={spec_nperseg}, noverlap={spec_noverlap}, 频率分辨率={fs/spec_nperseg:.1f}Hz")
    
    # 法兰角速度Z 时频图
    ax = axes4[0]
    t_spec, f_spec, Sxx = compute_spectrogram(omega_z, fs, nperseg=spec_nperseg, noverlap=spec_noverlap)
    freq_mask = f_spec <= min(freq_max, fs/2)
    im = ax.pcolormesh(t_spec * 1000, f_spec[freq_mask], 
                       10 * np.log10(Sxx[freq_mask, :] + 1e-10),
                       shading='gouraud', cmap='jet')
    ax.axhline(f_input, color='w', linestyle='--', linewidth=2, alpha=0.8, label=f'输入轴 {f_input:.1f}Hz')
    ax.axhline(f_crank, color='lime', linestyle='-.', linewidth=2, alpha=0.8, label=f'曲柄轴 {f_crank:.1f}Hz')
    ax.set_ylabel('频率 [Hz]')
    ax.set_title(f'时频谱图 - 法兰角速度Z (频率分辨率: {fs/spec_nperseg:.1f}Hz)')
    ax.legend(loc='upper right')
    plt.colorbar(im, ax=ax, label='功率 [dB]')
    
    # 法兰角加速度Z 时频图
    ax = axes4[1]
    t_spec, f_spec, Sxx = compute_spectrogram(alpha_z, fs, nperseg=spec_nperseg, noverlap=spec_noverlap)
    im = ax.pcolormesh(t_spec * 1000, f_spec[freq_mask], 
                       10 * np.log10(Sxx[freq_mask, :] + 1e-10),
                       shading='gouraud', cmap='jet')
    ax.axhline(f_input, color='w', linestyle='--', linewidth=2, alpha=0.8)
    ax.axhline(f_crank, color='lime', linestyle='-.', linewidth=2, alpha=0.8)
    ax.set_ylabel('频率 [Hz]')
    ax.set_title('时频谱图 - 法兰角加速度Z')
    plt.colorbar(im, ax=ax, label='功率 [dB]')
    
    # 主齿轮加速度XY 时频图
    ax = axes4[2]
    t_spec, f_spec, Sxx = compute_spectrogram(acc_xy, fs, nperseg=spec_nperseg, noverlap=spec_noverlap)
    im = ax.pcolormesh(t_spec * 1000, f_spec[freq_mask], 
                       10 * np.log10(Sxx[freq_mask, :] + 1e-10),
                       shading='gouraud', cmap='jet')
    ax.axhline(f_input, color='w', linestyle='--', linewidth=2, alpha=0.8)
    ax.axhline(f_crank, color='lime', linestyle='-.', linewidth=2, alpha=0.8)
    ax.set_ylabel('频率 [Hz]')
    ax.set_xlabel('时间 [ms]')
    ax.set_title('时频谱图 - 主齿轮加速度XY')
    plt.colorbar(im, ax=ax, label='功率 [dB]')
    
    fig4.tight_layout()
    fig4.savefig(os.path.join(output_dir, 'spectrogram.png'), dpi=150)
    print(f"已保存: spectrogram.png")
    
    # ============ 保存频谱数据到CSV ============
    freq_fft, amp_omega = compute_fft(omega_z, dt)
    _, amp_alpha = compute_fft(alpha_z, dt)
    _, amp_acc = compute_fft(acc_xy, dt)
    
    spectrum_data = np.column_stack([freq_fft, amp_omega, amp_alpha, amp_acc])
    spectrum_csv = os.path.join(output_dir, 'spectrum_data.csv')
    header = 'frequency[Hz],flange_omega_z[rad/s],flange_alpha_z[rad/s2],cycloid_acc_xy[m/s2]'
    np.savetxt(spectrum_csv, spectrum_data, delimiter=',', header=header, comments='')
    print(f"已保存: spectrum_data.csv")
    
    # ============ 统计信息 ============
    print(f"\n振动统计:")
    print(f"  法兰角速度Z: mean={np.mean(omega_z):.4f}, std={np.std(omega_z):.4f} rad/s")
    print(f"  法兰角加速度Z: mean={np.mean(alpha_z):.2f}, std={np.std(alpha_z):.2f} rad/s²")
    print(f"  主齿轮加速度XY: mean={np.mean(acc_xy):.2f}, std={np.std(acc_xy):.2f} m/s²")
    
    # 找主频
    freq_fft, amp_omega = compute_fft(omega_z, dt)
    peak_idx = np.argmax(amp_omega[1:]) + 1  # 排除直流
    print(f"\n主频分析:")
    print(f"  法兰角速度主频: {freq_fft[peak_idx]:.2f} Hz, 幅值: {amp_omega[peak_idx]:.4f} rad/s")
    
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='振动响应频谱分析')
    parser.add_argument('--csv', type=str, 
                        default=os.path.join(os.path.dirname(__file__), 'solution', 'vibration_response.csv'),
                        help='振动响应CSV文件路径')
    parser.add_argument('--output', type=str,
                        default=os.path.join(os.path.dirname(__file__), 'solution', 'spectrum'),
                        help='输出目录')
    parser.add_argument('--rpm', type=float, default=1000,
                        help='输入轴转速 (rpm)')
    args = parser.parse_args()
    
    print("=" * 60)
    print("振动响应频谱分析")
    print("=" * 60)
    
    if not os.path.exists(args.csv):
        print(f"错误: 找不到CSV文件: {args.csv}")
        print("请先运行 fewTeethDyn.py 生成振动数据")
        return
    
    print(f"读取: {args.csv}")
    data = load_vibration_data(args.csv)
    
    analyze_and_plot(data, args.output, args.rpm)
    
    print("\n" + "=" * 60)
    print("分析完成!")
    print("=" * 60)


if __name__ == "__main__":
    main()
