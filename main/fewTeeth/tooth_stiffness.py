"""
单齿刚度计算模块 (Tooth Stiffness Calculator)

严格按照 mesh_stiff250915_3.m 中的算法实现，包括：
- 齿体弯曲变形 (Bending deformation) - 基于梁理论
- 齿根变形 (Fillet-foundation deformation) - 基于 Ishikawa 公式
- 接触变形 (Hertz contact deformation) - 基于 Hertz 理论

单位制：SI (m, N, Pa) - 与 MATLAB 的 N-mm 单位制公式一致，仅单位转换

参考文献：
- Weber-Banaschek 方法
- Ishikawa 齿根变形公式
- Hertz 接触理论

用法:
    from tooth_stiffness import ToothStiffnessCalculator, GearParameters
    
    gear = GearParameters(z=60, module=0.002, alpha_deg=20)
    calc = ToothStiffnessCalculator(gear, face_width=0.01, E=2.1e11, poisson=0.3)
    stiffness_table = calc.compute_stiffness_table(n_points=5)
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Tuple, List, Optional
import math


@dataclass
class GearParameters:
    """齿轮几何参数 - 与 MATLAB 中的定义对应"""
    z: int                      # 齿数
    module: float               # 模数 [m]
    alpha_deg: float = 20.0     # 压力角 [度]
    ha_star: float = 1.0        # 齿顶高系数 (ha)
    c_star: float = 0.25        # 顶隙系数 (c)
    x: float = 0.0              # 变位系数
    is_internal: bool = False   # 是否为内齿轮
    
    # 派生参数 (自动计算)
    alpha: float = field(init=False)      # 压力角 [rad]
    inv_alpha: float = field(init=False)  # 渐开线函数值 inv(alpha)
    r: float = field(init=False)          # 分度圆半径
    rb: float = field(init=False)         # 基圆半径
    ra: float = field(init=False)         # 齿顶圆半径
    rf: float = field(init=False)         # 齿根圆半径
    s: float = field(init=False)          # 分度圆齿厚
    
    def __post_init__(self):
        """计算派生参数"""
        self.alpha = self.alpha_deg * math.pi / 180.0
        self.inv_alpha = math.tan(self.alpha) - self.alpha
        
        m = self.module
        z = self.z
        ha = self.ha_star
        c = self.c_star
        x = self.x
        
        # 分度圆 d = m*z, r = d/2
        self.r = 0.5 * m * z
        
        # 基圆 db = d*cos(alpha), rb = db/2
        self.rb = self.r * math.cos(self.alpha)
        
        if self.is_internal:
            # 内齿轮：齿顶向内
            self.ra = 0.5 * m * (z - 2 * ha + 2 * x)
            # df = m*(z + 2*ha + 2*c + 2*x)
            self.rf = 0.5 * m * (z + 2 * ha + 2 * c - 2 * x)
            # 分度圆齿厚
            self.s = m * (math.pi / 2 - 2 * x * math.tan(self.alpha))
        else:
            # 外齿轮
            self.ra = 0.5 * m * (z + 2 * ha + 2 * x)
            # df = m*(z - 2*ha - 2*c + 2*x)
            self.rf = 0.5 * m * (z - 2 * ha - 2 * c + 2 * x)
            # 分度圆齿厚 s = m*(pi/2 + 2*x*tan(alpha))
            self.s = m * (math.pi / 2 + 2 * x * math.tan(self.alpha))


@dataclass
class DeformationResult:
    """变形计算结果"""
    delta_bending: float    # 齿体弯曲变形 deltaB
    delta_fillet: float     # 齿根变形 deltaM
    delta_contact: float    # 接触变形 deltaC
    delta_total: float      # 总变形
    stiffness: float        # 总刚度 k = F / delta_total
    stiffness_structure: float = 0.0  # 结构刚度（不含接触）k = F / (δB + δM)
    stiffness_contact: float = 0.0    # 接触刚度 k = F / δC


class ToothStiffnessCalculator:
    """
    单齿刚度计算器
    """
    
    def __init__(self, gear: GearParameters, face_width: float, 
                 E: float = 2.1e11, poisson: float = 0.3,
                 n_segments: int = 50):
        """
        初始化计算器
        
        Args:
            gear: 齿轮参数
            face_width: 齿宽 B [m]
            E: 弹性模量 [Pa]
            poisson: 泊松比 miu
            n_segments: 齿体弯曲计算分段数 n (MATLAB 默认 50)
        """
        self.gear = gear
        self.B = face_width
        self.E = E
        self.miu = poisson
        self.n = n_segments
        
        # 齿根危险截面 x 坐标: xM = rf
        self.xM = gear.rf
        
        # 齿根处半齿厚 yM = rf*tan(pi/z)
        self.yM = gear.rf * math.tan(math.pi / gear.z)
        
        # 齿根齿厚 Hf = 2*yM
        self.Hf = 2 * self.yM
        
        # 齿高 Hp (用于判断平面应力/应变)
        if gear.is_internal:
            self.Hp = gear.rf - gear.ra  # 内齿轮
        else:
            self.Hp = gear.ra - gear.rf  # 外齿轮
    
    def _involute(self, alpha: float) -> float:
        """渐开线函数 inv(alpha) = tan(alpha) - alpha"""
        return math.tan(alpha) - alpha
    
    def _compute_geometry_at_contact_point(self, rp: float) -> dict:
        """
        计算接触点处的几何参数
        
        Args:
            rp: 接触点半径
            
        Returns:
            几何参数字典
        """
        gear = self.gear
        
        # 压力角 alphap = acos(rb/rp)
        if rp <= gear.rb:
            alphap = 0.0
            inv_alphap = 0.0
        else:
            alphap = math.acos(gear.rb / rp)
            inv_alphap = self._involute(alphap)
        
        # 齿厚角 faip = s/r - 2*(inv_alphap - inv_alpha)
        faip = gear.s / gear.r - 2 * (inv_alphap - gear.inv_alpha)
        
        # 接触点坐标
        xp = rp * math.cos(faip / 2)
        # yp = 0.5*(s*rp/r - 2*rp*(inv_alphap - inv_alpha))
        yp = 0.5 * (gear.s * rp / gear.r - 2 * rp * (inv_alphap - gear.inv_alpha))
        
        # 力作用方向角 beta = alphap - faip/2
        beta = alphap - faip / 2
        
        # 力臂 Lf = xp - xM - yp*tan(beta)
        Lf = xp - self.xM - yp * math.tan(beta) if xp > self.xM else 0.0
        
        # 曲率半径 rou = rb*tan(alphap)
        rou = gear.rb * math.tan(alphap) if alphap > 0 else 0.0
        
        return {
            'rp': rp,
            'alphap': alphap,
            'inv_alphap': inv_alphap,
            'faip': faip,
            'xp': xp,
            'yp': yp,
            'beta': beta,
            'Lf': Lf,
            'rou': rou,
        }
    
    def _compute_half_tooth_thickness_at_radius(self, rK: float) -> float:
        """
        计算给定半径处的半齿厚 yk
        
        Args:
            rK: 计算点半径
            
        Returns:
            半齿厚 yk
        """
        gear = self.gear
        
        # 渐开线齿厚公式：s_r = s * r/r_d - 2*r*(inv(α_r) - inv(α))
        # 这个公式对外齿轮和内齿轮都适用，因为 s 已经根据齿轮类型正确定义
        
        if rK <= gear.rb:
            # 基圆以下，使用基圆处齿厚
            alphaK = 0.0
            invalphaK = 0.0
            # 基圆处齿厚
            sb = gear.s * gear.rb / gear.r - 2 * gear.rb * (0 - gear.inv_alpha)
            yk = 0.5 * abs(sb)  # 取绝对值确保正值
        else:
            alphaK = math.acos(gear.rb / rK)
            invalphaK = self._involute(alphaK)
            # 渐开线齿厚公式
            s_at_rK = gear.s * rK / gear.r - 2 * rK * (invalphaK - gear.inv_alpha)
            yk = 0.5 * abs(s_at_rK)  # 取绝对值确保正值
        
        return max(yk, 1e-12)
    
    def compute_bending_deformation(self, rp: float, xp: float, beta: float, F: float) -> float:
        """
        计算齿体弯曲变形 deltaB
        
        基于梁理论，包括：弯曲应力、剪切变形、压缩变形
        
        Args:
            rp: 接触点半径
            xp: 接触点 x 坐标
            beta: 力作用方向角
            F: 法向载荷
            
        Returns:
            弯曲变形量
        """
        gear = self.gear
        E = self.E
        B = self.B
        miu = self.miu
        n = self.n
        
        deltaB = 0.0
        
        # 积分范围：从齿根到接触点
        # 外齿轮：rf < rp，积分从小半径到大半径
        # 内齿轮：rf > rp，积分从大半径到小半径
        r_start = gear.rf
        r_end = rp
        
        # 积分循环
        for k in range(1, n + 1):
            # 积分点半径
            rK = r_start + (r_end - r_start) * (k - 0.5) / n
            # 积分步长（取绝对值）
            Tk = abs(r_end - r_start) / n
            
            # 半齿厚 yk
            yk = self._compute_half_tooth_thickness_at_radius(rK)
            xk = rK
            
            # 截面特性
            Ak = 2 * yk * B          # 截面积
            Ik = B * (2 * yk)**3 / 12  # 惯性矩
            Lk = abs(xp - xk)         # 力臂（取绝对值）
            
            # 验证几何合理性
            if Ak <= 0 or Ik <= 0 or yk <= 0 or Tk <= 0:
                continue
            
            cos_beta = math.cos(beta)
            sin_beta = math.sin(beta)
            
            # 弯曲变形
            deltaB_bending = (F / E) * cos_beta**2 * Tk * \
                            (Tk**2 + 3 * Tk * Lk + 3 * Lk**2) / (3 * Ik)
            
            # 剪切变形
            deltaB_shear = (F / E) * 12 * (1 + miu) * cos_beta**2 * Tk / (5 * Ak)
            
            # 压缩变形
            deltaB_compression = (F / E) * sin_beta**2 * Tk / Ak
            
            deltaB += deltaB_bending + deltaB_shear + deltaB_compression
        
        return max(deltaB, 0.0)
    
    def compute_fillet_deformation(self, Lf: float, beta: float, F: float) -> float:
        """
        计算齿根变形 deltaM
        
        基于 Ishikawa 公式，系数：5.306, 2*(1-miu), 1.534, 0.4167
        
        Args:
            Lf: 力臂 (从齿根危险截面到接触点)
            beta: 力作用方向角
            F: 法向载荷
            
        Returns:
            齿根变形量
        """
        gear = self.gear
        E = self.E
        B = self.B
        miu = self.miu
        mm = gear.module
        
        # 临界齿根半径 rf_critical = rf + 0.5*mm
        rf_critical = gear.rf + 0.5 * mm
        
        # 计算实际齿根厚度 Hf_actual
        if rf_critical > gear.rb:
            alpha_critical = math.acos(gear.rb / rf_critical)
            invalpha_critical = self._involute(alpha_critical)
            sf_critical = gear.s * rf_critical / gear.r - \
                         2 * rf_critical * (invalpha_critical - gear.inv_alpha)
        else:
            sf_critical = gear.s * gear.rb / gear.r - 2 * gear.rb * (0 - gear.inv_alpha)
        
        Hf_actual = sf_critical
        
        # 验证齿根厚度合理性
        if Hf_actual <= 0:
            Hf_actual = self.Hf
        
        if Hf_actual <= 1e-12 or Lf <= 0:
            return 0.0
        
        cos_beta = math.cos(beta)
        tan_beta = math.tan(beta)
        
        # 根据齿宽比选择计算方法
        if B / self.Hp < 5:
            # 窄齿轮：平面应力状态
            deltaM = (F * cos_beta**2) / (B * E) * (
                5.306 * (Lf / Hf_actual)**2 +
                2 * (1 - miu) * (Lf / Hf_actual) +
                1.534 * (1 + 0.4167 * tan_beta**2 / (1 + miu))
            )
        else:
            # 宽齿轮：平面应变状态
            deltaM = (F * cos_beta**2) / (B * E) * (1 - miu**2) * (
                5.306 * (Lf / Hf_actual)**2 +
                2 * ((1 - miu - 2 * miu**2) / (1 - miu**2)) * (Lf / Hf_actual) +
                1.534 * (1 + 0.4167 * tan_beta**2 / (1 + miu))
            )
        
        return max(deltaM, 0.0)
    
    def compute_contact_deformation(self, rp: float, F: float, 
                                       rho_mate: float = None,
                                       is_internal_mesh: bool = None) -> float:
        """
        计算接触变形 deltaC - 基于 Hertz 理论
        
        对于内-外齿問合（少齿差传动），使用等效曲率半径：
        - 外-外問合（凸-凸）: ρ_eq = ρ₁·ρ₂ / (ρ₁ + ρ₂)
        - 内-外問合（凸-凹）: ρ_eq = ρ₁·ρ₂ / |ρ₂ - ρ₁|
        
        Args:
            rp: 接触点半径
            F: 法向载荷
            rho_mate: 配对齿轮接触点曲率半径 (可选)
            is_internal_mesh: 是否为内-外問合 (默认根据 gear.is_internal 判断)
            
        Returns:
            接触变形量
        """
        gear = self.gear
        E = self.E
        B = self.B
        miu = self.miu
        
        # 本齿轮曲率半径 rou = sqrt(rp^2 - rb^2) * (1 + x * 0.02)
        if rp > gear.rb:
            rho_self = math.sqrt(rp**2 - gear.rb**2) * (1 + gear.x * 0.02)
        else:
            rho_self = 1e-12
        
        if rho_self <= 1e-12:
            return 0.0
        
        # 确定是否为内-外問合
        if is_internal_mesh is None:
            is_internal_mesh = gear.is_internal
        
        # 计算等效曲率半径
        if rho_mate is not None and rho_mate > 1e-12:
            if is_internal_mesh:
                # 内-外問合（凸-凹接触）: ρ_eq = ρ₁·ρ₂ / |ρ₂ - ρ₁|
                diff = abs(rho_mate - rho_self)
                if diff > 1e-12:
                    rho_eq = rho_self * rho_mate / diff
                else:
                    # 曲率半径相等，等效曲率半径趋近无穷大（平面接触）
                    rho_eq = rho_self * 100  # 使用较大值
            else:
                # 外-外問合（凸-凸接触）: ρ_eq = ρ₁·ρ₂ / (ρ₁ + ρ₂)
                rho_eq = rho_self * rho_mate / (rho_self + rho_mate)
        else:
            # 没有配对曲率半径，使用本齿轮曲率半径
            rho_eq = rho_self
        
        # 线载荷 p = F/B
        p = F / B
        
        if p <= 0 or rho_eq <= 1e-12:
            return 0.0
        
        # Hertz 接触变形公式
        # deltaC = (2*p/π) * ((1-μ²)/E) * (ln(2ρ/b) + 0.407)
        # 其中 b = sqrt(4*p*ρ/(π*E/(1-μ²)))
        Ee = E / (1 - miu**2)  # 等效弹性模量
        
        # 半接触宽度 b
        b_sq = 4 * p * rho_eq / (math.pi * Ee)
        if b_sq <= 0:
            return 0.0
        b = math.sqrt(b_sq)
        
        # log 参数
        log_arg = 2 * rho_eq / b
        if log_arg <= 1:
            log_arg = 1.001  # 避免负对数
        
        deltaC = (2 * p / math.pi) * ((1 - miu**2) / E) * (math.log(log_arg) + 0.407)
        
        return max(deltaC, 0.0)
    
    def compute_tooth_stiffness(self, rp: float, F: float = 1.0,
                                  rho_mate: float = None,
                                  is_internal_mesh: bool = None) -> DeformationResult:
        """
        计算给定接触半径处的单齿刚度
        
        Args:
            rp: 接触点半径
            F: 法向载荷 (默认单位载荷)
            rho_mate: 配对齿轮接触点曲率半径 (用于 Hertz 接触变形)
            is_internal_mesh: 是否为内-外問合
            
        Returns:
            DeformationResult
        """
        gear = self.gear
        
        # 确保半径在有效范围内
        if not gear.is_internal:
            rp = max(min(rp, gear.ra), max(gear.rf, gear.rb + 1e-9))
        else:
            rp = min(max(rp, gear.ra), min(gear.rf, gear.rb - 1e-9) if gear.rb > gear.ra else gear.rf)
        
        # 计算几何参数
        geom = self._compute_geometry_at_contact_point(rp)
        
        xp = geom['xp']
        beta = geom['beta']
        Lf = geom['Lf']
        
        # 计算三个变形分量
        deltaB = self.compute_bending_deformation(rp, xp, beta, F)
        deltaM = self.compute_fillet_deformation(Lf, beta, F)
        deltaC = self.compute_contact_deformation(rp, F, rho_mate, is_internal_mesh)
        
        # 总变形 delta = deltaB + deltaM + deltaC
        delta_total = deltaB + deltaM + deltaC
        
        # 刚度 k = F / delta
        if delta_total > 1e-20:
            stiffness = F / delta_total
        else:
            stiffness = 1e20
        
        # 结构刚度（不含接触）
        # 注意：在齿根附近，力臂很小，变形趋近于零，刚度趋近于无穷大
        # 设置合理的最大刚度上限（约10倍于典型单齿刚度）
        MAX_STIFFNESS = 1e10  # 10 GN/m
        delta_structure = deltaB + deltaM
        if delta_structure > 1e-20:
            stiffness_structure = min(F / delta_structure, MAX_STIFFNESS)
        else:
            stiffness_structure = MAX_STIFFNESS
        
        # 接触刚度
        if deltaC > 1e-20:
            stiffness_contact = F / deltaC
        else:
            stiffness_contact = 1e20
        
        return DeformationResult(
            delta_bending=deltaB,
            delta_fillet=deltaM,
            delta_contact=deltaC,
            delta_total=delta_total,
            stiffness=stiffness,
            stiffness_structure=stiffness_structure,
            stiffness_contact=stiffness_contact
        )
    
    def compute_stiffness_table(self, n_points: int = 5, F: float = 1.0,
                                include_contact: bool = False) -> List[float]:
        """
        计算沿齿廓均匀分布的刚度表 (从齿根到齿顶)
        
        与 C++ InterpolateStiffness() 的对应关系：
            Python: stiffness_table[i] 对应 t = i/(n_points-1)
            C++:    InterpolateStiffness(t, table) 返回线性插值结果
            
            t = 0.0 → 索引 0   → 齿根 (r_root)
            t = 1.0 → 索引 n-1 → 齿顶 (r_tip)
            
            C++ 中的 flip 逻辑会根据 isInner 和 reverseParam 自动转换
            渐开线参数到刚度表参数，确保一一对应。
        
        Args:
            n_points: 采样点数
            F: 法向载荷
            include_contact: 是否包含接触刚度 (默认 False，只返回结构刚度)
            
        Returns:
            刚度列表 [k_root, ..., k_tip]
            - include_contact=False: 结构刚度（弯曲+齿根）
            - include_contact=True: 总刚度（弯曲+齿根+接触）
        """
        gear = self.gear
        
        # 确定半径范围：从齿根到齿顶
        if gear.is_internal:
            r_root = gear.rf
            r_tip = max(gear.ra, gear.rb + 1e-9)
        else:
            r_root = max(gear.rf, gear.rb + 1e-9)
            r_tip = gear.ra
        
        stiffness_table = []
        
        for i in range(n_points):
            t = i / (n_points - 1) if n_points > 1 else 0.5
            rp = r_root + t * (r_tip - r_root)
            
            result = self.compute_tooth_stiffness(rp, F)
            # 根据参数选择返回结构刚度或总刚度
            if include_contact:
                stiffness_table.append(result.stiffness)
            else:
                stiffness_table.append(result.stiffness_structure)
        
        return stiffness_table
    
    def compute_stiffness_curve(self, n_points: int = 50, F: float = 1.0) -> Tuple[np.ndarray, np.ndarray]:
        """
        计算详细的刚度曲线
        
        Args:
            n_points: 采样点数
            F: 法向载荷
            
        Returns:
            (radii, stiffnesses) 两个数组
        """
        gear = self.gear
        
        if gear.is_internal:
            r_root = gear.rf
            r_tip = max(gear.ra, gear.rb + 1e-9)
        else:
            r_root = max(gear.rf, gear.rb + 1e-9)
            r_tip = gear.ra
        
        radii = np.linspace(r_root, r_tip, n_points)
        stiffnesses = []
        
        for rp in radii:
            result = self.compute_tooth_stiffness(rp, F)
            stiffnesses.append(result.stiffness)
        
        return radii, np.array(stiffnesses)
    
    def compute_deformation_breakdown(self, rp: float, F: float = 1.0) -> dict:
        """
        计算并返回详细的变形分解
        
        Args:
            rp: 接触点半径
            F: 法向载荷
            
        Returns:
            包含各变形分量及占比的字典
        """
        result = self.compute_tooth_stiffness(rp, F)
        
        total = result.delta_total
        if total > 0:
            return {
                'delta_bending': result.delta_bending,
                'delta_fillet': result.delta_fillet,
                'delta_contact': result.delta_contact,
                'delta_total': total,
                'stiffness': result.stiffness,
                'ratio_bending': result.delta_bending / total * 100,
                'ratio_fillet': result.delta_fillet / total * 100,
                'ratio_contact': result.delta_contact / total * 100,
            }
        return {
            'delta_bending': 0, 'delta_fillet': 0, 'delta_contact': 0,
            'delta_total': 0, 'stiffness': 1e20,
            'ratio_bending': 0, 'ratio_fillet': 0, 'ratio_contact': 0,
        }


def compute_mesh_stiffness(k_outer: float, k_inner: float) -> float:
    """
    计算齿对啮合刚度 (串联)
    k = (k1*k2)/(k1+k2)
    
    Args:
        k_outer: 外齿轮单齿刚度
        k_inner: 内齿轮单齿刚度
        
    Returns:
        啮合刚度
    """
    if k_outer <= 0 or k_inner <= 0:
        return 0.0
    return (k_outer * k_inner) / (k_outer + k_inner)


# =============================================================================
# 便捷函数：为 fewTeeth.py 提供的接口
# =============================================================================

def create_stiffness_table_for_gear(
    z: int,
    module: float,
    face_width: float,
    alpha_deg: float = 20.0,
    ha_star: float = 1.0,
    c_star: float = 0.25,
    x: float = 0.0,
    is_internal: bool = False,
    E: float = 2.1e11,
    poisson: float = 0.3,
    n_points: int = 5,
    F: float = 1.0,
    include_contact: bool = False
) -> List[float]:
    """
    便捷函数：创建单齿刚度表
    
    Args:
        z: 齿数
        module: 模数 [m]
        face_width: 齿宽 [m]
        alpha_deg: 压力角 [度]
        ha_star: 齿顶高系数
        c_star: 顶隙系数
        x: 变位系数
        is_internal: 是否为内齿轮
        E: 弹性模量 [Pa]
        poisson: 泊松比
        n_points: 刚度表采样点数
        F: 计算用载荷 [N]
        include_contact: 是否包含接触刚度 (默认 False，只返回结构刚度)
        
    Returns:
        刚度列表 [N/m]，从齿根到齿顶
        - include_contact=False: 结构刚度（弯曲+齿根，不含接触）
        - include_contact=True: 总刚度（弯曲+齿根+接触）
    """
    gear = GearParameters(
        z=z,
        module=module,
        alpha_deg=alpha_deg,
        ha_star=ha_star,
        c_star=c_star,
        x=x,
        is_internal=is_internal
    )
    
    calc = ToothStiffnessCalculator(gear, face_width, E, poisson)
    return calc.compute_stiffness_table(n_points, F, include_contact)


def create_mesh_stiffness_table_internal_external(
    z_outer: int, z_inner: int,
    module: float, face_width: float,
    alpha_deg: float = 20.0,
    ha_star: float = 1.0, c_star: float = 0.25,
    x_outer: float = 0.0, x_inner: float = 0.0,
    E: float = 2.1e11, poisson: float = 0.3,
    n_points: int = 5, F: float = 1.0
) -> Tuple[List[float], List[float], List[float], List[float]]:
    """
    便捷函数：计算内-外齿問合的刚度表（少齿差传动）
    
    正确处理接触刚度：
    - 结构刚度（弯曲+齿根）：各齿轮单独计算
    - 接触刚度：使用等效曲率半径，只计算一次
    - 問合刚度 = 1 / (1/k_结构_外 + 1/k_结构_内 + 1/k_接触)
    
    Args:
        z_outer: 外齿轮齿数
        z_inner: 内齿轮齿数
        module: 模数 [m]
        face_width: 齿宽 [m]
        alpha_deg: 压力角 [度]
        ha_star: 齿顶高系数
        c_star: 顶隙系数
        x_outer: 外齿轮变位系数
        x_inner: 内齿轮变位系数
        E: 弹性模量 [Pa]
        poisson: 泊松比
        n_points: 刚度表采样点数
        F: 计算用载荷 [N]
        
    Returns:
        (stiffness_outer, stiffness_inner, stiffness_contact, stiffness_mesh) 四个刚度表
        - stiffness_outer: 外齿轮结构刚度（不含接触）
        - stiffness_inner: 内齿轮结构刚度（不含接触）
        - stiffness_contact: 接触刚度（使用等效曲率半径）
        - stiffness_mesh: 总問合刚度
    """
    # 创建齿轮参数
    gear_outer = GearParameters(
        z=z_outer, module=module, alpha_deg=alpha_deg,
        ha_star=ha_star, c_star=c_star, x=x_outer, is_internal=False
    )
    gear_inner = GearParameters(
        z=z_inner, module=module, alpha_deg=alpha_deg,
        ha_star=ha_star, c_star=c_star, x=x_inner, is_internal=True
    )
    
    calc_outer = ToothStiffnessCalculator(gear_outer, face_width, E, poisson)
    calc_inner = ToothStiffnessCalculator(gear_inner, face_width, E, poisson)
    
    # 计算半径范围
    r_root_outer = max(gear_outer.rf, gear_outer.rb + 1e-9)
    r_tip_outer = gear_outer.ra
    r_root_inner = gear_inner.rf
    r_tip_inner = max(gear_inner.ra, gear_inner.rb + 1e-9)
    
    stiffness_outer = []
    stiffness_inner = []
    stiffness_contact = []
    stiffness_mesh = []
    
    for i in range(n_points):
        t = i / (n_points - 1) if n_points > 1 else 0.5
        
        # 外齿轮接触点半径
        rp_outer = r_root_outer + t * (r_tip_outer - r_root_outer)
        # 内齿轮接触点半径 (从齿根到齿顶)
        rp_inner = r_root_inner + t * (r_tip_inner - r_root_inner)
        
        # 计算曲率半径
        if rp_outer > gear_outer.rb:
            rho_outer = math.sqrt(rp_outer**2 - gear_outer.rb**2)
        else:
            rho_outer = 1e-12
        if rp_inner > gear_inner.rb:
            rho_inner = math.sqrt(rp_inner**2 - gear_inner.rb**2)
        else:
            rho_inner = 1e-12
        
        # 计算单齿结构刚度（不含接触）
        result_outer = calc_outer.compute_tooth_stiffness(rp_outer, F)
        result_inner = calc_inner.compute_tooth_stiffness(rp_inner, F)
        
        # 结构刚度（弯曲+齿根）
        stiffness_outer.append(result_outer.stiffness_structure)
        stiffness_inner.append(result_inner.stiffness_structure)
        
        # 计算等效曲率半径（内-外問合：凸-凹接触）
        diff = abs(rho_inner - rho_outer)
        if diff > 1e-12 and rho_outer > 1e-12 and rho_inner > 1e-12:
            rho_eq = rho_outer * rho_inner / diff
        else:
            rho_eq = max(rho_outer, rho_inner) * 10  # 近似平面接触
        
        # 计算接触刚度（只算一次，使用等效曲率半径）
        # Hertz 接触刚度: k_c = F / δ_c
        p = F / face_width
        miu = poisson
        Ee = E / (1 - miu**2)
        if p > 0 and rho_eq > 1e-12:
            b_sq = 4 * p * rho_eq / (math.pi * Ee)
            if b_sq > 0:
                b = math.sqrt(b_sq)
                log_arg = 2 * rho_eq / b
                if log_arg > 1:
                    delta_c = (2 * p / math.pi) * ((1 - miu**2) / E) * (math.log(log_arg) + 0.407)
                    k_contact = F / delta_c if delta_c > 1e-20 else 1e20
                else:
                    k_contact = 1e20
            else:
                k_contact = 1e20
        else:
            k_contact = 1e20
        
        stiffness_contact.append(k_contact)
        
        # 問合刚度 = 1 / (1/k_结构_外 + 1/k_结构_内 + 1/k_接触)
        k_s_outer = result_outer.stiffness_structure
        k_s_inner = result_inner.stiffness_structure
        if k_s_outer > 0 and k_s_inner > 0 and k_contact > 0:
            k_mesh = 1.0 / (1.0/k_s_outer + 1.0/k_s_inner + 1.0/k_contact)
        else:
            k_mesh = 0.0
        stiffness_mesh.append(k_mesh)
    
    return stiffness_outer, stiffness_inner, stiffness_contact, stiffness_mesh


# =============================================================================
# 测试代码
# =============================================================================

if __name__ == "__main__":
    print("=" * 70)
    print("单齿刚度计算测试")
    print("=" * 70)
    
    # 测试参数 (SI 单位)
    print("\n--- 测试1: 标准齿轮参数 (z=22, m=8mm, B=5mm) ---")
    
    gear_test = GearParameters(
        z=22,
        module=0.008,  # 8mm -> 0.008m
        alpha_deg=20.0,
        ha_star=1.0,
        c_star=0.25,
        x=0.0,
        is_internal=False
    )
    
    face_width = 0.005  # 5mm -> 0.005m
    E = 2.1e11  # 210000 MPa = 2.1e11 Pa
    
    calc = ToothStiffnessCalculator(gear_test, face_width, E, 0.3)
    
    print(f"\n齿轮参数:")
    print(f"  齿数 z: {gear_test.z}")
    print(f"  模数 m: {gear_test.module * 1000:.2f} mm")
    print(f"  压力角 alpha: {gear_test.alpha_deg}°")
    print(f"  分度圆半径 r: {gear_test.r * 1000:.2f} mm")
    print(f"  基圆半径 rb: {gear_test.rb * 1000:.2f} mm")
    print(f"  齿顶圆半径 ra: {gear_test.ra * 1000:.2f} mm")
    print(f"  齿根圆半径 rf: {gear_test.rf * 1000:.2f} mm")
    print(f"  分度圆齿厚 s: {gear_test.s * 1000:.4f} mm")
    
    # 在分度圆处计算详细变形
    rp_test = gear_test.r
    breakdown = calc.compute_deformation_breakdown(rp_test, F=1000)  # 1000N 载荷
    
    print(f"\n在分度圆处 (r={rp_test*1000:.2f}mm) 的变形分解 (F=1000N):")
    print(f"  弯曲变形 deltaB: {breakdown['delta_bending']*1e6:.4f} μm ({breakdown['ratio_bending']:.1f}%)")
    print(f"  齿根变形 deltaM: {breakdown['delta_fillet']*1e6:.4f} μm ({breakdown['ratio_fillet']:.1f}%)")
    print(f"  接触变形 deltaC: {breakdown['delta_contact']*1e6:.4f} μm ({breakdown['ratio_contact']:.1f}%)")
    print(f"  总变形 delta: {breakdown['delta_total']*1e6:.4f} μm")
    print(f"  单齿刚度 k: {breakdown['stiffness']/1e6:.2f} MN/m = {breakdown['stiffness']/1e9:.4f} GN/m")
    
    # 计算刚度表
    stiffness_table = calc.compute_stiffness_table(n_points=5, F=1000)
    print(f"\n刚度表 (5点, 根→顶):")
    for i, k in enumerate(stiffness_table):
        print(f"  [{i}] {k/1e6:.2f} MN/m")
    
    # =============================================================================
    print("\n" + "=" * 70)
    print("--- 测试2: 摆线针轮参数 (z_out=60, z_in=62, m=2mm) ---")
    print("=" * 70)
    
    # 外齿轮 (摆线轮)
    gear_outer = GearParameters(
        z=60,
        module=0.002,  # 2mm
        alpha_deg=20.0,
        ha_star=1.0,
        c_star=0.25,
        x=0.0,
        is_internal=False
    )
    
    # 内齿轮 (针齿壳体)
    gear_inner = GearParameters(
        z=62,
        module=0.002,
        alpha_deg=20.0,
        ha_star=1.0,
        c_star=0.25,
        x=0.0,
        is_internal=True
    )
    
    face_width = 0.010  # 10mm
    
    calc_outer = ToothStiffnessCalculator(gear_outer, face_width, E, 0.3)
    calc_inner = ToothStiffnessCalculator(gear_inner, face_width, E, 0.3)
    
    print(f"\n外齿轮 (摆线轮):")
    print(f"  z={gear_outer.z}, r={gear_outer.r*1000:.2f}mm, rb={gear_outer.rb*1000:.2f}mm")
    print(f"  ra={gear_outer.ra*1000:.2f}mm, rf={gear_outer.rf*1000:.2f}mm")
    
    print(f"\n内齿轮 (针齿壳体):")
    print(f"  z={gear_inner.z}, r={gear_inner.r*1000:.2f}mm, rb={gear_inner.rb*1000:.2f}mm")
    print(f"  ra={gear_inner.ra*1000:.2f}mm, rf={gear_inner.rf*1000:.2f}mm")
    
    stiffness_outer = calc_outer.compute_stiffness_table(5)
    stiffness_inner = calc_inner.compute_stiffness_table(5)
    
    print(f"\n外齿轮刚度表 (根→顶):")
    for i, k in enumerate(stiffness_outer):
        print(f"  [{i}] {k/1e6:.2f} MN/m = {k:.3e} N/m")
    
    print(f"\n内齿轮刚度表 (根→顶):")
    for i, k in enumerate(stiffness_inner):
        print(f"  [{i}] {k/1e6:.2f} MN/m = {k:.3e} N/m")
    
    # 计算中点啮合刚度
    k_mesh = compute_mesh_stiffness(stiffness_outer[2], stiffness_inner[2])
    print(f"\n中点啮合刚度 (串联): {k_mesh/1e6:.2f} MN/m = {k_mesh:.3e} N/m")

    # =============================================================================
    print("\n" + "=" * 70)
    print("--- 测试3: 内-外問合刚度表（正确分离结构刚度和接触刚度） ---")
    print("=" * 70)
    
    k_outer, k_inner, k_contact, k_mesh_table = create_mesh_stiffness_table_internal_external(
        z_outer=60, z_inner=62,
        module=0.002, face_width=0.010,
        n_points=5
    )
    
    print(f"\n外齿轮结构刚度表 (弯曲+齿根, 不含接触):")
    for i, k in enumerate(k_outer):
        print(f"  [{i}] {k/1e6:.2f} MN/m")
    
    print(f"\n内齿轮结构刚度表 (弯曲+齿根, 不含接触):")
    for i, k in enumerate(k_inner):
        print(f"  [{i}] {k/1e6:.2f} MN/m")
    
    print(f"\n接触刚度表 (使用等效曲率半径):")
    for i, k in enumerate(k_contact):
        print(f"  [{i}] {k/1e6:.2f} MN/m")
    
    print(f"\n問合刚度表 (1/k_外 + 1/k_内 + 1/k_接触):")
    for i, k in enumerate(k_mesh_table):
        print(f"  [{i}] {k/1e6:.2f} MN/m")
    
    print("\n注意: 传给 fewTeeth.py 的应该是结构刚度 (k_outer, k_inner)，不含接触刚度")
