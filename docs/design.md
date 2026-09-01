# Ascento Dog 机器人建模与控制设计

本文是仓库唯一的技术设计文档，描述当前代码实际采用的坐标约定、单腿解析运动学、四腿支持力分配、VMC 闭环和跨台阶状态机。所有公式使用国际单位制；未由实物或 CAD 确认的参数均明确标为示意值。

---

# 第一章　坐标系、连杆拓扑与符号

## 1.1 世界系、机体系与腿系

系统使用右手坐标系：

- 世界系 $W$：$+x_W$ 为场景中的前进方向，$+y_W$ 向左，$+z_W$ 竖直向上；
- 机体系 $B$：原点位于底盘刚体参考点，$+x_B$ 向前，$+y_B$ 向左，$+z_B$ 向上；
- 第 $i$ 条腿的解析系 $L_i$：原点位于髋轴 $A_i$，$(x_L,z_L)$ 是腿的矢状面，$+z_L$ 向上。

roll、pitch、yaw 分别是绕机体系 $+x_B,+y_B,+z_B$ 的右手转角。按这一约定，正 pitch 使机头下俯。欧拉角按 yaw–pitch–roll 顺序组成

$$
R_{WB}=R_z(\mathrm{yaw})R_y(\mathrm{pitch})R_x(\mathrm{roll}).
$$

$R_{WB}$ 将机体系向量变换到世界系：

$$
v^W=R_{WB}v^B,\qquad v^B=R_{WB}^{\mathsf T}v^W.
$$

四条腿同向安装。所有解析腿系的 $+x_L$ 都指向机体 $-x_B$，即屈膝方向朝向机体后方；后腿不镜像，也不相对前腿增加额外的 $180^\circ$ 旋转。腿系到机体系的固定旋转为

$$
\boxed{R_{BL}=R_z(\pi)=
\begin{bmatrix}
-1&0&0\\
0&-1&0\\
0&0&1
\end{bmatrix}}.
$$

![整车坐标系、腿名和四腿同向安装关系](assets/coordinate_frames.svg)

当前 MuJoCo 示意安装点为

$$
\begin{aligned}
p_{A,FL}^B&=(+0.24,+0.20,0)^{\mathsf T},&
p_{A,FR}^B&=(+0.24,-0.20,0)^{\mathsf T},\\
p_{A,RL}^B&=(-0.24,+0.20,0)^{\mathsf T},&
p_{A,RR}^B&=(-0.24,-0.20,0)^{\mathsf T}.
\end{aligned}
$$

这些安装位置、机身 $0.60\times0.36\times0.15\ \mathrm m$ 外形和轮半径 $R=0.065\ \mathrm m$ 都是仿真示意值，不是实物标定结果。

## 1.2 单腿拓扑与几何参数

单腿有一个连杆主动自由度，车轮自转是独立执行器，不计入连杆自由度。点位定义如下：

| 点 | 含义 |
| --- | --- |
| $A$ | 髋部电机轴，也是解析腿系原点 |
| $B$ | 机身上的固定销轴 |
| $D$ | 主动杆上的内侧关节 |
| $C$ | 上、下支链闭合的膝关节 |
| $E$ | 轮心 |

![单腿闭环连杆点位、杆长、主动角和装配支路](assets/leg_linkage.svg)

几何真值为：

| 符号 | 杆件 | 长度（mm） | 长度（m） |
| --- | --- | ---: | ---: |
| $L_1$ | $DE$ | 235.00 | 0.23500 |
| $L_2$ | $AD$ | 238.00 | 0.23800 |
| $L_3$ | $BC$ | 244.00 | 0.24400 |
| $L_4$ | $AB$ | 108.89 | 0.10889 |
| $L_{23}$ | $CD$ | 57.00 | 0.05700 |

固定杆 $AB$ 与 $+x_L$ 的夹角为

$$
\gamma=45^\circ=\frac{\pi}{4}.
$$

主动角 $q$ 是从 $+x_L$ 到 $AD$ 的逆时针角，当前工作区间为

$$
\boxed{-65^\circ\le q\le-15^\circ}.
$$

$C,D,E$ 共线且 $D$ 位于 $C$ 与 $E$ 之间。二维叉积定义为

$$
u\times v=u_xv_z-u_zv_x,
$$

唯一允许的装配支路满足

$$
\boxed{(D-B)\times(C-D)>0}.
$$

## 1.3 三维装配与二维解析量的映射

解析运动学只处理腿系矢状面向量

$$
p_E^L=\begin{bmatrix}E_x&0&E_z\end{bmatrix}^{\mathsf T}.
$$

第 $i$ 个轮心在机体系中的位置为

$$
p_{E,i}^B=p_{A,i}^B+R_{BL}p_{E,i}^L,
$$

完整世界系点位变换为

$$
\boxed{p_{E,i}^W=p_B^W+R_{WB}p_{E,i}^B}.
$$

后文底盘高度统一指机体参考点的世界系 $z$ 坐标

$$
h=(p_B^W)_z.
$$

当前四个髋安装点的机体系 $z$ 坐标均为零，所以机体水平时 $h$ 也等于四个髋轴的世界系高度；它不是整车质心高度。

相对整车质心的位置为

$$
\boxed{r_i^B=p_{A,i}^B+R_{BL}p_{E,i}^L-r_{COM}^B}.
$$

MuJoCo 参考姿态采用 $q_0=-40^\circ$，其髋关节坐标记录相对偏移：

$$
q_{hip}=q-q_0.
$$

被动关节的参考偏移由第二章定义的 $\phi,\psi$ 给出：

$$
q_{inner}=(\psi-q)-(\psi_0-q_0),\qquad
q_{pin}=\phi-\phi_0.
$$

MJCF 使用两个具名 $C$ 站点和 `connect` 等式约束闭合上下支链，解析运动学始终作为 MuJoCo 点位的外部真值。

---

# 第二章　单腿 VMC、正逆运动学与解析雅可比

## 2.1 正运动学闭式解

给定髋角 $q$，固定点和主动点直接为

$$
A=\begin{bmatrix}0\\0\end{bmatrix},\qquad
B=L_4\begin{bmatrix}\cos\gamma\\\sin\gamma\end{bmatrix},\qquad
D=L_2\begin{bmatrix}\cos q\\\sin q\end{bmatrix}.
$$

点 $C$ 同时满足

$$
\|C-B\|=L_3,\qquad \|C-D\|=L_{23}.
$$

定义

$$
s=D-B,\qquad d=\|s\|,\qquad
e=\frac{s}{d},\qquad
e_\perp=\begin{bmatrix}-e_z\\e_x\end{bmatrix}.
$$

两圆公共弦在 $e$ 方向上的投影和垂直距离分别是

$$
a=\frac{L_3^2-L_{23}^2+d^2}{2d},\qquad
h=\sqrt{L_3^2-a^2}.
$$

两个候选闭合点为

$$
C_\pm=B+ae\pm he_\perp.
$$

因为

$$
(D-B)\times(C_\pm-D)=\pm dh,
$$

选定支路对应正号：

$$
\boxed{C=B+ae+he_\perp}.
$$

闭环存在实解的必要条件是

$$
|L_3-L_{23}|\le d\le L_3+L_{23}.
$$

其中 $d=0$ 时方向 $e$ 未定义；等号对应 $h=0$ 的两圆相切位形，此时严格支路条件不成立且雅可比奇异。它们即使有几何交点，也不是本项目可接受的工作支路状态。

由 $C,D,E$ 的共线关系得到

$$
\boxed{E=D+\frac{L_1}{L_{23}}(D-C)}.
$$

两根被动杆的绝对方向角为

$$
\phi=\operatorname{atan2}(C_z-B_z,C_x-B_x),\qquad
\psi=\operatorname{atan2}(C_z-D_z,C_x-D_x).
$$

至此，$A,B,C,D,E,\phi,\psi$ 都是 $q$ 和固定杆长的显式解析函数。

## 2.2 笛卡尔逆运动学闭式解

单自由度腿的轮心工作空间是一条一维曲线，不是二维区域。给定腿系目标轮心 $E^L=(E_x,E_z)$，先求同时位于圆 $(A,L_2)$ 与圆 $(E,L_1)$ 上的 $D$。必须先满足

$$
\rho=\|E-A\|>0,
\qquad
|L_2-L_1|\le\rho\le L_2+L_1;
$$

否则两圆无有效交点。定义

$$
t=E-A,\qquad \rho=\|t\|,\qquad
n=\frac{t}{\rho},\qquad
n_\perp=\begin{bmatrix}-n_z\\n_x\end{bmatrix},
$$

$$
p=\frac{L_2^2-L_1^2+\rho^2}{2\rho},\qquad
k=\sqrt{L_2^2-p^2}.
$$

两组候选主动点为

$$
\boxed{D_\pm=A+pn\pm kn_\perp}.
$$

对每个候选点重建

$$
C=D+\frac{L_{23}}{L_1}(D-E),
$$

然后依次要求

$$
\left|\|C-B\|-L_3\right|\le\varepsilon,
$$

$$
(D-B)\times(C-D)>0,
$$

$$
q_{min}\le \operatorname{atan2}(D_z,D_x)\le q_{max}.
$$

通过全部检查的闭式逆解为

$$
\boxed{q=\operatorname{atan2}(D_z,D_x)}.
$$

若 $k^2<0$、候选点退化或没有候选点通过，目标不是这条一维轨迹上的可达点，算法报错而不是投影。最后还必须回代正运动学并验证 $\|E_{FK}(q)-E_{target}\|\le\varepsilon$；公共接口默认 $\varepsilon=10^{-8}\ \mathrm m$。若数值容差内出现多个候选，选择回代误差最小者；在当前工作支路内测试期望唯一解。代码中的 `inverse_height(z)` 只针对工作支路上单调的 $E_z(q)$ 使用区间二分，它是高度目标辅助接口，不替代上述二维闭式逆解。

## 2.3 解析雅可比的显式结果

VMC 需要

$$
J_E(q)=\frac{\partial E}{\partial q}
=\begin{bmatrix}J_x(q)\\J_z(q)\end{bmatrix}.
$$

定义闭环约束

$$
g_1=(C-B)^{\mathsf T}(C-B)-L_3^2=0,
$$

$$
g_2=(C-D)^{\mathsf T}(C-D)-L_{23}^2=0.
$$

对 $q$ 求导：

$$
(C-B)^{\mathsf T}C'=0,
$$

$$
(C-D)^{\mathsf T}(C'-D')=0,
$$

其中

$$
D'=L_2\begin{bmatrix}-\sin q\\\cos q\end{bmatrix}.
$$

为得到可直接计算的显式标量结果，令

$$
u=C-B=\begin{bmatrix}u_x\\u_z\end{bmatrix},\qquad
v=C-D=\begin{bmatrix}v_x\\v_z\end{bmatrix},
$$

$$
\Delta=u_xv_z-u_zv_x,
$$

$$
\beta=v^{\mathsf T}D'
=L_2(-v_x\sin q+v_z\cos q).
$$

约束导数方程的解析解是

$$
\boxed{
C'=\frac{\beta}{\Delta}
\begin{bmatrix}-u_z\\u_x\end{bmatrix}}
$$

而

$$
E=(1+\lambda)D-\lambda C,\qquad
\lambda=\frac{L_1}{L_{23}}.
$$

因此轮心雅可比的两个标量分量为

$$
\boxed{
J_x(q)=-(1+\lambda)L_2\sin q
+\lambda\frac{u_z\beta}{\Delta}}
$$

和

$$
\boxed{
J_z(q)=(1+\lambda)L_2\cos q
-\lambda\frac{u_x\beta}{\Delta}}.
$$

于是

$$
\boxed{\dot E=J_E(q)\dot q}.
$$

$\Delta=0$ 表示两个闭环约束梯度线性相关，即机构处于几何奇异位形。实现会显式拒绝该状态。当前工作区间内的代表性结果为：

| $q$ | $E_x$（m） | $E_z$（m） | $J_x$（m/rad） | $J_z$（m/rad） |
| ---: | ---: | ---: | ---: | ---: |
| $-65^\circ$ | 0.012709 | -0.433653 | -0.204684 | 0.270075 |
| $-40^\circ$ | 0.002526 | -0.304309 | 0.017354 | 0.343462 |
| $-15^\circ$ | 0.000186 | -0.111206 | -0.022686 | 0.620168 |

轮心在全工作区间的竖直行程约为 $0.32245\ \mathrm m$，水平总偏移约为 $0.01352\ \mathrm m$，因此轨迹是“近似竖直”，不是严格竖直。

## 2.4 虚功与轮心力到髋力矩

轮心虚位移满足

$$
\delta E=J_E(q)\delta q.
$$

若作用在轮心上的平面外力为

$$
F_E=\begin{bmatrix}F_x\\F_z\end{bmatrix},
$$

则该外力产生的广义力由虚功得到：

$$
\delta W=F_E^{\mathsf T}\delta E
=F_E^{\mathsf T}J_E\delta q
=Q_q\delta q,
$$

$$
\boxed{Q_q=J_E^{\mathsf T}F_E=J_xF_x+J_zF_z}.
$$

控制器中的 $f_i>0$ 表示地面对轮子的世界系向上支持力。髋电机要静态抵消这个外力，因此命令符号为

$$
\tau_i+Q_{q,i}=0.
$$

将平面雅可比嵌入三维

$$
j_i^L=\begin{bmatrix}J_x(q_i)\\0\\J_z(q_i)\end{bmatrix},
$$

得到通用映射

$$
\boxed{
\tau_i=-f_i(e_z^W)^{\mathsf T}R_{WB}R_{BL}j_i^L}.
$$

机体水平时 $R_{WB}=I$，且 $R_{BL}$ 不改变 $z$ 分量，于是

$$
\boxed{\tau_i=-J_z(q_i)f_i}.
$$

正号的 $J^{\mathsf T}F$ 是外力产生的广义力；代码中的负号来自电机对地面支持力的静力平衡，二者不可混用。

## 2.5 单腿验证要求

`tests/test_four_bar.py` 与 `tests/test_mujoco_leg.py` 必须覆盖：

- 五条杆件的长度残差与装配支路符号；
- 工作区间和两端附近的 `IK(FK(q))`；
- 离开一维轮心轨迹的目标被拒绝；
- 上述解析雅可比与中心差分一致；
- MuJoCo 的 $A$–$E$ 站点与解析点位一致；
- 两个 $C$ 站点的闭环误差保持在既定容差内。

---

# 第三章　四腿支持力、力矩分配与力控映射

## 3.1 目标广义力

当前 VMC 只分配世界系竖直支持力，并控制机体 roll/pitch。目标广义力定义为

$$
w^*=\begin{bmatrix}F_z^*\\\tau_x^*\\\tau_y^*\end{bmatrix}.
$$

第 $i$ 个轮子的地面反力为

$$
F_i^W=f_i e_z^W,\qquad f_i\ge0,
$$

其中 $e_z^W=[0,0,1]^{\mathsf T}$。世界竖直方向在机体系中的表示为

$$
n^B=R_{WB}^{\mathsf T}e_z^W.
$$

## 3.2 姿态相关的支持力分配矩阵

利用第一章定义的轮心相对质心位置 $r_i^B$，第 $i$ 条腿对机体产生的力矩为

$$
m_i^B=r_i^B\times(f_i n^B).
$$

定义

$$
a_i=r_i^B\times n^B,
$$

以及腿序固定为

$$
f=\begin{bmatrix}f_{FL}&f_{FR}&f_{RL}&f_{RR}\end{bmatrix}^{\mathsf T}.
$$

则

$$
\boxed{Af=w^*},
$$

其中

$$
\boxed{
A=
\begin{bmatrix}
1&1&1&1\\
a_{FL,x}&a_{FR,x}&a_{RL,x}&a_{RR,x}\\
a_{FL,y}&a_{FR,y}&a_{RL,y}&a_{RR,y}
\end{bmatrix}}.
$$

$A$ 每个控制周期都由实时 $R_{WB}$、质心和四个轮心位置重建，所以公式在机体已有 roll/pitch 时仍成立。轮心到接触点沿世界竖直方向的偏移与支持力共线，不改变该力对质心的力矩，因此可直接使用轮心位置。

## 3.3 水平对称姿态的显式四腿解

当机体水平时

$$
n^B=\begin{bmatrix}0\\0\\1\end{bmatrix},\qquad
r_i^B\times n^B=\begin{bmatrix}y_i\\-x_i\\0\end{bmatrix}.
$$

若前后轮心位于 $x=\pm a$，左右轮心位于 $y=\pm b$，最小范数对称解为

$$
\boxed{
f_{FL}=\frac{F_z^*}{4}+\frac{\tau_x^*}{4b}-\frac{\tau_y^*}{4a}}
$$

$$
\boxed{
f_{FR}=\frac{F_z^*}{4}-\frac{\tau_x^*}{4b}-\frac{\tau_y^*}{4a}}
$$

$$
\boxed{
f_{RL}=\frac{F_z^*}{4}+\frac{\tau_x^*}{4b}+\frac{\tau_y^*}{4a}}
$$

$$
\boxed{
f_{RR}=\frac{F_z^*}{4}-\frac{\tau_x^*}{4b}+\frac{\tau_y^*}{4a}}.
$$

因此，总支持力改变四腿力之和；roll 力矩改变左右差；pitch 力矩改变前后差。实际控制不强制四腿等载，而是使用实时质心和轮心位置。

## 3.4 有界支持力解算

轮地法向力只能推地，不能拉地：

$$
f_{i,min}\le f_i\le f_{i,max},\qquad f_{i,min}\ge0.
$$

当前分配器始终求解同一个带边界目标：

$$
\boxed{
\min_f\ \|W(Af-w^*)\|_2^2
+\epsilon\|f-f_0\|_2^2}
$$

并满足全部力边界。令

$$
\ell=\max_i\sqrt{r_{i,x}^2+r_{i,y}^2},\qquad
W=\operatorname{diag}(1,1/\ell,1/\ell),
$$

$$
f_{0,i}=\operatorname{clip}\left(\frac{\max(0,F_z^*)}{4},f_{i,min},f_{i,max}\right),
\qquad \epsilon=10^{-12}.
$$

$W$ 用当前最大水平力臂对力和力矩残差做量纲归一，极小的 $\epsilon$ 用于在近似等残差解之间偏向均匀承载，而不是严格的词典序保证。四条腿各有“下界、自由、上界”三种状态，当前实现穷举

$$
3^4=81
$$

组有效集，在可行候选中选择代价最小者。分配器同时返回

$$
r_w=Af-w^*,
$$

以显式报告目标广义力的不可实现程度。

## 3.5 从四腿支持力到四个髋力矩

对每条腿分别计算 $J_i(q_i)$，然后执行第二章的静力映射：

$$
\boxed{
\tau_i=-f_i(e_z^W)^{\mathsf T}R_{WB}R_{BL}j_i^L,
\quad i\in\{FL,FR,RL,RR\}}.
$$

完整链路为

$$
w^*\longrightarrow
f^*=\underset{f_{min}\le f\le f_{max}}{\arg\min}\,
\left(\|W(Af-w^*)\|_2^2+\epsilon\|f-f_0\|_2^2\right)
\longrightarrow \tau_i=-J_i^{\mathsf T}F_i.
$$

支持力先经过单腿力上限，髋力矩再独立限幅。两级限幅是不同约束：前者限制可分配接触力，后者限制执行器命令。当前尚未实现接触状态估计；若车轮失联，分配器不会自动移除该腿，这是后续安全状态机必须解决的问题。

---

# 第四章　完整控制框架

## 4.1 闭环信号流

![底盘高度 PID、重力前馈、姿态 PD、支持力解算和雅可比力矩映射组成的 VMC 闭环](assets/vmc_control_flow.svg)

软件分层如下：

| 层 | 文件 | 职责 |
| --- | --- | --- |
| 解析运动学 | `ascento_dog/kinematics/four_bar.py` | FK、IK、工作支路、解析雅可比 |
| 控制器 | `ascento_dog/control/vmc.py` | 高度 PID、姿态 PD、有界支持力分配、髋力矩映射 |
| 轮速控制 | `ascento_dog/control/wheel_speed.py` | 驱动运动学、四轮独立 PI、键盘脉冲命令 |
| 仿真适配 | `mujoco/simulation/mujoco_vmc.py` | 状态读取、模型质量和质心读取、指令写入、扰动 |
| 动力学 | `mujoco/quadruped.xml` | 自由底盘、四个闭环腿、重力、轮地接触、IMU 和执行器 |
| 入口 | `ascento_dog/scripts/vmc.py` | 参数解析、Viewer、目标高度和遥控编排 |

## 4.2 底盘高度 PID

高度误差定义为

$$
e_h=h^*-h.
$$

高度反馈只产生相对重力前馈的附加推力：

$$
\boxed{
F_{PID}=K_{ph}e_h+K_{ih}I_h-K_{dh}\dot h}.
$$

微分项对测量高度速度 $\dot h$ 作用，不对误差差分，因此目标高度阶跃不会产生微分冲击。离散积分候选为

$$
I_{h,k}^{cand}=\operatorname{clip}
\left(I_{h,k-1}+e_{h,k}\Delta t,-I_{max},I_{max}\right).
$$

先用候选积分计算未限幅输出 $F_{raw}$，再执行

$$
F_{PID}=\operatorname{clip}(F_{raw},-F_{max},F_{max}).
$$

仅在输出未饱和，或当前误差会把饱和输出拉回允许区间时接受 $I_{h,k}^{cand}$，这就是条件积分抗饱和。`reset()` 会清空积分状态。高度环没有隐藏的腿长位置环，PID 输出单位直接是牛顿。当前抗饱和只感知 $F_{PID}$ 自身的限幅，不会根据后续的 $F_z^*\ge0$、单腿力饱和或髋力矩饱和做反算。

## 4.3 重力前馈

整车总质量 $M$ 从编译后的 MuJoCo 模型读取，不在控制器内重复硬编码：

$$
\boxed{F_g=Mg}.
$$

目标总支持力为

$$
\boxed{F_z^*=\max(0,F_g+F_{PID})}.
$$

前馈承担名义静载，PID 只修正高度误差和竖直运动。这样可以减小稳态积分需求，并使质量参数的来源单一。

## 4.4 roll/pitch 姿态控制

常规 VMC 的目标姿态为

$$
\phi^*=0,\qquad\theta^*=0.
$$

机体系恢复力矩为

$$
\boxed{
\tau_x^*=-K_{p\phi}\phi-K_{d\phi}\omega_x}
$$

和

$$
\boxed{
\tau_y^*=-K_{p\theta}\theta-K_{d\theta}\omega_y}.
$$

yaw 当前不进入闭环，所以持续 yaw 扰动会留下航向偏差。IMU 已提供四元数、陀螺仪和线加速度接口，但当前控制状态直接读取 MuJoCo 真值，尚未实现真实传感器融合。

## 4.5 支持力解算与执行器命令

控制器组合

$$
w^*=\begin{bmatrix}F_z^*&\tau_x^*&\tau_y^*\end{bmatrix}^{\mathsf T},
$$

用实时轮心、质心和姿态构建第三章的 $A$，求得四个有界 $f_i$，再通过解析雅可比得到四个髋力矩。当前普通 VMC 的示意限制是

$$
0\le f_i\le120\ \mathrm N,
\qquad
|\tau_i|\le40\ \mathrm{N\,m}.
$$

每个控制周期输出期望支持力、分配器得到的 $Af$ 和分配残差 $Af-w^*$，便于上层判断接触力层面的饱和或不可实现状态。随后若髋力矩被二次限幅，当前实现不会回算新的支持力、重新分配或更新该残差，因此 `achieved_wrench` 只表示力分配器输出，不表示执行器限幅后的真实机体广义力。

## 4.6 单个控制周期

整机 MJCF 时间步长为 $1\ \mathrm{ms}$，VMC 每个仿真步更新一次，即当前更新频率为 $1\ \mathrm{kHz}$：

1. 读取底盘高度、竖直速度、$R_{WB}$、roll/pitch/yaw、机体系角速度和四个髋角；
2. 从 `subtree_com` 计算整车质心 $r_{COM}^B$；
3. 计算 $F_g$、$F_{PID}$、$\tau_x^*$ 和 $\tau_y^*$；
4. 对四腿执行 FK，生成实时 $r_i^B$ 和支持力分配矩阵 $A$；
5. 在各腿上下界内求解 $f_i$，记录 $Af-w^*$；
6. 计算四个解析 $J_i(q_i)$，映射并限幅髋力矩；
7. 可选地运行四轮速度 PI，生成轮电机力矩；
8. 写入 MuJoCo 执行器并推进一个动力学步。

## 4.7 水平轮速控制

`vmc --teleop` 在 VMC 高度/姿态环外增加轮速环。对安装横坐标为 $y_i$ 的轮子，目标轮心线速度为

$$
v_{x,i}=v_x^*-\omega_{yaw}^*y_i,
$$

轮角速度目标为

$$
\boxed{\omega_i^*=\frac{v_x^*-\omega_{yaw}^*y_i}{R}}.
$$

四个轮子各自使用带条件抗饱和的 PI：

$$
\tau_{w,i}=\operatorname{clip}
\left(K_{pw}e_{\omega,i}+K_{iw}\int e_{\omega,i}dt,
-\tau_{w,max},\tau_{w,max}\right).
$$

数字键 `1/2/3/4` 分别触发前进、后退、左转、右转的锁存脉冲，默认在 $1\ \mathrm s$ 内线性衰减。该轮速环实现平移和开环 yaw 速率命令，但不构成 yaw 姿态保持。

## 4.8 当前示意增益、限幅与故障行为

普通 VMC 演示的当前参数为：

| 参数 | 当前示意值 |
| --- | ---: |
| 高度 $K_p,K_i,K_d$ | 1000 N/m，200 N/(m·s)，260 N·s/m |
| 高度积分限幅 | 0.15 m·s |
| PID 附加推力限幅 | ±180 N |
| roll $K_p,K_d$ | 180 N·m/rad，28 N·m·s/rad |
| pitch $K_p,K_d$ | 260 N·m/rad，38 N·m·s/rad |
| 单腿最大支持力 | 120 N |
| 单髋最大力矩 | 40 N·m |

这些值仅适用于当前占位质量、惯量、轮胎和接触参数。获得 CAD 和执行器辨识数据后必须重新整定。

明确的故障与降级行为是：

- 非有限输入、错误数组形状、越界髋角和奇异雅可比：拒绝计算并报错；控制器本身不负责清零、保持上一拍或切入硬件安全态，调用方必须处理异常；
- 目标广义力不可实现：返回最优有界力和非零残差；
- 髋力矩超限：单独截断到执行器限制，不触发重新分配；
- 车轮失联：当前没有接触估计与失联腿重分配；
- yaw 偏差：当前不恢复；
- 真实驱动器带宽、弹簧、轮胎和传感器噪声：尚未建模。

控制与动力学回归由 `tests/test_vmc.py`、`tests/test_mujoco_quadruped.py`、`tests/test_mujoco_teleop.py` 和 `tests/test_wheel_speed.py` 执行。测试通过只证明当前示意模型在覆盖工况内自洽，不代表实物稳定性或安全性。

---

# 第五章　上台阶状态机与特殊控制

## 5.1 场景与目标

`mujoco/quadruped_step.xml` 在整机模型前方放置一个默认高度

$$
H_s=0.150\ \mathrm m
$$

的平台，立面前缘为 $x=0.9\ \mathrm m$，轮半径示意值为 $R=0.065\ \mathrm m$，台阶摩擦系数 1.5 也是仿真示意值。目标是完成

$$
\text{接近}\rightarrow\text{前轮上台}\rightarrow\text{跨坐}
\rightarrow\text{后轮上台}\rightarrow\text{四腿伸展},
$$

常规阶段的 VMC 将 pitch 目标闭环到零；`REAR_CLIMB` 切换为纯关节位置伺服后不再计算姿态 PD，而是依靠左右对称目标、前腿几何锁定和后腿同步收缩近似维持水平。

工作区间内最短腿位于 $q_{max}=-15^\circ$，轮心相对安装座下探量为

$$
d_{min}=-E_z(q_{max})=0.111206\ \mathrm m.
$$

跨越阶段的水平低站姿选为

$$
\boxed{h_{stance}=H_s+R+d_{min}=0.326206\ \mathrm m}.
$$

这里 $h_{stance}$ 与 $h_{extend}$ 都是第一章定义的机体参考点世界系高度；由于当前髋安装点 $(p_A^B)_z=0$，水平姿态下也等于髋轴世界系高度。

这样前腿收缩到最短时，前轮中心恰位于台面上方一个轮半径。标称姿态 $q_0=-40^\circ$ 的下探量为

$$
d_0=-E_z(q_0)=0.304309\ \mathrm m,
$$

所以台上最终标称高度为

$$
\boxed{h_{extend}=H_s+R+d_0=0.519309\ \mathrm m}.
$$

## 5.2 六阶段状态机

![接近、前轮爬升、跨坐、后轮爬升、伸展和完成六阶段状态机](assets/step_state_machine.svg)

| 阶段 | 腿部控制 | 轮驱动与目标 | 转移条件 |
| --- | --- | --- | --- |
| `APPROACH` | 四腿 VMC 力控 | 低站姿前进 | 两个前轮都接近立面 |
| `FRONT_CLIMB` | 四腿仍为 VMC；前腿支持力上限降到 70 N | 前后轮保持较强正向速度，建立立面滚动摩擦 | 前轮越过边缘且轮心达到台面高度 |
| `STRADDLE` | 四腿 VMC 力控 | 前轮在台、后轮在地，继续低站姿推进 | 两个后轮都接近立面 |
| `REAR_CLIMB` | 四腿切换关节位置伺服；前腿锁最短，后腿按高度调度收缩 | 前轮低顶压，后轮自转辅助拖拽 | 后轮越过边缘且轮心达到台面高度 |
| `EXTEND` | 恢复四腿 VMC 力控 | $1.5\ \mathrm s$ 内将高度斜坡升到 $h_{extend}$ | 斜坡结束且高度误差不超过 0.02 m |
| `DONE` | 四腿 VMC 保持 | 轮速归零 | 保持终态 |

精确位置判据使用两个轮子的世界系坐标 $x_i^W,z_i^W$ 的最小值，保证左右轮都满足条件。当前接触余量 $m_c=0.005\ \mathrm m$，边缘余量 $m_e=0.040\ \mathrm m$。以立面位置 $x_s$ 表示：

$$
\min_{i\in front}x_i^W\ge x_s-R-m_c
$$

触发前轮爬升；前轮完成条件为

$$
\min_{i\in front}x_i^W\ge x_s+m_e,
\qquad
\min_{i\in front}z_i^W\ge H_s+R-m_c.
$$

后轴使用同样的两组判据。

## 5.3 前轮爬升：降低下压力并利用立面摩擦

前轮抵住立面时，水平轮驱动产生法向压力 $N$，轮子沿立面滚动所能获得的向上摩擦与 $\mu N$ 相关。若前腿向下支持力过大，轮子会被压在地面与立面交界处。当前特殊处理是：

1. 继续使用四腿 VMC，保持底盘高度和水平姿态；
2. 将每条前腿的支持力上限降到 $70\ \mathrm N$，减小爬升轴下压力；
3. 前后轮目标速度由常规 $0.25\ \mathrm{m/s}$ 增加到 $0.40\ \mathrm{m/s}$，建立足够顶压和滚动摩擦；
4. 不在此阶段切入腿位置环，前腿随轮心抬升自然收缩。

这里的立面摩擦爬升机制只是在当前高摩擦示意场景中的行为解释，不是对真实轮胎和台阶接触的保证。

## 5.4 后轮爬升：前腿几何锁定与后腿提升调度

后轮爬升阶段改用混合位置策略。前腿已经位于台面，控制目标固定为

$$
q_{front}^*=q_{max}=-15^\circ,
$$

即把前腿锁在最短位置，以几何支柱维持底盘高度。后轮中心的世界系目标高度 $z_{rear}^{W*}$ 从进入该阶段时的实测值开始，以

$$
\dot z_{rear}^{W*}=0.08\ \mathrm{m/s}
$$

向上推进。对后腿 $i$，先由当前机体姿态求髋轴世界高度

$$
z_{A,i}^W=h+\left(R_{WB}p_{A,i}^B\right)_z,
$$

一般姿态下，世界轮心高度的精确关系是

$$
z_{rear}^{W*}=z_{A,i}^W
+(e_z^W)^{\mathsf T}R_{WB}R_{BL}
\begin{bmatrix}E_x(q_i^*)\\0\\E_z(q_i^*)\end{bmatrix}.
$$

由于 $E_x$ 与 $E_z$ 都依赖 $q$，严格实现应在当前姿态下对这个标量方程做一维求根。当前代码采用水平机体近似 $R_{WB}\approx I$，忽略姿态引入的 $E_x$ 高度耦合以及对 $E_z$ 世界竖直投影的改变，只将世界高度目标换成腿系下探量和腿系轮心高度：

$$
d_i^*=z_{A,i}^W-z_{rear}^{W*},\qquad
E_{z,i}^{L*}=-d_i^*.
$$

高度逆运动学的输入是腿系 $E_z^L$，因此当前近似髋角目标为

$$
q_{rear,i}^*=IK_z(E_{z,i}^{L*}).
$$

`REAR_CLIMB` 又没有主动 pitch PD，所以明显姿态瞬态会降低这一近似的精度；这是当前状态机的已知限制，而不是完整姿态补偿。

四条腿的关节位置伺服为

$$
\boxed{
\tau_i=\operatorname{clip}
\left(K_{p,q}(q_i^*-q_i)-K_{d,q}\dot q_i,
-\tau_{max},\tau_{max}\right)}.
$$

后腿收缩把后轮沿立面拖上去。当前前轮速度目标为 $0.40\ \mathrm{m/s}$，后轮为 $0.45\ \mathrm{m/s}$。前轮顶压不能过大：在高摩擦台面上，过大的法向力会使静摩擦阻力超过后腿位置伺服能够提供的拖拽力矩。

## 5.5 伸展与阶段故障

四轮全部上台后，控制器恢复四腿 VMC，并把目标高度从进入 `EXTEND` 时的实测值线性插值到 $h_{extend}$：

$$
h^*(t)=h_{start}
+\operatorname{clip}\left(\frac{t}{T_{extend}},0,1\right)
(h_{extend}-h_{start}),
$$

其中 $T_{extend}=1.5\ \mathrm s$。roll/pitch 目标保持为零，四腿在支持力框架下同步伸展。

各阶段具有独立超时：`APPROACH` 20 s、`FRONT_CLIMB` 15 s、`STRADDLE` 20 s、`REAR_CLIMB` 15 s、`EXTEND` 8 s。超时后控制器设置 `failed` 并给出阶段名；腿集合错误或 $\Delta t\le0$ 会直接报错。多数非有限数值会在运动学、分配器或轮速 PI 中被拒绝，但状态机尚未对全部观测字段执行统一的入口有限性检查。控制器也没有专用的失败安全输出：当前脚本在观察到 `failed` 后停止循环，但实机调用方仍必须定义制动、卸力或撤退策略。状态机没有在线接触估计、障碍高度识别、失败后撤退或路径重规划。

## 5.6 参数来源、局限与复现

连杆尺寸来自用户给定几何真值。台阶高度、位置、摩擦、轮半径、机身尺寸、力帽、位置伺服增益、轮速和全部执行器限制都是当前仿真的示意值。主要局限包括：

- 前轮爬升依赖较高立面摩擦和足够轮驱动顶压；
- 后轮爬升依赖前腿锁定、低顶压和后腿位置伺服之间的平衡；
- 跨越中可能出现短暂 pitch 瞬态；
- 场景只验证直行，yaw 无闭环；
- 更高或更低摩擦的台阶不保证沿用同一参数成功；
- 当前成功不代表实物具备碰撞安全、驱动热安全或失联腿容错能力。

完整回归命令为

```bash
uv run ruff check
uv run ruff format --check
uv run pytest
uv run cross-step --headless --duration 30
```

`tests/test_cross_step.py` 检查台阶几何、四腿同向安装、状态机阶段顺序和完整无界面跨越终态。查看器入口为 `uv run cross-step`，按 `Esc` 退出。
