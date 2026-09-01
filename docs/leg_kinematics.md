# 单腿解析正逆运动学

## 1. 适用范围与符号

本文完整推导 Ascento 构型单自由度闭环腿的正运动学、逆运动学和解析雅可比。推导本身不依赖 MuJoCo，因此既是控制器的几何基础，也是检查仿真模型的独立基准。

所有运行时代码均使用米和弧度。附件论文用于说明构型来源；五个杆长严格采用用户提供的数据：

| 参数 | 杆件 | 长度（mm） | SI 长度（m） |
| --- | --- | ---: | ---: |
| `L1` | `DE` | 235.00 | 0.23500 |
| `L2` | `AD` | 238.00 | 0.23800 |
| `L3` | `BC` | 244.00 | 0.24400 |
| `L4` | `AB` | 108.89 | 0.10889 |
| `L23` | `CD` | 57.00 | 0.05700 |

在固定于髋部的矢状面坐标系中，横轴为 (+x)（向前），纵轴为 (+z)（向上）。(A) 为原点，固定杆 (AB) 与 (+x) 的夹角为 (gamma=45^\circ)。主动输入 (q) 是从 (+x) 到 (AD) 的逆时针角度。当前工作区间为

\[
-65^\circ\le q\le-15^\circ.
\]

(C,D,E) 共线，(D) 位于 (C,E) 之间。选用的装配支路满足

\[
(D-B)\times(C-D)>0,
\qquad
u\times v=u_xv_z-u_zv_x.
\]

## 2. 正运动学闭式解

固定点与主动点直接由几何关系得到：

\[
A=\begin{bmatrix}0\\0\end{bmatrix},\qquad
B=L_4\begin{bmatrix}\cos\gamma\\\sin\gamma\end{bmatrix},\qquad
D=L_2\begin{bmatrix}\cos q\\\sin q\end{bmatrix}.
\]

点 (C) 同时位于以 (B,D) 为圆心的两个圆上：

\[
\|C-B\|=L_3,\qquad \|C-D\|=L_{23}.
\]

令

\[
s=D-B,\quad d=\|s\|,\quad
u=\frac{s}{d},\quad
u_\perp=\begin{bmatrix}-u_z\\u_x\end{bmatrix}.
\]

将 (C-B) 分解到 (u,u_\perp) 上。两圆相减可得沿 (u) 的投影长度

\[
a=\frac{L_3^2-L_{23}^2+d^2}{2d},
\]

垂直投影为

\[
h=\sqrt{L_3^2-a^2}.
\]

因此两个装配候选解为

\[
C_\pm=B+a u\pm h u_\perp.
\]

由于

\[
(D-B)\times(C_\pm-D)=\pm dh,
\]

本项目规定的正支路唯一对应

\[
\boxed{C=B+a u+h u_\perp}.
\]

若 (d>L_3+L_{23})、(d<|L_3-L_{23}|) 或 (h^2<0)，闭环无实数解。由 (C,D,E) 的共线比例关系，轮心得到

\[
\boxed{E=D+\frac{L_1}{L_{23}}(D-C)}.
\]

被动杆的绝对方向角为

\[
\phi=\operatorname{atan2}(C_z-B_z,C_x-B_x),\qquad
\psi=\operatorname{atan2}(C_z-D_z,C_x-D_x).
\]

## 3. 轮心到髋角的闭式逆运动学

单自由度机构的轮心工作空间是一条一维曲线，不是二维区域。因此任意给定 ((E_x,E_z)) 通常没有精确解，算法必须拒绝离轨目标而不能悄悄投影。

给定目标 (E)，点 (D) 必须满足

\[
\|D-A\|=L_2,\qquad \|D-E\|=L_1.
\]

令

\[
t=E-A,\quad \rho=\|t\|,\quad
e=\frac{t}{\rho},\quad e_\perp=\begin{bmatrix}-e_z\\e_x\end{bmatrix},
\]

则两圆交点可写为

\[
p=\frac{L_2^2-L_1^2+\rho^2}{2\rho},\qquad
k=\sqrt{L_2^2-p^2},
\]

\[
D_\pm=A+p e\pm k e_\perp.
\]

对每个候选点，由共线比例重建

\[
C=D+\frac{L_{23}}{L_1}(D-E).
\]

依次检查

\[
\big|\|C-B\|-L_3\big|\le\varepsilon,
\quad (D-B)\times(C-D)>0,
\quad q_{\min}\le\operatorname{atan2}(D_z,D_x)\le q_{\max}.
\]

满足全部条件的解为

\[
\boxed{q=\operatorname{atan2}(D_z,D_x)}.
\]

程序还会将该 (q) 代回正运动学检查 (|E_{FK}-E_{target}|\le\varepsilon)。只给定高度 (E_z) 时，当前工作支路上的 (E_z(q)) 单调，代码使用带工作区间的二分法求根；这不是二维目标投影。

## 4. 闭环约束的解析雅可比

VMC 需要轮心速度与髋角速度之间的精确关系。定义两个闭环约束

\[
g_1=(C-B)^\mathsf{T}(C-B)-L_3^2=0,
\]

\[
g_2=(C-D)^\mathsf{T}(C-D)-L_{23}^2=0.
\]

对时间求导并消去共同因子 2：

\[
(C-B)^\mathsf{T}\dot C=0,
\]

\[
(C-D)^\mathsf{T}(\dot C-\dot D)=0.
\]

主动点关于 (q) 的导数为

\[
D'=\frac{\partial D}{\partial q}
=L_2\begin{bmatrix}-\sin q\\\cos q\end{bmatrix}.
\]

令 (C'=\partial C/\partial q)，可得二阶线性方程

\[
\underbrace{\begin{bmatrix}
(C-B)^\mathsf{T}\\
(C-D)^\mathsf{T}
\end{bmatrix}}_{M(q)}C'
=
\begin{bmatrix}
0\\ (C-D)^\mathsf{T}D'
\end{bmatrix}.
\]

只要 (det M\ne0)，便有

\[
C'=M^{-1}
\begin{bmatrix}0\\ (C-D)^\mathsf{T}D'\end{bmatrix}.
\]

令 (lambda=L_1/L_{23})，由 (E=(1+\lambda)D-\lambda C) 得轮心解析雅可比

\[
\boxed{
J_E(q)=\frac{\partial E}{\partial q}
=(1+\lambda)D'-\lambda C'
=\begin{bmatrix}J_x(q)\\J_z(q)\end{bmatrix}
}.
\]

于是

\[
\dot E=J_E(q)\dot q.
\]

(det M=0) 表示两条约束梯度线性相关，即四连杆的几何奇异位形。代码会显式报错，不会通过增大数值差分步长掩盖奇异性。单元测试在整个工作区间将该解析结果与中心差分作独立比较。

## 5. MuJoCo 坐标映射

MJCF 参考姿态为 (q_0=-40^\circ)，关节坐标表示相对参考姿态的偏移：

\[
q_{hip}=q-q_0,
\]

\[
q_{inner}=(\psi-q)-(\psi_0-q_0),
\qquad q_{pin}=\phi-\phi_0.
\]

三个铰接刚体先构成运动树，再由具名 `connect` 等式约束将上下两条支链在 (C) 点闭合。单腿文件保持零重力、无碰撞，用于纯运动学验证；整车文件另行启用重力与轮地接触。

## 6. 复现与验证

```bash
uv sync --dev
uv run pytest
uv run leg-kinematics
```

`pytest` 覆盖五个杆长残差、装配支路、(IK(FK(q)))、解析雅可比、MuJoCo 的 A—E 站点误差和闭环约束误差；`leg-kinematics` 只负责显示单腿悬空运动，按 ESC 或关闭窗口退出。动力学中的虚功与力矩映射见 `docs/vmc.md`。
