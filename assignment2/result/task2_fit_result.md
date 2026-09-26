# 任务2 结果：合成旋转视频参数拟合

## 模型
ω(t) = b + A·sin(Ωt + φ)   (rad/s)

## 方法
1. HSV 阈值(85~95) 提取青色目标，取最大轮廓质心作为目标中心。
2. 旋转中心取任务已知值 (480,360)；角度 θ=atan2(cy-y, x-cx)，逆时针为正，逐帧解缠。
3. 角速度用相邻帧中心差分 ω=Δθ/Δt；对 ω(t) 拟合。
4. 初值：DFT 找 ω 主频得 Ω，再用线性最小二乘求 b/A/φ；最后 Ceres 精修四参数。

## 参数估计
- b = 1.35001 rad/s (平均角速度)
- A = 0.549835 rad/s (振幅)
- Ω = 1.64987 rad/s (速度变化频率，周期 T=2π/Ω = 3.80828 s)
- φ = 0.702308 rad (归一化到 [-π, π))

## 误差指标（角速度，rad/s）
- RMSE = 0.0332543 rad/s
- 最大绝对残差 = 0.110315 rad/s
- 有效样本数 = 1438
- 参与计算帧范围 = [2, 1439]
- 时间范围 = [0.0166667, 23.9667] s

## 输出文件
- result/task2_fit/tracking_overlay.mp4
- result/task2_fit/fit_comparison.png
- result/task2_fit/angular_velocity.png（与 fit_comparison 同图）
- result/task2_fit/residuals.png
