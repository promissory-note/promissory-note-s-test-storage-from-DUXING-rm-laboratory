# Assignment 2：合成旋转视频的参数拟合（任务 2）

## 环境依赖
- Ubuntu 22.04、g++ 11（C++17）、CMake ≥ 3.10
- OpenCV（`libopencv-dev`）
- Eigen3（`libeigen3-dev`）
- Ceres Solver（`libceres-dev`，依赖 glog/gflags）

```bash
sudo apt install -y build-essential cmake libopencv-dev libeigen3-dev \
                    libceres-dev libgoogle-glog-dev libgflags-dev
```

## 目录结构
```
assignment2/
├── CMakeLists.txt
├── README.md
├── pro18.cpp               # 任务2 源码
├── resources/
│   └── task_2.mp4          # 输入素材（960×720, 60FPS, 24s, 1440帧）
└── result/
    ├── task2_fit_result.md
    └── task2_fit/
        ├── tracking_overlay.mp4
        ├── fit_comparison.png
        ├── angular_velocity.png
        └── residuals.png
```

## 构建与运行
```bash
cd assignment2
mkdir -p build && cd build
cmake ..
cmake --build . -j
cd ..
./build/task2_fit                        # 默认读 resources/task_2.mp4
./build/task2_fit resources/task_2.mp4   # 也可指定输入
```
> 需在本目录运行，结果写入 ./result/。

## 模型与方法
- 模型：`ω(t) = b + A·sin(Ωt + φ)`（rad/s），时间原点为第 0 帧
- 旋转中心：已知 (480,360)（半径约 220px）；角度 `θ = atan2(cy − y, x − cx)`，逆时针为正
- 青色目标：HSV `H∈[85,95], S∈[100,255], V∈[100,255]`，取最大轮廓质心
- 角速度：相邻帧中心差分 `ω = Δθ/Δt`
- 参数估计：DFT 找 ω 主频定 Ω 初值 → Eigen 线性最小二乘求 b/A/φ → Ceres 精修四参数

## 结果（最近一次）
| 参数 | 值 |
|---|---|
| b | 1.35001 rad/s |
| A | 0.549835 rad/s |
| Ω | 1.64987 rad/s（周期 T=3.808 s）|
| φ | 0.702308 rad（归一化 [−π,π)）|

误差（角速度）：RMSE = 0.0333 rad/s，最大绝对残差 0.110 rad/s，有效样本 1438，帧范围 [2,1439]。

## 输出文件
- `result/task2_fit/tracking_overlay.mp4`：带识别标记的视频
- `result/task2_fit/fit_comparison.png`：观测点与拟合曲线
- `result/task2_fit/angular_velocity.png`：角速度曲线（与对比图同图）
- `result/task2_fit/residuals.png`：残差曲线
- `result/task2_fit_result.md`：模型、参数、方法、误差指标

## 备注
- `build/` 为构建产物，不提交。
