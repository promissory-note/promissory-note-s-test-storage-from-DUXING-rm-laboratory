# Assignment 1：OpenCV 图像处理（任务 1）

## 环境依赖
- Ubuntu 22.04、g++ 11（C++17）、CMake ≥ 3.10
- OpenCV（`sudo apt install libopencv-dev`）

## 目录结构
```
assignment1/
├── CMakeLists.txt
├── README.md
├── pro16.cpp               # 任务1 源码
├── resources/
│   └── test_image.jpg      # 输入素材
└── result/
    └── task1_images/       # 处理结果
```

## 构建与运行
```bash
cd assignment1
mkdir -p build && cd build
cmake ..
cmake --build . -j
cd ..
./build/task1_image                       # 默认读 resources/test_image.jpg
./build/task1_image resources/test_image.jpg   # 也可指定输入
```
> 需在本目录运行，结果写入 ./result/。

## 处理内容与参数
| 步骤 | 说明 / 参数 | 输出 |
|---|---|---|
| 灰度 | `cvtColor(BGR2GRAY)` | `gray.png` |
| HSV | `cvtColor(BGR2HSV)`，三分量 | `hsv_h.png` / `hsv_s.png` / `hsv_v.png` |
| 红色掩膜 | H∈[0,10]∪[170,179]，S,V≥100 | `red_mask.png` |
| 腐蚀/膨胀 | 5×5 矩形核 | `erode.png` / `dilate.png` |
| 开/闭运算 | `MORPH_OPEN` / `MORPH_CLOSE` | `open.png` / `close.png` |
| 轮廓+外接矩形+面积 | 面积阈值 100 | `contours_boxes.png` |
| 滤波 | 均值 5×5；高斯 5×5 σ=1.5；中值 5 | `mean_filter.png` / `gaussian_filter.png` / `median_filter.png` |

运行输出：`valid_contours=5 total_area=896`（本图）。

## 备注
- 绘图/旋转/裁剪（`drawing.png`、`rotated_35deg.png`、`crop_top_left.png`）未包含在本程序内。
- `build/` 为构建产物，不提交。
