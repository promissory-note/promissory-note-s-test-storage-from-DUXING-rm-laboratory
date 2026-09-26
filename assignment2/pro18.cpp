#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <deque>
#include <string>
#include <numeric>
#include <algorithm>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <ceres/ceres.h>
using namespace cv;
using namespace std;

// ==================== 任务2 配置 ====================
static const string OUTDIR = "result/task2_fit/";      // 结果输出目录
static const string MD_FILE = "result/task2_fit_result.md";
static const Point2f KNOWN_CENTER(480, 360);           // 任务已知旋转中心(可检测替代)
static const bool USE_KNOWN_CENTER = true;             // true=用已知中心(更稳)

// ==================== 简单绘图工具 ====================
struct Plot {
    Mat img; int W, H, mL{80}, mR{30}, mT{50}, mB{60};
    double xmin, xmax, ymin, ymax; string title, xlab, ylab;

    Plot(int w, int h, double xmn, double xmx, double ymn, double ymx,
         const string& t, const string& xl, const string& yl)
        : W(w), H(h), xmin(xmn), xmax(xmx), ymin(ymn), ymax(ymx),
          title(t), xlab(xl), ylab(yl) {
        if (xmax - xmin < 1e-9) xmax = xmin + 1e-9;
        if (ymax - ymin < 1e-9) ymax = ymin + 1e-9;
        img = Mat(H, W, CV_8UC3, Scalar(255, 255, 255));
        line(img, Point(mL, mT), Point(mL, H - mB), Scalar(0, 0, 0), 1);
        line(img, Point(mL, H - mB), Point(W - mR, H - mB), Scalar(0, 0, 0), 1);
        putText(img, title, Point(mL, mT - 18), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 0), 2);
        putText(img, xlab, Point(W / 2 - 60, H - 15), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1);
        putText(img, ylab, Point(8, mT + 12), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1);
    }
    Point px(double x, double y) const {
        double px_ = mL + (x - xmin) / (xmax - xmin) * (W - mL - mR);
        double py_ = H - mB - (y - ymin) / (ymax - ymin) * (H - mT - mB);
        return Point((int)lround(px_), (int)lround(py_));
    }
    void points(const vector<double>& x, const vector<double>& y, Scalar c) {
        for (size_t i = 0; i < x.size(); i++) circle(img, px(x[i], y[i]), 3, c, -1);
    }
    void curve(const vector<double>& x, const vector<double>& y, Scalar c, int th = 2) {
        for (size_t i = 1; i < x.size(); i++)
            line(img, px(x[i - 1], y[i - 1]), px(x[i], y[i]), c, th);
    }
    void zero() { int py = px(xmin, 0).y; line(img, Point(mL, py), Point(W - mR, py), Scalar(150, 150, 150), 1); }
    void save(const string& p) { imwrite(p, img); }
};

// 取向量最小/最大值
static void minmaxOf(const vector<double>& v, double& mn, double& mx, bool includeZero = false) {
    mn = *min_element(v.begin(), v.end());
    mx = *max_element(v.begin(), v.end());
    if (includeZero) { mn = min(mn, 0.0); mx = max(mx, 0.0); }
    double pad = (mx - mn) * 0.1 + 1e-6; mn -= pad; mx += pad;
}

// ==================== 三点定圆（备用；已知中心时可不用） ====================
bool calccenter2(Point2f p1, Point2f p2, Point2f p3, Point2f& c) {
    double dx1 = p1.x - p2.x, dx2 = p2.x - p3.x;
    double dy1 = p1.y - p2.y, dy2 = p2.y - p3.y;
    if (fabs(dy1) < 1e-6 || fabs(dy2) < 1e-6) return false;
    double s1 = -dx1 / dy1, s2 = -dx2 / dy2;
    if (fabs(s2 - s1) < 1e-6) return false;
    double mx1 = 0.5 * (p1.x + p2.x), mx2 = 0.5 * (p2.x + p3.x);
    double my1 = 0.5 * (p1.y + p2.y), my2 = 0.5 * (p2.y + p3.y);
    Eigen::Matrix2d D, D1, D2;
    D << s1, -1, s2, -1;
    D1 << s1 * mx1 - my1, -1, s2 * mx2 - my2, -1;
    D2 << s1, s1 * mx1 - my1, s2, s2 * mx2 - my2;
    double det = D.determinant();
    if (fabs(det) < 1e-9) return false;
    c.x = (float)(D1.determinant() / det);
    c.y = (float)(D2.determinant() / det);
    return true;
}

// ==================== 初值估计：DFT 定频 + 线性最小二乘 ====================
void estimateInitialValues(const vector<double>& t, const vector<double>& w,
                           double& b, double& A, double& Omega, double& phi) {
    size_t N = t.size();
    if (N < 4 || w.size() != N) { b = 0; A = .5; Omega = 1.5; phi = 0; return; }

    b = accumulate(w.begin(), w.end(), 0.0) / N;          // 均值

    vector<double> y(N);                                   // 去均值
    for (size_t i = 0; i < N; i++) y[i] = w[i] - b;

    // DFT 扫频找主频 f0 -> Omega
    double dt = (t[1] > t[0]) ? (t[1] - t[0]) : 1.0 / 60.0;
    double fs = 1.0 / dt, fmin = 0.05, fmax = fs / 2.0, df = 0.005;
    double bestF = 0, bestMag = -1;
    for (double f = fmin; f <= fmax; f += df) {
        double re = 0, im = 0;
        for (size_t i = 0; i < N; i++) {
            double a = 2 * M_PI * f * t[i];
            re += y[i] * cos(a); im -= y[i] * sin(a);
        }
        double mag = re * re + im * im;
        if (mag > bestMag) { bestMag = mag; bestF = f; }
    }
    Omega = 2 * M_PI * bestF;
    if (Omega <= 1e-6) Omega = 1.5;

    // 固定 Omega，线性最小二乘解 w ≈ b + p·sin(Ωt) + q·cos(Ωt)
    Eigen::MatrixXd M(N, 3); Eigen::VectorXd wv(N);
    for (size_t i = 0; i < N; i++) {
        double a = Omega * t[i];
        M(i, 0) = 1; M(i, 1) = sin(a); M(i, 2) = cos(a);
        wv(i) = w[i];
    }
    Eigen::Vector3d x = (M.transpose() * M).ldlt().solve(M.transpose() * wv);
    b = x[0]; A = sqrt(x[1] * x[1] + x[2] * x[2]); phi = atan2(x[2], x[1]);
    if (A < 1e-9) A = 1e-9;
}

// ==================== Ceres 非线性精修 ====================
struct OmegaResidual {
    OmegaResidual(double t, double w) : t_(t), w_(w) {}
    template <typename T>
    bool operator()(const T* const p, T* r) const {        // p = [b, A, Omega, phi]
        r[0] = p[0] + p[1] * sin(p[2] * T(t_) + p[3]) - T(w_);
        return true;
    }
    double t_, w_;
};

void refineByCeres(const vector<double>& t, const vector<double>& w,
                   double& b, double& A, double& Omega, double& phi) {
    size_t N = t.size();
    if (N < 4 || w.size() != N) return;
    if (A < 0) { A = -A; phi += M_PI; }
    double p[4] = {b, A, Omega, phi};
    ceres::Problem problem;
    for (size_t i = 0; i < N; i++) {
        ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<OmegaResidual, 1, 4>(
            new OmegaResidual(t[i], w[i]));
        problem.AddResidualBlock(cost, new ceres::HuberLoss(0.1), p);
    }
    problem.SetParameterLowerBound(p, 1, 0.0);     // A >= 0
    problem.SetParameterLowerBound(p, 2, 1e-6);    // Omega > 0
    ceres::Solver::Options opt; opt.linear_solver_type = ceres::DENSE_QR;
    opt.max_num_iterations = 100;
    ceres::Solver::Summary summary;
    ceres::Solve(opt, &problem, &summary);
    b = p[0]; A = p[1]; Omega = p[2]; phi = p[3];
    cout << "Ceres: " << summary.BriefReport() << endl;
}

int main(int argc, char** argv) {
    // 输入默认取 resources/task_2.mp4，也可用命令行参数指定
    const string video = (argc > 1) ? argv[1] : "resources/task_2.mp4";
    VideoCapture cap(video);
    if (!cap.isOpened()) { cerr << "无法打开视频 " << video << endl; return 1; }

    double fps = cap.get(CAP_PROP_FPS); if (fps <= 0) fps = 60.0;
    int W = (int)cap.get(CAP_PROP_FRAME_WIDTH), H = (int)cap.get(CAP_PROP_FRAME_HEIGHT);
    cout << "video " << W << "x" << H << " fps=" << fps << endl;

    filesystem::create_directories(OUTDIR);           // 建结果目录
    VideoWriter vw; bool vwOpen = false;              // 标注视频

    // ---- 状态量 ----
    deque<Point2f> recent_points;                     // 检测中心(备用)
    long framecount = 0;
    double prev_wrapped = 0, current_unwrapped = 0;   // 角度(rad)
    bool angle_init = false;
    deque<double> ang_buf, t_buf; deque<long> idx_buf; // 三帧缓冲(前/后求中间)

    vector<double> t_omega, w_obs;                    // 观测: 时间(s) 与角速度(rad/s)
    vector<long>   frame_omega;                       // 每个 omega 对应帧号

    while (true) {
        Mat frame; cap >> frame;
        if (frame.empty()) break;
        framecount++;

        // 1) 识别青色目标(HSV 阈值 85~95)，取最大轮廓质心
        Mat hsv, mask;
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        inRange(hsv, Scalar(85, 100, 100), Scalar(95, 255, 255), mask);
        morphologyEx(mask, mask, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(3, 3)));
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        Point2f ball(-1, -1); double max_area = 0;
        for (auto& c : contours) {
            double a = contourArea(c);
            if (a > 100 && a > max_area) {
                max_area = a; Moments m = moments(c);
                if (m.m00 != 0) { ball.x = (float)(m.m10 / m.m00); ball.y = (float)(m.m01 / m.m00); }
            }
        }

        // 2) 旋转中心：优先已知中心
        recent_points.push_back(ball);                 // 备用检测
        if (framecount % 25 == 0) recent_points.push_back(ball);
        while (recent_points.size() > 3) recent_points.pop_front();
        Point2f center = KNOWN_CENTER;
        if (!USE_KNOWN_CENTER && recent_points.size() == 3) {
            Point2f c; if (calccenter2(recent_points[0], recent_points[1], recent_points[2], c)) center = c;
        }

        // 3) 角度计算 + 解缠，逐帧角速度(前后两帧定中间帧)
        if (ball.x >= 0) {
            double wrapped = atan2(center.y - ball.y, ball.x - center.x);   // 逆时针为正
            if (!angle_init) { current_unwrapped = wrapped; prev_wrapped = wrapped; angle_init = true; }
            else {
                double d = wrapped - prev_wrapped;
                if (d > M_PI) d -= 2 * M_PI; else if (d < -M_PI) d += 2 * M_PI;
                current_unwrapped += d; prev_wrapped = wrapped;
            }
            double t_now = (framecount - 1) / fps;
            ang_buf.push_back(current_unwrapped); t_buf.push_back(t_now); idx_buf.push_back(framecount);
            if (ang_buf.size() > 3) { ang_buf.pop_front(); t_buf.pop_front(); idx_buf.pop_front(); }

            double w_frame = 0; long frame_of_w = framecount;
            if (ang_buf.size() == 3) {
                w_frame = (ang_buf[2] - ang_buf[0]) / (t_buf[2] - t_buf[0]);
                frame_of_w = idx_buf[1];
                t_omega.push_back(t_buf[1]); w_obs.push_back(w_frame); frame_omega.push_back(frame_of_w);
            }
        }

        // 4) 绘制识别标记(目标中心/旋转中心/连线/角度/角速度)
        if (ball.x >= 0) {
            circle(frame, ball, 8, Scalar(255, 255, 255), 2);
            circle(frame, ball, 2, Scalar(0, 0, 255), -1);
            circle(frame, center, 4, Scalar(0, 255, 255), -1);
            line(frame, center, ball, Scalar(255, 0, 0), 1);
            putText(frame, "theta=" + to_string(current_unwrapped), Point(20, 30),
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
        }
        imshow("tracking", frame);
        if (!vwOpen) { vw.open(OUTDIR + "tracking_overlay.mp4", VideoWriter::fourcc('m','p','4','v'), fps, frame.size()); vwOpen = true; }
        if (vw.isOpened()) vw.write(frame);
        if (waitKey(1) == 'q') break;
    }
    if (vw.isOpened()) vw.release();
    destroyAllWindows();

    // 5) 拟合参数并精修
    double b, A, Omega, phi;
    estimateInitialValues(t_omega, w_obs, b, A, Omega, phi);
    refineByCeres(t_omega, w_obs, b, A, Omega, phi);
    while (phi >= M_PI) phi -= 2 * M_PI;               // 归一化到 [-π, π)
    while (phi < -M_PI) phi += 2 * M_PI;

    // 6) 计算拟合值、残差与 RMSE
    size_t N = t_omega.size();
    vector<double> w_fit(N), resid(N);
    double sse = 0, maxabs = 0;
    for (size_t i = 0; i < N; i++) {
        w_fit[i] = b + A * sin(Omega * t_omega[i] + phi);
        resid[i] = w_obs[i] - w_fit[i];
        sse += resid[i] * resid[i];
        maxabs = max(maxabs, fabs(resid[i]));
    }
    double rmse = sqrt(sse / N);

    // 7) 输出图表
    double xmin = t_omega.front(), xmax = t_omega.back();
    double ymn, ymx; { vector<double> all; all.insert(all.end(), w_obs.begin(), w_obs.end());
                       all.insert(all.end(), w_fit.begin(), w_fit.end()); minmaxOf(all, ymn, ymx); }
    Plot pFit(1000, 600, xmin, xmax, ymn, ymx, "Task2 fit: omega(t)", "time (s)", "omega (rad/s)");
    pFit.points(t_omega, w_obs, Scalar(255, 0, 0));     // 观测点(蓝)
    pFit.curve(t_omega, w_fit, Scalar(0, 0, 255), 2);   // 拟合曲线(红)
    pFit.save(OUTDIR + "fit_comparison.png");
    pFit.save(OUTDIR + "angular_velocity.png");         // 角速度曲线(可同图)

    double rmn, rmx; minmaxOf(resid, rmn, rmx, true);
    Plot pRes(1000, 600, xmin, xmax, rmn, rmx, "Task2 residuals", "time (s)", "residual (rad/s)");
    pRes.zero();
    pRes.curve(t_omega, resid, Scalar(0, 128, 0), 1);
    pRes.save(OUTDIR + "residuals.png");

    // 8) 写出结果说明 markdown
    ofstream md(MD_FILE);
    md << "# 任务2 结果：合成旋转视频参数拟合\n\n";
    md << "## 模型\nω(t) = b + A·sin(Ωt + φ)   (rad/s)\n\n";
    md << "## 方法\n";
    md << "1. HSV 阈值(85~95) 提取青色目标，取最大轮廓质心作为目标中心。\n";
    md << "2. 旋转中心取任务已知值 (480,360)；角度 θ=atan2(cy-y, x-cx)，逆时针为正，逐帧解缠。\n";
    md << "3. 角速度用相邻帧中心差分 ω=Δθ/Δt；对 ω(t) 拟合。\n";
    md << "4. 初值：DFT 找 ω 主频得 Ω，再用线性最小二乘求 b/A/φ；最后 Ceres 精修四参数。\n\n";
    md << "## 参数估计\n";
    md << "- b = " << b << " rad/s (平均角速度)\n";
    md << "- A = " << A << " rad/s (振幅)\n";
    md << "- Ω = " << Omega << " rad/s (速度变化频率，周期 T=2π/Ω = " << 2 * M_PI / Omega << " s)\n";
    md << "- φ = " << phi << " rad (归一化到 [-π, π))\n\n";
    md << "## 误差指标（角速度，rad/s）\n";
    md << "- RMSE = " << rmse << " rad/s\n";
    md << "- 最大绝对残差 = " << maxabs << " rad/s\n";
    md << "- 有效样本数 = " << N << "\n";
    md << "- 参与计算帧范围 = [" << frame_omega.front() << ", " << frame_omega.back() << "]\n";
    md << "- 时间范围 = [" << t_omega.front() << ", " << t_omega.back() << "] s\n\n";
    md << "## 输出文件\n";
    md << "- result/task2_fit/tracking_overlay.mp4\n";
    md << "- result/task2_fit/fit_comparison.png\n";
    md << "- result/task2_fit/angular_velocity.png（与 fit_comparison 同图）\n";
    md << "- result/task2_fit/residuals.png\n";
    md.close();

    cout << "参数 b=" << b << " A=" << A << " Omega=" << Omega << " phi=" << phi << endl;
    cout << "RMSE=" << rmse << " rad/s, 样本=" << N
         << ", 帧[" << frame_omega.front() << "," << frame_omega.back() << "]" << endl;
    cout << "结果已写入 " << OUTDIR << " 与 " << MD_FILE << endl;
    cap.release();
    return 0;
}
