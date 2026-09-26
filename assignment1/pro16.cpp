// 任务1：OpenCV 图像处理（灰度 / HSV / 滤波 / 红色掩膜 / 形态学 / 轮廓）
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    // 输入默认取 resources/test_image.jpg，也可用命令行参数指定
    const string imgPath = (argc > 1) ? argv[1] : "resources/test_image.jpg";
    const string outDir = "result/task1_images/";
    filesystem::create_directories(outDir);

    Mat img = imread(imgPath);
    if (img.empty()) { cerr << "fail: " << imgPath << endl; return 1; }

    // 灰度图
    Mat gray; cvtColor(img, gray, COLOR_BGR2GRAY);
    imwrite(outDir + "gray.png", gray);

    // HSV 三分量
    Mat hsv, ch[3]; cvtColor(img, hsv, COLOR_BGR2HSV);
    split(hsv, ch);
    imwrite(outDir + "hsv_h.png", ch[0]);
    imwrite(outDir + "hsv_s.png", ch[1]);
    imwrite(outDir + "hsv_v.png", ch[2]);

    // 红色掩膜（H 环绕：0~10 与 170~179）
    Mat masklow, maskhigh, mask;
    inRange(hsv, Scalar(0, 100, 100), Scalar(10, 255, 255), masklow);
    inRange(hsv, Scalar(170, 100, 255), Scalar(179, 255, 255), maskhigh);
    bitwise_or(masklow, maskhigh, mask);
    imwrite(outDir + "red_mask.png", mask);

    // 形态学（5x5 矩形核）
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat eroded, dilated, opened, closed;
    erode(mask, eroded, kernel);                      imwrite(outDir + "erode.png", eroded);
    dilate(mask, dilated, kernel);                    imwrite(outDir + "dilate.png", dilated);
    morphologyEx(mask, opened, MORPH_OPEN, kernel);   imwrite(outDir + "open.png", opened);
    morphologyEx(mask, closed, MORPH_CLOSE, kernel);  imwrite(outDir + "close.png", closed);

    // 轮廓 + 外接矩形 + 面积标注（面积阈值 100）
    vector<vector<Point>> contours;
    findContours(opened, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    Mat result = img.clone();
    int valid = 0; double total = 0;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area < 100) continue;
        valid++; total += area;
        drawContours(result, contours, (int)i, Scalar(0, 255, 0), 2);
        Rect r = boundingRect(contours[i]);
        rectangle(result, r, Scalar(255, 0, 0), 2);
        putText(result, "A:" + to_string((int)area), Point(r.x, r.y - 5),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
    }
    imwrite(outDir + "contours_boxes.png", result);
    cout << "valid_contours=" << valid << " total_area=" << total << endl;

    // 滤波：均值 / 高斯 / 中值
    Mat meanimg, gaussianimg, medianimg;
    blur(img, meanimg, Size(5, 5));
    GaussianBlur(img, gaussianimg, Size(5, 5), 1.5);
    medianBlur(img, medianimg, 5);
    imwrite(outDir + "mean_filter.png", meanimg);
    imwrite(outDir + "gaussian_filter.png", gaussianimg);
    imwrite(outDir + "median_filter.png", medianimg);

    cout << "任务1 结果已写入 " << outDir << endl;
    return 0;
}
