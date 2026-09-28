// 海康机器人 MVS SDK 的 ROS 2 Humble 封装节点
// 功能：枚举/连接相机(按IP或序列号)、采集并发布 sensor_msgs/Image、动态配置相机参数、断线重连
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <MvCameraControl.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace {

// MVS 像素格式 -> (ROS encoding, 每像素字节数)
bool pixelTypeToRos(unsigned int enType, std::string& enc, int& bpp) {
    switch (enType) {
        case PixelType_Gvsp_Mono8:       enc = "mono8";       bpp = 1; return true;
        case PixelType_Gvsp_RGB8_Packed: enc = "rgb8";        bpp = 3; return true;
        case PixelType_Gvsp_BGR8_Packed: enc = "bgr8";        bpp = 3; return true;
        case PixelType_Gvsp_BayerGR8:    enc = "bayer_grbg8"; bpp = 1; return true;
        case PixelType_Gvsp_BayerRG8:    enc = "bayer_rggb8"; bpp = 1; return true;
        case PixelType_Gvsp_BayerGB8:    enc = "bayer_gbrg8"; bpp = 1; return true;
        case PixelType_Gvsp_BayerBG8:    enc = "bayer_bggr8"; bpp = 1; return true;
        default: return false;
    }
}

bool isBayer(unsigned int t) {
    return t == PixelType_Gvsp_BayerGR8 || t == PixelType_Gvsp_BayerRG8 ||
           t == PixelType_Gvsp_BayerGB8 || t == PixelType_Gvsp_BayerBG8;
}

std::string ipToString(unsigned int ip) {
    return std::to_string((ip >> 24) & 0xff) + "." + std::to_string((ip >> 16) & 0xff) + "." +
           std::to_string((ip >> 8) & 0xff) + "." + std::to_string(ip & 0xff);
}

}  // namespace

class HikCameraNode : public rclcpp::Node {
public:
    HikCameraNode() : rclcpp::Node("hik_camera") {
        serial_number_ = declare_parameter<std::string>("serial_number", "");
        camera_ip_     = declare_parameter<std::string>("camera_ip", "");
        topic_         = declare_parameter<std::string>("topic", "image_raw");
        frame_id_      = declare_parameter<std::string>("frame_id", "camera");
        exposure_time_ = declare_parameter<double>("exposure_time", -1.0);
        gain_          = declare_parameter<double>("gain", -1.0);
        frame_rate_    = declare_parameter<double>("frame_rate", -1.0);
        pixel_format_  = declare_parameter<std::string>("pixel_format", "");

        // Reliable QoS：同时兼容 rviz2（Reliable）与 Best-Effort 订阅者
        pub_ = create_publisher<sensor_msgs::msg::Image>(topic_, rclcpp::QoS(rclcpp::KeepLast(10)));
        param_cb_ = add_on_set_parameters_callback(
            std::bind(&HikCameraNode::onSetParameters, this, std::placeholders::_1));

        MV_CC_Initialize();
        connect();
        reconnect_timer_ = create_wall_timer(1s, std::bind(&HikCameraNode::checkReconnect, this));
    }

    ~HikCameraNode() override {
        std::lock_guard<std::mutex> lk(mtx_);
        closeLocked();
        MV_CC_Finalize();
    }

private:
    // ---- 参数修改：逐个校验并下发到 SDK，仅成功才提交到成员（失败回滚）----
    rcl_interfaces::msg::SetParametersResult onSetParameters(
        const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        std::lock_guard<std::mutex> lk(mtx_);
        if (!handle_) { res.successful = false; res.reason = "相机未连接"; return res; }

        std::string why;
        for (const auto& p : params) {
            const std::string& n = p.get_name();
            bool ok = true;
            if (n == "exposure_time") {
                double v = p.as_double(); ok = setExposure(v, why); if (ok) exposure_time_ = v;
            } else if (n == "gain") {
                double v = p.as_double(); ok = setGain(v, why); if (ok) gain_ = v;
            } else if (n == "frame_rate") {
                double v = p.as_double(); ok = setFrameRate(v, why); if (ok) frame_rate_ = v;
            } else if (n == "pixel_format") {
                std::string v = p.as_string(); ok = setPixelFormat(v, why); if (ok) pixel_format_ = v;
            } else {
                continue;  // 其它参数（topic/frame_id 等）运行中不接受修改
            }
            if (!ok) { res.successful = false; res.reason = why; return res; }
        }
        return res;
    }

    // ---- 各参数的校验 + 下发（调用方负责提交成员）----
    bool setExposure(double v, std::string& why) {
        MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);  // 手动曝光前关闭自动
        MVCC_FLOATVALUE fv; std::memset(&fv, 0, sizeof(fv));
        if (MV_CC_GetFloatValue(handle_, "ExposureTime", &fv) == MV_OK &&
            (v < fv.fMin || v > fv.fMax)) {
            why = "exposure_time 超出范围 [" + std::to_string(fv.fMin) + "," + std::to_string(fv.fMax) + "]";
            return false;
        }
        if (MV_CC_SetFloatValue(handle_, "ExposureTime", (float)v) != MV_OK) { why = "设置 ExposureTime 失败"; return false; }
        return true;
    }
    bool setGain(double v, std::string& why) {
        MV_CC_SetEnumValue(handle_, "GainAuto", 0);      // 手动增益前关闭自动
        MVCC_FLOATVALUE fv; std::memset(&fv, 0, sizeof(fv));
        if (MV_CC_GetFloatValue(handle_, "Gain", &fv) == MV_OK && (v < fv.fMin || v > fv.fMax)) {
            why = "gain 超出范围 [" + std::to_string(fv.fMin) + "," + std::to_string(fv.fMax) + "]";
            return false;
        }
        if (MV_CC_SetFloatValue(handle_, "Gain", (float)v) != MV_OK) { why = "设置 Gain 失败"; return false; }
        return true;
    }
    bool setFrameRate(double v, std::string& why) {
        if (v <= 0) { why = "frame_rate 必须 > 0"; return false; }
        MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
        if (MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", (float)v) != MV_OK) {
            why = "设置 AcquisitionFrameRate 失败"; return false;
        }
        return true;
    }
    bool setPixelFormat(const std::string& s, std::string& why) {
        int type;
        if (s == "Mono8") type = PixelType_Gvsp_Mono8;
        else if (s == "RGB8_Packed") type = PixelType_Gvsp_RGB8_Packed;
        else if (s == "BGR8_Packed") type = PixelType_Gvsp_BGR8_Packed;
        else if (s == "BayerRG8") type = PixelType_Gvsp_BayerRG8;
        else { why = "不支持的 pixel_format: " + s; return false; }

        bool was = connected_;
        if (was) MV_CC_StopGrabbing(handle_);            // 部分格式切换需先停流
        int ret = MV_CC_SetEnumValue(handle_, "PixelFormat", (unsigned)type);
        if (was) MV_CC_StartGrabbing(handle_);
        if (ret != MV_OK) { why = "切换 PixelFormat 失败(相机可能不支持该格式)"; return false; }
        last_frame_time_ = now();                        // 避免停止/恢复造成的误触发重连
        return true;
    }

    // ---- 连接相机 ----
    void connect() {
        std::lock_guard<std::mutex> lk(mtx_);
        if (handle_) return;

        MV_CC_DEVICE_INFO_LIST list;
        std::memset(&list, 0, sizeof(list));
        int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list);
        if (ret != MV_OK || list.nDeviceNum == 0) {
            RCLCPP_WARN(get_logger(), "未发现相机 (ret=0x%x)", ret);
            return;
        }

        MV_CC_DEVICE_INFO* dev = nullptr;
        for (unsigned i = 0; i < list.nDeviceNum; i++) {
            auto* d = list.pDeviceInfo[i];
            if (!d) continue;
            // 打印枚举到的设备，演示“自动发现”
            if (d->nTLayerType == MV_USB_DEVICE) {
                RCLCPP_INFO(get_logger(), "发现设备[%u] USB: 型号=%s 序列号=%s", i,
                            reinterpret_cast<const char*>(d->SpecialInfo.stUsb3VInfo.chModelName),
                            reinterpret_cast<const char*>(d->SpecialInfo.stUsb3VInfo.chSerialNumber));
            } else if (d->nTLayerType == MV_GIGE_DEVICE) {
                RCLCPP_INFO(get_logger(), "发现设备[%u] GigE: 型号=%s 序列号=%s IP=%s", i,
                            reinterpret_cast<const char*>(d->SpecialInfo.stGigEInfo.chModelName),
                            reinterpret_cast<const char*>(d->SpecialInfo.stGigEInfo.chSerialNumber),
                            ipToString(d->SpecialInfo.stGigEInfo.nCurrentIp).c_str());
            }
            if (!serial_number_.empty()) {
                const char* sn = (d->nTLayerType == MV_USB_DEVICE)
                                     ? reinterpret_cast<const char*>(d->SpecialInfo.stUsb3VInfo.chSerialNumber)
                                     : reinterpret_cast<const char*>(d->SpecialInfo.stGigEInfo.chSerialNumber);
                if (serial_number_ == sn) { dev = d; break; }
            }
            if (!camera_ip_.empty() && d->nTLayerType == MV_GIGE_DEVICE &&
                ipToString(d->SpecialInfo.stGigEInfo.nCurrentIp) == camera_ip_) { dev = d; break; }
        }
        if (!dev) dev = list.pDeviceInfo[0];

        if (MV_CC_CreateHandle(&handle_, dev) != MV_OK) { handle_ = nullptr; return; }
        if (MV_CC_OpenDevice(handle_) != MV_OK) { MV_CC_DestroyHandle(handle_); handle_ = nullptr; return; }

        if (dev->nTLayerType == MV_GIGE_DEVICE) {
            int ps = MV_CC_GetOptimalPacketSize(handle_);
            if (ps > 0) MV_CC_SetIntValueEx(handle_, "GevSCPSPacketSize", ps);
        }

        MV_CC_SetEnumValue(handle_, "TriggerMode", 0);   // 关闭触发，连续采集

        // 恢复/应用已配置参数
        std::string why;
        if (exposure_time_ >= 0) setExposure(exposure_time_, why);
        if (gain_ >= 0) setGain(gain_, why);
        if (frame_rate_ >= 0) setFrameRate(frame_rate_, why);
        if (!pixel_format_.empty()) setPixelFormat(pixel_format_, why);

        MV_CC_RegisterImageCallBackEx2(handle_, &HikCameraNode::imageCallback, this, true);
        if (MV_CC_StartGrabbing(handle_) != MV_OK) {
            RCLCPP_ERROR(get_logger(), "StartGrabbing 失败");
            closeLocked();
            return;
        }
        connected_ = true;
        last_frame_time_ = now();
        RCLCPP_INFO(get_logger(), "相机已连接, 发布话题 %s", pub_->get_topic_name());
    }

    void closeLocked() {
        if (handle_) {
            MV_CC_StopGrabbing(handle_);
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
        }
        connected_ = false;
    }

    // ---- 采集回调：SDK 线程内直接发布 ----
    static void __stdcall imageCallback(MV_FRAME_OUT* pFrame, void* pUser, bool /*bAutoFree*/) {
        auto* self = static_cast<HikCameraNode*>(pUser);
        if (!pFrame || !self) return;
        auto& info = pFrame->stFrameInfo;
        const void* data = pFrame->pBufAddr;
        std::string enc; int bpp = 0; size_t bytes = 0;

        if (isBayer(info.enPixelType) && self->handle_) {
            // Bayer -> RGB8 去马赛克，便于 rviz2 显示彩色
            unsigned w = info.nWidth, h = info.nHeight;
            self->cvtBuf_.resize((size_t)w * h * 3);
            MV_CC_PIXEL_CONVERT_PARAM_EX cvt;
            std::memset(&cvt, 0, sizeof(cvt));
            cvt.nWidth = w; cvt.nHeight = h;
            cvt.enSrcPixelType = (MvGvspPixelType)info.enPixelType;
            cvt.pSrcData = (unsigned char*)pFrame->pBufAddr;
            cvt.nSrcDataLen = info.nFrameLen;
            cvt.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
            cvt.pDstBuffer = self->cvtBuf_.data();
            cvt.nDstBufferSize = (unsigned)self->cvtBuf_.size();
            if (MV_CC_ConvertPixelTypeEx(self->handle_, &cvt) != MV_OK) return;
            enc = "rgb8"; bpp = 3; data = self->cvtBuf_.data(); bytes = (size_t)w * h * 3;
        } else {
            if (!pixelTypeToRos(info.enPixelType, enc, bpp)) return;
            bytes = (size_t)info.nWidth * info.nHeight * bpp;
        }

        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = self->now();
        msg->header.frame_id = self->frame_id_;
        msg->height = info.nHeight;
        msg->width = info.nWidth;
        msg->encoding = enc;
        msg->is_bigendian = false;
        msg->step = (sensor_msgs::msg::Image::_step_type)(info.nWidth * bpp);
        msg->data.resize(bytes);
        std::memcpy(msg->data.data(), data, bytes);
        self->pub_->publish(std::move(msg));
        self->last_frame_time_ = self->now();
    }

    // ---- 断线重连：超过 3s 无帧则重连（重连后恢复参数）----
    // 注意：connect() 内部会锁 mtx_，此处不可持锁调用，避免死锁；用 reconnecting_ 防重入
    void checkReconnect() {
        if (!connected_) {
            if (reconnecting_.exchange(true)) return;
            connect();
            reconnecting_ = false;
            return;
        }
        if ((now() - last_frame_time_).seconds() <= 3.0) return;
        if (reconnecting_.exchange(true)) return;
        RCLCPP_WARN(get_logger(), "超过 3s 未收到图像，尝试重连...");
        {
            std::lock_guard<std::mutex> lk(mtx_);
            closeLocked();
        }
        connect();
        reconnecting_ = false;
    }

    // ---- 成员 ----
    std::string serial_number_, camera_ip_, topic_, frame_id_, pixel_format_;
    double exposure_time_{-1}, gain_{-1}, frame_rate_{-1};
    void* handle_{nullptr};
    std::atomic<bool> connected_{false};
    std::atomic<bool> reconnecting_{false};
    rclcpp::Time last_frame_time_;
    std::mutex mtx_;
    std::vector<uint8_t> cvtBuf_;  // Bayer->RGB 去马赛克缓冲（仅采集线程使用）
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // 多线程 executor：避免 SDK 阻塞调用（连接/取流）卡住参数服务
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<HikCameraNode>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
