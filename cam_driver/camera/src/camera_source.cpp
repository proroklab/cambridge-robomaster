#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_converter.h"
#include <jetson-utils/videoSource.h>


using std::placeholders::_1;


class CameraSource : public rclcpp::Node
{
public:
    CameraSource()
        : Node("camera_source")
        , image_cvt_{}
        , camera_info_manager_{this}
        , image_publisher_{}
    {
        declare_parameter("camera_idx", 0);
        declare_parameter("camera_info_url", "");
        declare_parameter("camera_info_name", "camera");
        declare_parameter("codec", "");
        declare_parameter("width", 1920);
        declare_parameter("height", 1080);
        declare_parameter("framerate", 30.0);
        declare_parameter("loop", 0);

        videoOptions video_options;
        std::string codec_str;
        std::string camera_info_url, camera_info_name;
        int camera_idx;
        get_parameter("camera_idx", camera_idx);
        get_parameter("camera_info_name", camera_info_name);
        get_parameter("camera_info_url", camera_info_url);
        get_parameter("codec", codec_str);
        get_parameter("width", video_options.width);
        get_parameter("height", video_options.height);
        get_parameter("framerate", video_options.frameRate);
        get_parameter("loop", video_options.loop);

        std::ostringstream resource_str;
        resource_str << "csi://" << camera_idx;

        if ( codec_str.size() != 0 )
        {
            video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());
        }

        stream_ = videoSource::Create(resource_str.str().c_str(), video_options);

        if ( !stream_ )
        {
            throw std::runtime_error("failed to open video source");
        }

        camera_info_manager_.setCameraName(camera_info_name);

        if (camera_info_manager_.validateURL(camera_info_url))
        {
            camera_info_manager_.loadCameraInfo(camera_info_url);
            RCLCPP_INFO(get_logger(), "Loaded camera calibration from %s", camera_info_url.c_str());
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Camera info url '%s' is not valid, missing 'file://' prefix?", camera_info_url.c_str());

        }

        camera_info_manager::CameraInfo cur_cinfo = camera_info_manager_.getCameraInfo();
        camera_info_ = sensor_msgs::msg::CameraInfo(cur_cinfo);

        rclcpp::PublisherOptions pub_options;

        image_publisher_ = image_transport::create_publisher(this, "image_raw", rmw_qos_profile_sensor_data);

        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);

        timer_frames_ = rclcpp::create_timer(
                            this,
                            get_clock(),
                            rclcpp::Duration(0, (int) (1.f / video_options.frameRate) * 1e9),
                            std::bind(&CameraSource::timer_frames_callback, this)
                        );

        RCLCPP_INFO(get_logger(), "Initialized");
    }

private:
    void timer_frames_callback(void)
    {
        imageConverter::PixelType* nextFrame = NULL;

        if ( !stream_->Capture(&nextFrame, 1000) )
        {
            RCLCPP_ERROR(get_logger(), "failed to capture next frame");
            return;
        }

        auto time_stamp = get_clock()->now();

        if ( !image_cvt_.Resize(stream_->GetWidth(), stream_->GetHeight(), imageConverter::ROSOutputFormat) )
        {
            RCLCPP_ERROR(get_logger(), "failed to resize camera image converter");
            return;
        }

        sensor_msgs::Image msg;
        msg.header.stamp = time_stamp;

        if ( !image_cvt_.Convert(msg, imageConverter::ROSOutputFormat, nextFrame) )
        {
            RCLCPP_INFO(get_logger(), "failed to convert video stream frame to sensor_msgs::Image");
            return;
        }

        camera_info_.header.stamp = time_stamp;
        camera_info_pub_->publish(std::move(camera_info_));

        image_publisher_.publish(std::move(msg));
        RCLCPP_DEBUG(get_logger(), "published %ux%u video frame", stream_->GetWidth(), stream_->GetHeight());
    }

    rclcpp::TimerBase::SharedPtr timer_frames_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

    camera_info_manager::CameraInfoManager camera_info_manager_;
    image_transport::Publisher image_publisher_;

    sensor_msgs::msg::CameraInfo camera_info_;

    videoSource* stream_;
    imageConverter image_cvt_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraSource>());
    rclcpp::shutdown();
    return 0;
}
