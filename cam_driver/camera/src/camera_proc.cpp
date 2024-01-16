#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/mat.hpp>
#include "image_geometry/pinhole_camera_model.h"
#include <iostream>

using std::placeholders::_1;


class CameraProc : public rclcpp::Node
{
public:
    CameraProc()
        : Node("camera_proc")
        , img_raw_sub_{}
        , img_proc_pub_{}
        , camera_model_{}
    {
        declare_parameter("image_height", 224);
        declare_parameter("fov", 120.0);
        declare_parameter("aperture_width", 6.287);
        declare_parameter("aperture_height", 4.712);

        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
                               "camera_info",
                               1,
                               std::bind(&CameraProc::camera_info_callback, this, _1)
                           );
        img_proc_pub_ = image_transport::create_publisher(
                            this,
                            "image_proc",
                            rmw_qos_profile_sensor_data
                        );
        img_raw_sub_ = image_transport::create_subscription(
                           this,
                           "image_raw",
                           std::bind(&CameraProc::img_raw_callback, this, _1),
                           "raw",
                           rmw_qos_profile_sensor_data
                       );

        RCLCPP_INFO(get_logger(), "Initialized");
    }

private:
    image_transport::Subscriber img_raw_sub_;
    image_transport::Publisher img_proc_pub_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    image_geometry::PinholeCameraModel camera_model_;
    cv::Mat map1_;
    cv::Mat map2_;
    cv::Mat proc_;

    void img_raw_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        if (map1_.empty())
        {
            RCLCPP_INFO(get_logger(), "Received image but waiting for camera_info...");
            return;
        }

        auto cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        float dt_rx = (get_clock()->now() - msg->header.stamp).nanoseconds() / 1e9;
        cv::remap(cv_ptr->image, proc_, map1_, map2_, cv::INTER_LINEAR, CV_HAL_BORDER_CONSTANT);
        float dt_proc = (get_clock()->now() - msg->header.stamp).nanoseconds() / 1e9;

        cv_bridge::CvImage cv_img(msg->header, msg->encoding, proc_);
        img_proc_pub_.publish(std::move(cv_img.toImageMsg()));
        RCLCPP_DEBUG(get_logger(), "Processed img rx %f proc %f", dt_rx, dt_proc);
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (camera_model_.fromCameraInfo(msg))
        {
            const auto img_size = camera_model_.fullResolution();
            const float scale = (float) get_parameter("image_height").as_int() / img_size.height;
            const auto new_size = cv::Size(std::round(img_size.width * scale), std::round(img_size.height * scale));
            const auto r = cv::Mat::eye(3, 3, CV_32F);
            const auto k = camera_model_.intrinsicMatrix();
            const auto d = camera_model_.distortionCoeffs();
            float balance = 1.0;
            auto p = cv::Matx33d{};

            for (int i = 0; i < 100; i++)
            {
                cv::fisheye::estimateNewCameraMatrixForUndistortRectify(k, d, img_size, r, p, balance, new_size);
                p(1, 2) = (float) new_size.height / 2;
                p(0, 2) = (float) new_size.width / 2;

                double fovx, fovy, focal_length, aspect_ratio;
                auto principal_point = cv::Point2d{};
                cv::calibrationMatrixValues(
                    p,
                    new_size,
                    get_parameter("aperture_width").as_double(),
                    get_parameter("aperture_height").as_double(),
                    fovx, fovy,
                    focal_length, principal_point, aspect_ratio
                );
                balance *= get_parameter("fov").as_double() / std::min(fovx, fovy);
                std::ostringstream os;
                os << "P" << p << "\n" << fovx << "," << fovy << "," << balance;
                RCLCPP_DEBUG(get_logger(), "%s", os.str().c_str());
            }

            cv::fisheye::initUndistortRectifyMap(k, d, r, p, new_size, CV_32FC1, map1_, map2_);
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraProc>());
    rclcpp::shutdown();
    return 0;
}
