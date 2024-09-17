#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include <publisher/euroc_util.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <popl.hpp>

// added to read cmd args
#include <vector>
#include <string>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto img_dir_path = op.add<popl::Value<std::string>>("i", "img-dir", "directory path which contains images");
    auto img_fps = op.add<popl::Value<unsigned int>>("", "fps", "FPS of images", 20);

    try
    {
        op.parse(argc, argv);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set())
    {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!img_dir_path->is_set())
    {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // initialize this node
    std::string name_space = std::string("/") + std::string(argv[3]);
    auto node = std::make_shared<rclcpp::Node>("image_publisher");
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    custom_qos.depth = 1;

    auto publisher_left = image_transport::create_publisher(node.get(), name_space + "/stereo_camera/left/image_rect_color", custom_qos);
    auto publisher_right = image_transport::create_publisher(node.get(), name_space + "/stereo_camera/right/image_rect_color", custom_qos);
    // camera info publisher
    auto left_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(name_space + "/stereo_camera/left/camera_info", 10);
    auto right_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(name_space + "/stereo_camera/right/camera_info", 10);

    auto camera_info_left = std::make_shared<sensor_msgs::msg::CameraInfo>();
    auto camera_info_right = std::make_shared<sensor_msgs::msg::CameraInfo>();
    camera_info_left->header.frame_id = "camera_link";
    camera_info_left->height = 480;
    camera_info_left->width = 752;
    camera_info_left->distortion_model = "plumb_bob";
    camera_info_left->d = std::vector<double>{-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
    camera_info_left->k = std::array<double, 9>{458.654, 0.0, 367.215, 0.0, 457.296, 248.375, 0.0, 0.0, 1.0};
    camera_info_left->r = std::array<double, 9>{0.99996635, -0.00142274, 0.00807958,
                                                0.00136574, 0.99997418, 0.00705563,
                                                -0.00808941, -0.00704436, 0.99994247};
    camera_info_left->p = std::array<double, 12>{436.24429565, 0, 364.44123459, 0,
                                                 0, 436.24429565, 256.95167542, 0,
                                                 0, 0, 1, 0};

    camera_info_right->header.frame_id = "camera_right";
    camera_info_right->height = 480;
    camera_info_right->width = 752;
    camera_info_right->distortion_model = "plumb_bob";
    camera_info_right->d = std::vector<double>{-0.28368365, 0.07451284, -0.00010473, -3.55590700e-05};
    camera_info_right->k = std::array<double, 9>{457.587, 0.0, 379.999, 0.0, 456.134, 255.238, 0.0, 0.0, 1.0};
    camera_info_right->r = std::array<double, 9>{0.99996335, -0.00362581, 0.00775544,
                                                 0.0036804, 0.99996848, -0.00703585,
                                                 -0.00772969, 0.00706413, 0.99994517};
    camera_info_right->p = std::array<double, 12>{436.24429565, 0, 364.44123459, -48.02083073,
                                                  0, 436.24429565, 256.95167542, 0,
                                                  0, 0, 1, 0};

    sensor_msgs::msg::Image::SharedPtr msg_left, msg_right;
    rclcpp::WallRate pub_rate(img_fps->value());
    rclcpp::executors::SingleThreadedExecutor exec;

    exec.add_node(node);

    euroc_sequence sequence(img_dir_path->value());
    const auto frames = sequence.get_frames();

    for (unsigned int i = 0; i < frames.size(); ++i)
    {

        const auto &frame = frames.at(i);
        const auto img_left = cv::imread(frame.left_img_path_, cv::IMREAD_COLOR);
        const auto img_right = cv::imread(frame.right_img_path_, cv::IMREAD_COLOR);
        const int sec = frame.sec_;
        const unsigned long nsec = frame.nsec_;

        msg_left = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_left).toImageMsg();
        // msg_left->header.stamp = node->now();
        // using index instead of ros time
        msg_left->header.stamp = rclcpp::Time(sec, nsec);
        msg_left->header.frame_id = "camera_link";

        msg_right = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_right).toImageMsg();
        msg_right->header.stamp = msg_left->header.stamp;
        msg_right->header.frame_id = "camera_right";

        camera_info_left->header.stamp = msg_left->header.stamp;
        camera_info_right->header.stamp = msg_left->header.stamp;
        left_info_pub->publish(*camera_info_left);
        right_info_pub->publish(*camera_info_right);
        publisher_left.publish(msg_left);
        publisher_right.publish(msg_right);

        exec.spin_some();
        pub_rate.sleep();
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
