#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <baxter_core_msgs/CameraSettings.h>
#include <baxter_core_msgs/CameraControl.h>
#include <baxter_core_msgs/OpenCamera.h>
#include <baxter_core_msgs/CloseCamera.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>


class Camera
{
public:
    enum CameraId {LEFT_HAND, HEAD, RIGHT_HAND};
    enum RequestType {REQUEST_SELECTED_SHAPE, REQUEST_SHAPES};
    enum RequestStatus {STATUS_AVAILABLE, STATUS_REQUESTING, STATUS_IN_PROGRESS};

    Camera(int id, ros::NodeHandle nh);
    ~Camera();
    void callback(const sensor_msgs::ImageConstPtr &msg);
    void request(RequestType request_type);
    bool isResultAvailable();
    std::vector<std::vector<cv::Point> >* getResult();

private:
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img_gray, img_hsv;
    ros::NodeHandle nh;
    baxter_core_msgs::OpenCamera open_camera;
    baxter_core_msgs::CameraControl camera_control;
    image_transport::ImageTransport *it;
    image_transport::Subscriber is;
    std::vector<std::vector<cv::Point> > contours, *request_result;
    int hmin, hmax, vmin, threshold1, threshold2, minarea;
    RequestType request_type;
    RequestStatus request_status;
};
