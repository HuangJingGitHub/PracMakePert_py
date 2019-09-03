/* 

    HUANG Jing
    haungjing@mae.cuhk.edu.hk
    Partially based on correll lab's script http://104.236.202.16/?p=3064.
    Track red feature shape contour in an image.

*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo.hpp>    // denoising module
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <vector>
#include <math.h>
#include "dvrk_retraction/visual_feedback.h"
#include "dvrk_retraction/visual_info_srv.h"

// namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "New OpenCV Image";

class ImageConverter
{
    ros::NodeHandle nh_;
    ros::Publisher image_info_pub = nh_.advertise<dvrk_retraction::visual_feedback>("visual_info", 1000);
    ros::ServiceServer visual_info_service = nh_.advertiseService("visual_info_srv", &ImageConverter::get_visual_info_srv, this);
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    float focal_x = 683;
    float focal_y = 683;
    float depth = 58;

public:
    std::vector<float> class_centroid;
    std::vector<float> class_rotate_origin;
    std::vector<float> class_principal_axis;
    float class_rotate_angle;
    std::vector<float> class_target_region_center;
    float class_target_region_radius;
  
public:
    ImageConverter(char* ros_image_stream): it_(nh_)
    {
        image_pub_ = it_.advertise("image_processed", 1);
        image_sub_ = it_.subscribe(ros_image_stream, 1, &ImageConverter::imageCb, this);

        cv::namedWindow(WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        //std::cout << "I am trying" << std::endl;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        //std::cout << "Try failed" << std::endl;
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

   // Find countors of red shape
    cv::Point imageOrigin(320, 240),
              xAxisEnd(220, 240), yAxisEnd(320, 340);
    cv::Scalar redLow(0, 173, 152), redHigh(10, 255, 255), blackLow(0, 0, 0), blackHigh(149, 218, 71);
    cv::Scalar towelRedLow(0, 152, 124), towelRedHigh(12, 255, 255);
    std::vector< std::vector<cv::Point>> contours;
    cv::Mat trackedImage, trackedImage_denoised, dst, trackedImage_undist;
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 683.9731, 0, 320., 0, 683.9731, 240, 0, 0, 1);
    cv::Mat distCoef = (cv::Mat_<double>(1, 5) << -0.7175, 2.8528, 0, 0, -5.1548);
    
    cv::cvtColor(cv_ptr->image, trackedImage, CV_BGR2HSV);
    // cv::fastNlMeansDenoisingColored(trackedImage,trackedImage_denoised, 3, 3, 7, 21); // This function costs much time.
    cv::blur(trackedImage, trackedImage, cv::Size(3, 3));   // blurring is very important to get rid of noise.
    cv::inRange(trackedImage, towelRedLow, towelRedHigh, dst);
    //cv::imshow("Binary Image", dst);

    /* Use bgr difference to generate binary image rather than inRange()
    cv::Mat bgr_thr = cv::Mat::zeros(trackedImage.size(), CV_8UC1);
    int Rows = bgr_thr.rows, Cols = bgr_thr.cols;
    for (int r = 0; r < Rows; r ++)
        for (int c = 0; c < Cols; c++)
        {
            cv::Vec3b intensity = cv_ptr->image.at<cv::Vec3b>(cv::Point(c, r));
            if (intensity[2] - intensity[0] > 100 && intensity[2] - intensity[1] > 100)
            {
                bgr_thr.at<uchar>(cv::Point(c, r)) = 255;
            }
        }
    */
   
    cv::Moments m_dst = moments(dst, true);
    cv::Point p(m_dst.m10 / m_dst.m00, m_dst.m01 / m_dst.m00);
    dvrk_retraction::visual_feedback image_info_msg;
    if (p.x < 0 || p.y < 0)
    {
      std::cout << "No feature shape detected." << std::endl;
    }
    else
    {
      // cv::findContours(bgr_thr, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
      cv::findContours(dst, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
      int largestContourIndex = 0;
      for (int i = 0; i < contours.size(); i++)
        largestContourIndex = (contours[i].size() > contours[largestContourIndex].size()) ? i : largestContourIndex;
      int sndlargestContourIndex = (largestContourIndex == 0 ) ? 1 : 0;         // NEW
      for(int i = 0; i < contours.size(); i++)
        sndlargestContourIndex = (contours[i].size() > contours[sndlargestContourIndex].size() && i != largestContourIndex ) ? i : sndlargestContourIndex;
      contours[largestContourIndex].insert(contours[largestContourIndex].end(), contours[sndlargestContourIndex].begin(), contours[sndlargestContourIndex].end());

      cv::Moments shapeMoments = moments(contours[largestContourIndex]);
      cv::Point2f shapeCenter(static_cast<float>(shapeMoments.m10 / shapeMoments.m00),
                              static_cast<float>(shapeMoments.m01 / shapeMoments.m00));
      Eigen::Matrix2f I;        // inertia tensor
      I <<  shapeMoments.mu20, shapeMoments.mu11,
            shapeMoments.mu11, shapeMoments.mu02;
      Eigen::EigenSolver<Eigen::Matrix2f> es(I);
      Eigen::Vector2d eigenVector1(es.eigenvectors().col(0).real()[0], es.eigenvectors().col(0).real()[1]),
                      eigenVector2(es.eigenvectors().col(1).real()[0], es.eigenvectors().col(1).real()[1]);

      // ***The procedure is to get rid of the possible random switch of the principal axis display.
      // ***There is a need of look at the characters of the retrun eigenvectors first to know how to
      // ***better set the processing.
      if (eigenVector1[0] < 0 )
      {
        Eigen::Vector2d tempVector = eigenVector1;
        eigenVector1 = eigenVector2;
      } 
      eigenVector2 = Eigen::Vector2d(eigenVector1[1], -eigenVector1[0]);
      /* float angleX1 = atan2(eigenVector1[1], eigenVector1[0]),
            angleX2 = atan2(eigenVector2[1], eigenVector2[0]);
      if (angleX1 > angleX2)
      {
        Eigen::Vector2d tempVector = eigenVector1;
        eigenVector1 = eigenVector2;
        eigenVector2 = tempVector;
      } */
      // std::cout << es.eigenvectors().col(0).real()[0] << "*****" << std::endl;
     
      cv::RotatedRect rectShape = cv::minAreaRect(contours[largestContourIndex]);
      cv::Point2f rectShapeVertices[4];
      rectShape.points(rectShapeVertices);    // order of vertices: bottomLeft, topLeft, topRight, bottomRight. And the bottom
                                              // is the most bottom.
      //std::vector<std::vector<cv::Point2f>> rectShapeContour;
      //rectShapeContour[0].assign(rectShapeVertices, rectShapeVertices + 4);  // this cause segmentation fault (core dumped)
      // cv::drawContours(cv_ptr->image, rectShapeContour, 0, cv::Scalar(0, 255, 0), 2);
      std::vector<float> rectLine0_1{rectShapeVertices[0].x - rectShapeVertices[1].x,
                                     rectShapeVertices[0].y - rectShapeVertices[1].y };
      std::vector<float> rectLine1_2{rectShapeVertices[2].x - rectShapeVertices[1].x,
                                     rectShapeVertices[2].y - rectShapeVertices[1].y };
      float rectLineLengthSquare0_1 = pow(rectLine0_1[0], 2) + pow(rectLine0_1[1], 2), 
            rectLineLengthSquare1_2 = pow(rectLine1_2[0], 2) + pow(rectLine1_2[1], 2);
      cv::Point2f rotateOrigin;
      //if (rectShapeVertices[2].x > rectShapeVertices[0].x)
      if (rectLineLengthSquare0_1 < rectLineLengthSquare1_2)
      {
        rotateOrigin.x = (rectShapeVertices[0].x + rectShapeVertices[1].x) / 2;
        rotateOrigin.y = (rectShapeVertices[0].y + rectShapeVertices[1].y) / 2;
      }
      else
      {
        rotateOrigin.x = (rectShapeVertices[1].x + rectShapeVertices[2].x) / 2; 
        rotateOrigin.y = (rectShapeVertices[1].y + rectShapeVertices[2].y) / 2;      
      }
      /*for (int i = 0; i < 4; i++)  // draw the rectangle    0903
      {
        cv::line(cv_ptr->image, rectShapeVertices[i], rectShapeVertices[(i+1) % 4], cv::Scalar(0, 255, 0), 2);
      }*/
      // ***0903 cv::circle(cv_ptr->image, rotateOrigin, 8, cv::Scalar(155, 245, 66), -1);      0903

      cv::Point2f principleAxie1 = shapeCenter + 80 * cv::Point2f( eigenVector1[0], eigenVector1[1]),
                  principleAxie2 = shapeCenter + 80 * cv::Point2f( eigenVector2[0], eigenVector2[1]);      
      cv::arrowedLine(cv_ptr->image, shapeCenter, principleAxie1, cv::Scalar(0, 0, 255), 2);  
      //cv::arrowedLine(cv_ptr->image, shapeCenter, principleAxie2, cv::Scalar(0, 255, 0), 2);  
      cv::circle(cv_ptr->image, shapeCenter, 5, cv::Scalar(0, 0, 0), -1); 
      cv::drawContours(cv_ptr->image, contours, largestContourIndex, cv::Scalar(255, 0, 0), 2);

      //***** visual_feedback ros message *****
      double rotateAngle = acos(eigenVector1[0]) / M_PI * 180;   // eigenvector is given with norm = 1.
      double targetAngle = 45;
      cv::Point2f rotateRadius = shapeCenter - rotateOrigin;
      Eigen::Vector2d rotateRadiusVec(rotateRadius.x, rotateRadius.y);
      cv::Point2f targetRegionCenterVec(rotateRadiusVec.norm() * cos(targetAngle / 180 * M_PI),
                                        -rotateRadiusVec.norm() * sin(targetAngle / 180 * M_PI));
      cv::Point2f targetRegionCenter = rotateOrigin + targetRegionCenterVec;
      const double targetRegionRadius = 20;
      // ***0903 cv::circle(cv_ptr->image, targetRegionCenter, targetRegionRadius, cv::Scalar(155, 245, 66), 2);   0903
      image_info_msg.centroid.push_back(shapeCenter.x);
      image_info_msg.centroid.push_back(shapeCenter.y);
      image_info_msg.rotate_origin.push_back(rotateOrigin.x);
      image_info_msg.rotate_origin.push_back(rotateOrigin.y);
      image_info_msg.principal_axis.push_back(eigenVector1[0]);
      image_info_msg.principal_axis.push_back(eigenVector1[1]);
      image_info_msg.rotate_angle = rotateAngle;
      image_info_msg.target_angle = targetAngle;
      image_info_msg.target_region_center.push_back(targetRegionCenter.x);
      image_info_msg.target_region_center.push_back(targetRegionCenter.y);
      image_info_msg.target_region_radius = targetRegionRadius;

      std::cout << "key visual info: \nrotate angle: " << rotateAngle << std::endl;
      std::cout << "centroid: [" << shapeCenter.x << " " << shapeCenter.y << "]" << std::endl;
      std::cout << "principal_axis: [" << eigenVector1[0] << " " << eigenVector1[1] << "]" << std::endl;
      //***** visual_info_srv response assignment*****
      class_centroid.clear();
      class_rotate_origin.clear();
      class_principal_axis.clear();
      class_target_region_center.clear();

      class_centroid.push_back(shapeCenter.x);
      class_centroid.push_back(shapeCenter.y);
      class_rotate_origin.push_back(rotateOrigin.x);
      class_rotate_origin.push_back(rotateOrigin.y);
      class_principal_axis.push_back(eigenVector1[0]);
      class_principal_axis.push_back(eigenVector1[1]);
      class_rotate_angle = rotateAngle;
      class_target_region_center.push_back(targetRegionCenter.x);
      class_target_region_center.push_back(targetRegionCenter.y);
      class_target_region_radius = targetRegionRadius; 

    }

    // cv::circle(cv_ptr->image, imageOrigin, 5, cv::Scalar(0, 0, 0), -1);
    // cv::arrowedLine(cv_ptr->image, imageOrigin, xAxisEnd, cv::Scalar(0, 0, 255), 2);
    // cv::putText(cv_ptr->image, "x", xAxisEnd - cv::Point(10, 10), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
    // cv::arrowedLine(cv_ptr->image, imageOrigin, yAxisEnd, cv::Scalar(0, 255, 0), 2);
    // cv::putText(cv_ptr->image, "y", yAxisEnd + cv::Point(10, 10), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 0), 2);

   // end of processing
    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // image_pub_.publish(cv_ptr->toImageMsg()); 
    image_info_pub.publish(image_info_msg);
    }

  bool get_visual_info_srv(dvrk_retraction::visual_info_srv::Request &req,
                          dvrk_retraction::visual_info_srv::Response &res)
  {
    res.centroid.push_back(class_centroid[0]);
    res.centroid.push_back(class_centroid[1]);
    res.rotate_origin.push_back(class_rotate_origin[0]);
    res.rotate_origin.push_back(class_rotate_origin[1]);
    res.principal_axis.push_back(class_principal_axis[0]);
    res.principal_axis.push_back(class_principal_axis[1]);
    res.rotate_angle = class_rotate_angle;
    res.target_region_center.push_back(class_target_region_center[0]);
    res.target_region_center.push_back(class_target_region_center[1]);
    res.target_region_radius = class_target_region_radius;

    // ROS_INFO("Successfully obtain visual information.\n");
    return true;
  }  
};

int main(int argc, char** argv)
{
  if (argc == 2) 
  {
    ros::init(argc, argv, "image_processing");
    ImageConverter ic(argv[1]);
    ros::spin();
    return 0;
  } 
  else if (argc == 1)
  {
  // set default ros image source， HUANG Jing
    char *image_topic = (char*) "/cameras/source_camera/image";
    ros::init(argc, argv, "image_processing");
    ImageConverter ic(image_topic);
    ros::spin();
    return 1;
  }
  else
  {
    std::cout << "ERROR:\tusage - RosToOpencvImage <ros_image_topic>" << std::endl; 
    return 1;    
  }
}
