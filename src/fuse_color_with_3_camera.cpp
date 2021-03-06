//
// 这个文件是用来进行测试使用4个camera 同时和 一个velodyne  points 进行点云加颜色...
// 今天基本上是完成了对四个激光雷达点加颜色的实验，明天再吧另外三个程序加上去就可以了。。。可以回去了
//#include "velo2cam_utils.h"
#include <ros/ros.h>
#include "ros/package.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace Eigen;

struct Get_color
 {
     int b;
     int g;
     int r;
 } ;


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
ros::Publisher pcl_pub;
image_transport::Publisher pub;

Eigen::Matrix4f
transform_mat_camera_16362016(4,4),transform_mat_camera_17023550(4,4),
transform_mat_camera_17082012(4,4),transform_mat_camera_17082022(4,4);

Get_color assign_color(Mat img,int uv_y,int uv_x ){

    Get_color s1;

   int  pixel_i = uv_y;//here we convert double to int
   int  pixle_j = uv_x;
    if(pixel_i == 2048){
        pixel_i = pixel_i - 1;
    }
    if(pixle_j == 2448){
        pixle_j = pixle_j - 1;
    }

    s1.b  = img.at<cv::Vec3b>(pixel_i,pixle_j)[0];
    s1.g  = img.at<cv::Vec3b>(pixel_i,pixle_j)[1];
    s1.r  = img.at<cv::Vec3b>(pixel_i,pixle_j)[2];

    return s1;

}


pcl::PointXYZRGB one_camera_assign_color( Mat img, pcl::PointXYZRGB coloured_temp , Eigen::Matrix4f transform_mat,  image_geometry::PinholeCameraModel cam_model_ ){
    Eigen::Vector4f  homogeneous_points;
    Eigen::Vector4f  trans_result;
    cv::Point2d uv;

    homogeneous_points << coloured_temp.x, coloured_temp.y, coloured_temp.z, 1;
    trans_result =  transform_mat * homogeneous_points;

    cv::Point3d pt_cv(trans_result(0), trans_result(1), trans_result(2));

    uv = cam_model_.project3dToPixel(pt_cv); //世界坐标系下面的点在相机坐标系下面的表示

    if( trans_result(2) >0 && uv.x>0 && uv.x < img.cols && uv.y > 0 && uv.y < img.rows){

        Get_color s_16362016 = assign_color(img ,uv.y,uv.x);

        coloured_temp.b = s_16362016.b;
        coloured_temp.g = s_16362016.g;
        coloured_temp.r = s_16362016.r;

    }

    return coloured_temp;
}



void callback(const PointCloud2::ConstPtr& pcl_msg,
        const CameraInfoConstPtr& camera_info_1_msg, const ImageConstPtr& camera_1_msg,
        const CameraInfoConstPtr& camera_info_2_msg, const ImageConstPtr& camera_2_msg,

        const CameraInfoConstPtr& camera_info_3_msg, const ImageConstPtr& camera_3_msg,
        const CameraInfoConstPtr& camera_info_4_msg, const ImageConstPtr& camera_4_msg
        ){
    std::cout<<"received image!!!"<<std::endl;
    ROS_INFO("Projecting poincloud to the image %ld", pcl_msg->header.stamp.toNSec()-camera_1_msg->header.stamp.toNSec());

    cv::Mat img_16362016,img_pub, img_17023550,img_17082012,img_17082022;

    try{

        img_16362016  = cv_bridge::toCvCopy(camera_1_msg, sensor_msgs::image_encodings::BGR8)->image;
        img_17023550  = cv_bridge::toCvCopy(camera_2_msg, sensor_msgs::image_encodings::BGR8)->image;
        img_17082012  = cv_bridge::toCvCopy(camera_3_msg, sensor_msgs::image_encodings::BGR8)->image;
        img_17082022  = cv_bridge::toCvCopy(camera_4_msg, sensor_msgs::image_encodings::BGR8)->image;

     //   img_pub = img_16362016.clone();
     //   cv::imwrite("image_with_points.jpg",img_16362016);

    }catch (cv_bridge::Exception& e){

        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

  //  imshow("img_17023550",img_17023550);
  //  imshow("img_17082012",img_17082012);
  //  waitKey(10);

    image_geometry::PinholeCameraModel cam_model_16362016,cam_model_17023550,cam_model_17082012,cam_model_17082022;
    cam_model_16362016.fromCameraInfo(camera_info_1_msg);
    cam_model_17023550.fromCameraInfo(camera_info_2_msg);
    cam_model_17082012.fromCameraInfo(camera_info_3_msg);
    cam_model_17082022.fromCameraInfo(camera_info_4_msg);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr  pcl_cloud_temp;

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    output = *pcl_msg;

    pcl::fromROSMsg(output,*cloud);

    std::cout<<"transform_mat is :"<<endl<<transform_mat_camera_16362016<<std::endl;

    ROS_INFO("Projecting every single point. Please wait. ");

    int cnt=0;
    PointCloudXYZRGB::Ptr coloured = PointCloudXYZRGB::Ptr(new PointCloudXYZRGB); // For coloring purposes

    pcl::copyPointCloud(*cloud, *coloured);

    int cPointR=-1, cPointG=-1, cPointB=-1;

    int pixel_i= 0,pixle_j =0;
    pcl::PointXYZRGB coloured_temp;

    for(int i = 0; i < cloud->points.size(); i++){

        coloured->points[i].b = 0;
        coloured->points[i].g = 255;
        coloured->points[i].r = 0;

        coloured_temp.x = coloured->points[i].x;
        coloured_temp.y = coloured->points[i].y;
        coloured_temp.z = coloured->points[i].z;
        coloured_temp.r = coloured->points[i].r;
        coloured_temp.g = coloured->points[i].g;
        coloured_temp.b = coloured->points[i].b;

        pcl::PointXYZRGB coloured_return_16362016 = one_camera_assign_color(img_16362016,coloured_temp,transform_mat_camera_16362016,cam_model_16362016);

        pcl::PointXYZRGB coloured_return_17023550 = one_camera_assign_color(img_17023550,coloured_return_16362016,transform_mat_camera_17023550,cam_model_17023550);

        pcl::PointXYZRGB coloured_return_17082012 = one_camera_assign_color(img_17082012,coloured_return_17023550,transform_mat_camera_17082012,cam_model_17082012);

        pcl::PointXYZRGB coloured_return_17082022 = one_camera_assign_color(img_17082022,coloured_return_17082012,transform_mat_camera_17082022,cam_model_17082022);
//
//        coloured->points[i].x = coloured_return_17082022.x;
//        coloured->points[i].y = coloured_return_17082022.y;
//
//        coloured->points[i].z = coloured_return_17082022.z;
//        coloured->points[i].r = coloured_return_17082022.r;
//
//        coloured->points[i].g = coloured_return_17082022.g;
//        coloured->points[i].b = coloured_return_17082022.b;



        coloured->points[i].x = coloured_return_17082022.x;
        coloured->points[i].y = coloured_return_17082022.y;

        coloured->points[i].z = coloured_return_17082022.z;
        coloured->points[i].r = coloured_return_17082022.r;

        coloured->points[i].g = coloured_return_17082022.g;
        coloured->points[i].b = coloured_return_17082022.b;



    }


    std::cout<<"successful !!!"<<cnt<<std::endl;

//cv::imwrite("image_with_points.jpg",image);
//pcl::io::savePCDFileASCII ("umbrella.pcd", *coloured); //将点云保存到PCD文件中

//pub.publish(cv_bridge::CvImage(image_msg->header, image_msg->encoding, img).toImageMsg());
//pub.publish(cv_bridge::CvImage(image_msg->header, "bgr8", img_pub).toImageMsg());

    ROS_INFO("Done");

    sensor_msgs::PointCloud2 pcl_colour_ros;
    pcl::toROSMsg(*coloured, pcl_colour_ros);
    pcl_colour_ros.header.frame_id="chy";
    pcl_colour_ros.header.stamp = pcl_msg->header.stamp ;
    pcl_pub.publish(pcl_colour_ros);

}


string left_velodyne ,

camera_1_info_param, camera_1_image_param,
camera_2_info_param, camera_2_image_param,

camera_3_info_param, camera_3_image_param,
camera_4_info_param, camera_4_image_param;

int main(int argc, char **argv){

    ros::init(argc, argv, "pcl_projection");
    ros::NodeHandle nh_ ;

  //  nh_ = getPrivateNodeHandle();
    image_transport::ImageTransport it(nh_);

    if(!nh_.getParam("/chy_one/velodyne_topic_all", left_velodyne)){

        ROS_ERROR("fail to read the left_velodyne");
    }

// camera1
    if(!nh_.getParam("/chy_one/camera_16360216", camera_1_image_param)){
        ROS_ERROR("fail to read the camera_1_image_param");
    }
    if(!nh_.getParam("/chy_one/camera_16360216_info", camera_1_info_param)){
        ROS_ERROR("fail to read the camera_1_info_param");
    }


//camera2
    if(!nh_.getParam("/chy_one/camera_17023550", camera_2_image_param)){
        ROS_ERROR("fail to read the camera_2_image_param");
    }
    if(!nh_.getParam("/chy_one/camera_17023550_info", camera_2_info_param)){
        ROS_ERROR("fail to read the camera_2_info_param");
    }

//camera3
    if(!nh_.getParam("/chy_one/camera_17082012", camera_3_image_param)){
        ROS_ERROR("fail to read the camera_3_image_param");
    }
    if(!nh_.getParam("/chy_one/camera_17082012_info", camera_3_info_param)){
        ROS_ERROR("fail to read the camera_3_info_param");
    }



//camera4
    if(!nh_.getParam("/chy_one/camera_17082022", camera_4_image_param)){
        ROS_ERROR("fail to read the camera_4_image_param");
    }
    if(!nh_.getParam("/chy_one/camera_17082022_info", camera_4_info_param)){
        ROS_ERROR("fail to read the camera_4_info_param");
    }



    std::vector<float> trans_matrix;

    if(!nh_.getParam("/chy_one/matrix_16362016", trans_matrix)){
        ROS_ERROR("fail to read the matrix_16362016");
    }


    for(int i=0; i < trans_matrix.size(); i++) {

        transform_mat_camera_16362016(i/4,i%4) = trans_matrix[i];
    }

    std::cout<<"transform_mat_camera_16362016 is: "<<std::endl<<transform_mat_camera_16362016<<std::endl;

    if(!nh_.getParam("/chy_one/matrix_17023550", trans_matrix)){
        ROS_ERROR("fail to read the matrix_17023550");
    }


    for(int i=0; i < trans_matrix.size(); i++) {

        transform_mat_camera_17023550(i/4,i%4) = trans_matrix[i];
    }
    std::cout<<"transform_mat_camera_17023550 is: "<<std::endl<<transform_mat_camera_17023550<<std::endl;


    if(!nh_.getParam("/chy_one/matrix_17082012", trans_matrix)){
        ROS_ERROR("fail to read the matrix_17082012");
    }


    for(int i=0; i < trans_matrix.size(); i++) {

        transform_mat_camera_17082012(i/4,i%4) = trans_matrix[i];
    }
    std::cout<<"transform_mat_camera_17082012 is: "<<std::endl<<transform_mat_camera_17082012<<std::endl;


    if(!nh_.getParam("/chy_one/matrix_17082022", trans_matrix)){
        ROS_ERROR("fail to read the matrix_17082022");
    }


    for(int i=0; i < trans_matrix.size(); i++) {

        transform_mat_camera_17082022(i/4,i%4) = trans_matrix[i];
    }
    std::cout<<"transform_mat_camera_17082022 is: "<<std::endl<<transform_mat_camera_17082022<<std::endl;


    std::cout<<"velodyne_topic is: "<<left_velodyne<<std::endl;

    std::cout<<"image topic  is: "<<camera_1_image_param<<std::endl;
    std::cout<<"img_info_topic is: "<<camera_1_info_param<<std::endl;


    std::cout<<"image topic  is: "<<camera_2_image_param<<std::endl;
    std::cout<<"img_info_topic is: "<<camera_2_info_param<<std::endl;

    std::cout<<"image topic  is: "<<camera_3_image_param<<std::endl;
    std::cout<<"img_info_topic is: "<<camera_3_info_param<<std::endl;


    std::cout<<"image topic  is: "<<camera_4_image_param<<std::endl;
    std::cout<<"img_info_topic is: "<<camera_4_info_param<<std::endl;


    message_filters::Subscriber<PointCloud2> velodyne(nh_, left_velodyne, 1000);


    message_filters::Subscriber<CameraInfo> camera_1_info(nh_, camera_1_info_param, 1000);
    message_filters::Subscriber<Image> camera_1_image(nh_, camera_1_image_param, 1000);


    message_filters::Subscriber<CameraInfo> camera_2_info(nh_, camera_2_info_param, 1000);
    message_filters::Subscriber<Image> camera_2_image(nh_, camera_2_image_param, 1000);


    message_filters::Subscriber<CameraInfo> camera_3_info(nh_, camera_3_info_param, 1000);
    message_filters::Subscriber<Image> camera_3_image(nh_, camera_3_image_param, 1000);


    message_filters::Subscriber<CameraInfo> camera_4_info(nh_, camera_4_info_param, 1000);
    message_filters::Subscriber<Image> camera_4_image(nh_, camera_4_image_param, 1000);



    pcl_pub = nh_.advertise<sensor_msgs::PointCloud2>("/chy/pcl_output",1000, true);

    typedef message_filters::sync_policies::ApproximateTime<PointCloud2,
    CameraInfo,Image,
    CameraInfo,Image,
    CameraInfo,Image,
    CameraInfo,Image > SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), velodyne,
            camera_1_info, camera_1_image,
            camera_2_info,camera_2_image,
            camera_3_info,camera_3_image,
            camera_4_info,camera_4_image);
    sync.registerCallback(boost::bind(&callback,_1,_2,_3,_4,_5,_6,_7,_8,_9));

    ros::spin();
    return 0;
}
