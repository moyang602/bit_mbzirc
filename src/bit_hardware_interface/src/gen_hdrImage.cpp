#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <bit_hardware_msgs/MER_srv.h>
#include <sensor_msgs/Image.h>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "test_hdr");
    
    ros::NodeHandle nh;

    cv::namedWindow(OPENCV_WINDOW);

    ros::ServiceClient client = nh.serviceClient<bit_hardware_msgs::MER_srv>("/CameraMER_Left/GrabMERImage");
    
    ros::Duration five_seconds(5,0);
    client.waitForExistence(five_seconds);

    bit_hardware_msgs::MER_srv  srv;

    ros::Rate loop_rate(10);
    int numImages = 3;
    static const float timesArray[] = {100*0.000001,10000*0.000001,50000*0.000001};
    std::vector<float> times;
    times.assign(timesArray, timesArray + numImages);
    std::vector<cv::Mat> images;
    while(ros::ok()) 
    {
        images.clear();

        for (size_t i = 0; i < numImages; i++)
		{
            srv.request.exposure_time = times[i]*1000000;
            srv.request.BalanceRatioRed = 2.0;
            srv.request.BalanceRatioGreen = 1.6;
            srv.request.BalanceRatioBlue = 2.5;

            client.call(srv);
            sensor_msgs::Image Image;
            Image = srv.response.MER_image;

            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(Image, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
            
            images.push_back(cv_ptr->image);
            
            
		}

        // 对齐输入图像
		Ptr<AlignMTB> alignMTB = createAlignMTB();
		alignMTB->process(images, images);

        // 获取图像响应函数 (CRF)
		Mat responseDebevec;
		Ptr<CalibrateDebevec> calibrateDebevec = createCalibrateDebevec();
		calibrateDebevec->process(images, responseDebevec, times);


		// 将图像合并为HDR线性图像
		Mat hdrDebevec;
		Ptr<MergeDebevec> mergeDebevec = createMergeDebevec();
		mergeDebevec->process(images, hdrDebevec, times, responseDebevec);

        //! [Tonemap HDR image]
        Mat ldr;
        Ptr<Tonemap> tonemap = createTonemap(2.2f);
        tonemap->process(hdrDebevec, ldr);
        //! [Tonemap HDR image]

        //! [Perform exposure fusion]
        Mat fusion;
        Ptr<MergeMertens> merge_mertens = createMergeMertens();
        merge_mertens->process(images, fusion);
        //! [Perform exposure fusion]

		// // 使用Reinhard色调映射算法获得24位彩色图像
		// Mat ldrReinhard;
		// Ptr<TonemapReinhard> tonemapReinhard = createTonemapReinhard(1.0, 0, 0.1, 0);
		// tonemapReinhard->process(hdrDebevec, ldrReinhard);

        cv::imshow(OPENCV_WINDOW, fusion);
        cv::waitKey(1);


        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    }

			

		
        
		

	// 	ROS_ERROR("I am here!!!");
	// 	imwrite("ldr-Reinhard.jpg", ldrReinhard * 255);




    return 0;
}