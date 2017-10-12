
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


// Global variables
cv_bridge::CvImagePtr cv_ptr;

// Define callbacks
void imageCallback(const sensor_msgs::ImageConstPtr& msg){

	try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
	}
}

// -------- Main --------------------
int main(int argc, char** argv){

	ros::init(argc, argv, "blobs_detector_node");
	ros::NodeHandle nh_;
	ros::NodeHandle nhp_("~");
	ros::Rate rate(20.0);

	// Publishers and subscribers
	ros::Subscriber img_sub = nh_.subscribe("input_image", 5, imageCallback);
	image_transport::ImageTransport it(nh_);
	image_transport::Publisher pub_image = it.advertise("output_image", 1);
	
	//  Script variables
	cv_bridge::CvImage out_msg;
	cv::Mat frame, imgHSV;
	int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;

	while (ros::ok()){

		// Load the color parameters
		nhp_.getParam("iLowH",iLowH);
		nhp_.getParam("iHighH",iHighH);
		nhp_.getParam("iLowS",iLowS);
		nhp_.getParam("iHighS",iHighS);
		nhp_.getParam("iLowV",iLowV);
		nhp_.getParam("iHighV",iHighV);

		// Check that the callback already received the image
		if(cv_ptr){ 
			cv_ptr->image.copyTo(frame);

			cv::cvtColor(frame, imgHSV, cv::COLOR_BGR2HSV);
			cv::Mat imgThresholded;
			cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);		

			//morphological opening (remove small objects from the foreground)
  			cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
  			cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

   			//morphological closing (fill small holes in the foreground)
  			cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
  			cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

			// Debugging stuff
			//std::cout <<" "<<iLowH<<"  "<<iHighH<<" "<<iLowS<<" "<<iHighS<<" "<<iLowV<<" "<<iHighV<<std::endl; 
			//cv::imshow("Readings", imgThresholded);
	  		//cv::waitKey(3);

			out_msg.encoding = sensor_msgs::image_encodings::MONO8;
			out_msg.image = imgThresholded;		
			pub_image.publish(out_msg.toImageMsg());
		}
   		ros::spinOnce(); 
    }

 return 0;
}
