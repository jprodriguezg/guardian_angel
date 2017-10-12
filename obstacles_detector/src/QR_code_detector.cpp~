#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <visp3/detection/vpDetectorDataMatrixCode.h>
#include <visp3/detection/vpDetectorQRCode.h>
#include <visp3/core/vpImageConvert.h>


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

	ros::init(argc, argv,"QR_code_detector_node");
	ros::NodeHandle nh_;
	ros::NodeHandle nhp_("~");
	ros::Rate rate(20.0);

	// Publishers and subscribers
	ros::Subscriber img_sub = nh_.subscribe("input_image", 5, imageCallback);
	image_transport::ImageTransport it(nh_);
	image_transport::Publisher pub_image = it.advertise("output_image", 1);
	
	//  Script variables
	cv_bridge::CvImage out_msg;
	cv::Mat frame;

	// ViSP variables
	vpDetectorQRCode detector;
	vpImage<unsigned char> I;
	bool status; 
	vpImagePoint QR_centroid;
	while (ros::ok()){


		// Check that the callback already received the image
		if(cv_ptr){ 
			cv_ptr->image.copyTo(frame);
			//cv::flip(frame, frame, -1);
			vpImageConvert::convert(frame, I);
			status = detector.detect(I);
			
			if (status){
				//std::cout<<" Detecto algo " << std::endl;
				for(size_t i=0; i < detector.getNbObjects(); i++) {
      				/*std::cout << "Bar code " << i << ":" << std::endl;
      				std::vector<vpImagePoint> p = detector.getPolygon(i);
					for(size_t j=0; j < p.size(); j++)
        				std::cout << "  Point " << j << ": " << p[j] << std::endl;*/
					QR_centroid = detector.getCog(i);
					std::cout <<"Centroid :" << i <<" u: " <<QR_centroid.get_u()<<"  v : " <<QR_centroid.get_v()<<std::endl;
					//std::cout <<"u " << QR_centroid.get_u()  <<"  v  :" << QR_centroid.get_v()<< std::endl;
					//std::cout <<"i " << QR_centroid.get_i()  <<"  j  :" << QR_centroid.get_j()<< std::endl;
				}
			}
	
			//cv::imshow("Readings", frame);
	  		//cv::waitKey(3);

		}
   		ros::spinOnce(); 
    }

 return 0;
}
