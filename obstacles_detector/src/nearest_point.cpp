
#include <iostream>
#include <stdlib.h>  
 
#include <Eigen/Dense>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <visp3/detection/vpDetectorDataMatrixCode.h>
#include <visp3/detection/vpDetectorQRCode.h>
#include <visp3/core/vpImageConvert.h>


// Global variables
cv_bridge::CvImagePtr cv_ptr;
sensor_msgs::CameraInfo camera_info;
double drone_height = 1.5;

// -------------------------- Define callbacks -------------------------
// Retrieve drone's camera images
void imageCallback(const sensor_msgs::ImageConstPtr& msg){

	try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
	}
}
// Retrieve drone's camera info
void camera_infoCallback(const sensor_msgs::CameraInfo& msg){
	camera_info = msg;
	return;
}
// Retrieve drone's height
void hasDroneHeight(const std_msgs::Float32::ConstPtr& msg){
	drone_height = msg->data;
	return;
}
// Publishing functions
void PublishROSVectorStamped(geometry_msgs::Vector3Stamped &msg, std::vector<double> vector , ros::Publisher &publisher, std::string frame){
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = frame;
	msg.vector.x = vector[0];
	msg.vector.y = vector[1];
		if(vector.size()>2)
			msg.vector.z = vector[2];
	publisher.publish(msg);
}


//  -------------------Functions -------------------------------- 
double pixeldistance(cv::Point candidate ,std::vector<int> target){

	double distance = sqrt(pow(candidate.x-target[0],2)+pow(candidate.y-target[1],2));	
	//double distance = abs(obstacle_u-target[0])+abs(obstacle_v-target[1]);	
	return distance;
}
std::vector<int> find_nearest_point(cv::Mat image,std::vector<int> target){
	double prev_distance = 100000;
	std::vector<int> nearest_point(2,0);

	int count_white = 0;
	int count_black = 0;
	int count_distance = 0;

	std::vector<cv::Point> locations;   // output, locations of non-zero pixels
	cv::findNonZero(image, locations);

	for(int i=0; i < locations.size(); i++){
		if (pixeldistance(locations[i],target) <= prev_distance){
			nearest_point[0] =locations[i].x;
			nearest_point[1] =locations[i].y;
			prev_distance = pixeldistance(locations[i],target);
			count_distance++;
		}
	}
	return nearest_point;
}

// This function computes the position of 3D objects with respect to the ground! The origin of the system is the center of the camera
std::vector<double> compute_camera_position(double focal_length, double u_0, double v_0, std::vector<double> scale, float heigth, std::vector<int> image_point){

	std::vector<double> camera_position(2,0);
	camera_position[0] = scale[0]*((heigth*(image_point[0]-u_0))/focal_length);
	camera_position[1] = scale[1]*((heigth*(image_point[1]-v_0))/focal_length);
	return camera_position;
}

// Perform the transformation from the camera frame to the drone frame
std::vector<double> camera2drone(Eigen::MatrixXd R, std::vector<double> point2D){

	std::vector<double> new_point(2,0);
	Eigen::VectorXd point_3D(3), out_point_3D(3);

	point_3D(0) = point2D[0];
	point_3D(1) = point2D[1];
	point_3D(2) = 0.0;

	out_point_3D = R*point_3D;

	new_point[0] = out_point_3D(0);
	new_point[1] = out_point_3D(1);
	
	return new_point; 
}
// Find the position of the obstacle w.r.t the frame of the QR code (2D translation)
std::vector<double> obstacle2QR(std::vector<double> obstacle, std::vector<double> QR){
	std::vector<double> output(2,0);
	output[0] = obstacle[0]-QR[0];
	output[1] = obstacle[1]-QR[1];
	return output;
}
// -------- Main --------------------
int main(int argc, char** argv){

	ros::init(argc, argv, "nearest_point_node");
	ros::NodeHandle nh_;
	ros::NodeHandle nhp_("~");
	ros::Rate rate(20.0);

	// Publishers and subscribers
	ros::Subscriber img_sub = nh_.subscribe("input_image", 5, imageCallback);
	image_transport::ImageTransport it(nh_);
	image_transport::Publisher pub_segmented_image = it.advertise("image_segmented", 1);
	image_transport::Publisher pub_nearest_point_image = it.advertise("image_nearest_point", 1);
	ros::Subscriber sub_camera_info = nh_.subscribe("camera_info", 5, camera_infoCallback);
	ros::Subscriber sub_drone_height = nh_.subscribe("drone_height", 5, hasDroneHeight);
	ros::Publisher QR_position_camera =nh_.advertise<geometry_msgs::Vector3Stamped>("QR_position_camera_frame", 1);
	ros::Publisher QR_position_drone =nh_.advertise<geometry_msgs::Vector3Stamped>("QR_position_drone_frame", 1);
	ros::Publisher obstacle_position_camera =nh_.advertise<geometry_msgs::Vector3Stamped>("obstacle_position_camera_frame", 1);
	ros::Publisher obstacle_position_drone =nh_.advertise<geometry_msgs::Vector3Stamped>("obstacle_position_drone_frame", 1);
	ros::Publisher obstacle_position_QR =nh_.advertise<geometry_msgs::Vector3Stamped>("obstacle_position_QR_frame", 1);
	
	//  Script variables
	cv_bridge::CvImage image_msg;
	cv::Mat frame, imgHSV, imgThresholded;
	int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;

	std::vector<cv::Vec4i> hierarchy;
	// ViSP variables
	vpDetectorQRCode detector;
	vpImage<unsigned char> vp_Image;
	bool status; 
	vpImagePoint QR_centroid;
	double focal_length = 700.50;
	// position of QR code and nearest point in pixels
	std::vector<int>QR_code_poistion(2,0);
	std::vector<int>nearest_obstacle(2,0);
	// position of QR code and nearest point in meters (camera frame)
	std::vector<double>QR_code_camera_position(2,0);
	std::vector<double>nearest_obstacle_camera_position(2,0);
	// Additional pixel scale
	std::vector<double>pixel_scale(2,1);
	
	// Rotation matrix from the camera frame to the drone frame
	Eigen::MatrixXd R(3,3);
	R << 0.0, -1.0, 0.0,
	 -1.0, 0.0, 0.0,
 	  0.0, 0.0, -1.0;

	// position of QR code and nearest point in meters (camera frame)
	std::vector<double>QR_code_drone_position(2,0);
	std::vector<double>nearest_obstacle_drone_position(2,0);
	// Position of the obstacle in the QR frame 
	std::vector<double>nearest_obstacle_QR_position(2,0);

	// Message to publish the points
	geometry_msgs::Vector3Stamped msg_point;
	while (ros::ok()){

		// Load the color parameters
		nhp_.getParam("iLowH",iLowH);
		nhp_.getParam("iHighH",iHighH);
		nhp_.getParam("iLowS",iLowS);
		nhp_.getParam("iHighS",iHighS);
		nhp_.getParam("iLowV",iLowV);
		nhp_.getParam("iHighV",iHighV);
		nhp_.getParam("scale_u",pixel_scale[0]);
		nhp_.getParam("scale_v",pixel_scale[1]);

		// Check that the callback already received the image
		if(cv_ptr){ 
			cv_ptr->image.copyTo(frame);
			// Do no flip the image, ROS do it for you. So, the ROS image is the flipped version of the V-REP image
			//cv::flip(frame, frame, -1);

			// Extract the centroid position of the QR code
			vpImageConvert::convert(frame, vp_Image);
			status = detector.detect(vp_Image);

			// Load the position of the QR code
			if (status){
				//std::cout<<" Detecto algo " << std::endl;
				for(size_t i=0; i < detector.getNbObjects(); i++)
					QR_centroid = detector.getCog(i);
				// Get column index
				QR_code_poistion[0]=(int)QR_centroid.get_u();
				// Get row index
				QR_code_poistion[1]=(int)QR_centroid.get_v();
			}
	
			

			cv::cvtColor(frame, imgHSV, cv::COLOR_BGR2HSV);
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

			nearest_obstacle = find_nearest_point(imgThresholded,QR_code_poistion);		

			cv::circle(frame,cv::Point(QR_code_poistion[0],QR_code_poistion[1]),7,cv::Scalar(255,0,0),4);
			cv::circle(frame,cv::Point(nearest_obstacle[0],nearest_obstacle[1]),7,cv::Scalar(0,255,0),4);

			QR_code_camera_position = compute_camera_position(camera_info.K[0], camera_info.K[2], camera_info.K[5], pixel_scale, drone_height, QR_code_poistion);
			nearest_obstacle_camera_position = compute_camera_position(camera_info.K[0], camera_info.K[2], camera_info.K[5], pixel_scale, drone_height, nearest_obstacle);

			QR_code_drone_position = camera2drone(R,QR_code_camera_position);
			nearest_obstacle_drone_position = camera2drone(R,nearest_obstacle_camera_position);

	
			nearest_obstacle_QR_position = obstacle2QR(nearest_obstacle_drone_position,QR_code_drone_position);

			// Publish the images	
			image_msg.encoding = sensor_msgs::image_encodings::MONO8;
			image_msg.image = imgThresholded;		
			pub_segmented_image.publish(image_msg.toImageMsg());
			
			image_msg.encoding = sensor_msgs::image_encodings::BGR8;
			image_msg.image = frame;
			pub_nearest_point_image.publish(image_msg.toImageMsg());

			// Publish the positions computed
			PublishROSVectorStamped(msg_point, QR_code_camera_position, QR_position_camera, "camera_frame");
			PublishROSVectorStamped(msg_point, nearest_obstacle_camera_position, obstacle_position_camera, "camera_frame");
			PublishROSVectorStamped(msg_point, QR_code_drone_position, QR_position_drone, "drone_frame");
			PublishROSVectorStamped(msg_point, nearest_obstacle_drone_position, obstacle_position_drone, "drone_frame");
			PublishROSVectorStamped(msg_point, nearest_obstacle_QR_position, obstacle_position_QR, "QR_frame");
			
		}
   		ros::spinOnce(); 
    }
}

