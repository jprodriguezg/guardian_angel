#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
# define PI 3.14159265358979323846

// Some global variables
Eigen::VectorXd imageft(3);
std::vector<double> drone_velocity(6,0),drone_pose(6,0), drone2cameraPose(3,0); // [u,v]
double drone_thrust = 9.81; // Force equivalent to the gravity in an object of 1 kg


// -------- Callbacks to retrieve topics information  --------------------
void hasReceivedVisualFeatures(const geometry_msgs::PoseStamped::ConstPtr& msg){
	imageft(0) = msg->pose.position.x;
	imageft(1) = msg->pose.position.y;
	imageft(2) = msg->pose.position.z;
	//std::cout << msg->pose.position.x << " , " << msg->pose.position.y <<  std::endl;
  return;
}

void hasReceivedDronePosition(const geometry_msgs::PoseStamped::ConstPtr& msg){
	drone_pose[0] = msg->pose.position.x;
	drone_pose[1] = msg->pose.position.y;
	drone_pose[2] = msg->pose.position.z;
	drone_pose[3] = msg->pose.orientation.x;
	drone_pose[4] = msg->pose.orientation.y;
	drone_pose[5] = msg->pose.orientation.z;
  return;
}


void hasReceivedDroneVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg){
	drone_velocity[0] = msg->twist.linear.x;
	drone_velocity[1] = msg->twist.linear.y;
	drone_velocity[2] = msg->twist.linear.z;
	drone_velocity[3] = msg->twist.angular.x;
	drone_velocity[4] = msg->twist.angular.y;
	drone_velocity[5] = msg->twist.angular.z;
  return;
}

void hasReceivedDrone2CameraPosition(const geometry_msgs::PointStamped::ConstPtr& msg){
	drone2cameraPose[0] = 0.0; //msg->point.x;
	drone2cameraPose[1] = 0.0; //msg->point.y;
	drone2cameraPose[2] = 0.0; //msg->point.z;
	return;
}

void hasDroneThrustForce(const std_msgs::Float32::ConstPtr& msg){
	drone_thrust = msg->data;
	return;
}

// -------- Functions --------------------
/* This function retrives the variable and its derivatives. It is lazy function but saves space. Unfortunately I
 	did not find how to get a subverctor from a vector in c++ */
void VariablesandDerivatives(std::vector<double> x, std::vector<double> xd, Eigen::VectorXd &var, Eigen::VectorXd &dvar, int starting_ref){

	int j = 0;
	for (int i = starting_ref; i<starting_ref+3; i++){
		var(j) = x[i];
		dvar(j) = xd[i];
		j++;
	}
}

/* This functions makes the transformation between the derivatives of the euler angles and the angular velocities
	NOTE: Be careful with the notation of the angles, otherwise the function returns a wrong transformation */

Eigen::VectorXd fromAngleDerivatives2AngularVelocity(Eigen::VectorXd euler_derivatives, Eigen::VectorXd euler_angles){
	Eigen::VectorXd angular_vel(3);
	// NOTE: The relation of the Euler angles to RPY is the following (The same as in my notebook)
	// gm -> Roll, ph -> yaw, th -> pitch.
	double gm = euler_angles[0];
	double gmd = euler_derivatives[0];
	double th = euler_angles[1];
	double thd = euler_derivatives[1];
	double ph = euler_angles[2];
	double phd = euler_derivatives[2];

	// Computes the correspondent angular velocity
	angular_vel(0) = phd*sin(th)*sin(gm) + thd*cos(gm);
	angular_vel(1) = phd*sin(th)*cos(gm) - thd*sin(gm);
	angular_vel(2) = phd*cos(th) + gmd;


	return angular_vel;
}

Eigen::MatrixXd PseudoInverse(Eigen::MatrixXd matrix){
	Eigen::MatrixXd aux, pseudoInv;
	aux = matrix*matrix.transpose();
	pseudoInv = matrix.transpose()*aux.inverse();
	return pseudoInv;
}

Eigen::MatrixXd JvPseudoInverse(Eigen::VectorXd vf, double depth, double f){

	double u = vf(0);
	double v = vf(1);

	Eigen::MatrixXd Jv_pseudo(3,2);
	Jv_pseudo(0,0) = -(depth*(pow(f,2) + pow(v,2)))/(f*(pow(f,2) + pow(u,2) + pow(v,2)));
	Jv_pseudo(0,1) = (depth*u*v)/(f*(pow(f,2) + pow(u,2) + pow(v,2)));
	Jv_pseudo(1,0) = (depth*u*v)/(f*(pow(f,2) + pow(u,2) + pow(v,2)));
	Jv_pseudo(1,1) = -(depth*(pow(f,2) + pow(u,2)))/(f*(pow(f,2) + pow(u,2) + pow(v,2)));
	Jv_pseudo(2,0) = (depth*u)/(pow(f,2) + pow(u,2) + pow(v,2));
	Jv_pseudo(2,1) = (depth*v)/(pow(f,2) + pow(u,2) + pow(v,2));

	return Jv_pseudo;
}


Eigen::MatrixXd SkewSymmetricMatrix(std::vector<double> input_vec){
	Eigen::MatrixXd S(3,3);
	S << 0.0, -input_vec[2], input_vec[1],
    input_vec[2], 0.0, -input_vec[0],
    -input_vec[1], input_vec[0], 0.0;
	return S;
}

Eigen::MatrixXd ComputeLinearVelJacobian(Eigen::VectorXd vf, double depth, double f){
	Eigen::MatrixXd Jv(2,3);

	double u = vf(0);
	double v = vf(1);

	Jv(0,0) = -f/depth;
	Jv(0,1) = 0.0;
	Jv(0,2) = u/depth;
	Jv(1,0) = 0.0;
	Jv(1,1) = -f/depth;
	Jv(1,2) = v/depth;

	return Jv;
}

Eigen::MatrixXd ComputeAngularVelJacobian(Eigen::VectorXd vf, double depth, double f, std::vector<double> drone2cameraVec){
	Eigen::MatrixXd Jw(2,3);

	double px = drone2cameraVec[0];
	double py = drone2cameraVec[1];
	double pz = drone2cameraVec[2];
	double u = vf(0);
	double v = vf(1);

	Jw(0,0) = (u*(py + v))/depth;
	Jw(0,1) = pow(u,2)/f - (px*u)/depth + f - (f*pz)/depth;
	Jw(0,2) = (f*py)/depth - v;
	Jw(1,0) = pow(v,2)/f + (py*v)/depth + f + (f*pz)/depth;
	Jw(1,1) = (u*v)/f - (px*v)/depth;
	Jw(1,2) = u - (f*px)/depth;

	return Jw;
}

// Returns the min-max values for the asin function of mat.h [-1,1]
double asin_boundaries(double input){

	double output;
	if (input >1)
		output = 1.0;
	if (input < -1)
		output = -1.0;
	return output;
}

std::vector<double> ComputeControlOuputs(double T, double m, double K, Eigen::VectorXd vel_error){

	std::vector<double> output(2,0);

	double aux_x = (m*K/T)*(vel_error(0)/cos(drone_pose[3]));
	double aux_y = (m*K/T)*vel_error(1);


	// Avoids values out of [-1,1]
	//aux_x = asin_boundaries(aux_x);
	//aux_y = asin_boundaries(aux_y);

	output[0] = asin(aux_x);
	output[1] = asin(aux_y);

	return output;
}


//// --------------- Publisher functions ----------------------
void PublishROSMultyarray(std_msgs::Float32MultiArray &msg, std::vector<double> data, ros::Publisher &publisher){
	msg.data.insert(msg.data.end(), data.begin(), data.end());
	publisher.publish(msg);
	msg.data.clear();
}

void PublishImageftVelocity(geometry_msgs::TwistStamped &msg, Eigen::VectorXd linear_vel, Eigen::VectorXd angular_vel, ros::Publisher &publisher){
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "world_frame";
	msg.twist.linear.x = linear_vel(0);
	msg.twist.linear.y = linear_vel(1);
	msg.twist.linear.z = 0.0;
	msg.twist.angular.x = angular_vel(0);
	msg.twist.angular.y = angular_vel(1);
	msg.twist.angular.z = 0.0;
	publisher.publish(msg);
	//msg.data.clear();
}

void PublishROSVectorStamped(geometry_msgs::Vector3Stamped &msg, Eigen::VectorXd vector , ros::Publisher &publisher){
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "world_frame";
	msg.vector.x = vector(0);
	msg.vector.y = vector(1);
		if(vector.size()>2)
			msg.vector.z = vector(2);
	publisher.publish(msg);
}


// -------- Main --------------------
int main(int argc, char** argv){

ros::init(argc, argv, "vs_control_node");
ros::NodeHandle nh_;
ros::NodeHandle nhp_("~");
ros::Rate rate(20.0);


// Intialize some local variables
Eigen::MatrixXd Jv, Jw, Jvps;
Eigen::VectorXd scaled_imageft(3), rot_imageft(3);

// Initialize the publisher message
geometry_msgs::Vector3Stamped msg_rot_imft;


// ROS Subscribers and Publishers
ros::Subscriber visual_tracker_sub_=nh_.subscribe("visual_tracker", 10, hasReceivedVisualFeatures);
ros::Subscriber vrep_drone_pose_sub_=nh_.subscribe("vrep_drone_pose", 10, hasReceivedDronePosition);
ros::Subscriber vrep_drone2camera_pose_sub_=nh_.subscribe("vrep_drone2camera_pose", 10, hasReceivedDrone2CameraPosition);
ros::Subscriber vrep_drone_velocity_sub_=nh_.subscribe("vrep_drone_velocity", 10, hasReceivedDroneVelocity);
ros::Subscriber vrep_drone_thrust_sub_=nh_.subscribe("vrep_drone_thrust_force", 10, hasDroneThrustForce);
ros::Publisher rot_imgft_pub_=nh_.advertise<geometry_msgs::Vector3Stamped>("vs_image_features_rot", 1);

double Sx, Sy;
// X axis Scale
nhp_.getParam("Sx",Sx);
// Y axis scale
nhp_.getParam("Sy",Sy);


// Rotation matrix
Eigen::MatrixXd R(3,3);
R << 0.0, -1.0, 0.0,
 -1.0, 0.0, 0.0,
 0.0, 0.0, -1.0;

	while (ros::ok()){

		scaled_imageft(0) = Sx*imageft(0);
		scaled_imageft(1) = Sy*imageft(1);
		rot_imageft = R*scaled_imageft;
		//std::cout << rot_imageft(0)<< " , " << rot_imageft(1)<<  std::endl;
		//std::cout << imageft(0)<< " , " << imageft(1)<<  std::endl;
		PublishROSVectorStamped(msg_rot_imft, rot_imageft, rot_imgft_pub_);

   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    }

 return 0;
}
