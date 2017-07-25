#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
# define PI 3.14159265358979323846

// Some global variables
std::vector<double> drone_velocity(6,0),drone_pose(6,0), visual_features(3,0), drone2cameraPose(3,0); // [u,v]
double drone_thrust = 9.81; // Force equivalent to the gravity in an object of 1 kg


// -------- Callbacks to retrieve topics information  --------------------
void hasReceivedVisualFeatures(const geometry_msgs::PoseStamped::ConstPtr& msg){
	visual_features[0] = msg->pose.position.x;
	visual_features[1] = msg->pose.position.y;
	visual_features[2] = msg->pose.position.z;
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

Eigen::MatrixXd SkewSymmetricMatrix(std::vector<double> input_vec){
	Eigen::MatrixXd S(3,3);
	S << 0.0, -input_vec[2], input_vec[1],
    input_vec[2], 0.0, -input_vec[0],
    -input_vec[1], input_vec[0], 0.0;
	return S;
}

Eigen::MatrixXd ComputeLinearVelJacobian(std::vector<double> vf, double depth, double f){
	Eigen::MatrixXd Jv(2,3);

	double u = vf[0];
	double v = vf[1];

	Jv(0,0) = -f/depth;
	Jv(0,1) = 0.0;
	Jv(0,2) = u/depth;
	Jv(1,0) = 0.0;
	Jv(1,1) = -f/depth;
	Jv(1,2) = v/depth;

	return Jv;
}

Eigen::MatrixXd ComputeAngularVelJacobian(std::vector<double> vf, double depth, double f, std::vector<double> drone2cameraVec){
	Eigen::MatrixXd Jw(2,3);

	double px = drone2cameraVec[0];
	double py = drone2cameraVec[1];
	double pz = drone2cameraVec[2];
	double u = vf[0];
	double v = vf[1];

	Jw(0,0) = (u*(py + v))/depth;
	Jw(0,1) = pow (u,2)/f - (px*u)/depth + f - (f*pz)/depth;
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


void fillROSMultyarray(std_msgs::Float32MultiArray &msg, std::vector<double> data, ros::Publisher &publisher){
	msg.data.insert(msg.data.end(), data.begin(), data.end());
	publisher.publish(msg);
	msg.data.clear();
}



// -------- Main --------------------
int main(int argc, char** argv){

ros::init(argc, argv, "ibvs_controller_node");
ros::NodeHandle nh_;
ros::NodeHandle nhp_("~");
ros::Rate rate(20.0);

// Initialize the publisher message
std_msgs::Float32MultiArray msg_vf;
std::vector<double> control_angles(2,0);
msg_vf.layout.dim.push_back(std_msgs::MultiArrayDimension());
msg_vf.layout.dim[0].size = control_angles.size();
msg_vf.layout.dim[0].stride = 1;


// Intialize some local variables
Eigen::MatrixXd Jv, Jw, Jvps;
Eigen::VectorXd angular_vel(3), imagef(2), imgefd(2), error, angles(3), angles_derivatives(3), position(3), linear_vel(3);

// Control gain
double K = 1.0;
// Visual gain
double Ka = 1.0;
// Quadrotor mass
double mass = 1.0; // kg
// Focal length
double focal_length = 1.0; // m

// ROS Subscribers and Publishers
ros::Subscriber visual_tracker_sub_=nh_.subscribe("visual_tracker", 10, hasReceivedVisualFeatures);
ros::Subscriber vrep_drone_pose_sub_=nh_.subscribe("vrep_drone_pose", 10, hasReceivedDronePosition);
ros::Subscriber vrep_drone2camera_pose_sub_=nh_.subscribe("vrep_drone2camera_pose", 10, hasReceivedDrone2CameraPosition);
ros::Subscriber vrep_drone_velocity_sub_=nh_.subscribe("vrep_drone_velocity", 10, hasReceivedDroneVelocity);
ros::Subscriber vrep_drone_thrust_sub_=nh_.subscribe("vrep_drone_thrust_force", 10, hasDroneThrustForce);
ros::Publisher control_output_pub_=nh_.advertise<std_msgs::Float32MultiArray>("ibvs_control_output", 1);


// Load simulation scene parameters
// Quadrotor mass
nhp_.getParam("quadrotor_mass",mass);
// Camera focal length
nhp_.getParam("camera_focal_length",focal_length);

	while (ros::ok()){

		// Update the local variables
		VariablesandDerivatives(drone_pose, drone_velocity,angles,angles_derivatives,3);
		VariablesandDerivatives(drone_pose, drone_velocity, position, linear_vel,0);
		//Compute the angular velocity
		angular_vel = fromAngleDerivatives2AngularVelocity(angles_derivatives,angles);
		// Compute the Jacobian matrices
		Jv= ComputeLinearVelJacobian(visual_features,drone_pose[2],focal_length);
		Jw = ComputeAngularVelJacobian(visual_features,drone_pose[2],focal_length, drone2cameraPose);

		// Computes the pseudo inverse of Jv with dimension 3x2
		Jvps = PseudoInverse(Jv);

		// Computes the derivatives of the image features
		imgefd = Jv*linear_vel + Jw* angular_vel;
		// Assigning the virtual features to the eigen vector
		imagef(0) = visual_features[0];
		imagef(1) = visual_features[1];

		// Takes the error in velocity (ex,ey,ez)
		error = Jvps*(-Ka*imagef-imgefd);

		// Compute the IBVS control outputs
		control_angles = ComputeControlOuputs(drone_thrust, mass, Ka, error);

		fillROSMultyarray(msg_vf, control_angles,control_output_pub_);
		// Insert the value of the control outputs in the ros message
		//msg_vf.data.insert(msg_vf.data.end(), control_angles.begin(), control_angles.end());
		// Publish the data
		//control_output_pub_.publish(msg_vf);
		// Clearing the data for the next iteration
		//msg_vf.data.clear();
   	ros::spinOnce(); // if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
    	rate.sleep();
    }

 return 0;
}
