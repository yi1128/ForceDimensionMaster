#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <dhdc.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#define KP    100.0
#define KVP    10.0
#define MAXF    4.0
#define KR      0.3
#define KWR     0.02
#define MAXT    0.1
#define KG    100.0
#define KVG     5.0
#define MAXG    1.0

#define REFRESH_INTERVAL    0.1   //  seconds
#define LINEAR_VISCOSITY   30.0   //  N/(m/s)
#define MAX_FRICTION_FORCE 0.75
// #define MAX_FRICTION_FORCE 0.00001
#define FRICTION_SAT_REGION 0.05
#define ANGULAR_VISCOSITY   0.04  // Nm/(rad/s)


#define _USE_MATH_DEFINES

#define MAX_FEEDBACK_FORCE 10 // N


#define SERIAL_NUMBER_OMEGA3 11415 // Omega3
#define SERIAL_NUMBER_OMEGA6 11359 // Omega6
#define SERIAL_NUMBER_SIGMA7 3030 // Sigma7

double position_based_velocity_x_ = 0.0;
double position_based_velocity_y_ = 0.0;
double position_based_velocity_z_ = 0.0;

double omega_velocity_x_ = 0.0;
double omega_velocity_y_ = 0.0;
double omega_velocity_z_ = 0.0;


class Omega_CH2
{
public:
	ros::NodeHandlePtr node_;
	ros::NodeHandlePtr pnode_;

	ros::Publisher master_dt_pub_;
	ros::Publisher master_position_cmd_pub_;
	ros::Publisher master_velocity_cmd_pub_;
	ros::Publisher indexing_button_pub_;


	ros::Subscriber force_feedback_sub_;
	ros::Subscriber attractive_force_sub_;


	double ForceFeedback_[3];
	int teleoperation_mode_;
	bool indexing_;

	// Force feedback callback
	void ForceFeedbackCallback(const geometry_msgs::WrenchStampedConstPtr& msg){

		static double Dt = 0.0;
		static ros::Time old_t = msg->header.stamp;
		Dt = (msg->header.stamp - old_t).toSec();
		old_t = msg->header.stamp;
		if(!(Dt>0)) return;

		// Dt = 1.0/frequency;
		Dt = 0.001;

		double Fs[3] = {msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z}; // Force from Slave
		static double Xm[3] =  {0.0, 0.0, 0.0}; // Position of Master Device
		double Vm[3] = {omega_velocity_x_, omega_velocity_y_, omega_velocity_z_}; // Velocity of Master Device
		double final_forcefeedback[3] = {0.0, 0.0, 0.0};
		
		if(indexing_){
			// Master Position Update
			for(unsigned i=0; i<3; i++){
				Xm[i] += Vm[i]*Dt;
			}

			// Force Saturation
			for(unsigned int i=0;i<3;i++){
				if(Fs[i] > MAX_FEEDBACK_FORCE){
					Fs[i] = MAX_FEEDBACK_FORCE;
				}
				else if(Fs[i] < -MAX_FEEDBACK_FORCE){
					Fs[i] = -MAX_FEEDBACK_FORCE;
				}
				ForceFeedback_[i] = Fs[i];
			}
		}
	}


	double LowPassFilter(const double measured_value, const double filtered_value_old, const float alpha){
	  double filtered_value;
	  filtered_value = alpha*measured_value + (1.0-alpha)*filtered_value_old;
	  return filtered_value;
	}

	// Load parameters etc
	int init()
	{
		node_ = ros::NodeHandlePtr(new ros::NodeHandle(""));
		pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

		pnode_->param("teleoperation_mode", teleoperation_mode_, 1);

		master_dt_pub_ = node_->advertise<std_msgs::Float64>("Master/CH2/Dt",10);
		master_position_cmd_pub_ = node_->advertise<geometry_msgs::TwistStamped>("/Master/CH2/command/position", 10);
		master_velocity_cmd_pub_ = node_->advertise<geometry_msgs::PoseStamped>("/Master/CH2/command/velocity",10);
		indexing_button_pub_ = node_->advertise<std_msgs::Bool>("/Master/CH2/indexing", 10);
		force_feedback_sub_ = node_->subscribe<geometry_msgs::WrenchStamped>("/Master/CH2/FeedbackForce", 10, &Omega_CH2::ForceFeedbackCallback, this);


		indexing_ = false;

		return 0;
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Omega_CH2_node");

	Omega_CH2 omega_ch2;
	if (omega_ch2.init())
	{
		ROS_FATAL("Omega_CH2_node initialization failed");
		return -1;
	}

  	std_msgs::Float64 master_dt;
	double px, py, pz;
	double roll, pitch, yaw;
	double vx, vy, vz;
	double wx, wy, wz;
	double vg;
	double fx, fy, fz;
	double tx, ty, tz;
	double fg;
  	double freq   = 0.0;
	
	int    done  = 0;
	double t0    = dhdGetTime ();
	double t1    = t0;

	double final_force[DHD_MAX_DOF];
		
	omega_ch2.ForceFeedback_[0] = 0.0;
	omega_ch2.ForceFeedback_[1] = 0.0;
	omega_ch2.ForceFeedback_[2] = 0.0;

  	// message
	int major, minor, release, revision;
	dhdGetSDKVersion (&major, &minor, &release, &revision);
	printf ("\n");
	printf ("Force Dimension - Viscosity Example %d.%d.%d.%d\n", major, minor, release, revision);
	printf ("(C) 2014 Force Dimension\n");
	printf ("All Rights Reserved.\n\n");

	// required to change asynchronous operation mode
	dhdEnableExpertMode ();

	// open the first available device
	if (dhdOpenSerial(SERIAL_NUMBER_OMEGA6) < 0) {
		printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
	  	dhdSleep (2.0);
	    return -1;
	}
        /*
        if (dhdOpen() < 0) {
                      printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
                      dhdSleep (2.0);
                  return -1;
              }
              */
	ushort Serial_;
	dhdGetSerialNumber(&Serial_);
	std::cout << Serial_ << std::endl;

	// identify device
	printf ("%s device detected\n\n", dhdGetSystemName());

	// display instructions
	printf ("press BUTTON or 'q' to quit\n\n");

	// enable force
	dhdEnableForce (DHD_ON);

	
	ros::Rate loop_rate(1000);
	double Dt = 0.001;
	// loop while the button is not pushed
	while (!done) {
		ros::spinOnce();

		ros::Time t_now = ros::Time::now();
		static ros::Time t_old = t_now;
		double dt = (t_now - t_old).toSec();
		master_dt.data = dt;
		t_old = t_now;
    	// printf("dt: %f\n",dt);
		if(dt == 0.0) continue;


		//---Get Omega State
		dhdGetPosition(&px,&py,&pz);
		dhdGetOrientationRad(&roll, &pitch, &yaw);
		dhdGetLinearVelocity (&vx, &vy, &vz);
		dhdGetAngularVelocityRad (&wx, &wy, &wz);
		dhdGetGripperLinearVelocity (&vg);

		
		omega_velocity_x_ = vx; // [m] to [mm]
		omega_velocity_y_ = vy; // [m] to [mm]
		omega_velocity_z_ = vz; // [m] to [mm]


		//--- Force Init
		final_force[0] = 0.0;
		final_force[1] = 0.0;
		final_force[2] = 0.0;


		//---1. Centering Force
		double centering_force[3];
		
		// double kp_centering = 3.0;
		double kp_centering = 27.0;

		double p[3];

		p[0] = px;
		p[1] = py;
		p[2] = pz;

		double spring_range = 0.05;
		for(unsigned int i=0;i<3;i++){
			if(p[i] > spring_range){
				p[i] = spring_range;
			}
			else if(p[i] < -spring_range){
				p[i] = -spring_range;
			}
			centering_force[i] = -p[i]*kp_centering;
			final_force[i] += centering_force[i];
		}


		// centering_force[0] = -px*kp_centering;
		// centering_force[1] = -py*kp_centering;
		// centering_force[2] = -pz*kp_centering;

		//   final_force[0] += centering_force[0];
		//   final_force[1] += centering_force[1];
		//   final_force[2] += centering_force[2];

		//---2. Viscosity Damping Force
		double damping_force[3];
		//tx = 0.0;
		//ty = 0.0;
		//tz = 0.0;

		//fg = 0.0;

		tx = -ANGULAR_VISCOSITY * wx;
		ty = -ANGULAR_VISCOSITY * wy;
		tz = -ANGULAR_VISCOSITY * wz;

		fg = -LINEAR_VISCOSITY * vg;

		double velocity = sqrt(pow(vx,2)+pow(vy,2)+pow(vz,2));
		double friction_force = 0.0;

		friction_force = -MAX_FRICTION_FORCE/FRICTION_SAT_REGION * velocity;



		if(friction_force > MAX_FRICTION_FORCE) friction_force = MAX_FRICTION_FORCE;
		else if(friction_force < -MAX_FRICTION_FORCE) friction_force = -MAX_FRICTION_FORCE;

		if(fabs(velocity)>0) {
			damping_force[0] = vx / velocity * friction_force;
			damping_force[1] = vy / velocity * friction_force;
			damping_force[2] = vz / velocity * friction_force;
		}
		else{
			damping_force[0] = 0.0;
			damping_force[1] = 0.0;
			damping_force[2] = 0.0;
		}

		final_force[0] += damping_force[0];
		final_force[1] += damping_force[1];
		final_force[2] += damping_force[2];


		//--- Master Position Increments Data Publish for Teleoperation
		double master_dx = 0.0; 
		double master_dy = 0.0;
		double master_dz = 0.0;
		double master_droll = 0.0;
		double master_dpitch = 0.0;
		double master_dyaw = 0.0;

		static double old_px = px;
		static double old_py = py;
		static double old_pz = pz;
		static double old_roll = roll;
		static double old_pitch = pitch;
		static double old_yaw = yaw;

		master_dx = px - old_px;
		master_dy = py - old_py;
		master_dz = pz - old_pz;
		master_droll = roll - old_roll;
		master_dpitch = pitch - old_pitch;
		master_dyaw = yaw - old_yaw;

		old_px = px;
		old_py = py;
		old_pz = pz;
		old_roll = roll;
		old_pitch = pitch;
		old_yaw = yaw;

		position_based_velocity_x_ = master_dx/dt;
		position_based_velocity_y_ = master_dy/dt;
		position_based_velocity_z_ = master_dz/dt;
		
		std_msgs::Bool indexing;
		geometry_msgs::TwistStamped MasterPositionCmd;
		MasterPositionCmd.header.frame_id = "/Omega_CH2";
		MasterPositionCmd.header.stamp = ros::Time::now();

		geometry_msgs::PoseStamped MasterVelocityCmd;
		MasterVelocityCmd.header.frame_id = "/Omega_CH2";
		MasterVelocityCmd.header.stamp = ros::Time::now();
			
		MasterVelocityCmd.pose.position.x = position_based_velocity_x_;
		MasterVelocityCmd.pose.position.y = position_based_velocity_y_;
		MasterVelocityCmd.pose.position.z = position_based_velocity_z_;
    
		static geometry_msgs::PoseStamped prevMasterVelocityCmd = MasterVelocityCmd;


		if(dhdGetButtonMask()){
			omega_ch2.indexing_ = true;

				// Force Feedback reflect to the omega6
			final_force[0] += omega_ch2.ForceFeedback_[0];
			final_force[1] += omega_ch2.ForceFeedback_[1];
			final_force[2] += omega_ch2.ForceFeedback_[2];

			MasterPositionCmd.twist.linear.x = omega_velocity_x_*Dt;
			MasterPositionCmd.twist.linear.y = omega_velocity_y_*Dt;
			MasterPositionCmd.twist.linear.z = omega_velocity_z_*Dt;

      			// Low Pass Filtering
			double alpha = 0.3;
			MasterVelocityCmd.pose.position.x = omega_ch2.LowPassFilter(position_based_velocity_x_,prevMasterVelocityCmd.pose.position.x,alpha);
			MasterVelocityCmd.pose.position.y = omega_ch2.LowPassFilter(position_based_velocity_y_,prevMasterVelocityCmd.pose.position.y,alpha);
			MasterVelocityCmd.pose.position.z = omega_ch2.LowPassFilter(position_based_velocity_z_,prevMasterVelocityCmd.pose.position.z,alpha);

			prevMasterVelocityCmd = MasterVelocityCmd;
		}
		else{
			omega_ch2.indexing_ = false;

			// final_force[0] = 0.0;
			// final_force[1] = 0.0;
			// final_force[2] = 0.0;

			MasterPositionCmd.twist.linear.x = 0.0;
			MasterPositionCmd.twist.linear.y = 0.0;
			MasterPositionCmd.twist.linear.z = 0.0;

			MasterVelocityCmd.pose.position.x = 0.0;
			MasterVelocityCmd.pose.position.y = 0.0;
			MasterVelocityCmd.pose.position.z = 0.0;
		}

		// Orientation increment always update independent on indexing
		MasterPositionCmd.twist.angular.x = master_droll;
		MasterPositionCmd.twist.angular.y = master_dpitch;
		MasterPositionCmd.twist.angular.z = master_dyaw;

		MasterVelocityCmd.pose.orientation.w = 1.0;
		MasterVelocityCmd.pose.orientation.x = master_droll/dt;
		MasterVelocityCmd.pose.orientation.y = master_dpitch/dt;
		MasterVelocityCmd.pose.orientation.z = master_dyaw/dt;

		// Master Position Increment Data(command) publish
		omega_ch2.master_position_cmd_pub_.publish(MasterPositionCmd);
		omega_ch2.master_velocity_cmd_pub_.publish(MasterVelocityCmd);
		omega_ch2.master_dt_pub_.publish(master_dt);
		indexing.data = omega_ch2.indexing_;
		omega_ch2.indexing_button_pub_.publish(indexing);

		//---Force Set To Omega
    	if (dhdSetForceAndTorqueAndGripperForce (final_force[0], final_force[1], final_force[2], tx, ty, tz, fg) < DHD_NO_ERROR) {
      		printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
      		done = 1;
    	}

		// display refresh rate and position at 10Hz
		t1 = dhdGetTime ();
		if ((t1-t0) > REFRESH_INTERVAL) {
			// retrieve information to display
      		freq = dhdGetComFreq ();
      		t0 = t1;

			// write down velocity
			// printf ("p (%+0.03f %+0.03f %+0.03f) m  ", px, py, pz);
			// printf ("| v (%+0.03f %+0.03f %+0.03f) m/s  ", vx, vy, vz);
			// printf ("| o (%+0.03f %+0.03f %+0.03f) rad  ", oa, ob, og);
			// printf ("\r");
			// printf("    %.3f,    %.3f,    %.3f\r",(double)final_force[0],(double)final_force[1],(double)final_force[2]);

			// test for exit condition
			//if (dhdGetButtonMask ()) done = 1;

	        if (dhdKbHit ()) {
				switch (dhdKbGet ()) 
				{
		  			case 'q': done = 1; break;
					case 'b':
						// base = !base;
						// drdRegulatePos  (base);
						break;
					case 'w':
						// wrist = !wrist;
						// drdRegulateRot  (wrist);
						break;
					case 'g':
						// grip = !grip;
						// drdRegulateGrip (grip);
						break;
				}
			}	
		}
		loop_rate.sleep();
	}

	// close the connection
  	dhdClose ();

  	// happily exit
  	printf ("\ndone.\n");
  	return 0;
}
