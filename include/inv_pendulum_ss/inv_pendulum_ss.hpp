#ifndef _inv_pendulum_ss_hpp
#define _inv_pendulum_ss_hpp

#include "inv_pendulum_ss/InvPendulumSSConfig.h"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_broadcaster.h>

namespace inv_pendulum_ss
{
	class InvPendulumSS : public nodelet::Nodelet
	{
	public:
		InvPendulumSS( );
		virtual void onInit( );
		void UAVOdomCB( const nav_msgs::OdometryPtr &msg );
		void PendulumOdomCB( const nav_msgs::OdometryPtr &msg );
	private:
		void DynReCB( inv_pendulum_ss::InvPendulumSSConfig &cfg, const uint32_t lvl );
		void Iterate( );

		// ROS Interface
		ros::NodeHandle nh;
		ros::NodeHandle nh_priv;
		ros::Publisher twist_pub;
		ros::Subscriber uav_odom_sub;
		ros::Subscriber pendulum_odom_sub;
		dynamic_reconfigure::Server<inv_pendulum_ss::InvPendulumSSConfig> *dyn_re;
		dynamic_reconfigure::Server<inv_pendulum_ss::InvPendulumSSConfig>::CallbackType *dyn_re_cb_type;
		tf::TransformBroadcaster br;

		double gain;

		// Last Messages
		nav_msgs::OdometryPtr uav_odom;
		nav_msgs::OdometryPtr pendulum_odom;
		bool have_uav;
		bool have_pendulum;
		double last_theta;
		double last_phi;
	};
}

#endif /* _inv_pendulum_ss_hpp */
