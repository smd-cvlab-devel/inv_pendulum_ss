#include "inv_pendulum_ss/inv_pendulum_ss.hpp"

#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include <iomanip>

PLUGINLIB_DECLARE_CLASS(inv_pendulum_ss, InvPendulumSS, inv_pendulum_ss::InvPendulumSS, nodelet::Nodelet)

namespace inv_pendulum_ss
{
	InvPendulumSS::InvPendulumSS( ) :
                gain( 0.0 ),
		have_uav( false ),
		have_pendulum( false ),
		last_theta( 0.0 ),
		last_phi( 0.0 )
	{
	}

	void InvPendulumSS::onInit( )
	{
		nh = getNodeHandle( );
		nh_priv = getPrivateNodeHandle( );
		dyn_re = new dynamic_reconfigure::Server<inv_pendulum_ss::InvPendulumSSConfig>( nh_priv );
		dyn_re_cb_type = new dynamic_reconfigure::Server<inv_pendulum_ss::InvPendulumSSConfig>::CallbackType( boost::bind( &InvPendulumSS::DynReCB, this, _1, _2) );
		nh_priv.param( "gain", gain, 0.0 );
		dyn_re->setCallback( *dyn_re_cb_type );
		twist_pub = nh.advertise<geometry_msgs::Twist>( "cmd_vel", 1, false );
		uav_odom_sub = nh.subscribe( "uav_odom", 1, &InvPendulumSS::UAVOdomCB, this );
		pendulum_odom_sub = nh.subscribe( "pendulum_odom", 1, &InvPendulumSS::PendulumOdomCB, this );
	}

	void InvPendulumSS::UAVOdomCB( const nav_msgs::OdometryPtr &msg )
	{
		uav_odom = msg;
		have_uav = true;
		if( have_pendulum )
		{
			Iterate( );
			have_uav = false;
			have_pendulum = false;
		}
	}

	void InvPendulumSS::PendulumOdomCB( const nav_msgs::OdometryPtr &msg )
	{
		pendulum_odom = msg;
		have_pendulum = true;
		if( have_uav )
		{
			Iterate( );
			have_uav = false;
			have_pendulum = false;
		}
	}

	void InvPendulumSS::DynReCB( inv_pendulum_ss::InvPendulumSSConfig &cfg, const uint32_t lvl )
	{
		gain = cfg.gain;
	}

	void InvPendulumSS::Iterate( )
	{
		//
		// GET DATA
		//

		// Get the tf::Quaternions
		const tf::Quaternion uav_quat( uav_odom->pose.pose.orientation.x, uav_odom->pose.pose.orientation.y,
			uav_odom->pose.pose.orientation.z, uav_odom->pose.pose.orientation.w );
		const tf::Quaternion pendulum_quat( pendulum_odom->pose.pose.orientation.x, pendulum_odom->pose.pose.orientation.y,
			pendulum_odom->pose.pose.orientation.z, pendulum_odom->pose.pose.orientation.w );

		const double w = pendulum_quat.w( );
		const double x = pendulum_quat.x( );
		const double y = pendulum_quat.y( );
		const double z = pendulum_quat.z( );

		const double theta = atan2(2.0 * (y*z + w*x), w*w - x*x - y*y + z*z);
		const double phi = asin(-2.0 * (x*z - w*y));

		const double theta_dot = pendulum_odom->twist.twist.angular.x;
		const double phi_dot = pendulum_odom->twist.twist.angular.y;

		//
		// CONTROLLER
		//
		//
		//vx ~ phi;
		//vy ~ -theta;
		//
		//K = { { k1, k2, k3, k4,  0,  0,  0,  0 },
		//      {  0,  0,  0,  0, k5, k6, k7, k8 } };
		//
		//X = { { x },
		//      { x_dot },
		//      { phi },
		//      { phi_dot },
		//      { y },
		//      { y_dot },
		//      { theta },
		//      { theta_dot } };
		//
		//vx = k1 * x + k2 * x_dot + k3 * phi + k4 * phi_dot;
		//vy = k5 * y + k6 * y_dot + k7 * theta + k8 * theta_dot

		static const double K[2][8] = { {  -3.8730,  -4.4827, -28.8321,  -8.5346,  -0.0000,  -0.0000,  -0.0000, -0.0000 },
						{  -0.0000,  -0.0000,  -0.0000,  -0.0000,  -5.0000,  -5.3863, -32.1270, -9.5314 } };

		const double vx = -K[0][0] * uav_odom->pose.pose.position.x - K[0][1] * uav_odom->twist.twist.linear.x
				- K[0][2] * phi - K[0][3] * phi_dot;
		const double vy = -K[1][4] * uav_odom->pose.pose.position.y - K[1][5] * uav_odom->twist.twist.linear.y
				+ K[1][6] * theta + K[1][7] * theta_dot;
		//const double vx = -K[0][0] * uav_odom->pose.pose.position.x - K[0][1] * uav_odom->twist.twist.linear.x;
		//const double vy = -K[1][4] * uav_odom->pose.pose.position.y - K[1][5] * uav_odom->twist.twist.linear.y;

		//
		// CONSTRUCT TWIST MESSAGE
		//

		geometry_msgs::TwistPtr msg( new geometry_msgs::Twist );

		msg->linear.x = gain * vx;
		msg->linear.y = gain * vy;

		if( msg->linear.x > 1.0 )
			msg->linear.x = 1.0;
		else if( msg->linear.x < -1.0 )
			msg->linear.x = -1.0;
		if( msg->linear.y > 1.0 )
			msg->linear.y = 1.0;
		else if( msg->linear.y < -1.0 )
			msg->linear.y = -1.0;

		twist_pub.publish( msg );

		last_theta = theta;
		last_phi = phi;
	}
}
