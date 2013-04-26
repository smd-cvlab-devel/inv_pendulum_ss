#!/usr/bin/env python

PACKAGE='inv_pendulum_ss'

from driver_base.msg import SensorLevels
from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator( )

gen.add( "gain", double_t, SensorLevels.RECONFIGURE_RUNNING, "Gain", 0.0, 0.0, 10.0 )

exit( gen.generate( PACKAGE, "inv_pendulum_ss", "InvPendulumSS" ) )

