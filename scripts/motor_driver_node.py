#!/usr/bin/env python3

import rospy
import ZLAC8015D
import tf2_ros
import numpy as np
import time
import yaml
from decimal import Decimal, ROUND_HALF_UP
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped

class MotorDriverNode:
	def __init__(self):
		print("\033[32m" + "Start motor driver node!!" + "\033[0m")
		print("#########################")
		#-----Initialize publisher-----
		self.wheels_rpm_pub = rospy.Publisher("/wheels_rpm", Float32MultiArray, queue_size=10)
		self.wheels_rpm_msg = Float32MultiArray()
		self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
		self.odom_msg = Odometry()

		#-----Initialize subscriber-----
		rospy.Subscriber("/zlac8015d/twist/cmd_vel", Twist, self.zlac8015d_twist_cmd_callback)
		rospy.Subscriber("/zlac8015d/vel/cmd_vel", Float32MultiArray, self.zlac8015d_vel_cmd_callback)
		rospy.Subscriber("/zlac8015d/vel/cmd_rpm", Float32MultiArray, self.zlac8015d_rpm_cmd_callback)
		rospy.Subscriber("/zlac8015d/pos/cmd_deg", Float32MultiArray, self.zlac8015d_deg_cmd_callback)
		rospy.Subscriber("/zlac8015d/pos/cmd_dist", Float32MultiArray, self.zlac8015d_dist_cmd_callback)
		
		#-----Import parameters from yaml file-----
		self.load_file = "/home/ubuntu/catkin_ws/src/zlac8015d_ros/params/motor_driver_params.yaml"
		with open(self.load_file) as file:
			self.obj = yaml.safe_load(file)
		
		#-----Initialize ZLAC8015D-----
		self.zlc = ZLAC8015D.Controller(self.obj["port"])
		self.zlc.travel_in_one_rev = float(self.obj["travel_in_one_rev"])
		self.zlc.cpr = int(self.obj["cpr"])
		self.zlc.R_Wheel = float(self.obj["R_Wheel"])
		self.control_mode = int(self.obj["control_mode"])
		if self.control_mode == 3:
			self.zlac8015d_speed_mode_init()
		elif self.control_mode == 1:
			self.zlac8015d_position_mode_init()
		
		#-----Initialize variable-----
		self.last_subscribed_time = 0.0
		self.callback_timeout = float(self.obj["callback_timeout"])
		self.deadband_rpm = int(self.obj["deadband_rpm"])
		self.linear_vel_cmd = 0.0
		self.angular_vel_cmd = 0.0
		self.got_twist_cmd = False
		
		self.left_vel_cmd = 0.0
		self.right_vel_cmd = 0.0
		self.got_vel_cmd = False

		self.left_rpm_cmd = 0.0
		self.right_rpm_cmd = 0.0
		self.got_vel_rpm_cmd = False

		self.left_pos_deg_cmd  = 0.0
		self.right_pos_deg_cmd  = 0.0
		self.got_pos_deg_cmd = False

		self.left_pos_dist_cmd = 0.0
		self.right_pos_dist_cmd = 0.0
		self.got_pos_dist_cmd = False
		
		self.theta = 0.0
		self.x = 0.0
		self.y = 0.0
		self.period = 0.05
		self.l_meter = 0.0
		self.r_meter = 0.0
		self.l_meter_init, self.r_meter_init = self.zlc.get_wheels_travelled()
		self.prev_l_meter = 0.0
		self.prev_r_meter = 0.0

		self.br = tf2_ros.TransformBroadcaster()
		self.t = TransformStamped()
	
	"""
	++++++++++++++++++++++++++++++++++++++++++
		zlac8015d_twist_cmd_callback function
	++++++++++++++++++++++++++++++++++++++++++
	Callback function to subscribe for "/zlac8015d/twist/cmd_vel".
	"""
	def zlac8015d_twist_cmd_callback(self, msg):
		self.linear_vel_cmd = msg.linear.x
		self.angular_vel_cmd = msg.angular.z
		self.got_twist_cmd = True
		self.last_subscribed_time = rospy.Time.now().nsecs
	
	"""
	++++++++++++++++++++++++++++++++++++++++++
		zlac8015d_vel_cmd_callback function
	++++++++++++++++++++++++++++++++++++++++++
	Callback function to subscribe for "/zlac8015d/vel/cmd_vel".
	"""
	def zlac8015d_vel_cmd_callback(self, msg):
		self.left_vel_cmd = msg.data[0]
		self.right_vel_cmd = -msg.data[1]
		self.got_vel_cmd = True
		self.last_subscribed_time = time.perf_counter()

	"""
	++++++++++++++++++++++++++++++++++++++++++
		zlac8015d_rpm_cmd_callback function
	++++++++++++++++++++++++++++++++++++++++++
	Callback function to subscribe for "/zlac8015d/vel/cmd_rpm".
	"""
	def zlac8015d_rpm_cmd_callback(self, msg):
		self.left_rpm_cmd = msg.data[0]
		self.right_rpm_cmd = -msg.data[1]
		self.got_vel_rpm_cmd = True
		self.last_subscribed_time = time.perf_counter()

	"""
	++++++++++++++++++++++++++++++++++++++++++
		zlac8015d_deg_cmd_callback function
	++++++++++++++++++++++++++++++++++++++++++
	Callback function to subscribe for "/zlac8015d/pos/cmd_deg".
	"""
	def zlac8015d_deg_cmd_callback(self, msg):
		self.left_pos_deg_cmd = msg.data[0]
		self.right_pos_deg_cmd = -msg.data[1]
		self.got_pos_deg_cmd = True

	"""
	++++++++++++++++++++++++++++++++++++++++++
		zlac8015d_dist_cmd_callback function
	++++++++++++++++++++++++++++++++++++++++++
	Callback function to subscribe for "/zlac8015d/pos/cmd_dist".
	"""
	def zlac8015d_dist_cmd_callback(self, msg):
		self.left_pos_dist_cmd = msg.data[0]
		self.right_pos_dist_cmd = msg.data[1]
		self.got_pos_dist_cmd = True

	"""
	++++++++++++++++++++++++++++++++++++++++++
		zlac8015d_speed_mode_init function
	++++++++++++++++++++++++++++++++++++++++++
	Initialization of speed rpm control mode.
	"""
	def zlac8015d_speed_mode_init(self):
		self.zlc.disable_motor()
		self.zlc.set_accel_time(int(self.obj["set_accel_time_left"]), int(self.obj["set_accel_time_right"]))
		self.zlc.set_decel_time(int(self.obj["set_decel_time_left"]), int(self.obj["set_decel_time_right"]))
		self.zlc.set_mode(3)
		self.zlc.enable_motor()

	"""
	++++++++++++++++++++++++++++++++++++++++++
		zlac8015d_position_mode_init function
	++++++++++++++++++++++++++++++++++++++++++
	Initialization of relative position control mode.
	"""
	def zlac8015d_position_mode_init(self):
		self.zlc.disable_motor()
		self.zlc.set_accel_time(int(self.obj["set_accel_time_left"]), int(self.obj["set_accel_time_right"]))
		self.zlc.set_decel_time(int(self.obj["set_decel_time_left"]), int(self.obj["set_decel_time_right"]))
		self.zlc.set_mode(1)
		self.zlc.set_position_async_control()
		self.zlc.set_maxRPM_pos(int(self.obj["max_left_rpm"]), int(self.obj["max_right_rpm"]))
		self.zlc.enable_motor()
	
	"""
	++++++++++++++++++++++++++++++++++++++++++
		twist_to_rpm function
	++++++++++++++++++++++++++++++++++++++++++
	Convert from twist to rpm.
	"""
	def twist_to_rpm(self, linear_vel, angular_vel):
		left_vel = linear_vel - float(self.obj["wheels_base_width"]) / 2 * angular_vel
		right_vel = linear_vel + float(self.obj["wheels_base_width"]) / 2 * angular_vel
		left_rpm, right_rpm = self.vel_to_rpm(left_vel, -right_vel)
		return left_rpm, right_rpm
	
	"""
	++++++++++++++++++++++++++++++++++++++++++
		vel_to_rpm function
	++++++++++++++++++++++++++++++++++++++++++
	Convert from speed to rpm.
	"""
	def vel_to_rpm(self, left_vel, right_vel):
		left_rpm = 60 * left_vel / (2 * np.pi * self.zlc.R_Wheel)
		right_rpm = 60 * right_vel / (2 * np.pi * self.zlc.R_Wheel)
		return left_rpm, right_rpm

	"""
	++++++++++++++++++++++++++++++++++++++++++
		zlac8015d_set_rpm_with_limit function
	++++++++++++++++++++++++++++++++++++++++++
	Set rpm with limit.
	"""
	def zlac8015d_set_rpm_with_limit(self, left_rpm, right_rpm):
		if (-int(self.obj["max_left_rpm"]) < left_rpm < int(self.obj["max_left_rpm"])) and (-int(self.obj["max_right_rpm"]) < right_rpm < int(self.obj["max_right_rpm"])):
			if (-self.deadband_rpm < left_rpm < self.deadband_rpm):
				left_rpm = 0
			if (-self.deadband_rpm < right_rpm < self.deadband_rpm):
				right_rpm = 0

			self.zlc.set_rpm(int(left_rpm), int(right_rpm))
		else:
			rospy.logerr("Set RPM exceeds the limit.")

	"""
	++++++++++++++++++++++++++++++++++++++++++
		calculate_odometry function
	++++++++++++++++++++++++++++++++++++++++++
	Odometry computation.
	"""
	def calculate_odometry(self):
		self.l_meter, self.r_meter = self.zlc.get_wheels_travelled()
		self.l_meter = self.l_meter - self.l_meter_init
		self.r_meter = (-1 * self.r_meter) - (-1 * self.r_meter_init)
		if self.control_mode == 3:
			vl, vr = self.zlc.get_linear_velocities()
		elif self.control_mode == 1:
			vl = (self.l_meter - self.prev_l_meter) / self.period
			vr = (self.r_meter - self.prev_r_meter) / self.period
		
		round_vl = float(Decimal(str(vl)).quantize(Decimal(self.obj["decimil_coefficient"]), rounding = ROUND_HALF_UP))
		round_vr = float(Decimal(str(vr)).quantize(Decimal('0.001'), rounding = ROUND_HALF_UP))
        self.obj["TF_header_flame"]
	
		#-----Rotatiing-----
		if ((round_vl * round_vr) < 0.0) and (abs(round_vl) == abs(round_vr)):
			V = 0.0
			Wz = 2.0 * vr / float(self.obj["wheels_base_width"])
			self.theta = self.theta + Wz * self.period
			path = "Rotatiing"
		
		#-----Curving-----
		elif (round_vl - round_vr) != 0:
			V = (vl + vr) / 2.0
			Wz = (vr - vl) / float(self.obj["wheels_base_width"])
			R_ICC = (float(self.obj["wheels_base_width"]) / 2.0) * ((vr + vl) / (vr - vl))
			self.x = self.x - R_ICC * np.sin(self.theta) + R_ICC * np.sin(self.theta + Wz * self.period)
			self.y = self.y + R_ICC * np.cos(self.theta) - R_ICC * np.cos(self.theta + Wz * self.period)
			self.theta = self.theta + Wz * self.period
			path = "Curving"
			
		#-----Going straight-----
		else:
			V = (vl + vr)/2.0
			Wz = 0.0
			self.x = self.x + V * np.cos(self.theta) * self.period
			self.y = self.y + V * np.sin(self.theta) * self.period
			path = "Going straight"

		#-----Construct tf-----
		q = quaternion_from_euler(0, 0, self.theta)
		self.t.header.stamp = rospy.Time.now()
		self.t.header.frame_id = self.obj["TF_header_flame"]
		self.t.child_frame_id = self.obj["TF_child_flame"]
		self.t.transform.translation.x = self.x
		self.t.transform.translation.y = self.y
		self.t.transform.translation.z = 0.0
		self.t.transform.rotation.x = q[0]
		self.t.transform.rotation.y = q[1]
		self.t.transform.rotation.z = q[2]
		self.t.transform.rotation.w = q[3]
		self.br.sendTransform(self.t)

		#-----Construct odom message-----
		self.odom_msg.header.stamp = rospy.Time.now()
		self.odom_msg.header.frame_id = self.obj["odom_header_flame"]
		self.odom_msg.child_frame_id = self.obj["odom_child_flame"]
		self.odom_msg.pose.pose.position.x = self.x
		self.odom_msg.pose.pose.position.y = self.y
		self.odom_msg.pose.pose.position.z = 0.0
		self.odom_msg.pose.pose.orientation.x = q[0]
		self.odom_msg.pose.pose.orientation.y = q[1]
		self.odom_msg.pose.pose.orientation.z = q[2]
		self.odom_msg.pose.pose.orientation.w = q[3]
		self.odom_msg.pose.covariance[0] = 0.0001
		self.odom_msg.pose.covariance[7] = 0.0001
		self.odom_msg.pose.covariance[14] = 0.000001
		self.odom_msg.pose.covariance[21] = 0.000001
		self.odom_msg.pose.covariance[28] = 0.000001
		self.odom_msg.pose.covariance[35] = 0.0001
		self.odom_msg.twist.twist.linear.x = V
		self.odom_msg.twist.twist.linear.y = 0.0
		self.odom_msg.twist.twist.angular.z = Wz
		self.odom_pub.publish(self.odom_msg)
		
		return vl, vr

	"""
	++++++++++++++++++++++++++++++++++++++++++
		control_loop function
	++++++++++++++++++++++++++++++++++++++++++
	Control loop.
	"""
	def control_loop(self):
		rate = rospy.Rate(20) # 20Hz
		while True:
			if rospy.is_shutdown():
				self.zlc.disable_motor()
				break
				
			start_time = time.perf_counter()

			#-----Speed rpm control mode-----
			if self.control_mode == 3:
				if self.got_twist_cmd:
					self.left_rpm_cmd, self.right_rpm_cmd = self.twist_to_rpm(self.linear_vel_cmd, self.angular_vel_cmd)
					self.zlac8015d_set_rpm_with_limit(self.left_rpm_cmd, self.right_rpm_cmd)
					self.got_twist_cmd = False
				elif self.got_vel_cmd:
					self.left_rpm_cmd, self.right_rpm_cmd = self.vel_to_rpm(self.left_vel_cmd, self.right_vel_cmd)
					self.zlac8015d_set_rpm_with_limit(self.left_rpm_cmd, self.right_rpm_cmd)
					self.got_vel_cmd = False
				elif self.got_vel_rpm_cmd:
					self.zlac8015d_set_rpm_with_limit(self.left_rpm_cmd, self.right_rpm_cmd)
					self.got_vel_rpm_cmd = False
				elif (time.perf_counter() - self.last_subscribed_time > self.callback_timeout):
					self.left_rpm_cmd = 0.0
					self.right_rpm_cmd = 0.0
					self.zlac8015d_set_rpm_with_limit(self.left_rpm_cmd, self.right_rpm_cmd)
				
				fb_L_rpm, fb_R_rpm = self.zlc.get_rpm()
				self.wheels_rpm_msg.data = [fb_L_rpm, fb_R_rpm]
				self.wheels_rpm_pub.publish(self.wheels_rpm_msg)
			
			#-----relative position control mode-----
			elif self.control_mode == 1:
				if self.got_pos_deg_cmd:
					self.zlc.set_relative_angle(self.left_pos_deg_cmd ,self.right_pos_deg_cmd)
					self.zlc.move_left_wheel()
					self.zlc.move_right_wheel()
					self.got_pos_deg_cmd = False
				elif self.got_pos_dist_cmd:
					left_cmd_deg = (self.left_pos_dist_cmd * 360.0) / self.zlc.travel_in_one_rev
					right_cmd_deg = (-self.right_pos_dist_cmd * 360.0) / self.zlc.travel_in_one_rev
					self.zlc.set_relative_angle(left_cmd_deg, right_cmd_deg)
					self.zlc.move_left_wheel()
					self.zlc.move_right_wheel()
					self.got_pos_dist_cmd = False

			#-----Odometry computation-----
			vl, vr = self.calculate_odometry()
	
			self.period = time.perf_counter() - start_time
			self.prev_l_meter = self.l_meter
			self.prev_r_meter = self.r_meter
			
			#-----Logging to screen-----
			if self.control_mode == 1:
				print("\033[32m" + "\rcontrol_mode: {:d} | l_meter: {:.2f} | r_meter: {:.2f} | VL: {:.3f} | VR: {:.3f}".format(\
				self.control_mode, self.l_meter, self.r_meter, vl, vr) + "\033[0m", end = "")
			else:
				print("\033[32m" + "\rcontrol_mode: {:d} | left_rpm: {:.2f} | right_rpm: {:.2f} | l_meter: {:.2f} | r_meter: {:.2f} | VL: {:.3f} | VR: {:.3f}".format(\
				self.control_mode, fb_L_rpm, -fb_R_rpm, self.l_meter, self.r_meter, vl, vr) + "\033[0m", end = "")
			
			rate.sleep()

"""
++++++++++++++++++++++++++++++++++++++++++
	main function
++++++++++++++++++++++++++++++++++++++++++
Main.
"""
def main():
	rospy.init_node("motor_driver_node")
	MDN = MotorDriverNode()
	MDN.control_loop()
	rospy.spin()

if __name__ == "__main__":
	main()