import xml.etree.cElementTree as ET
import numpy as np
from xml.etree import ElementTree
from xml.dom import minidom
import argparse
import math
import matplotlib.pyplot as plt

def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def GenerateRelPosition(agent_number, circle_radius):
	num_agent = agent_number
	radius = circle_radius
	coord2D = np.zeros((agent_number,3))
	base = 10/180.0
	print("base is ", base)

	for i in range(1,agent_number):
		theta = 2 * math.pi * (i-1) /  (agent_number-1) + base
		print("theta is ",theta)
		coord2D[i] = [radius * math.cos(theta), radius * math.sin(theta), theta]

	return np.round(coord2D,10)


def GenerateMavrosLaunch(agent_number, formation_radius, file_name):
	num_agent = agent_number
	fname = file_name
	radius = formation_radius

	"""
	MAVROS posix SITL environment launch script
	launches Gazebo environment and 2x: MAVROS, PX4 SITL, and spawns vehicle
	vehicle model and world
	"""
	
	root = ET.Element("launch")
	ET.SubElement(root, "arg", name="est", value="ekf2")
	ET.SubElement(root, "arg", name="vehicle", value="iris")
	ET.SubElement(root, "arg", name="world", value="$(find mavlink_sitl_gazebo)/worlds/empty.world")

	#Gazebo configs
	ET.SubElement(root, "arg", name="gui", value="true")
	ET.SubElement(root, "arg", name="debug", value="false")
	ET.SubElement(root, "arg", name="verbose", value="false")
	ET.SubElement(root, "arg", name="paused", value="false")

	#Gazebo sim
	include = ET.SubElement(root, "include", file="$(find gazebo_ros)/launch/empty_world.launch")
	ET.SubElement(include, "arg", name="gui", value="$(arg gui)")
	ET.SubElement(include, "arg", name="world_name", value="$(arg world)")
	ET.SubElement(include, "arg", name="debug", value="$(arg debug)")
	ET.SubElement(include, "arg", name="verbose", value="$(arg verbose)")
	ET.SubElement(include, "arg", name="paused", value="$(arg paused)")

	start_coord2D = GenerateRelPosition(num_agent, radius)
	#start_coord2D = np.random.rand(num_agent,2)
	print(start_coord2D)

	for i in range(num_agent):
		name_space = "uav"+str(i)
		udp_port = 14540 + i
		local_host_port = 14580 + i
		mavlink_udp_port = 14560 + i
		mavlink_tcp_port = 4560 + i
		gst_udp_port = 5600 + i
		video_uri = 5600 + i
		mavlink_cam_udp_port = 14530 + i
		fcu_url = "udp://:"+str(udp_port)+"@localhost:"+str(local_host_port)
		tgt_system = 1 + i

		group = ET.SubElement(root, "group", ns=name_space)
		ET.SubElement(group, "arg", name="ID", value=str(i))
		ET.SubElement(group, "arg", name="fcu_url", default=fcu_url)
		include_sub = ET.SubElement(group, "include", file="$(find px4)/launch/single_vehicle_spawn.launch")
		ET.SubElement(include_sub, "arg", name="x", value=str(start_coord2D[i][0]))
		ET.SubElement(include_sub, "arg", name="y", value=str(start_coord2D[i][1]))
		ET.SubElement(include_sub, "arg", name="z", value=str(0))
		ET.SubElement(include_sub, "arg", name="R", value=str(0))
		ET.SubElement(include_sub, "arg", name="P", value=str(0))
		ET.SubElement(include_sub, "arg", name="Y", value=str(0))
		ET.SubElement(include_sub, "arg", name="vehicle", value="$(arg vehicle)")
		ET.SubElement(include_sub, "arg", name="mavlink_udp_port", value=str(mavlink_udp_port))
		ET.SubElement(include_sub, "arg", name="mavlink_tcp_port", value=str(mavlink_tcp_port))
		ET.SubElement(include_sub, "arg", name="ID", value=str(i))
		ET.SubElement(include_sub, "arg", name="gst_udp_port", value=str(gst_udp_port))
		ET.SubElement(include_sub, "arg", name="video_uri", value=str(video_uri))
		ET.SubElement(include_sub, "arg", name="mavlink_cam_udp_port", value=str(mavlink_cam_udp_port))
		include_sub1 = ET.SubElement(group, "include", file="$(find mavros)/launch/px4.launch")
		ET.SubElement(include_sub1, "arg", name="fcu_url", value=fcu_url)
		ET.SubElement(include_sub1, "arg", name="gcs_url", value="")
		ET.SubElement(include_sub1, "arg", name="tgt_system", value=str(tgt_system))
		ET.SubElement(include_sub1, "arg", name="tgt_component", value="1")


		pretty_root = prettify(root)
		tree = ET.ElementTree(ET.fromstring(pretty_root))
		tree.write(fname+".launch")

def GenerateFormationLaunch(agent_number, formation_radius, file_name):
	num_agent = agent_number
	fname = file_name + ".launch"
	fobj = open("../launch/" + fname, "w")
	radius = formation_radius

	"""
	formation controller node launchers
	"""
	
	root = ET.Element("launch")
	start_coord2D = GenerateRelPosition(num_agent, radius)
	#start_coord2D = np.random.rand(num_agent,2)
	print(start_coord2D)

	for i in range(num_agent):
		agent_id = "uav"+str(i)
		uav_id = i
		node_name = "formation"+str(i)

		node = ET.SubElement(root, "node", pkg = "formation_controller", type = "formation_controller_node", name = node_name, output = "screen")
		ET.SubElement(node, "param", name="agent_id", value=agent_id)
		ET.SubElement(node, "param", name="uav_id", value=str(uav_id))
		ET.SubElement(node, "param", name="x_offset", value=str(start_coord2D[i][0]))
		ET.SubElement(node, "param", name="y_offset", value=str(start_coord2D[i][1]))
		ET.SubElement(node, "param", name="z_offset", value=str(0))

		pretty_root = prettify(root)
		tree = ET.ElementTree(ET.fromstring(pretty_root))

		
		
		#tree.write(fname+".launch")
	tree.write(fobj)

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Generate ROS launch file for multi-uav sitl')
	parser.add_argument('--num_agent', required=True, help='number of agent')
	parser.add_argument('--radius', required=True, help='radius of circle formation')
	args = parser.parse_args()
	agent_number = int(args.num_agent)
	radius = int(args.radius)
	assert agent_number<=10, "Agent number must be less than or equal to 10"
	assert radius>0, "Radius must be positive"
	file_name = str(radius) + "m_radius_" + str(agent_number) + "_uav_mavros_sitl"
	GenerateMavrosLaunch(agent_number, radius, file_name)

	file_name_formation = "formation_flt_" + str(radius) + "m_radius_" + str(agent_number) + "_uav"
	GenerateFormationLaunch(agent_number, radius, file_name_formation)
