#
# import os 
  
# uav_num = 10
# root = minidom.Document()
  
# xml = root.createElement('launch') 
# root.appendChild(xml)
  
# productChild = root.createElement('group')
# root.appendChild(xml)
# productChild.setAttribute('ns', 'uav0')
# #productChild.setAttribute('default', 'ekf2')

  
# xml.appendChild(productChild)
  
# xml_str = root.toprettyxml(indent ="\t") 
  
# save_path_file = str(uav_num) + "test_uav_mavros_sitl.launch"
  
# with open(save_path_file, "w") as f:
#     f.write(xml_str) 

import xml.etree.cElementTree as ET
#from ElementTree_pretty import prettify
import numpy as np
from xml.etree import ElementTree
from xml.dom import minidom
import argparse

def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def GenerateLaunch(agent_number, file_name):
	num_agent = agent_number
	fname = file_name
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

	#num_agent = 25
	start_coord2D = np.random.rand(num_agent,2)
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


		# doc = ET.SubElement(root, "doc")
		# #arg1 = ET.SubElement(root, "arg")
		# ET.SubElement(doc, "arg", name="x", value="0")

		# string = "blah"
		# ET.SubElement(doc, "field1", ns=string).text = "some value1"
		# ET.SubElement(doc, "field2", name="asdfasd", value="0").text = "some vlaue2"

		pretty_root = prettify(root)
		tree = ET.ElementTree(ET.fromstring(pretty_root))

		#tree = ET.ElementTree(root)
		#fname = str(num_agent) + "_uav_mavros_sitl"
		tree.write(fname+".launch")

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Generate ROS launch file for multi-uav sitl')
	parser.add_argument('--num_agent', required=True, help='number of agent')
	args = parser.parse_args()
	agent_number = int(args.num_agent)
	assert agent_number<=10, "Agent number must be less than or equal to 10"
	file_name = str(agent_number) + "_uav_mavros_sitl"
	GenerateLaunch(agent_number, file_name)
