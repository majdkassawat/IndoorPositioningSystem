# In this file we manage the scanning of the scene, this works by getting the
# markers dictionary from aruco(or others in the future) and storing
# the relations between the visible markers over the scanning time.
# We record a certain number of iterations for each relation between the markers
# and produce a list containing all the relations with all their iterations.
# When the scanning is done, the list is simplified to contain the averages of the iterations.
# The list is then stored. Here we note that the list does not have a unified reference.
# That is to be selected by the user in IndPose.py
from __future__ import print_function
import rospy
import roslib
import tf
import signal
import time
import threading
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import UInt32MultiArray
import geometry_msgs.msg
from itertools import combinations
import pprint
import json

markers_map_file_path = "/home/majd/average_markers.json"
max_iterations = 600
total_number_of_markers = 5
freq = 30
# Dictionary visible markers
refrences_dict = {}
connections_dict = {}
markers_list_msg = UInt32MultiArray()

cancel_signal = False
t = tf.Transformer(True)
br = tf.TransformBroadcaster()
overall_markers_list = []


def Markers_list_Callback(msg):
    global refrences_dict
    for id, refrence in refrences_dict.items():
        if int(id) not in msg.data:
            del refrences_dict[id]


def MarkersCallback(msg):
    global refrences_dict

    for marker in msg.markers:
        refrences_dict[marker.id] = marker.pose.pose


rospy.init_node('scan_scene', log_level=rospy.INFO)
sub = rospy.Subscriber('aruco_marker_publisher/markers',
                       MarkerArray, MarkersCallback)
sub_markers_list = rospy.Subscriber(
    'aruco_marker_publisher/markers_list', UInt32MultiArray, Markers_list_Callback)


def sigint_handler(signum, frame):
    global cancel_signal
    cancel_signal = True


signal.signal(signal.SIGINT, sigint_handler)


def pair_exists(pair):
    result = False
    id = 99999999
    for k, connection in connections_dict.items():
        if connection["m1"] == pair[0] and connection["m2"] == pair[1]:
            result = True
            id = k
            break
    return result, id


def calculate_pair_transformation(pair):
    #  calculates transformation from pair[0] to pair[1]
    global t
    transformation = t.lookupTransform(
        str(pair[0]), str(pair[1]), rospy.Time())
    transformation_dict = {}
    transformation_dict["position"] = {}
    transformation_dict["rotation"] = {}
    transformation_dict["position"]["x"] = transformation[0][0]
    transformation_dict["position"]["y"] = transformation[0][1]
    transformation_dict["position"]["z"] = transformation[0][2]
    transformation_dict["rotation"]["x"] = transformation[1][0]
    transformation_dict["rotation"]["y"] = transformation[1][1]
    transformation_dict["rotation"]["z"] = transformation[1][2]
    transformation_dict["rotation"]["w"] = transformation[1][3]
    return transformation_dict


def br_t_calculate_pair_transformation(pair, br_t):
    #  calculates transformation from pair[0] to pair[1]
    transformation = br_t.lookupTransform(
        str(pair[0]), str(pair[1]), rospy.Time())
    transformation_dict = {}
    transformation_dict["position"] = {}
    transformation_dict["rotation"] = {}
    transformation_dict["position"]["x"] = transformation[0][0]
    transformation_dict["position"]["y"] = transformation[0][1]
    transformation_dict["position"]["z"] = transformation[0][2]
    transformation_dict["rotation"]["x"] = transformation[1][0]
    transformation_dict["rotation"]["y"] = transformation[1][1]
    transformation_dict["rotation"]["z"] = transformation[1][2]
    transformation_dict["rotation"]["w"] = transformation[1][3]
    return transformation_dict


def Reverse(tuples):
        # Reversing a tuple using slicing technique
        # New tuple is created
    new_tup = tuples[::-1]
    return new_tup


def register_transformation(pair):
    res1, id1 = pair_exists(pair)
    res2, id2 = pair_exists(Reverse(pair))
    if res1:
        # insert iteration in id1
        transformation = calculate_pair_transformation(pair)
        new_transformation_iteration_key = str(
            len(connections_dict[id1]["transformation_iterations"]))
        if int(new_transformation_iteration_key) >= max_iterations:
            return
        connections_dict[id1]["transformation_iterations"][new_transformation_iteration_key] = transformation
    elif res2:
        # insert iteration in id2
        transformation = calculate_pair_transformation(Reverse(pair))
        new_transformation_iteration_key = str(
            len(connections_dict[id2]["transformation_iterations"]))
        if int(new_transformation_iteration_key) >= max_iterations:
            return
        connections_dict[id2]["transformation_iterations"][new_transformation_iteration_key] = transformation
    else:
        # insert new connection
        new_connection = {"m1": pair[0], "m2": pair[1]}
        transformation_iterations = {}
        transformation = calculate_pair_transformation(pair)
        transformation_iterations["0"] = transformation
        new_connection["transformation_iterations"] = transformation_iterations
        new_connection_key = str(len(connections_dict))
        connections_dict[new_connection_key] = new_connection


def fill_overall_markers_list():
    for k, connection in connections_dict.items():
        if overall_markers_list.count(connection["m1"]) == 0:
            overall_markers_list.insert(
                len(overall_markers_list), connection["m1"])
        if overall_markers_list.count(connection["m2"]) == 0:
            overall_markers_list.insert(
                len(overall_markers_list), connection["m2"])


def get_least_scanned():
    least_iterations = max_iterations
    least_scanned = None
    for k, connection in connections_dict.items():
        if len(connection["transformation_iterations"]) <= least_iterations:
            least_iterations = len(connection["transformation_iterations"])
            least_scanned = [connection["m1"], connection["m2"]]
    return least_scanned, least_iterations


def broadcast():
    # set inter-marker connections
    
    for k, connection in connections_dict.items():
        m1 = connection["m1"]
        m2 = connection["m2"]
        t_pos_x = connection["transformation_iterations"][str(len(
            connection["transformation_iterations"]) - 1)]["position"]["x"]
        t_pos_y = connection["transformation_iterations"][str(len(
            connection["transformation_iterations"]) - 1)]["position"]["y"]
        t_pos_z = connection["transformation_iterations"][str(len(
            connection["transformation_iterations"]) - 1)]["position"]["z"]
        t_rot_x = connection["transformation_iterations"][str(len(
            connection["transformation_iterations"]) - 1)]["rotation"]["x"]
        t_rot_y = connection["transformation_iterations"][str(len(
            connection["transformation_iterations"]) - 1)]["rotation"]["y"]
        t_rot_z = connection["transformation_iterations"][str(len(
            connection["transformation_iterations"]) - 1)]["rotation"]["z"]
        t_rot_w = connection["transformation_iterations"][str(len(
            connection["transformation_iterations"]) - 1)]["rotation"]["w"]
        br.sendTransform((t_pos_x, t_pos_y, t_pos_z),
                         (t_rot_x, t_rot_y,
                          t_rot_z, t_rot_w),
                         rospy.Time.now(),
                         str(m2),
                         str(m1))
      #   e_m_t = geometry_msgs.msg.TransformStamped()
      #   e_m_t.header.frame_id = str(m1)
      #   e_m_t.child_frame_id = str(m2)
      #   e_m_t.transform.translation.x = t_pos_x
      #   e_m_t.transform.translation.y = t_pos_y
      #   e_m_t.transform.translation.z = t_pos_z
      #   e_m_t.transform.rotation.x = t_rot_x
      #   e_m_t.transform.rotation.y = t_rot_y
      #   e_m_t.transform.rotation.z = t_rot_z
      #   e_m_t.transform.rotation.w = t_rot_w
      #   br_t.setTransform(e_m_t)
    # set visible markers to camera connections
    for k, reference in refrences_dict.items():
        br.sendTransform((reference.position.x, reference.position.y, reference.position.z),
                         (reference.orientation.x, reference.orientation.y,
                          reference.orientation.z, reference.orientation.w),
                         rospy.Time.now(),
                         str(k),
                         '/camera_rgb_frame')
        
      #   m_c_t = geometry_msgs.msg.TransformStamped()
      #   m_c_t.header.frame_id = '/camera_rgb_frame'
      #   m_c_t.child_frame_id = str(k)
      #   m_c_t.transform.translation.x = reference.position.x
      #   m_c_t.transform.translation.y = reference.position.y
      #   m_c_t.transform.translation.z = reference.position.z
      #   m_c_t.transform.rotation.x = reference.orientation.x
      #   m_c_t.transform.rotation.y = reference.orientation.y
      #   m_c_t.transform.rotation.z = reference.orientation.z
      #   m_c_t.transform.rotation.w = reference.orientation.w
      #   br_t.setTransform(m_c_t)
   #  for marker in overall_markers_list:

   #      if br_t.canTransform(str(marker), '/camera_rgb_frame', rospy.Time()) == True:
   #          print("True", [str(marker), '/camera_rgb_frame'])
   #      else:
   #          print("False", [str(marker), '/camera_rgb_frame'])
   #      transformation = br_t_calculate_pair_transformation(
   #          [str(marker), '/camera_rgb_frame'], br_t)
   #      br.sendTransform((transformation["position"]["x"], transformation["position"]["y"], transformation["position"]["z"]),
   #                       (transformation["rotation"]["x"], transformation["rotation"]["y"],
   #                        transformation["rotation"]["z"], transformation["rotation"]["w"]),
   #                       rospy.Time(),
   #                       '/camera_rgb_frame',
   #                       str(marker))


def process_markers():
    global refrences_dict, cancel_signal
    for k, reference in refrences_dict.items():
        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = '/camera_rgb_frame'
        m.child_frame_id = str(k)
        m.transform.translation.x = reference.position.x
        m.transform.translation.y = reference.position.y
        m.transform.translation.z = reference.position.z
        m.transform.rotation.x = reference.orientation.x
        m.transform.rotation.y = reference.orientation.y
        m.transform.rotation.z = reference.orientation.z
        m.transform.rotation.w = reference.orientation.w
        t.setTransform(m)
    pairs = list(combinations(refrences_dict, 2))
    for pair in pairs:
        register_transformation(pair)

    fill_overall_markers_list()
    least_scanned, least_iterations = get_least_scanned()
    output_string = "#mrk scn:" + \
        (str(len(overall_markers_list))) + \
        ", least scanned:" + str(least_scanned) + ":" + str(least_iterations)
    print(output_string)
    broadcast()

    #  stop and save when total number of markers and maximum number of iterations reached
    if (len(overall_markers_list) == total_number_of_markers) and (least_iterations == max_iterations):
        average_connections_dic = {}
        for k, connection in connections_dict.items():
            average_connection = {}
            average_connection["m1"] = connection["m1"]
            average_connection["m2"] = connection["m2"]
            average_connection["transformation"] = {
                "position": {}, "rotation": {}}
            position_sum_x = 0
            position_sum_y = 0
            position_sum_z = 0
            rotation_sum_x = 0
            rotation_sum_y = 0
            rotation_sum_z = 0
            rotation_sum_w = 0

            for v, tranformation in connection["transformation_iterations"].items():
                position_sum_x = position_sum_x + \
                    float(tranformation["position"]["x"])
                position_sum_y = position_sum_y + \
                    float(tranformation["position"]["y"])
                position_sum_z = position_sum_z + \
                    float(tranformation["position"]["z"])
                rotation_sum_x = rotation_sum_x + \
                    float(tranformation["rotation"]["x"])
                rotation_sum_y = rotation_sum_y + \
                    float(tranformation["rotation"]["y"])
                rotation_sum_z = rotation_sum_z + \
                    float(tranformation["rotation"]["z"])
                rotation_sum_w = rotation_sum_w + \
                    float(tranformation["rotation"]["w"])
            average_connection["transformation"]["position"]["x"] = position_sum_x / \
                len(connection["transformation_iterations"])
            average_connection["transformation"]["position"]["y"] = position_sum_y / \
                len(connection["transformation_iterations"])
            average_connection["transformation"]["position"]["z"] = position_sum_z / \
                len(connection["transformation_iterations"])
            average_connection["transformation"]["rotation"]["x"] = rotation_sum_x / \
                len(connection["transformation_iterations"])
            average_connection["transformation"]["rotation"]["y"] = rotation_sum_y / \
                len(connection["transformation_iterations"])
            average_connection["transformation"]["rotation"]["z"] = rotation_sum_z / \
                len(connection["transformation_iterations"])
            average_connection["transformation"]["rotation"]["w"] = rotation_sum_w / \
                len(connection["transformation_iterations"])
            average_connections_dic[len(
                average_connections_dic)] = average_connection

        with open(markers_map_file_path, 'w') as fp:
            json.dump(average_connections_dic, fp)
        print("saved to:", markers_map_file_path)
        cancel_signal = True

    # If cancel signal is not received (ctrl+c), then execute this function again respecting the frequency (here no need for ros sleep)
    if cancel_signal != True:
        sleep_time = 1/float(freq)
        threading.Timer(sleep_time, process_markers).start()


process_markers()
