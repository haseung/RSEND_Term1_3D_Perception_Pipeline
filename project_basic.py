#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:
    #--------------------------------------------------------------------------
    # TODO: Convert ROS msg to PCL data
    #--------------------------------------------------------------------------    
    cloud = ros_to_pcl(pcl_msg)   

    #--------------------------------------------------------------------------    
    # TODO: Statistical Outlier Filtering
    #--------------------------------------------------------------------------    
    # Create a statistical outlier filter object 
    outlier_filter = cloud.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(50)

    # Set threshold scale factor
    x = 0.001

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Call the filter function with statistical outlier filter applied for resultant point cloud
    cloud = outlier_filter.filter()   

    # filename = 'statistical_outlier_filtered.pcd'
    # pcl.save(cloud, filename)

    #--------------------------------------------------------------------------    
    # TODO: Voxel Grid Downsampling (DS)
    #--------------------------------------------------------------------------    
    # Create a voxel grid DS filter object
    vox = cloud.make_voxel_grid_filter() #mod 1

    # Choose a voxel (also known as leaf) size
    LEAF_SIZE = .005   

    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function with Voxel Grid DS applied for resultant point cloud
    cloud_filtered = vox.filter()
    
    # filename = 'voxel_downsampled.pcd'
    # pcl.save(cloud_filtered, filename)   

    #--------------------------------------------------------------------------    
    # TODO: PassThrough Filter
    #--------------------------------------------------------------------------    

    ### Z-axis    
    # Create a passthrough filter object
    passthrough = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.61
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)

    # Call the filter function with passthrough filter applied for resultant point cloud
    cloud_filtered = passthrough.filter()

    # filename = 'pass_through_filtered_z.pcd'
    # pcl.save(cloud_filtered, filename)


    ### Y-axis    
    # Create a passthrough filter object
    passthrough = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.45
    axis_max = 0.45
    passthrough.set_filter_limits(axis_min, axis_max)

    # Call the filter function with passthrough filter applied for resultant point cloud
    cloud_filtered = passthrough.filter()

    # filename = 'pass_through_filtered_zy.pcd'
    # pcl.save(cloud_filtered, filename)

    #--------------------------------------------------------------------------    
    # TODO: RANSAC Plane Segmentation
    #--------------------------------------------------------------------------    
    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()

    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    max_distance = .01
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    #--------------------------------------------------------------------------    
    # TODO: Extract inliers and outliers
    #--------------------------------------------------------------------------    
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)   
    cloud_table = extracted_inliers

    # filename = 'extracted_inliers.pcd'
    # pcl.save(extracted_inliers, filename)
   
    extracted_outliers = cloud_filtered.extract(inliers, negative=True)    
    cloud_objects = extracted_outliers

    filename = 'extracted_outliers.pcd'
    pcl.save(extracted_outliers, filename)
  
    #--------------------------------------------------------------------------    
    # TODO: Euclidean Clustering
    #--------------------------------------------------------------------------    
    # Convert PCL 'cloud objects' point cloud to an XYZ point cloud (removes color info)   
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(.03)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(10000)

    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)

    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    #--------------------------------------------------------------------------    
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #--------------------------------------------------------------------------    
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)    

    #--------------------------------------------------------------------------    
    # TODO: Convert PCL data to ROS messages
    #--------------------------------------------------------------------------    
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    #--------------------------------------------------------------------------    
    # TODO: Publish ROS messages
    #--------------------------------------------------------------------------    
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

#--------------------------------------------------------------------------    
# Exercise-3 TODOs:
#--------------------------------------------------------------------------    

    #--------------------------------------------------------------------------    
    # Classify the clusters! (loop through each detected cluster one at a time)
    #--------------------------------------------------------------------------    
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        #----------------------------------------------------------------------    
        # Grab the points for the cluster
        #----------------------------------------------------------------------    
        pcl_cluster = cloud_objects.extract(pts_list)
    
        #----------------------------------------------------------------------    
        # Compute the associated feature vector
        #----------------------------------------------------------------------    
        # Convert the cluster from pcl to ROS using helper function    
        ros_cluster = pcl_to_ros(pcl_cluster)        
        
        # Extract histogram features
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
   
        #----------------------------------------------------------------------    
        # Make the prediction
        #----------------------------------------------------------------------    
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        #----------------------------------------------------------------------    
        # Publish a label into RViz
        #----------------------------------------------------------------------    
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        #----------------------------------------------------------------------    
        # Add the detected object to the list of detected objects.
        #----------------------------------------------------------------------    
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    #----------------------------------------------------------------------    
    # Publish the list of detected objects
    #----------------------------------------------------------------------    
    detected_objects_pub.publish(detected_objects)

    #----------------------------------------------------------------------    
    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    #----------------------------------------------------------------------        

    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

#------------------------------------------------------------------------------
# function to load parameters and request PickPlace service
#----------------------------------------------------------------------------
def pr2_mover(object_list):

    #------------------------------------------------------------------------------
    # TODO: Initialize variables
    #------------------------------------------------------------------------------
    test_scene_num = Int32()
    object_name = String()
    object_group = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()
    dict_list = []

    #------------------------------------------------------------------------------
    # Set test scene number
    #------------------------------------------------------------------------------
    test_scene_num.data = 3

    #------------------------------------------------------------------------------
    # TODO: Get/Read parameters
    #------------------------------------------------------------------------------
    object_list_param = rospy.get_param('/object_list')
    # print(object_list_param)
    
    dropbox_param = rospy.get_param('/dropbox')

     #------------------------------------------------------------------------------
    # Grab labels, access the XYZ coords of each point and compute centroid of detected objects
    #------------------------------------------------------------------------------
    labels = []
    centroids = [] # to be list of tuples (x, y, z)
    for object in object_list:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])

    # print(centroids)
    #------------------------------------------------------------------------------
    # TODO: Loop through the pick list
    #------------------------------------------------------------------------------

    for i in range(0, len(object_list_param)):
        #------------------------------------------------------------------------------
        # TODO: Parse parameters into individual variables
        #------------------------------------------------------------------------------
        object_name.data = object_list_param[i]['name']
        object_group = object_list_param[i]['group']        

        #----------------------------------------------------------------------
        # TODO: Get the PointCloud for a given object and obtain it's centroid
        #----------------------------------------------------------------------    
        pick_pose.position.x = np.asscalar(centroids[i][0])
        pick_pose.position.y = np.asscalar(centroids[i][1])
        pick_pose.position.z = np.asscalar(centroids[i][2])  
      
        #--------------------------------------------------------------------------
        # TODO: Create 'place_pose' for the object and assign the arm to be used for pick_place
        #--------------------------------------------------------------------------
        if object_group == 'red':
            dropbox_position = dropbox_param[0]['position'] 
            arm_name.data = 'left'           
        elif object_group == 'green':
            dropbox_position = dropbox_param[1]['position']
            arm_name.data = 'right'
        else:        
            print('Error reading dropbox position')    
      
        place_pose.position.x = dropbox_position[0]
        place_pose.position.y = dropbox_position[1]
        place_pose.position.z = dropbox_position[2]
     
        #--------------------------------------------------------------------------
        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        #--------------------------------------------------------------------------
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            #----------------------------------------------------------------------
            # TODO: Insert your message variables to be sent as a service request
            #----------------------------------------------------------------------
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    #------------------------------------------------------------------------------
    # TODO: Output your request parameters into output yaml file
    #------------------------------------------------------------------------------
    yaml_filename = 'output_'+str(test_scene_num.data)+'.yaml'
    send_to_yaml(yaml_filename, dict_list)

if __name__ == '__main__':
    #------------------------------------------------------------------------------
    # TODO: ROS node initialization
    #------------------------------------------------------------------------------
    rospy.init_node('clustering', anonymous=True)

    #------------------------------------------------------------------------------
    # TODO: Create Subscribers
    #------------------------------------------------------------------------------
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    #------------------------------------------------------------------------------
    # TODO: Create Publishers
    #------------------------------------------------------------------------------
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    #------------------------------------------------------------------------------
    # TODO: Initialize color_list
    #------------------------------------------------------------------------------
    get_color_list.color_list = []

    #------------------------------------------------------------------------------
    # TODO: Load Model From disk
    #------------------------------------------------------------------------------
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    #------------------------------------------------------------------------------
    # TODO: Spin while node is not shutdown
    #------------------------------------------------------------------------------
    while not rospy.is_shutdown():
     rospy.spin()

    ##################################### END #################################### 
