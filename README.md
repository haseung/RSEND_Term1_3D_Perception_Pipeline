## Project: Perception Pick & Place
### Author: Harrison Seung
### Date: 07/30/2018
--

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

The following README is for the Udacity Robotics Software Engineering Nanodegree, Term 1, Project 3, 3D Perception.  

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

The first step in developing the 3D perception pipeline for a tabletop pick and place is to take an image from an RGBD sensor of the PR2 robot in the form of Point Cloud Data (PCD) and apply some of the most commonly used filters from the Point Cloud Library to aid in extracting a subset of desired information.  The filters used are described below in the order applied.   

1.  Statistical Outlier Removal Filter - The first filter applied is the statistical outlier filter. This eliminates any noisy data in the data file.

![Statistical Outlier Filter](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/statistical_outlier.JPG)
(PCD with Statistical Outlier Filter applied)

2. Voxel Downsampling Filter - The Voxel Downsample filter is used to derive a point cloud with fewer points but still does a good job representing the overall point cloud.

![Voxel Downsampling Filter](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/voxel_downsampling.JPG)
(PCD with Voxel Downsampling Filter applied)

3. Pass Through Filter - Next, a passthrough filter in the Z and Y axis is used to eliminate areas of the point cloud that are not pertinent.  In this case, the point cloud is isolated to purely the objects on the table.

![Passthrough Filter](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/pass_through_filtered_zy.JPG)
(PCD with Pass Through Filter in Z and Y axis applied)

4. RANSAC plane segmentation - Lastly, the objects are removed from the table using a Random Sample Consensus or RANSAC algorithm.  RANSAC works by assuming all of the data in the dataset is composed of inliers and outliers.  Whether a set of data is considered an inlier if it fits a particular model based on a specific set parameter.  In this scenario the model type is pcl.SACMODEL_PLANE and the parameter is a max distance from that predefined model.  As the filtered Point Cloud Data only contains the table and objects, the table fits a PLANE and thus will be considered an INLIER.  By extracting the outliers we obtain the desired objects on the table.

![Extracted Inliers](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/extracted_inliers.JPG)
(PCD with RANSAC plane segmentation applied.  Extracted inliers)

![Extracted Outliers](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/extracted_outliers.JPG)
(PCD with RANSAC plane segmentation applied.  Extracted outliers)

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

The second step in the 3D Perception pipeline is to implement the steps from Exercise 1 into a ROS node and subscribe to '/pr2/world/points' topic.  This will extract the image from the PR2 RGBD sensor to be filtered.  Once the image and filtering has been performed, the next step is to apply Euclidean clustering to create separate clusters for each item on the table.

!['/pr2/world/points' topic shown](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/Topics.JPG)
(Required topics subscribed and published too)

!['/pr2/world/points' topic shown](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/pcl_cluster.JPG)
(PCL clustered)

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

The third step in the 3D Perception project is to implement Object recognition.  This is done by first generating a training set of features corresponding to the pick lists for test worlds 1, 2 or 3 using the 'captures_features.py' script resulting in the 'training_set.sav' file.

![capture features](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/capture_features.JPG)
(Capturing features for pick list 3)

The 'training_set.sav' file is then used to train the SVM with features from the new models, saved in 'model.sav'.  The relative accuracy of the model is shown below.

!['confusion matrix'](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/CMWN%20Test1.png)
(Confusion matrix without normalization for Test World 1)

!['normalized confusion matrix'](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/NCM%20Test1.png.JPG)
(Normalized confusion matrix for Test World 1)

!['confusion matrix'](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/CMWN%20Test2.png)
(Confusion matrix without normalization for Test World 2)

!['normalized confusion matrix'](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/NCM%20Test2.png)
(Normalized confusion matrix for Test World 2)

!['confusion matrix'](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/CMWN%20Test3.png)
(Confusion matrix without normalization for Test World 3)

!['normalized confusion matrix'](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/NCM%20Test3.png)
(Normalized confusion matrix for Test World 3)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

To complete the pick and place for the PR2 environment, code developed through exercises 1 through 3 were implemented in the 'project_basic.py' file to scan three different tabletop setups.  

In order to switch between test worlds the following modifications were required:

1. In 'capture_features.py', change the 'models' variable to the desired pick_list
  
  !['pick_list_1'](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/pick_list_1.jpg)
  (Example: Pick List 1)

2. In 'pick_place_project.launch', modify the following line to the desired test world:  
  <arg name="world_name" value="$(find pr2_robot)/worlds/test3.world"/>

   and the following line to desired pick list:
  <rosparam command="load" file="$(find pr2_robot)/config/pick_list_3.yaml"/>

3. In 'project_basic.py', change the 'test_scene_num.data' to the desired test world Number

4. Perform feature extraction and SVM training per exercise 3 to obtain 'training_set.sav' and 'model.sav' files.  Move these files to the same script folder as 'project_basic.py'

5. Launch Pick and Place 3D perception RVIZ and GAZEBO environment using 'roslaunch pr2_robot pick_place_project.launch'

6. In a separate terminal, run the './project_basic.py' script.  Run the script for each of the three test worlds and this will produce the required 3 output_*.yaml files.

The success of the PR2 robot in recognizing an object and correctly placing it in the designated bin relies on the accuracy of the SVM model trained.  For the purposes of this project, the accuracy of the models ranged from 83% to 93% were sufficient to meet the minimum required object recognition.  For an industrial application where thousands of products would be processed, a 10% error could lead to significant attrition.  With more time, I would modify the parameters for the capture_features operation, increasing iterations per object and experimenting with different kernels to improve accuracy.  

!['Test World 1'](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/Capture_test1.JPG)
(Test World 1 Object Recognition)

!['Test World 2'](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/Capture_test2.JPG)
(Test World 2 Object Recognition)

!['Test World 3'](https://github.com/haseung/term1_project3_3Dperception/blob/master/Figures/Capture_test3.JPG)
(Test World 3 Object Recognition)
