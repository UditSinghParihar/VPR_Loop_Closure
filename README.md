# Commands


1. Record Depth, RGB, Pose and TF
2. `python getPose.py slam_images/frame002417.jpg slam_images/frame003400.jpg`
3. `python getPoseDir.py slam_images_comb5/rgb/ slam_images_comb5/depth/ loop_pairs5.txt`
4. `python getPose.py slam_images_comb5/rgb/rgb000631.jpg slam_images_comb5/rgb/rgb002819.jpg` # Outside virtual env; For extracting correspondences
5. `python ../test_manual_icp.py rgb/rgb000631.jpg depth/depth000631.png rgb/rgb002819.jpg depth/depth002819.png ../src_pts.txt ../trg_pts.txt` # Inside virtual env; For ICP based on correspondences

## RTABMAP

### To ensure same time use by all nodes, launch commands in sequence.  

1. `roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" visual_odometry:=false frame_id:=/base_link odom_topic:=/RosAria/pose depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info rtabmapviz:=true rviz:=false use_sim_time:=true`    
2. `rosbag play corridor5.bag --clock`    
3. `rosrun tf static_transform_publisher 0.27 0 0.54 0 0.331613 0 base_link camera_link 100`    
4. `rosrun tf tf_echo base_link camera_depth_optical_frame` # Should be same as time according to bag file and not according to wall clock (current) time.    
5. `date +%s` # To get current time


## TF problem while working with Bagfiles    

1. `rosparam set use_sim_time true` $ Ensuring all nodes use same time  
2. `rosbag play <file.bag> --clock` $ To use simulation time  
3. `rosrun tf static_transform_publisher 
x y z yaw pitch roll frame_id child_frame_id period_in_ms` $ tf would also be published in simulated time  
4. `roslaunch rtabmap_ros rtabmap.launch` $ Launch any node using rosbag topics and static tf info  

## Optimizing pose graph  
`python readPose.py poses5.txt loop_pairs5.txt` # Based on fixed transformation  
`python readPose2.py poses5.txt transformation5.txt` # Based on transformations from a file

## Register
`python register.py slam_images_comb5/rgb/ slam_images_comb5/depth/ ~/Desktop/rtabmap/poses.g2o`  


## EVO  
1. For alignment:  
	`evo_traj kitti vpr.kitti --ref denseGt2.kitti -a --n_to_align 70 --plot --plot_mode xy --save_as_kitti`  
2. For ate caculation:  
	`evo_ape kitti denseGt2.kitti vpr5.kitti -v --plot --plot_mode xy`  


## Kdenlive  
1. `iros4.kdenlive` == `iros_vid8.mp4` was submitted to IROS.  
2. Video animation
	1. `python video2.py ../slam_images_comb5/rgb/ ../poses_comb5.txt ../transformations5.txt`  