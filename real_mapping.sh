colcon build --symlink-install
cmds=(

	"
		ros2 launch livox_ros_driver2  msg_MID360_launch.py
	"

	"	ros2 launch pb_rm_simulation rm_real.launch.py \
		use_sim_time:=false
	"

	"	
		ros2 launch imu_complementary_filter complementary_filter.launch.py \
		use_sim_time:=false
	"

	# "
    # 	ros2 launch fast_lio mapping.launch.py \
	# 	use_sim_time:=false 
	# "


	"
		ros2 launch point_lio point_lio.launch.py	\
		use_sim_time:=false
	"
	
	# "
    # 	ros2 launch point_lio_cxr mapping_mid360.launch.py \
	# 	use_sim_time:=false rviz:=true
	# "

	# "
	# 	ros2 launch terrain_analysis terrain_analysis.launch \
	# 	use_sim_time:=false
	# "

	# "
	# 	ros2 launch terrain_analysis_ext terrain_analysis_ext.launch \
	# 	use_sim_time:=false
	# "

	"
		ros2 launch linefit_ground_segmentation_ros segmentation.launch.py \
		use_sim_time:=false
	"
	
	"
		ros2 launch merge_cloud merge_cloud_code.launch.py \
		use_sim_time:=false
	"

	"
		ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py \
		use_sim_time:=false
	"


	"
		ros2 launch rm_nav_bringup online_async_launch.py \
		use_sim_time:=false
	"

	"./transhform_map_to_odom.sh"

	# "
	# 	ros2 launch fake_vel_transform fake_vel_transform_launch.py \
	# 	use_sim_time:=false
	# "

	"
		ros2 launch rm_nav_bringup bringup_no_amcl_launch.py \
		use_sim_time:=false
	"
		
	"	
		ros2 launch rm_serial_driver serial_driver.launch.py \
		use_sim_time:=false
	"

	# " ros2 launch hx_decsion hx_decsion_launch.py file_path:=src/hx_decsion/config/homec01.xml          "
)

for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.5
done
