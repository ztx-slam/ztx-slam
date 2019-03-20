author:1617709246@qq.com

this packages is for learn the frame of 3D lidar slam

1.pcl_ros
PCL_ROS是在ROS中涉及n维点云和3D几何处理的3D应用的首选桥梁
eg：rosrun pcl_ros bag_to_pcd <input_file.bag> <topic> <output_directory> 读取一个包文件，保存所有ROS点云消息在指定的PCD文件
    rosrun pcl_ros convert_pcd_to_image <cloud.pcd>  从PCD文件中生成的图像流
    rosrun pcl_ros convert_pointcloud_to_image input:=/my_cloud output:=/my_image 订阅一个ros点云话题和重发布图像消息
    rosrun pcl_ros pcd_to_pointcloud cloud_file.pcd 0.1 _frame_id:=/odom  加载一个PCD文件，把它发布为一个点云消息，发布10次/秒 在/odom参考坐标系
    rosrun pcl_ros pointcloud_to_pcd input:=/velodyne/pointcloud2 订阅/ velodyne / pointcloud2话题并将每条消息保存在当前目录中。文件名看起来像1285627014.833100319.pcd
