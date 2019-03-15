1. 在自定义ros消息类型中，如果包含数组，可以直接使用std::vector来进行数组操作（eg: face_detection.cpp）

   python节点中自定义消息类型的使用

2. 通过ROS param服务器获取参数时，存在全局参数，私有参数，以及命名空间下参数三种，可以通过不同的方式进行访问

   c++：

   ​	全局参数：nh.getParam(``"/global_name"``, global_name)

   ​	当前命名空间下的参数：nh.getParam(``"relative_name"``, relative_name)

   ​	私有参数：handle 接口必须创建一个新的 ros::NodeHandle 以一个私有名称空间作为其名称空间。

   ```
   `ros::NodeHandle nh(``"~"``);``std::string param;``nh.getParam(``"private_name"``, param);`
   ```

   python：

   ​	获取全局参数：rospy.get_param(‘/global_param_name')

   ​	获取当前命名空间的参数：rospy.get_param('param_name')

   ​	获取私有命名空间的参数：rospy.get_param('~private_param_name')

   命名空间的设置和param赋值可以在launch文件中进行

   ​	

3. C++节点类内回调函数的调用（eg: face_detection.cpp）

4. dlib的相关用法，dlib在检测人脸时存在速度较慢的情况

5. Object detection 节点使用

   依次运行：

   roslaunch turtlebot_bringup minimal.launch

   roslaunch astra_launch astra.launch

   roslaunch darknet_ros darknet_ros.launch

   rosrun kamerider_image_detection object_detection

   rostopic pub /kamerider_navigation/nav_pub std_msgs/String "in_grasp_position"

   就可以看到检测出物体的画面，以及选取的点，现在问题是tf变换的变换矩阵有明显问题，以及最后选取估计点的时候，因为需要寻找valid point，导致真实选取的点和由bounding_box确定的点偏差较大。
