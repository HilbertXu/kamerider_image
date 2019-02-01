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