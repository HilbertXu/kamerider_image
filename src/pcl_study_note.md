#这是学习PCL基本操作以及如何在ROS中应用的笔记

**使用须知**
必须安装pcl-1.9.1 否则会缺少一系列与点云分割有关的头文件
'
	git clone https://github.com/PointCloudLibrary/pcl.git

	cd pcl
	mkdir release
	cd release
	cmake -DCMAKE_BUILD_TYPE=None -DCMAKE_INSTALL_PREFIX=/usr \
		-DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_examples=ON \
		-DCMAKE_INSTALL_PREFIX=/usr ..
	make -j8
	sudo make install
'
源码安装完之后在cmakelists.txt里添加如下语句
find_package(PCL 1.9 REQUIRED)
然后在对每一个cpp文件编译时的target_link_libraries里加入${PCL_LIBRARIES}
eg: target_link_libraries(pcl_segmentate ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})


##ROS点云与PCL点云之间的转换

###PCL基础点云类的数据结构
PCL包含了一个很重要得到数据结构，叫PointCloud，这是一个模板类，把点的类型作为模板参数。

下面是最重要的在点云里面的公共域

header 这个域是pcl::PCLHeader类型和指定点云的获取时间。

points:这个域是std::vector<PointT,..>类型，是点云存储的容器。PointT是类模板参数。

width:这个域指定点云的宽度在组织一个图像的时候，否则只有一个。

height:这个域指定点云的高度，没有指定则只有一个。

is_dense:这个域指定点云是否包含无效值(infinite或者NaN)

sensor_origin_:这个域是Eigen::Vector4f 类型，它定义 了传感器的获取姿势就一个区域的转换而言。

sensor_orientation_：这个域是Eigen::Quaternion类型，他定义了sensor作为一个旋转的角度而言。

###PCL中不同的点云类型
1. pcl::PointXYZ 这是最简单的点的类型，存储着点的x,y,z信息。

2. pcl::PointXYZI 这个类型的点是和前面的那个很相似的，但是他也包含一个域描述了点的密集程度。另外还有两个其他的标准的特殊的点的类型:第一个是pcl::InterestPoint，它有一个域去存储长处而不是密集度。pcl::PointWithRange，存储了点的范围(深度).

3. pcl::PointXYZRGBA 这个类型的点存储了3D信息同时和RGB与Alpha(透明度）

4. pcl::PointXYZRGB 坐标+RGB值

5. pcl::Normal 这是一个最常用的点的类型，它代表了给定点的 曲面法线(normal翻译为法线有点奇怪)和曲率的测量。

6. pcl::PointNormal 这个类型和前面那个一样。只不过它多了坐标(x,y,z)。他的变体有PointXYZRGBNormal和PointXYZINormal,就像名字所说的一样，前者包含颜色，后者包含密集度

###PCL对ROS的接口
PCL对ROS的接口提供了PCL数据结构的交流，通过ROS提供的以消息为基础的交流系统，在ROS系统中点云信息一般使用**sensor_msgs::PointCloud2**来储存点云数据
使用*pcl_conversions.h*中的如下函数进行PCL格式的点云与ROS message格式点云的转换:
1. **ROS转PCL数据格式**
-sensor_msgs::PointCloud2转pcl::PCLPointCloud2

'pcl_conversions::toPCL (sensor_msgs::PointCloud2, pcl::PCLPointCloud2)'

-sensor_msgs::PointCloud2转pcl::PointCloud<pcl::PointXYZ>

'pcl::fromROSMsg (sensor_msgs::PointCloud2, pcl::PointCloud<pcl::PointXYZ>)'

2. **PCL转ROS数据格式**
-pcl::PCLPointCloud2转sensor_msgs::PointCloud2

'pcl_conversions::fromPCL (pcl::PCLPointCloud2, sensor_msgs::PointCloud2)'

-pcl::PointCloud<pcl::PointXYZ>转sensor_msgs::PointCloud2

'pcl::toROSMsg (pcl::PointCloud<pcl::PointXYZ>, sensor_msgs::PointCloud2)'

3. **PCL中数据互转**
-pcl::PCLPointCloud2转pcl::PointCloud<pcl::PointXYZ>

'pcl::fromPCLPointCloud2(pcl::PCLPointCloud2, pcl::PointCloud<pcl::PointXYZ>)'

-pcl::PointCloud<pcl::PointXYZ>转
pcl::PCLPointCloud2

'pcl::toPClPointCloud2(pcl::PointCloud<pcl::PointXYZ>, pcl::PCLPointCloud2)'


##点云的基本操作
###从ROS Topic中读取点云并保存为PCD文件
注意要先将ROS Message 格式的点云数据转换成为PCL::PointCloud<pcl::PointXYZ>格式
'pcl::fromROSMsg (sensor_msgs::PointCloud2, pcl::PointCloud<pcl::PointXYZ>)'
然后使用pcl的io接口进行pcd文件的写入
'pcl::io::savePCDFileASCII(path, cloud);'

###从PCD文件实现点云的可视化
'
//1. 最基础的点云可视化操作
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	// -----Open 3D viewer and add point cloud
	//创建视窗对象并给标题栏设置一个名称“3D Viewer”并将它设置为boost::shared_ptr智能共享指针，这样可以保证指针在程序中全局使用，而不引起内存错误
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	//设置视窗的颜色，这里设置为白色
	viewer->setBackgroundColor(255,251,240);

	/*这是最重要的一行，我们将点云添加到视窗对象中，并定一个唯一的字符串作为ID 号，利用此字符串保证在其他成员中也能
	标志引用该点云，多次调用addPointCloud可以实现多个点云的添加，每调用一次就会创建一个新的ID号，如果想更新一个
	已经显示的点云，必须先调用removePointCloud（），并提供需要更新的点云ID 号*/
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");

	//用于改变显示点云的尺寸，可以利用该方法控制点云在视窗中的显示方法，
	//通过"sample cloud"这个ID来访问刚刚添加的点云
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	
    //查看复杂的点云，经常让人感到没有方向感，为了保持正确的坐标判断，需要显示坐标系统方向，可以通过使用X（红色）
	//Y（绿色 ）Z （蓝色）圆柱体代表坐标轴的显示方式来解决，圆柱体的大小可以通过scale参数来控制，本例中scale设置为1.0
    viewer->addCoordinateSystem(1.0);
	//通过设置照相机参数使得从默认的角度和方向观察点云
	viewer->initCameraParameters();
	return (viewer);
}
'

##点云关键点检测
已经完成了Harris和Sift关键点检测的程序
但是效果并不理想

