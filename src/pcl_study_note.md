#这是学习PCL基本操作以及如何在ROS中应用的笔记

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

