#include <iostream> 
#include <string> 
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h> 
#include <pcl/visualization/pcl_visualizer.h> 
using namespace std; 
using namespace pcl;


// --------------
// -----Help-----
// --------------
void printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation example\n"
            << "-r           RGB colour visualisation example\n"
            << "-c           Custom colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-a           Shapes visualisation example\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
            << "\n\n";
}

/*
@TODO
添加多个可视化模块
实现多视角可视化
法线特征与其他特征可视化
*/
//最基础的点云可视化操作
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{

	// -----Open 3D viewer and add point cloud
	//创建视窗对象并给标题栏设置一个名称“3D Viewer”并将它设置为boost::shared_ptr智能共享指针，这样可以保证指针在程序中全局使用，而不引起内存错误
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	//设置视窗的背景色，可以任意设置RGB的颜色，这里是设置为黑色
	viewer->setBackgroundColor(0, 0, 0);

	/*这是最重要的一行，我们将点云添加到视窗对象中，并定一个唯一的字符串作为ID 号，利用此字符串保证在其他成员中也能
	标志引用该点云，多次调用addPointCloud可以实现多个点云的添加，每调用一次就会创建一个新的ID号，如果想更新一个
	已经显示的点云，必须先调用removePointCloud（），并提供需要更新的点云ID 号*/
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	
    //用于改变显示点云的尺寸，可以利用该方法控制点云在视窗中的显示方法，
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	
    //查看复杂的点云，经常让人感到没有方向感，为了保持正确的坐标判断，需要显示坐标系统方向，可以通过使用X（红色）
	//Y（绿色 ）Z （蓝色）圆柱体代表坐标轴的显示方式来解决，圆柱体的大小可以通过scale参数来控制，本例中scale设置为1.0
    viewer->addCoordinateSystem(1.0);
	//通过设置照相机参数使得从默认的角度和方向观察点云
	viewer->initCameraParameters();
	return (viewer);
}

//可视化RGB点云的函数
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

//自定义点云颜色的函数
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	//创建一个自定义的颜色处理器PointCloudColorHandlerCustom对象，并设置颜色为纯绿色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
	//addPointCloud<>()完成对颜色处理器对象的传递
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

//可视化点云法线以及其他特征
//显示法线是理解点云的一个重要步骤，点云法线特征是非常重要的基础特征，PCL visualizer可视化类可用于绘制法线，
//也可以绘制表征点云的其他特征，比如主曲率和几何特征，normalsVis函数中演示了如何实现点云的法线
boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	
    //实现对点云法线的显示
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 10, 0.05, "normals");
	
    viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}


//绘制普通形状
//PCL visualizer可视化类允许用户在视窗中绘制一般图元，这个类常用于显示点云处理算法的可视化结果，例如 通过可视化球体
//包围聚类得到的点云集以显示聚类结果，shapesVis函数用于实现添加形状到视窗中，添加了四种形状：从点云中的一个点到最后一个点
//之间的连线，原点所在的平面，以点云中第一个点为中心的球体，沿Y轴的椎体
boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	/************************************************************************************************
	绘制形状的实例代码，绘制点之间的连线，
	*************************************************************************************************/
	viewer->addLine<pcl::PointXYZRGB>(cloud->points[0],
		cloud->points[cloud->size() - 1], "line");
	//添加点云中第一个点为中心，半径为0.2的球体，同时可以自定义颜色
	viewer->addSphere(cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

	//---------------------------------------
	//-----Add shapes at other locations添加绘制平面使用标准平面方程ax+by+cz+d=0来定义平面，这个平面以原点为中心，方向沿着Z方向-----
	//---------------------------------------
	pcl::ModelCoefficients coeffs;
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	viewer->addPlane(coeffs, "plane");
	//添加锥形的参数
	coeffs.values.clear();
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(5.0);
	viewer->addCone(coeffs, "cone");

	return (viewer);
}

//多视角显示
//viewportsVis函数演示如何用多视角来显示点云计算法线的方法结果对比
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->initCameraParameters();
	//以上是创建视图的标准代码

	int v1(0);  //创建新的视口
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);  //4个参数分别是X轴的最小值，最大值，Y轴的最小值，最大值，取值0-1，v1是标识
	viewer->setBackgroundColor(0, 0, 0, v1);    //设置视口的背景颜色
	viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);  //添加一个标签区别其他窗口  利用RGB颜色着色器并添加点云到视口中
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);
	//对第二视口做同样的操作，使得做创建的点云分布于右半窗口，将该视口背景赋值于灰色，以便明显区别，虽然添加同样的点云，给点云自定义颜色着色
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, single_color, "sample cloud2", v2);
	//为所有视口设置属性，
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
	viewer->addCoordinateSystem(1.0);
	//添加法线  每个视图都有一组对应的法线
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals1, 10, 0.05, "normals1", v1);
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals2, 10, 0.05, "normals2", v2);

	return (viewer);
}

//鼠标事件
//每次响应这种事件都会在鼠标按下的位置上生成一个文本标签
unsigned int text_id = 0;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
	void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym() == "r" && event.keyDown())
	{
		std::cout << "r was pressed => removing all text" << std::endl;

		char str[512];
		for (unsigned int i = 0; i < text_id; ++i)
		{
			sprintf(str, "text#%03d", i);
			viewer->removeShape(str);
		}
		text_id = 0;
	}
}

//键盘事件
//所以在PCL中视窗中注册事件响应回调函数，不会覆盖其他成员对同一事件的响应
void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

int main (int argc, char** argv)

{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  bool simple(false), rgb(false), custom_c(false), normals(false),
    shapes(false), viewports(false), interaction_customization(false);
  if (pcl::console::find_argument (argc, argv, "-s") >= 0)
  {
    simple = true;
    std::cout << "Simple visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-c") >= 0)
  {
    custom_c = true;
    std::cout << "Custom colour visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-r") >= 0)
  {
    rgb = true;
    std::cout << "RGB colour visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-n") >= 0)
  {
    normals = true;
    std::cout << "Normals visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-a") >= 0)
  {
    shapes = true;
    std::cout << "Shapes visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-v") >= 0)
  {
    viewports = true;
    std::cout << "Viewports example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-i") >= 0)
  {
    interaction_customization = true;
    std::cout << "Interaction Customization example\n";
  }
  else
  {
    printUsage (argv[0]);
    return 0;
  }

  // ------------------------------------
  // -----Create example point cloud-----
  // ------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "Genarating example point clouds.\n\n";
  // We're going to make an ellipse extruded along the z-axis. The colour for
  // the XYZRGB cloud will gradually go from red to green to blue.
  uint8_t r(255), g(15), b(15);
  for (float z(-1.0); z <= 1.0; z += 0.05)
  {
    for (float angle(0.0); angle <= 360.0; angle += 5.0)
    {
      pcl::PointXYZ basic_point;
      basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
      basic_point.y = sinf (pcl::deg2rad(angle));
      basic_point.z = z;
      basic_cloud_ptr->points.push_back(basic_point);

      pcl::PointXYZRGB point;
      point.x = basic_point.x;
      point.y = basic_point.y;
      point.z = basic_point.z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
    }
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  point_cloud_ptr->height = 1;

  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (point_cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.05);
  ne.compute (*cloud_normals1);

  // ---------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.1-----
  // ---------------------------------------------------------------
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.1);
  ne.compute (*cloud_normals2);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  if (simple)
  {
    viewer = simpleVis(basic_cloud_ptr);
  }
  else if (rgb)
  {
    viewer = rgbVis(point_cloud_ptr);
  }
  else if (custom_c)
  {
    viewer = customColourVis(basic_cloud_ptr);
  }
  else if (normals)
  {
    viewer = normalsVis(point_cloud_ptr, cloud_normals2);
  }
  else if (shapes)
  {
    viewer = shapesVis(point_cloud_ptr);
  }
  else if (viewports)
  {
    viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
  }
  else if (interaction_customization)
  {
    viewer = interactionCustomizationVis();
  }

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}



/*
int main (int argc, char** argv)
{ 
    typedef pcl::PointXYZRGBA PointT; 
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>); 

    std::cout << argv[1];

    std::string dir = "/home/kamerider/catkin_ws/src/robot_vision/pcd_files/"; 
    std::string filename = argv[1]; 

    if (pcl::io::loadPCDFile<PointT> ((dir+filename), *cloud) == -1)
    { 
        //* load the file 
        PCL_ERROR ("Couldn't read PCD file \n"); 
        return (-1); 
    } 
    printf("Loaded %d data points from PCD\n", 
    cloud->width * cloud->height); 

    for (size_t i = 0; i < cloud->points.size (); i+=10000) 
    printf("%8.3f %8.3f %8.3f %5d %5d %5d %5d\n", 
    cloud->points[i].x, 
    cloud->points[i].y, 
    cloud->points[i].z, 
    cloud->points[i].r, 
    cloud->points[i].g, 
    cloud->points[i].b, 
    cloud->points[i].a 
    ); 

    pcl::visualization::PCLVisualizer viewer("Cloud viewer"); 
    viewer.setBackgroundColor(255,251,240);
    //设置一个自定义的颜色处理器
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(*cloud, 0, 255, 0);
    viewer.setCameraPosition(0,0,-3.0,0,-1,0); 
    viewer.addCoordinateSystem(0.3); 

    //viewer.addPointCloud(cloud, single_color); 
    viewer.addPointCloud(cloud);
    while(!viewer.wasStopped()) 
    viewer.spinOnce(100); 
    return (0); 
} 
*/
