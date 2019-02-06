记录一下配置openpose的详细步骤以及这么多次踩的坑

1. 获取openpose源码，建议使用git clone --recursive指令，这样可以clone下来openpose项目下的子模块，建议使用官方修改过的子模块以减少错误
2. 配置caffe（./openpose/3rdparty/caffe）可以参考openbotics/wiki中我写过得配置方法，但是在修改Makefile文件时，如果使用的是opencv3那么在修改LIBRARIES一行时，需要修改为LIBRARIES += glog gflags protobuf leveldb snappy lmdb boost_system hdf5_hl hdf5 m   opencv_core opencv_imgproc opencv_imgcodecs 这是因为opencv3.0把imread相关函数放到imgcodecs.lib中了，而非原来的imgproc.lib。在这之后需要把caffe制作成发布版，在进行完runtest之后输入 make distribute -j8
3. 配置pybind11，openpose使用了pybind11来实现python和c++的互相调用，这个的配置也是一个坑。推荐使用CMake-gui进行配置，这样可以直观地设置flags。记得一定要在编译openpose之前进行pybind11的编译。其中一项PYBIND11_PYTHON_VERSION要设置成打算使用的python版本(ros下目前只支持2.7， 所以设置为2.7)
4. 配置openpose，推荐先使用models文件夹中的脚本下载模型，不然会等很久。。。虽然就算用脚本下也是很慢，后期我会考虑将模型传到网络硬盘中。之后的步骤按照openpose的github上的安装指南进行就可以了