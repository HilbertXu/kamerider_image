/*
Date: 2019/02/02
Author: Xu Yucheng
Abstract: Code for finding waving person
Methods: 首先使用人脸检测(dlib, opencv)检测出当前画面内人脸所在区域，
         然后取人脸两侧区域，对两侧的区域内使用Openpose检测手部，便可判断人是否在挥手
*/