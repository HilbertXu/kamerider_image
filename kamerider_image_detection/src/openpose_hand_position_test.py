#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2

image = cv2.imread('/home/kamerider/openpose/examples/media/COCO_val2014_000000000241.jpg')
image = cv2.rectangle(image, (320, 377, 69, 69),(0,0,255))
image = cv2.rectangle(image, (80, 407, 80, 80), (0,0,255))
image = cv2.rectangle(image, (46, 404, 98, 98), (0,0,255))
image = cv2.rectangle(image, (185, 303, 100, 157), (0,0,255))
image = cv2.rectangle(image, (88, 268, 117, 117), (0,0,255))



cv2.imshow('test', image)
cv2.waitKey(0)