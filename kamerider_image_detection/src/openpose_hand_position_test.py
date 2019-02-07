#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2

image = cv2.imread('/home/kamerider/openpose/examples/media/COCO_val2014_000000000241.jpg')
image = cv2.rectangle(image, (320, 377), (320+69, 377+69),(0,0,255),2)
image = cv2.rectangle(image, (80, 407), (80+80, 407+80), (0,0,255),2)
image = cv2.rectangle(image, (46, 404), (46+98, 404+98), (0,0,255),2)
image = cv2.rectangle(image, (185, 303), (185+100, 303+157), (0,0,255),2)
image = cv2.rectangle(image, (88, 268), (88+117, 268+117), (0,0,255),2)



cv2.imshow('test', image)
cv2.waitKey(0)