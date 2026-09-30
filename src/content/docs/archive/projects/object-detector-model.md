---
title: "Object Detector Model"
---
**DISCLAIMER** The object detector model project will have a heavy research emphasis

**Context**: During the URC competition we must be able to detect two types of objects: mallet and water bottle. These objects will exist at two distinct waypoints during the autonomous mission. For a full context on the auton mission please read the [URC Rules](https://urc.marssociety.org/home/requirements-guidelines), specifically **1.f Autonomous Navigation Mission**.

**Problem**: The majority of the R&D went into the object detector's execution/inference environment, leaving the model left somewhat rudimentary. The current model is a [YOLOv8 small](https://docs.ultralytics.com/models/yolov8/) with fine tuning using our own mallet and water bottle dataset and [RoboFlow](https://roboflow.com/).

**Solution**: 
First Step:
- Get the model to work with [YOLOv8n](https://docs.ultralytics.com/models/yolov8/#performance-metrics) it is a lighter weight model and a great starting point.
- Perform fine tuning using [RoboFlow](https://roboflow.com/)

Future Research:
Use a smaller, lighter 🪶 model using one of:
- **FIND A MODEL THAT ACCEPTS BGRA/Non-Blob format**: we are currently spending more time doing image conversions than actually processing the data.
- Tensor Flow:
     - [Instructions to do](https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/training.htmlf)
     - [Model Bank](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md)
     - The smallest one I found was 9.4 MB
     - Convert [.tflite to onnx ](https://onnxruntime.ai/docs/tutorials/tf-get-started.html)
- YoloV8:
     - YoloV8n has a size of 6.4 MB
- Training Custom Model (Using YOLO)
     - [Process](https://github.com/ultralytics/ultralytics/issues/6430)

<hr>

**Interface** (subject to change): Use the existing TensorRT framework to execute the model and then publish the objects into the tf tree. Here are some of the tf tags:
```
bottle
mallet
immediateMallet
immediateBottle
```
	
	