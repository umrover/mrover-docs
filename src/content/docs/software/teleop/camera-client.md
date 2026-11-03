---
title: "Camera Client"
---

## Overview

The camera client is a hub for viewing the rover's various camera feeds.  Controls for cameras appear on the left (including adding new cameras and changing configuration), while the actual displays are on the right. The camera client does not work with the simulator.

## Opening

The camera client is opened by all launch files that include "```camera_client```".

- `arm_camera_client.launch.py`
- `auton_camera_client.launch.py`
- `science_camera_client.launch.py`

It will open with two windows. The first will always start with existing cameras, while the second only sometimes starts with them.

## Qt

The camera client uses **[Qt](https://doc.qt.io/)** to create the interface. **Qt** is a framework that lets the same GUI run well on multiple different types of devices. We currently use Qt6 with c++.

### File Structure

Our components are classes defined by header files (`.hpp` in `/include`), and implemented in source files (`.cpp` in `/src`).

`QWidget` is inherited by nearly all graphical elements. `QWidget` is a basic block that you can add other elements to. Each element you add to it should be stored in a pointer. For example, in our config selector (`teleoperation/camera_client/include/CameraConfigWidget.hpp`):

```cpp
class CameraConfigWidget : public QWidget {
        Q_OBJECT

        QVBoxLayout* mMainLayout;

        QWidget* mFormWidget;
        QFormLayout* mFormLayout;

        QPushButton* mArmButton;
        QPushButton* mScienceButton;
        QPushButton* mAutonButton;
...
```

The object itself is defined as a class that inherits `QWidget`. Objects that are added to it, such as buttons and even another widget, are assigned to pointers of the corresponding type. 

Now, pointers aren't actually objects. They point to objects. These objects have to actually be created.

```cpp
CameraConfigWidget::CameraConfigWidget(QWidget* parent) : QWidget(parent) {
    mMainLayout = new QVBoxLayout(this);

    mFormWidget = new QWidget(this);
    mFormLayout = new QFormLayout(mFormWidget);

...
```

In `CameraConfigWidget`'s constructor, the pointers are assigned to newly created objects. The argument `this` is set as the parent of these new objects.
```cpp
    mMainLayout->addWidget(mArmButton);
    mMainLayout->addWidget(mScienceButton);
    mMainLayout->addWidget(mAutonButton);
```
Then, the objects are added to a layout using `mMainLayout->addWidget(<pointer>)`, and the `CameraConfigWidget` uses this layout by invoking `setLayout(mMainLayout)`.

So, a general Qt element has a

**Header File**

- `#include "pch.hpp"` \- This includes miscellaneous dependencies needed for the camera client to run
- Class definition
- Pointers
- Signal definitions
- Slot definitions

**Source File**

- Constructor
- Implementations
  - Functions
  - Slots

### Signals and Slots

**[Signals](https://doc.qt.io/qt-6/signalsandslots.html)** provide a way for Qt objects to communicate with each other. They are somewhat similar to a ROS service in that way \- one object emits/publishes a signal, and another receives/subscribes to it.

Signals are defined in the definition of a class:

```cpp
signals:
        void loadCameraConfigSignal(std::string const& config);
```

So are *slots*:

```cpp
slots:
         void onArmClicked();
         void onScienceClicked();
         void onAutonClicked();
 ```

Stock classes will their own signals and slots, which you can read about on the Qt docs.

:::note
 A *slot* is a function. The only difference is that it can receive a signal, and is defined in the *slots* section of a class.
:::

To connect a signal, write:

```cpp
connect(emitter, &EmitterType::signalName, receiver, &ReceiverType::slotName);
```


### ROS Integration

Like the basestation, the camera client has a ROS node `/camera-client` that communicates with those on the rover. Its primary use is NOT to get video feeds, however. Instead, it is used for discovering cameras, pausing cameras, and checking *metadata* of received images. Video streams are sent over [RTP](https://en.wikipedia.org/wiki/Real-time_Transport_Protocol) and processed by a gst pipeline (see below section).

### Documentation

[Main Docs](https://doc.qt.io/)

[Class Reference](https://doc.qt.io/qt-6/qtwidgets-module.html)

## GStreamer

**[GStreamer](https://gstreamer.freedesktop.org/documentation/?gi-language=c)** is a pipelining tool that processes video data incoming from the rover. ESW creates a **source** on the rover, which is sent through a gst pipeline to a **sink** on the basestation. When a new frame is processed by the sink, it is painted to the GUI in a `GSTVideoWidget`.
