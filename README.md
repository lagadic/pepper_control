# pepper_control

This module allows controlling the joints of Pepper in velocity. 

## Installation

* Install [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* Install libqi:

        $ sudo apt-get install ros-kinetic-naoqi-libqi ros-kinetic-naoqi-libqicore

* Clone the repository in your workspace:

        $ git clone https://github.com/lagadic/pepper_control.git
        $ cd pepper_control
        $ mkdir build-libqi; cd build-libqi
        $ source /opt/ros/kinetic/setup.bash
        $ ccmake ..
        $ make -j4

### Run the module in remote (just for testing)

* Run the module in remote (change the argument `--qi-url`  with your robot ip and port):

        $  ./pepper_control --qi-url=tcp://127.0.0.1:9559

    Arguments reference: [link](http://doc.aldebaran.com/2-4/dev/libqi/guide/qi-app-arguments.html
