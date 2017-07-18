# pepper_control

This module allows controlling the joints of Pepper in velocity.

## Installation

* Go via terminal to your qibuild workspace
* Clone the repository in your workspace:

        $ git clone https://github.com/lagadic/pepper_control.git
        $ cd pepper_control

### Run the module in remote (just for testing)

Following instructions are valid with Ubuntu 14.04. If you are using Ubuntu 16.04 you should use 2.5 toolchain. But in that case Pepper should also use naoqi 2.5.

* Configure remote module:

        $ qibuild configure -c toolchain_2_4 --release

* Build:

        $ qibuild make -c toolchain_2_4

* Run the module in remote (change the argument `--qi-url`  with your robot ip and port):

        $  ./build-toolchain_2_4/sdk/bin/pepper_control --qi-url=tcp://127.0.0.1:9559 

    Arguments reference: [link](http://doc.aldebaran.com/2-4/dev/libqi/guide/qi-app-arguments.html)

### Run the module locally 

* Download the Cross Toolchain 2.4.3 Linux 64
* Create a toolchain for the cross compilation (I have called it `atom_2_4`) ([Instructions](http://doc.aldebaran.com/2-4/dev/cpp/install_guide.html#e-compile-and-run-an-example))

* Configure local module:

        $ qibuild configure -c atom_2_4 --release

* Build:

        $ qibuild build -c atom_2_4 --release

* Create pkg:

        $ qipkg make-package pepper_control.pml -c atom_2_4

* Deploy in Pepper:

        $ qipkg deploy-package pepper_control-0.1.0.pkg --url nao@127.0.0.1

### Launch module
 
* Open Choregraphe
* Connect to Pepper
* Click on Robot Applications (right corner)
* Find pepper_control and click on the green start buttons

### Call function with qicli

* Connect via terminal to Pepper

        $ ssh nao@127.0.0.1  # change 127.0.0.1 with the right IP
        
* Check if `pepper_control is running` with 

        $ qicli info

### Example:

You can use visp_naoqi to control the joints of Pepper in velocity, otherwise you can directly create a proxy to the module pepper_control running in the robot:

```
// Connect to module
qi::SessionPtr session = qi::makeSession();
session->connect("tcp://131.254.10.126:9559");
qi::AnyObject proxy = session->service("pepper_control");
  
// Start the controller
proxy.call<void >("start");
  
std::vector<std::string> jointNames_head; // Fill with names of the joints to control
std::vector<float> vel; // Fill with the joint velocities (rad/s)

while (1)
{
  // Update velocities
 
  proxy.async<void >("setDesJointVelocity", jointNames_head, vel );
}

// Stop the joint motion
proxy.call<void >("stopJoint");
//Stop the controller
proxy.call<void >("stop");

``` 



### Possible problems

```
[ERROR]: Unable to install package on target
[ERROR]: Error was:  No module named qi
```
Remember to add the python sdk to the path:
```
$ export PYTHONPATH=/home/common/naoqi-sdk/pynaoqi-python2.7-2.4.3.28-linux64:$PYTHONPATH 
```


## TODO:
* Launch automatic the module at startup of the robot
* Use DCM to set joint positions.
