# pepper_control
This module allows controlling the joints of Pepper in velocity. 

## Installation
* Go via terminal to your qibuild workspace:
* Clone pepper_control:s
`$ git clone https://github.com/lagadic/pepper_control.git`   
`$ cd pepper_control`   
`$ qibuild configure -c toolchain_atom_2.4 --release`  
`$ cd build-toolchain_atom_2.4-release`   
`$ qipkg make-package --release pepper_control.pml -c toolchain_atom_2.4`   



### Example:
You can use visp_naoqi to control in velocity Romeo or Pepper, otherwise you can directly create a proxy to the module pepper_control running in the robot:

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



TODO:
* Add velocity control for the base (vx, vy, wz)
* Use DCM to set joint positions.
