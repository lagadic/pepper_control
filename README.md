# pepper_control
This module allows controlling the joints of Pepper in velocity.   

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
* Add possibility to control only one joint
* Add velocity control for the base (vx, vy, wz)
* Use DCM to set joint positions.
