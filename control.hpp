#include <string> 
#include <vector>

#include <qi/anyobject.hpp>
#include <qi/applicationsession.hpp>
#include <qi/periodictask.hpp>


class Control
{

private:
  qi::SessionPtr m_session;
  qi::SteadyClock::time_point m_prev;
  qi::PeriodicTask m_task;
  std::vector<float> m_vel;
  std::vector<float> m_pos;
  std::vector<std::string> m_jointNames;
  qi::AnyObject m_motion;
  boost::mutex lock_;


  bool m_taskStarted;
  bool m_firstTimeTask;


public:
  Control(qi::SessionPtr session);

  std::vector<float> getJointValues (std::vector<std::string> jointNames) const;
  void setDesJointVelocity (std::vector<std::string> jointNames, std::vector<float> vel);
  void setTask();
  qi::PeriodicTask::Callback printTime() ;
  void applyJointVelocity() ;
  void start();
//  void stopJoint();
  void stop();

};
QI_REGISTER_OBJECT(Control, getJointValues, printTime, setDesJointVelocity, start, stop);
