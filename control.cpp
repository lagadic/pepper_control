#include <iostream>
#include <sstream> 
#include <qi/clock.hpp>
#include <boost/chrono/chrono_io.hpp>
#include <control.hpp>


Control::Control(qi::SessionPtr session)
  : m_session(session), m_prev(), m_task(), m_vel(), m_pos(), m_jointNames(),lock_(), m_start(false)
{
  m_prev = qi::SteadyClock::now();
  m_motion = m_session->service("ALMotion");
}


void Control::start()
{
  setTask();
  std::cout << "Task started " << std::endl;
}

void Control::stop()
{
  m_task.stop();
  std::cout << "Task stopped " << std::endl;
}

void Control::setDesJointVelocity (std::vector<std::string> jointNames, std::vector<float> vel)
{
  lock_.lock();
  m_vel = vel;
  m_jointNames = jointNames;
  lock_.unlock();
}


std::vector<float> Control::getJointValues (std::vector<std::string> jointNames) const
{
  std::cout << "Function getJointValues called" << std::endl;
  qi::AnyObject tts = m_session->service("ALMotion");
  return tts.call<std::vector<float> >("getAngles", jointNames, 1 );
}

qi::PeriodicTask::Callback Control::printTime()
{
  qi::SteadyClock::time_point now = qi::SteadyClock::now();
  //qi::MilliSeconds ms = boost::chrono::duration_cast<qi::MilliSeconds>(now - prev);
  qi::MicroSeconds us = boost::chrono::duration_cast<qi::MicroSeconds>(now - m_prev);
  double t =  us.count();
  std::cout << "spent " << t/1000000<< " sec" << std::endl;
  m_prev = now;
}

void Control::applyJointVelocity()
{
  lock_.lock();
  if (m_vel.size()>0 && m_jointNames.size()>0)
  {

    static bool first_time = true;
    //std::cout << "m_vel: " << m_vel[0] << std::endl << m_vel[1] << std::endl  << "m_jointNames" <<   m_jointNames[0] << std::endl<<   m_jointNames[1] << std::endl;
    if (first_time)
    {
      m_pos = m_motion.call<std::vector<float> >("getAngles", m_jointNames, 0 );
      first_time = false;
    }

    qi::SteadyClock::time_point now = qi::SteadyClock::now();
    qi::MicroSeconds us = boost::chrono::duration_cast<qi::MicroSeconds>(now - m_prev);
    double delta_t =  us.count()/1000000.;

    std::cout << "delta_t " << delta_t << " sec" << std::endl;

    std::vector<float> new_pos(m_jointNames.size());

    for (unsigned int i=0 ; i<m_jointNames.size(); i++)
      new_pos[i] = m_pos[i] + m_vel[i]*delta_t;

    std::cout << "new_pos " << new_pos[0] << " " << std::endl;
    std::cout << "m_jointNames " << m_jointNames[0] << " " << std::endl;

    m_motion.async<void>("setAngles", m_jointNames, new_pos, 1.0);
    m_prev = now;
    m_pos = new_pos;
  }

  lock_.unlock();


}


void Control::setTask()
{
  m_task.setName("Setvelocity");
  m_task.setUsPeriod(2000);
  m_task.setCallback(&Control::applyJointVelocity, this);
  m_task.start();
}
