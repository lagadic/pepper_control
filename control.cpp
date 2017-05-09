#include <iostream>
#include <sstream> 
#include <qi/clock.hpp>
#include <boost/chrono/chrono_io.hpp>
#include <control.hpp>

#define FLAGACC		1	// Axe en acceleration
#define FLAGCTE		2 // Axe en vitesse constante
#define FLAGDEC		3	// Axe en deceleration
#define FLAGSTO		4	// Axe stoppe

#define DELTAQMIN	0.0001	// Delta Q minimum (rad)
#define OFFSET_BUTEE    0 //0.01	// on s'arrete un peu avant les butees


Control::Control(qi::SessionPtr session)
  : m_session(session), m_prev(), m_task(), m_vel(), m_pos(),
    m_jointNames(),lock_(), m_taskStarted(false), m_firstTimeTask(true),
    Status(), deltaQmax(), deltaQ(), Qacc(), Qacc_sav(), ConsFin(), SigneDep(),
    PQc(), dist_AD(), pcspeed(), pcspeed_old(), ecart(), FlagSpeed(), vmax(false), flagbutee(false),
    AccMax(), VitMax(), QMin(), QMax(), period_ms(2)
{
  m_prev = qi::SteadyClock::now();
  m_motion = m_session->service("ALMotion");
}

Control::~Control()
{
 m_session->close();

}

void Control::start()
{
  setTask();
  std::cout << "Task started " << std::endl;
}

void Control::stopJoint()
{
  lock_.lock();
  std::vector<float> zero (m_vel.size(), 0.0);
  m_vel = zero;
  lock_.unlock();
  std::cout << "Stop joints" << std::endl;
}

void Control::stop()
{
  m_task.stop();
  std::vector<float> change(m_jointNames.size(), 0.0);
  m_motion.async<void>("changeAngles", m_jointNames, change, 1.0);
  //m_started = false;
  std::cout << "Task stopped " << std::endl;
}

void Control::setOneDesJointVelocity (std::string jointName, float vel)
{
  std::vector<std::string> jointName_v;
  jointName_v.push_back(jointName);
  std::vector<float> vel_v;
  vel_v.push_back(vel);
  setDesJointVelocity(jointName_v, vel_v);
//  std::cout << "Calling setOneDesJointVelocity" << std::endl;
}



void Control::setDesJointVelocity (std::vector<std::string> jointNames, std::vector<float> vel)
{
  lock_.lock();
  std::vector<std::string> jointNames_prev = m_jointNames;
  lock_.unlock();

//  std::cout << "jointNames.size()" << jointNames.size() << std::endl;


  if (jointNames != jointNames_prev) {
    // TODO: Call next only if joint names modified
    size_t njoint = jointNames.size();
    Status.resize(njoint);
    deltaQmax.resize(njoint);
    deltaQ.resize(njoint);
    Qacc.resize(njoint);
    Qacc_sav.resize(njoint);
    ConsFin.resize(njoint);
    SigneDep.resize(njoint);
    PQc.resize(njoint);
    dist_AD.resize(njoint);
    pcspeed.resize(njoint);
    pcspeed_old.resize(njoint);
    ecart.resize(njoint);
    FlagSpeed.resize(njoint);
    AccMax.resize(njoint);
    VitMax.resize(njoint);
    QMin.resize(njoint);
    QMax.resize(njoint);


    //  for (unsigned int i=0; i<jointNames.size(); i++)
    //  {
    //    qi::AnyValue limits = m_motion.call< qi::AnyValue >("getLimits", jointNames[i]);
    //    //std::cout << limits << std::endl;
    //    QMin[i] = limits[0].asFloat();
    //    QMax[i] = limits[1].asFloat();
    //    VitMax[i] = limits[2].asFloat();
    //    AccMax[i] = 0.1; // To improve (~10deg in 2 sec)
    //    std::cout << " jointNames " << jointNames[i] << " " << QMin[i] << " " << QMax[i] << " " << VitMax[i] << std::endl;
    //  }

    for (unsigned int i=0; i<jointNames.size(); i++)
    {
      std::vector<std::vector<float>> limits = m_motion.call< std::vector<std::vector<float>> >("getLimits", jointNames[i]);
      //std::cout << limits << std::endl;
      QMin[i] = limits[0][0];
      QMax[i] = limits[0][1];
      VitMax[i] = limits[0][2];
      AccMax[i] = 5.; // To improve (~10deg in 2 sec)
      std::cout << " jointNames " << jointNames[i] << " " << QMin[i] << " " << QMax[i] << " " << VitMax[i] << std::endl;

      FlagSpeed[i] = false;
      Status[i] 	= FLAGSTO;
      deltaQ[i] 	= deltaQmax[i] 	= 0.0;
      Qacc_sav[i] = Qacc[i] = AccMax[i] * period_ms * period_ms / 1000000.;
      ConsFin[i] 	= PQc[i] = 0.0; //get_q(i);
      dist_AD[i] 	= 0.0;
      pcspeed[i]	= pcspeed_old[i] = 0.0;
      Status[i]   = FLAGSTO;
      SigneDep[i]	= 0;
      ConsFin[i] = 0;
    }

    m_pos = m_motion.call<std::vector<float> >("getAngles", jointNames, 1);

    for (unsigned int i=0; i<jointNames.size(); i++)
      PQc[i] = m_pos[i];

  }

  //vmax = false;
  //  m_pos = m_motion.call<std::vector<float> >("getAngles", jointNames, 1);

  //  for (unsigned int i=0; i<jointNames.size(); i++)
  //    PQc[i] = m_pos[i];

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
  qi::SteadyClock::time_point now = qi::SteadyClock::now();

  lock_.lock();
  std::vector<std::string> jointNames = m_jointNames;
  std::vector<float> vel = m_vel;
  lock_.unlock();

  if (vel.size()>0 && jointNames.size()>0)
  {

#if 1
    qi::MicroSeconds us = boost::chrono::duration_cast<qi::MicroSeconds>(now - m_prev);
    double delta_t =  us.count()/1000000.;

    //      std::cout << " delta " << delta_t << std::endl;

    /*
     * Test si changement de consigne.
     */
    for (int i=0;i<jointNames.size();i++) {
      pcspeed[i] = vel[i]; // TODO fuse in same var

      if (pcspeed[i] != pcspeed_old[i]) flagbutee = false;

      if (pcspeed[i] != pcspeed_old[i]) {
        Qacc[i] = Qacc_sav[i];

        if (pcspeed[i] > VitMax[i]) {
          pcspeed[i] = VitMax[i];
          if (vmax == false)
          {
            vmax = true;
          }
        }
        else if (pcspeed[i] < (-VitMax[i])) {
          pcspeed[i] = -VitMax[i];
          if (vmax == false)
          {
            vmax = true;
          }
        }
        else vmax = false;

        if (FlagSpeed[i] == false) {
          //if (pt_movespeed.FlagSpeed[i] == false) {
          /* Changement de consigne et non en phase de chang sens */

          if ( Status[i] == FLAGSTO) /* Si arret */
          {
            if (pcspeed[i] > 0)
            {
              deltaQmax[i] = pcspeed[i]*delta_t;
              SigneDep[i] = 1;
              ConsFin[i] = QMax[i] - OFFSET_BUTEE;
              deltaQ[i] = 0;
              Status[i] = FLAGACC;
            }
            else if (pcspeed[i] < 0)
            {
              deltaQmax[i] = - pcspeed[i]*delta_t;
              SigneDep[i] = -1;
              ConsFin[i] = QMin[i] + OFFSET_BUTEE;
              deltaQ[i] = 0;
              Status[i] = FLAGACC;
            }
          }
          // Si non en arret et changement de sens

          else if ( (pcspeed[i] * SigneDep[i]) < 0) {
            FlagSpeed[i] = true;
            Status[i] = FLAGDEC;
            deltaQmax[i] = 0;
          }
          // Pas de changement de sens

          else {	/* Non arret et pas de changement de sens */
            if ( SigneDep[i] == 1) {
              if ( pcspeed[i] > pcspeed_old[i])
                Status[i] = FLAGACC;
              else  Status[i] = FLAGDEC;
              deltaQmax[i] = pcspeed[i]*delta_t;
            }
            else {
              if ( pcspeed[i] > pcspeed_old[i])
                Status[i] = FLAGDEC;
              else  Status[i] = FLAGACC;
              deltaQmax[i] = - pcspeed[i]*delta_t;
            }
          }

          int n = (int) (deltaQmax[i] / Qacc[i]);
          dist_AD[i]=n*(deltaQmax[i]-(n+1)*Qacc[i]/2);
        }
        pcspeed_old[i] = pcspeed[i];
      }
    }

    //    std::cout << "ConsFin: ";
    //    for (int i=0;i<jointNames.size();i++)
    //      std::cout << jointNames[i] << "(" << ConsFin[i] << ") ";

    /*
     * calcul des consignes selon les cas:
     *		- acceleration
     *		- deceleration
     *		- arret
     */

    for (int i=0;i<jointNames.size();i++) {
      /*
       * Securite butee en vitesse constante
       */
      ecart[i] = ( ConsFin[i] - PQc[i]) * SigneDep[i];
      if ((ecart[i] - deltaQmax[i]) <=  dist_AD[i]) {
        if (dist_AD[i] > 0) {
          if (!flagbutee) printf("Flagbutee axe %d\n",i);
          flagbutee = true;
          for(int k=0;k<jointNames.size();k++)
          {
            if (Status[k] != FLAGSTO) Status[k] = FLAGDEC;
            deltaQmax[k] = 0;
          }
        }
      }
      /*
       * Deceleration.
       */
      if ( Status[i] == FLAGDEC) {
        deltaQ[i] -=  Qacc[i];
        if (deltaQ[i] <=  deltaQmax[i]) {
          if (deltaQmax[i] < DELTAQMIN)  {
            Status[i] = FLAGSTO;
            deltaQ[i] = 0.0;
            // Test si on etait en phase de changement de sens.
            if (FlagSpeed[i] == true) {
              //if (pt_movespeed.FlagSpeed[i] == true) {
              if (pcspeed[i] > 0) {
                deltaQmax[i] = pcspeed[i]*delta_t;
                SigneDep[i] = 1;
                ConsFin[i] = QMax[i] - OFFSET_BUTEE;
              }
              else if (pcspeed[i] < 0) {
                deltaQmax[i] = -pcspeed[i]*delta_t;
                SigneDep[i] = -1;
                ConsFin[i] = QMin[i] + OFFSET_BUTEE;
              }
              Status[i] = FLAGACC;
              FlagSpeed[i] = false;

              int n = (int) (deltaQmax[i] / Qacc[i]);
              dist_AD[i]=n*(deltaQmax[i]-(n+1)*Qacc[i]/2);
            }
          }
          else if ((deltaQmax[i] > 0) && !flagbutee)  {
            if (deltaQmax[i] < (deltaQ[i] + 2*Qacc[i])) {
              deltaQ[i] = deltaQmax[i];
              Status[i] = FLAGCTE;
            }
            else if (!flagbutee) {
              /* acceleration moins rapide*/
              deltaQ[i] += (2*Qacc[i]);
              Status[i] = FLAGACC;
            }
          }
        }
      }
      /*
       * Acceleration.
       */
      else if (Status[i] == FLAGACC) {
        deltaQ[i] += Qacc[i];

        if (deltaQ[i] >= deltaQmax[i]) {
          deltaQ[i] = deltaQmax[i];
          Status[i] = FLAGCTE;
        }
      }
      /*
       * Sinon a vitesse constante increment non change.
       */
      PQc[i] += SigneDep[i] * deltaQ[i];

    } /* endfor */

    // Test si un axe arrive pres des butees. Si oui, arret de tous les axes
    for (int i=0;i<jointNames.size();i++) {
      float butee = QMin[i] + OFFSET_BUTEE;
      if (PQc[i] < butee) {
        for (int j=0;j<jointNames.size();j++) PQc[j] -= SigneDep[j]*deltaQ[j];
        PQc[i] = butee;
        printf("Butee axe %d\n",i);
        break;
      }
      butee = (float) (QMax[i] - OFFSET_BUTEE);
      if (PQc[i] > butee) {
        for (int j=0;j<jointNames.size();j++) PQc[j] -= SigneDep[j]*deltaQ[j];
        PQc[i] = butee;
        printf("Butee axe %d\n",i);
        break;
      }
    }
    //    std::cout << "new_pos: ";
    //    for (int i=0;i<jointNames.size();i++)
    //      std::cout << jointNames[i] << "(" << PQc[i] << ") ";
    //    std::cout << std::endl;

    // Apply new position
    m_motion.async<void>("setAngles", jointNames, PQc, 1.0);
#else
    //std::cout << "m_vel: " << m_vel[0] << std::endl << m_vel[1] << std::endl  << "m_jointNames" <<   m_jointNames[0] << std::endl<<   m_jointNames[1] << std::endl;
    if (m_firstTimeTask)
    {
      m_pos = m_motion.call<std::vector<float> >("getAngles", m_jointNames, 1);
      m_firstTimeTask = false;
      std::cout << "Reset pos to" << m_pos[0] <<" "<<m_pos[1] << std::endl;
    }

    qi::MicroSeconds us = boost::chrono::duration_cast<qi::MicroSeconds>(now - m_prev);
    double delta_t =  us.count()/1000000.;

    //std::cout << "delta_t " << delta_t << " sec" << std::endl;

    std::vector<float> new_pos(m_jointNames.size());

    for (unsigned int i=0 ; i<m_jointNames.size(); i++)
      new_pos[i] = m_pos[i] + m_vel[i]*delta_t;

    //    std::cout << "new_pos " << new_pos[0] << " " << std::endl;
    //    std::cout << "m_jointNames " << m_jointNames[0] << " " << std::endl;

    m_motion.async<void>("setAngles", m_jointNames, new_pos, 1.0);
    m_pos = new_pos;
#endif
  }


  m_prev = now;
}


void Control::setTask()
{
  if (!m_taskStarted)
  {
    m_task.setName("Setvelocity");
   // m_task.setUsPeriod(period_ms * 1000);
    m_task.setPeriod(qi::MilliSeconds(period_ms));

    m_task.setCallback(&Control::applyJointVelocity, this);
    m_taskStarted = true;
  }
  m_vel.clear();
  m_jointNames.clear();
  m_pos.clear();
  m_prev = qi::SteadyClock::now();
  m_firstTimeTask = true;
  m_task.start();
}
