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
  std::vector<float> m_vel; // protected by mutex
  std::vector<float> m_pos;
  std::vector<std::string> m_jointNames; // protected by mutex
  qi::AnyObject m_motion;
  boost::mutex lock_;

  int period_ms;

  bool m_taskStarted;
  bool m_firstTimeTask;

  std::vector<int>	 Status;	        // Flag d'etat des axes
  std::vector<float> deltaQmax;        // Increment de consigne maximum
  std::vector<float> deltaQ;		// Increment de consigne en cours
  std::vector<float> Qacc;		// Increment d'increment de consigne
  std::vector<float> Qacc_sav;         // Sauvegarde
  std::vector<float> ConsFin;    	// Consigne finale (butee)
  std::vector<int>   SigneDep;	        // Signe du deplacement: +1 = incrementer consigne
  std::vector<float> PQc;              // Position mesuree et position calculee en rad
  std::vector<float> dist_AD;		// Distance requise pour acce et decel
  std::vector<float> pcspeed;		// Recopie des consigne de vitesse
  std::vector<float> pcspeed_old;
  std::vector<float> ecart;		// Difference entre consigne et consigne finale
  std::vector<float> FlagSpeed;
  bool vmax;
  bool flagbutee;

  // Constant
  std::vector<float> AccMax;    // Acceleration maximale
  std::vector<float> VitMax;    // Vitesses maximales
  std::vector<float> QMax;	// Butee softs maximale
  std::vector<float> QMin;	// Butee softs minimale



public:
  Control(qi::SessionPtr session);
  virtual ~Control();

  std::vector<float> getJointValues (std::vector<std::string> jointNames) const;
  void setDesJointVelocity (std::vector<std::string> jointNames, std::vector<float> vel);
  void setTask();
  qi::PeriodicTask::Callback printTime() ;
  void applyJointVelocity() ;
  void start();
  void stopJoint();
  void stop();

};
QI_REGISTER_OBJECT(Control, getJointValues, printTime, setDesJointVelocity, start, stop, stopJoint);
