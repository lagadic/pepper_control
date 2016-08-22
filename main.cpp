#include <control.hpp>

int main(int argc, char** argv)
{
  qi::ApplicationSession app(argc, argv);
  app.start();

  qi::SessionPtr session = app.session();

  boost::shared_ptr<Control> ctr = boost::make_shared<Control>(session);
  session->registerService("pepper_control", ctr);

   app.run();

  //ctr->stopTask();

}
