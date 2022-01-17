#include <iostream>
#include <mosaiqpp/app.hpp>
#include <mosaiqpp/intercom/publication.hpp>
#include "profilegenerator.h"

class ProfileGenApp : public MosaiqSdk::CyclicApp
{
public:
  ProfileGenApp(MosaiqSdk::Intercom::Broker&& broker)
    : MosaiqSdk::CyclicApp{std::move(broker)}
  {}

  void onInitialization() 
  { 
      profGen.init();
  }
  void onCyclicExecution()
  {
      profGen.generateProfile();
  }
  void onExit() 
  {
      profGen.stop();
  }

private:
    ProfileGenerator profGen;
};

MOSAIQ_DECLARE_APP(ProfileGenApp);
