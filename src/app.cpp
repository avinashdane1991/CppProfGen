#include <mosaiqpp/app.hpp>
#include <mosaiqpp/intercom/publication.hpp>
#include <mosaiqpp/intercom/subscription.hpp>
#include <sstream>
#include <string>

/// A cyclic Mosaiq App in C++ is controlled by 3 fundamental functions:
/// Main::onInitialization()  -> will run once before the cyclic execution starts (optional)
/// Main::onCyclicExecution() -> will run periodically according to a preconfigured cycle time (mandatory)
/// Main::onExit()            -> will run once after finishing the cyclic execution (optional)
class Main : public MosaiqSdk::CyclicApp
{
private:
  /// Declaration and initialization of member variables used in this app
  uint32_t m_Var1 = 0;
  /// Declaration and initialization of Real-time Intercom variables for publishing and subscribing.
  /// Possible datatypes are:
  /// "int8_t", "uint8_t", "int16_t", "uint16_t", "int32_t", "uint32_t", "int64_t", "uint64_t",
  /// "float", "double", "std::string", "std::vector<std::byte>"
  MosaiqSdk::Intercom::Subscription<uint32_t> m_SubVar1;          // subscription
  MosaiqSdk::Intercom::Publication<std::string> m_PubVar1String;  // publication

public:
  /// Constructor of class of cyclic app (do not change the line below apart from class name)
  Main(MosaiqSdk::Intercom::Broker &&broker) : MosaiqSdk::CyclicApp{std::move(broker)}, 
      // Set up topics to be published and subscribed and link it to member variables
      // Recommended: use public app id (to be found in the app details in the App Inventory) to ensure uniqueness across all apps
      // NOTE: the subscription of an app must match the datatype and topic name of the publication app
      m_SubVar1{getBroker().template subscribe<uint32_t>("Var1")},
      m_PubVar1String{getBroker().template publish<std::string>("Var1String")}
  {
  }
  /// Called once before cyclic execution (optional)
  void onInitialization()
  {
  }
  /// Called periodically according to the predefined cycle time (mandatory)
   void onCyclicExecution()
  {
    m_SubVar1.read(m_Var1);                      // read value of subscribed topic and write it to m_Var1
    std::stringstream Var1String{};
    m_PubVar1String.write(Var1String.str());     // publish value
  }
  /// Called once after cyclic execution (optional)
  void onExit()
  {
  }
};
/// This macro needs to be called for Main to function in the Mosaiq Runtime
MOSAIQ_DECLARE_APP(Main);