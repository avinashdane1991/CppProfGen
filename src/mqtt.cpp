#include "mqtt.h"
#include "profilegenerator.h"

bool Mqtt::isMessageReceived = false;
std::string Mqtt::topicInMessage = "";
std::string Mqtt::dataInMessage = "";

Mqtt::Mqtt()
{
    mosquitto_lib_init();
    mosq = mosquitto_new(CLIENT_ID.c_str()/*NULL*/, IS_CLEAN_SESSION, NULL);
    if(!mosq)
    {
        fprintf(stderr, "Error: Out of memory.\n");
        exit(1);
    }
}

Mqtt::~Mqtt()
{
    if(!mosq)
    {
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
    }
}

bool Mqtt::connect()
{   
    if(mosquitto_connect(mosq, BROKER_HOST.c_str(), BROKER_PORT, KEEP_ALIVE))
    {
        //fprintf(stderr, "Unable to connect.\n");
        return false;
    }
    
    mosquitto_subscribe(mosq, NULL, SUB_TPOIC_COMMAND.c_str(), 0);
    mosquitto_subscribe(mosq, NULL, SUB_TOPIC_PARAMETER.c_str(), 0);

    mosquitto_log_callback_set(mosq, mosq_log_callback);
    mosquitto_message_callback_set(mosq, message_callback);
        
    int loop = mosquitto_loop_start(mosq);
    if(loop != MOSQ_ERR_SUCCESS)
    {
        //fprintf(stderr, "Unable to start loop: %i\n", loop);
        return false;
    }
    return true;
}

void Mqtt::disconnect()
{
    //(*m_client)->disconnect()->wait();
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    mosq = NULL;
}


void Mqtt::publishForTopic_Monitor(float actVelocity, float actPosition, float actRotarySpeed)
{
    std::string jsonStr = "{\"actVelocity\": "+std::to_string(actVelocity)+", \"actPosition\":"+std::to_string(actPosition)+", \"actRotarySpeed\": "+std::to_string(actRotarySpeed)+"}";
    mosquitto_publish(mosq, NULL, PUB_TOPIC_MONITOR.c_str(), strlen(jsonStr.c_str()), jsonStr.c_str(), 0, 0);
}


void Mqtt::publishForTopic_ParameterOnConnect(std::vector<float> allParameters)
{
    std::string jsonStr = "{\"setDistance\": "+std::to_string(allParameters[0])+", \"setVelocity\": "+std::to_string(allParameters[1])
    +", \"setAcceleration\": "+std::to_string(allParameters[2])+", \"maxPosition\": "+std::to_string(allParameters[3])
    +",\"minPosition\": "+std::to_string(allParameters[4])+", \"maxVelocity\": "+std::to_string(allParameters[5])
    +", \"maxAccleration\": "+std::to_string(allParameters[6])+", \"maxMotorRotarySpeed\": "+std::to_string(allParameters[7])
    +",\"automaticCycleStopTime\": "+std::to_string(allParameters[8]/1000)+"}";

    mosquitto_publish(mosq, NULL, PUB_TOPIC_PARAMETER_ON_CONNECT.c_str(), strlen(jsonStr.c_str()), jsonStr.c_str(), 0, 0);
}

void Mqtt::mosq_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
/* Pring all log messages regardless of level. */
  
  switch(level){
    //case MOSQ_LOG_DEBUG:
    //case MOSQ_LOG_INFO:
    //case MOSQ_LOG_NOTICE:
    case MOSQ_LOG_WARNING:
    case MOSQ_LOG_ERR: {
      //printf("%i:%s\n", level, str);
      //use Mosaiq logging
    }
  }	
}

void Mqtt::message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message)
{
	bool match = 0;
	//printf("got message '%.*s' for topic '%s'\n", message->payloadlen, (char*) message->payload, message->topic);

	/*mosquitto_topic_matches_sub("/devices/wb-adc/controls/+", message->topic, &match);
	if (match) {
		printf("got message for ADC topic\n");
	}*/
    isMessageReceived = true;
    topicInMessage = (const char*)message->topic;
    dataInMessage = (const char*)message->payload;

    //std::cout << "topic in message_callback:" << topicInMessage<< "\n";
}


bool Mqtt::checkForMessagesFromBroker(std::string& topic, std::vector<std::string>& data)
{
    if(isMessageReceived)
    {
        isMessageReceived = false;
        topic = topicInMessage;
        //std::cout<< "topicInMessage:" << topicInMessage<<"\n";
        std::string jsonResponse = dataInMessage;
        jsonResponse.erase(std::remove(jsonResponse.begin(), jsonResponse.end(), '{'), jsonResponse.end());
        jsonResponse.erase(std::remove(jsonResponse.begin(), jsonResponse.end(), '}'), jsonResponse.end());

        std::stringstream check(jsonResponse);
        std::string intermediate;      
        while(getline(check, intermediate, ':'))
        {
            intermediate.erase(std::remove(intermediate.begin(), intermediate.end(), '"'), intermediate.end());
            data.push_back(intermediate);       
        }
        return true;
    }
    else
    {
        topic = "";
        data.clear();
        return false;
    }
}
