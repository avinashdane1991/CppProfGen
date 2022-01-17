#pragma once

#include <iostream>
#include <cstdlib>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <memory>
#include <sstream>

#include <mosquitto.h>
#include <string.h>
#include <vector>
#include <algorithm>

const std::string CLIENT_ID                            { "cppProfGen" };
const std::string PUB_TOPIC_MONITOR                   { "pyprofgen.lenze.mosaiq/monitor" };
const std::string PUB_TOPIC_PARAMETER_ON_CONNECT      { "pyprofgen.lenze.mosaiq/parameteronconnect" };
const std::string  SUB_TPOIC_COMMAND                   {"pyprofgen.lenze.mosaiq/command"};
const std::string  SUB_TOPIC_PARAMETER                 {"pyprofgen.lenze.mosaiq/parameter"};

const std::string BROKER_HOST                         {"localhost"};
const int BROKER_PORT                                  {1883};
const int KEEP_ALIVE                                 {60};
const bool IS_CLEAN_SESSION                         {true};

class Mqtt 
{
    struct mosquitto *mosq;
    static bool isMessageReceived;
    static std::string topicInMessage;
    static std::string dataInMessage;
    static void message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message);
    static void mosq_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str);
public:
    Mqtt();
    ~Mqtt();
    bool connect();
    void disconnect();
    void publishForTopic_Monitor(float actVelocity, float actPosition, float actRotarySpeed);
    void publishForTopic_ParameterOnConnect(std::vector<float> allPara);
    bool checkForMessagesFromBroker(std::string& topic, std::vector<std::string>& data);
};
