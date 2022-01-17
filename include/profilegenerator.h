#pragma once

#include <sys/time.h>
#include "mqtt.h"

const float TARGET_POSITION         {200.0f};
const float MAX_VELOCITY            {300.0f};
const float TARGET_ACCELERATION     {200.0f};
const float MAX_ACCELERATION        {1000.0f};
const float MAX_MOTOR_ROTARY_SPEED  {2000.0f};
const long int SAMPLING_TIME_MS     {200};
const float AUTOCYCLE_STOP_TIME_MS  {1000};
const float MAX_POSITION            {100000000.0f};
const float MIN_POSITION            {-100000000.0f};


class ProfileGenerator {        
    public:
        ProfileGenerator();
        
        void init();

        void generateProfile();

        void stop();
        
    private:
        //Updates the state, generating new setpoints
        //@param setpoint The current setpoint.
        float update(float aSetpoint);

        void checkPosition(float setpoint);

        bool getFinished();

        void reset();

        unsigned long milliSeconds();

        unsigned long microSeconds();

        void moveAutomatic();

        void moveVelocity();

        void moveAbsolute();

        void moveRelative();

        bool timeCalculation();

        //Calculate velocity and acceleration during profile generation operation
        void stateCalculation();

        void calculateTrapezoidalProfile(float);

        void parseMqttMessage(std::string topic,std::vector<std::string> dataInMessage);

        void parseMqttMessageForTopicCommand(std::string command);

        void parseMqttMessageForTopicParameter(std::vector<std::string> parameters);

        void executeMqttCommand();

        Mqtt m_Mqtt;

        bool m_isMoveAutoCmdReceived;
        bool m_isMoveRelativeCmdReceived;
        bool m_isMoveAbsCmdReceived; 
        bool m_isMoveVelocityCmdReceived;
        bool m_isStopCmdReceived;
        bool m_isFinished;
        bool m_isDirectionPositive;
        
        float m_maxVelocity;
        float m_maxAcceleration;
        float m_position;
        float m_oldPosition;
        float m_velocity;
        float m_oldVelocity;
        float m_acceleration;
        float m_rotarySpeed;
        float m_setVelocity;
        float m_targetPosition;
        float m_autoCycleStopTime;
        float m_maxMotorRotarySpeed;
        float m_delta;

        unsigned long m_lastTime;    
        unsigned long m_sampleTime;
};
