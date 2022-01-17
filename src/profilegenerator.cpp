#include "profilegenerator.h"

unsigned long ProfileGenerator::milliSeconds()
{
    unsigned long timeInMs = 0;
    struct timeval start_time;
    gettimeofday(&start_time, 0);
    timeInMs = ((start_time.tv_sec) * 1000 + start_time.tv_usec/1000.0);
    return timeInMs;
}

unsigned long ProfileGenerator::microSeconds()
{
    unsigned long timeInUs = 0;
    struct timeval start_time;
    gettimeofday(&start_time, 0);
    timeInUs = ((start_time.tv_sec) * 1000000 + start_time.tv_usec);
    return timeInUs;
}

ProfileGenerator::ProfileGenerator()
{
    m_lastTime = 0;
    m_delta = 0;
    m_isFinished = false;
    m_isDirectionPositive = true;
    m_setVelocity = MAX_VELOCITY;
    m_position = 0;

    m_isMoveAutoCmdReceived = false;
    m_isMoveRelativeCmdReceived = false;
    m_isMoveAbsCmdReceived = false; 
    m_isMoveVelocityCmdReceived = false;
    m_isStopCmdReceived = false;
    m_isFinished = false;
    m_isDirectionPositive = true;
    m_targetPosition = TARGET_POSITION;

    m_maxVelocity = MAX_VELOCITY;
    m_maxAcceleration = TARGET_ACCELERATION;
    m_sampleTime = SAMPLING_TIME_MS;

    reset();
}

void ProfileGenerator::checkPosition(float setpoint) 
{
    if(m_isDirectionPositive)
    {
        if(m_position >= setpoint)
        {
            m_position = setpoint;
            m_velocity = 0;
            m_lastTime = 0;
            m_isFinished = true;
        }    
        else
        {
            m_isFinished = false;
        }        
    }
    else
    {
        if(m_position <= setpoint)
        {
            m_position = setpoint;
            m_velocity = 0;
            m_lastTime = 0;
            m_isFinished = true;
        }
        else
        {
            m_isFinished = false;
        }
    }
}
    
//Updates the state, generating new setpoints
//@param setpoint The current setpoint.
float ProfileGenerator::update(float setpoint) 
{
    unsigned long now = milliSeconds();
    unsigned long timeChange = (now - m_lastTime);    
    if(timeChange >= m_sampleTime) 
    {    
        bool valid = timeCalculation();    
        checkPosition(setpoint);        
        if (!m_isFinished) 
        {
            // Shift state variables
            m_oldPosition = m_position;
            m_oldVelocity = m_velocity;
            if (valid) 
            {
                calculateTrapezoidalProfile(setpoint);
                // Update state
                stateCalculation();
            }
        }        
    }
    return m_position;
}


//Calculate delta time in seconds
bool ProfileGenerator::timeCalculation() {
    // Get current time in microseconds
    long currentTime = microSeconds();//micros();
    if (m_lastTime == 0) {
        m_lastTime = currentTime;
        return false;
    }
    // Delta is measured in seconds, thus divide by 1e6 because we're using microseconds.
    m_delta = static_cast<float>(currentTime - m_lastTime) / 1000000;
    m_lastTime = currentTime;
    return true;
}

//Calculate velocity and acceleration during profile generation operation 
void ProfileGenerator::stateCalculation() 
{
    m_velocity = (m_position - m_oldPosition) / m_delta;
    m_acceleration = (m_velocity - m_oldVelocity) / m_delta;
}


void ProfileGenerator::calculateTrapezoidalProfile(float setpoint) 
{
    // Check if we need to de-accelerate
    if (((m_velocity * m_velocity) / m_maxAcceleration) / 2 >= abs(setpoint - m_position)) 
    {
        if (m_velocity < 0) 
        {
            m_position += (m_velocity + m_maxAcceleration * m_delta) * m_delta;
        }
        else if (m_velocity > 0) 
        {
            m_position += (m_velocity - m_maxAcceleration * m_delta) * m_delta;
        }
    }
    else 
    {
        // We're not too close yet, so no need to de-accelerate. Check if we need to accelerate or maintain velocity.
        if (abs(m_velocity) < m_maxVelocity || (setpoint < m_position && m_velocity > 0) || (setpoint > m_position && m_velocity < 0)) 
        {
            // We need to accelerate, do so but check the maximum acceleration.
            // Keep velocity constant at the maximum
            float suggestedVelocity = 0.0;
            if (setpoint > m_position) 
            {
                suggestedVelocity = m_velocity + m_maxAcceleration * m_delta;
                if (suggestedVelocity > m_maxVelocity) 
                {
                    suggestedVelocity = m_maxVelocity;
                }
            }
            else 
            {
                suggestedVelocity = m_velocity - m_maxAcceleration * m_delta;
                if (abs(suggestedVelocity) > m_maxVelocity) 
                {
                    suggestedVelocity = -m_maxVelocity;
                }                
            }
            m_position += suggestedVelocity * m_delta;
        }
        else 
        {
            // Keep velocity constant at the maximum
            if (setpoint > m_position) 
            {
                m_position += m_maxVelocity * m_delta;
            }
            else 
            {
                m_position += -m_maxVelocity * m_delta;
            }
        }
    }

    if(m_isDirectionPositive && m_position >= setpoint)
    {
            m_position = setpoint;    
    }
    else if((m_isDirectionPositive == false) && m_position <= setpoint)
    {
        m_position = setpoint;
    }
} 

bool ProfileGenerator::getFinished() {
    return m_isFinished;
}

void ProfileGenerator::reset() {
    // Reset all state variables
    m_oldPosition = 0;
    m_velocity = 0;
    m_oldVelocity = 0;
    m_acceleration = 0;
    m_rotarySpeed = 0;

    m_lastTime = 0;

    //m_targetPosition = TARGET_POSITION;
    m_autoCycleStopTime = AUTOCYCLE_STOP_TIME_MS;
    m_maxMotorRotarySpeed = MAX_MOTOR_ROTARY_SPEED;
    m_maxVelocity = MAX_VELOCITY;
    m_acceleration = TARGET_ACCELERATION;
}

void ProfileGenerator::moveAutomatic()
{
    static bool s_isAutoCycleStopTimeRequired = false;
    static float s_targetDistance = 0;
    static unsigned long s_oldTime = 0;

    static bool s_isNewCall = true;
    if(s_isNewCall)
    {
        s_isNewCall = false;
        s_targetDistance = m_position + m_targetPosition;
        (m_targetPosition < 0) ? m_isDirectionPositive = false: m_isDirectionPositive = true;
        m_velocity = 0; 

        std::cout << "m_targetPosition: " << std::to_string(m_targetPosition) << " m_position:" << std::to_string(m_position) 
        << " direction:"<< std::to_string(m_isDirectionPositive) << "\n";
    }
    if(m_isStopCmdReceived == false)
    {
        if(s_isAutoCycleStopTimeRequired == false)
        {
            // Calculate actual position    
            update(s_targetDistance);
            // Calculate actual motor rotary speed
            m_rotarySpeed = m_velocity * m_maxMotorRotarySpeed / m_maxVelocity;

            if(getFinished())
            {
                s_isAutoCycleStopTimeRequired = true;
                s_oldTime = milliSeconds();
            }
        }
        else
        {
            // Calculate actual position    
            update(s_targetDistance);

            // Calculate actual motor rotary speed
            m_rotarySpeed = m_velocity * m_maxMotorRotarySpeed / m_maxVelocity;

            if((milliSeconds() - s_oldTime) > m_autoCycleStopTime)
            {
                s_isAutoCycleStopTimeRequired = false;
                s_targetDistance = s_targetDistance + m_targetPosition;
            }
        }
    }
    else
    {
        if(m_targetPosition > 0)
        {
            m_velocity = m_velocity - abs(m_acceleration)*(SAMPLING_TIME_MS/(float)1000);
            if(m_velocity <= 0)
            {
                m_velocity = 0;
                m_isMoveAutoCmdReceived = false;
                m_isStopCmdReceived = false;
                s_isAutoCycleStopTimeRequired = false;
                s_isNewCall = true;
                reset();
            }            
        }
        else
        {
            m_velocity = m_velocity + abs(m_acceleration)*(SAMPLING_TIME_MS/(float)1000);
            if(m_velocity >= 0)
            {
                m_velocity = 0;
                m_isMoveAutoCmdReceived = false;
                m_isStopCmdReceived = false;
                s_isAutoCycleStopTimeRequired = false;
                s_isNewCall = true;
                reset();
            }
        }
        m_position = m_position + m_velocity * (SAMPLING_TIME_MS/(float)1000);
        m_rotarySpeed = m_velocity * m_maxMotorRotarySpeed / m_maxVelocity;
    } 
}

void ProfileGenerator::moveVelocity()
{
    static bool s_isNewCall = true;
    if(s_isNewCall)
    {
        s_isNewCall = false;
        (m_setVelocity < 0) ? m_isDirectionPositive = false : m_isDirectionPositive = true; 
    }

    if(m_isStopCmdReceived == false)
    {
        if(m_isDirectionPositive)
        {
            m_velocity = m_velocity + abs(m_acceleration)*(SAMPLING_TIME_MS/(float)1000);
            if(m_velocity > m_setVelocity)
            {
                m_velocity = m_setVelocity;
            }        
        }
        else
        {
            m_velocity = m_velocity - abs(m_acceleration)*(SAMPLING_TIME_MS/(float)1000);
            if(m_velocity < m_setVelocity)
            {
                m_velocity = m_setVelocity;
            }
        }
        m_position = m_position + m_velocity * (SAMPLING_TIME_MS/(float)1000);
        m_rotarySpeed = m_velocity * m_maxMotorRotarySpeed / m_maxVelocity;
    }
    else
    {
        if(m_isDirectionPositive)
        {
            m_velocity = m_velocity - abs(m_acceleration)*(SAMPLING_TIME_MS/(float)1000);
            if(m_velocity <= 0)
            {
                m_velocity = 0;
                m_isMoveVelocityCmdReceived = false;
                m_isStopCmdReceived = false;
                s_isNewCall = true;
                reset();
            }            
        }
        else
        {
            m_velocity = m_velocity + abs(m_acceleration)*(SAMPLING_TIME_MS/(float)1000);
            if(m_velocity >= 0)
            {
                m_velocity = 0;
                m_isMoveVelocityCmdReceived = false;
                m_isStopCmdReceived = false;
                s_isNewCall = true;
                reset();
            }
        }
        m_position = m_position + m_velocity * (SAMPLING_TIME_MS/(float)1000);
        m_rotarySpeed = m_velocity * m_maxMotorRotarySpeed / m_maxVelocity;    
    }    
}


void ProfileGenerator::moveRelative()
{
    static bool s_isAutoCycleStopTimeRequired = false;
    static float s_targetDistance = 0;//position + targetPosition;
    static unsigned long s_oldTime = 0;

    static bool s_isNewCall = true;
    if(s_isNewCall)
    {
        s_isNewCall = false;
        s_targetDistance = m_position + m_targetPosition;
        (m_targetPosition < 0) ? m_isDirectionPositive = false: m_isDirectionPositive = true;
    }

    if(m_isStopCmdReceived == false)
    {
        // Calculate actual position    
        update(s_targetDistance);
        // Calculate actual motor rotary speed
        m_rotarySpeed = m_velocity * m_maxMotorRotarySpeed / m_maxVelocity;

        if(getFinished())
        {
            m_isMoveRelativeCmdReceived = false;
            m_isStopCmdReceived = false;
            s_isNewCall = true;
            reset();
        }        
    }
    else
    {        
        if(m_targetPosition > 0)
        {
            m_velocity = m_velocity - abs(m_acceleration)*(SAMPLING_TIME_MS/(float)1000);
            if(m_velocity <= 0)
            {
                m_velocity = 0;
                m_isMoveRelativeCmdReceived = false;
                m_isStopCmdReceived = false;
                s_isNewCall = true;
                reset();
            }            
        }
        else
        {
            m_velocity = m_velocity + abs(m_acceleration)*(SAMPLING_TIME_MS/(float)1000);
            if(m_velocity >= 0)
            {
                m_velocity = 0;
                m_isMoveRelativeCmdReceived = false;
                m_isStopCmdReceived = false;
                s_isNewCall = true;
                reset();
            }
        }
        m_position = m_position + m_velocity * (SAMPLING_TIME_MS/(float)1000);
        m_rotarySpeed = m_velocity * m_maxMotorRotarySpeed / m_maxVelocity;
    } 
}

void ProfileGenerator::moveAbsolute()
{
    static float s_targetDistance = m_targetPosition;
    static bool s_isNewCall = true;

    //std::cout << "isNewCall:" << std::to_string(s_isNewCall) <<"\n";
    if(s_isNewCall)
    {
        s_isNewCall = false;
        s_targetDistance = m_targetPosition; 
        (m_targetPosition < m_position) ? m_isDirectionPositive = false: m_isDirectionPositive = true;
        m_velocity = 0;

        std::cout << "m_targetPosition: " << std::to_string(m_targetPosition) << " m_position:" << std::to_string(m_position) 
        << " direction:"<< std::to_string(m_isDirectionPositive) << "\n";
    }
    if(m_isStopCmdReceived == false)
    {
        // Calculate actual position    
        update(s_targetDistance);
        // Calculate actual motor rotary speed
        m_rotarySpeed = m_velocity * m_maxMotorRotarySpeed / m_maxVelocity;

        if(getFinished())
        {                
            m_isMoveAbsCmdReceived = false;
            m_isStopCmdReceived = false;
            s_isNewCall = true;
            reset();    
        }
    }
    else
    {        
        if(m_targetPosition > 0)
        {
            m_velocity = m_velocity - abs(m_acceleration)*(SAMPLING_TIME_MS/(float)1000);
            if(m_velocity <= 0)
            {
                m_velocity = 0;
                m_isMoveAbsCmdReceived = false;
                m_isStopCmdReceived = false;
                s_isNewCall = true;
                reset();
            }            
        }
        else
        {
            m_velocity = m_velocity + abs(m_acceleration)*(SAMPLING_TIME_MS/(float)1000);
            if(m_velocity >= 0)
            {
                m_velocity = 0;
                m_isMoveAbsCmdReceived = false;
                m_isStopCmdReceived = false;
                s_isNewCall = true;
                reset();
            }
        }
        m_position = m_position + m_velocity * (SAMPLING_TIME_MS/(float)1000);
        m_rotarySpeed = m_velocity * m_maxMotorRotarySpeed / m_maxVelocity;
    } 
}


void ProfileGenerator::parseMqttMessageForTopicCommand(std::string command)
{
    if(command == "MC_MoveVelocity")
    {                        
        m_isMoveVelocityCmdReceived = true;    
    }
    else if(command == "MC_MoveRelative")
    {
        m_isMoveRelativeCmdReceived = true;
    }
    else if(command == "MC_LE_AutomaticMode")
    {
        m_isMoveAutoCmdReceived = true;
    }
    else if(command == "MC_MoveAbsolute")
    {
        m_isMoveAbsCmdReceived = true;
    }
    else if(command == "MC_MoveStop")
    {
        m_isStopCmdReceived = true;
    }
}

void ProfileGenerator::parseMqttMessageForTopicParameter(std::vector<std::string> parameters)
{
    if(parameters[0] == "setVelocity")
    {
        m_setVelocity = atof(parameters[1].c_str());    
        if(m_setVelocity < 0)
        {
            m_isDirectionPositive = false;
        }
        else
        {
            m_isDirectionPositive = true;
        }            
    }
    else if(parameters[0] == "setAcceleration")
    {
        m_acceleration = atof(parameters[1].c_str());
    }
    else if(parameters[0] == "maxVelocity")
    {
        m_maxVelocity    = atof(parameters[1].c_str());
    }
    else if(parameters[0] == "maxMotorRotarySpeed")
    {
        m_maxMotorRotarySpeed = atof(parameters[1].c_str());
    }
    else if(parameters[0] == "automaticCycleStopTime")
    {
        m_autoCycleStopTime = (atof(parameters[1].c_str())) * 1000;
    }
    else if(parameters[0] == "setDistance")
    {
        m_targetPosition = atof(parameters[1].c_str());
        std::cout << "setDistance: " << parameters[1] << "\n";
        (m_targetPosition < 0) ? m_isDirectionPositive = false : m_isDirectionPositive = true;
    }
}

void ProfileGenerator::parseMqttMessage(std::string topic,std::vector<std::string> dataInMessage)
{
    std::string topicCommand(SUB_TPOIC_COMMAND);
    std::string topicParameter(SUB_TOPIC_PARAMETER);

    if(topic == topicCommand)
    {                    
        parseMqttMessageForTopicCommand(dataInMessage[1]);
    }
    else if(topic == topicParameter)
    {
        parseMqttMessageForTopicParameter(dataInMessage);
    }
}

void ProfileGenerator::executeMqttCommand()
{
    if(m_isMoveAutoCmdReceived)
    {                
        moveAutomatic();
    }
    else if(m_isMoveVelocityCmdReceived)
    {
        moveVelocity();
    }
    else if(m_isMoveRelativeCmdReceived)
    {
        moveRelative();
    }
    else if(m_isMoveAbsCmdReceived)
    {
        moveAbsolute();
    }
    else
    {
        if(m_isStopCmdReceived)
        {
            m_isStopCmdReceived = false;
            reset();
        }
    }
    m_Mqtt.publishForTopic_Monitor(m_velocity,m_position,m_rotarySpeed);
}

void ProfileGenerator::init()
{
  m_Mqtt.connect();
  std::vector<float> allConfigPara;
  allConfigPara.push_back(TARGET_POSITION);
  allConfigPara.push_back(MAX_VELOCITY);
  allConfigPara.push_back(TARGET_ACCELERATION);
  allConfigPara.push_back(MAX_POSITION);
  allConfigPara.push_back(MIN_POSITION);
  allConfigPara.push_back(MAX_VELOCITY);
  allConfigPara.push_back(MAX_ACCELERATION);
  allConfigPara.push_back(MAX_MOTOR_ROTARY_SPEED);
  allConfigPara.push_back(AUTOCYCLE_STOP_TIME_MS);
  m_Mqtt.publishForTopic_ParameterOnConnect(allConfigPara);
}

void ProfileGenerator::generateProfile()
{
    std::string topic;
    std::vector<std::string> dataInMessage;
    if(m_Mqtt.checkForMessagesFromBroker(topic,dataInMessage) == true)
    {
        parseMqttMessage(topic,dataInMessage);
    }
    executeMqttCommand();
}

void ProfileGenerator::stop()
{
    m_Mqtt.disconnect();
}

