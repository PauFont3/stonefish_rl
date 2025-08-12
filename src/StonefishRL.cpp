#include "StonefishRL.h"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <sstream> // To build the JSON for observations
#include <map>
#include <unordered_map>
#include <unordered_set>

#include <zmq.hpp> 

// Sensor includes
#include <Stonefish/sensors/Sensor.h>
#include <Stonefish/sensors/Sample.h>
#include <Stonefish/sensors/ScalarSensor.h> // For getLastSample()
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/scalar/RotaryEncoder.h>
#include <Stonefish/sensors/vision/Camera.h>
#include <Stonefish/sensors/scalar/Pose.h>
#include "sensors/scalar/LinkSensor.h"
#include "sensors/scalar/JointSensor.h"

// Actuator includes
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/actuators/Motor.h>
#include <Stonefish/actuators/Actuator.h>
#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/actuators/LinkActuator.h>
#include <Stonefish/actuators/JointActuator.h>

#include <Stonefish/core/Robot.h>

#include <Stonefish/entities/SolidEntity.h>

#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/core/SimulationApp.h>
#include <Stonefish/core/ScenarioParser.h> // To load the XML file
#include <Stonefish/StonefishCommon.h>

#include <SDL2/SDL.h>

// Constructor of the StonefishRL class
StonefishRL::StonefishRL(const std::string &path, double frequency)
    : sf::SimulationManager(frequency),
      scenePath(path),
      context(1),                            // Initialize ZeroMQ context with 1 thread
      socket(context, zmq::socket_type::rep) // Create a REP socket
{
    InitializeZMQ();
}


std::string StonefishRL::RecieveInstructions(sf::SimulationApp& simApp)
{
    zmq::message_t request;

    // Wait to receive a message --> It will receive: "RESET:robot_name;"
    auto result = socket.recv(request, zmq::recv_flags::none);

    // Convert the message (buffer) to string and show it
    std::string cmd = request.to_string();
    // std::cout << "[ZMQ] Received: (" << result.value() << " bytes) in command: " << cmd << std::endl;

     // Remove the instruction type prefix
    int pos = cmd.find(":");                 // Take up to the first ":" found
    std::string prefix = cmd.substr(0, pos); // Keep what the command must do
    cmd = cmd.substr(pos + 1);               // +1 to drop the ":"

    if (prefix == "RESET")
    {
        std::vector<InfoObject> command_data;
        command_data = ParseResetCommand(cmd);

        SetRobotPosition(command_data);        
        SendObservations();
            
        return "RESET";
    }
    else if (prefix == "EXIT")
    {
       std::cout << "[ZMQ] Received EXIT\n";
        socket.send(zmq::buffer("EXIT OK"), zmq::send_flags::none);
        return "EXIT";
    }
    else if (prefix == "CMD")
    {
        ApplyCommands(cmd);
        return "CMD";
    }
    else
    {
        std::cout << "[ERROR] UPS! Something was not written correctly in the command.\n";
        return "INVALID";
    }
}


void StonefishRL::SendObservations()
{
    StateScene scalar_observations = GetStateScene();
    //PrintAll();
    // Convert observations to string
    std::string obs_str_json = SerializeScene(scalar_observations.observations);
    
    this->socket.send(zmq::buffer(obs_str_json), zmq::send_flags::none);
}


void StonefishRL::ApplyCommands(const std::string& str_cmds)
{
    // Assign new values to the structure kept as StonefishRL members.
    ParseCommandsAndObservations(str_cmds);

    unsigned int id = 0;
    sf::Actuator *actuator_ptr;

    while ((actuator_ptr = getActuator(id++)) != nullptr)
    {
        std::string actuator_name = actuator_ptr->getName();

        // Continue only if there are commands for the current actuator
        if (commands_.count(actuator_name) > 0)
        {
            switch (actuator_ptr->getType())
            {
            // SERVO
            case sf::ActuatorType::SERVO:
            {
                sf::Servo *servo = dynamic_cast<sf::Servo *>(actuator_ptr);
                if (!servo)
                {
                    std::cout << "[WARNING] Not converted " << actuator_name << " to SERVO.\n";
                    break;
                }

                // Apply all actions specified for the SERVO
                for (const auto &[action, action_value] : commands_[actuator_name])
                {
                    if (action == "VELOCITY" || action == "TORQUE")
                    {
                        servo->setControlMode(sf::ServoControlMode::VELOCITY);
                        servo->setDesiredVelocity(action_value);
                    }
                    else if (action == "POSITION") 
                    {
                        servo->setControlMode(sf::ServoControlMode::POSITION);
                        servo->setDesiredPosition(action_value);
                    }
                    else
                    {
                        std::cout << "[WARNING] Unknown command '" << action << "' for servo '" << actuator_name << "'\n";
                    }
                }
                break;
            }

            // THRUSTER
            case sf::ActuatorType::THRUSTER:
            {
                sf::Thruster *thruster = dynamic_cast<sf::Thruster *>(actuator_ptr);
                if (!thruster)
                {
                    std::cout << "[WARNING] Not converted " << actuator_name << " to THRUSTER.\n";
                    break;
                }

                // Apply all actions specified for the THRUSTER
                for (const auto &[action, action_value] : commands_[actuator_name])
                {
                    if (action == "VELOCITY" || action == "TORQUE")
                    {
                        thruster->setSetpoint(action_value);
                    }
                    else
                    {
                        std::cout << "[WARNING] Unknown command '" << action << "' for servo '" << actuator_name << "'\n";
                    }
                }
                break;
            }

            default:
                std::cout << "[WARNING] Actuator type not supported: " << actuator_name << "\n";
                break;
            }
        }
    }
}

void StonefishRL::BuildScenario()
{
    std::cout << "[INFO] Building scenario from: " << scenePath << std::endl;
    sf::ScenarioParser parser(this);

    if (!parser.Parse(scenePath))
    {
        std::cerr << "[ERROR] Error charging the scenario: " << scenePath << "\n";
        for (const auto &msg : parser.getLog())
        {
            std::cerr << "[ScenarioParser Log] " << msg.text << "\n";
        }
        return;
    }

    sf::Sensor *sensor_ptr;
    unsigned int sensor_id = 0;
    bool item_found = false;

    while ((sensor_ptr = getSensor(sensor_id++)) != nullptr)
    {
      item_found = true;
    }
    if (!item_found) std::cout << "[WARN] No sensors registered in this scenario!" << std::endl;

    sf::Actuator *actuator_ptr;
    unsigned int actuator_id = 0;
    item_found = false;
    while ((actuator_ptr = getActuator(actuator_id++)) != nullptr)
    {
        item_found = true;
    }
    if (!item_found) std::cout << "[WARN] No actuators registered in this scenario!" << std::endl;

    sf::Robot *robot_ptr;
    unsigned int robot_id = 0;
    item_found = false;
    while ((robot_ptr = getRobot(robot_id++)) != nullptr)
    {
        item_found = true;
    }
    if (!item_found) std::cout << "[WARN] No robots registered in this scenario!" << std::endl;

    std::cout << "[INFO] Scenario loaded succesfully.\n";
}


StonefishRL::StateScene StonefishRL::GetStateScene()
{
    sf::Sensor *sensor_ptr;
    sf::Robot *robot_ptr;
    sf::Actuator *actuator_ptr;
    StateScene state;

    unsigned int id = 0;
    while ((robot_ptr = getRobot(id++)) != nullptr)
    {
        std::string robot_name = robot_ptr->getName();
        
        if(ObjImportantForObs(robot_name))
        {
            sf::Vector3 origin = robot_ptr->getTransform().getOrigin();
            sf::Scalar yawZ, pitchY, rollX;
            robot_ptr->getTransform().getRotation().getEulerZYX(yawZ, pitchY, rollX);

            InfoObject obs;
            FillWithNanInfoObject(obs);
            obs.name = robot_name;
            obs.position[0] = origin.getX();
            obs.position[1] = origin.getY();
            obs.position[2] = origin.getZ();
            obs.rotation[0] = rollX;
            obs.rotation[1] = pitchY;
            obs.rotation[2] = yawZ;

            state.observations.push_back(obs);
        } 
    }

    id = 0;
    while ((sensor_ptr = getSensor(id++)) != nullptr)
    {
        std::string sensor_name = sensor_ptr->getName();

        if (ObjImportantForObs(sensor_name))
        {
            sf::ScalarSensor *sensor = dynamic_cast<sf::ScalarSensor *>(sensor_ptr);
            if (!sensor) continue;
            
            InfoObject obs; // I declare it here so each new object cannot carry over previous data
            FillWithNanInfoObject(obs);
            
            obs.name = sensor_name;
            
            // ENCODING SENSOR
            if(sensor->getScalarSensorType() == sf::ScalarSensorType::ENCODER)
            {
                obs.angle = sensor->getLastSample().getValue(0);   // Sensor angle
                obs.angular_velocity[2] = sensor->getLastSample().getValue(1); // Put the single value on Z axis, Python will handle it later

                state.observations.push_back(obs);
            }
        
            // ODOMETRY SENSOR
            if(sensor->getScalarSensorType() == sf::ScalarSensorType::ODOM)
            {
                obs.position[0] = sensor->getLastSample().getValue(0); // Position X
                obs.position[1] = sensor->getLastSample().getValue(1); // Position Y 
                obs.position[2] = sensor->getLastSample().getValue(2); // Position Z

                obs.linear_velocity[0] = sensor->getLastSample().getValue(3); // Linear velocity X
                obs.linear_velocity[1] = sensor->getLastSample().getValue(4); // Linear velocity Y
                obs.linear_velocity[2] = sensor->getLastSample().getValue(5); // Linear velocity Z
                
                obs.rotation[0] = sensor->getLastSample().getValue(6); // Rotation X
                obs.rotation[1] = sensor->getLastSample().getValue(7); // Rotation Y
                obs.rotation[2] = sensor->getLastSample().getValue(8); // Rotation Z
                float rot_w = sensor->getLastSample().getValue(9); // Not stored in the struct because it's not needed now,
                                                                   // otherwise, you must follow the same procedure as adding
                                                                   // a new sensor, but since the sensor already exists, we 
                                                                   // would just add this missing parameter to the (Odometry) sensor
                                                                   
                obs.angular_velocity[0] = sensor->getLastSample().getValue(10); // Angular velocity X
                                                                   
                obs.angular_velocity[1] = sensor->getLastSample().getValue(11); // Angular velocity Y
                obs.angular_velocity[2] = sensor->getLastSample().getValue(12); // Angular velocity Z
                
                state.observations.push_back(obs);
            }

            // PRESSURE SENSOR
            else if(sensor->getScalarSensorType() == sf::ScalarSensorType::PRESSURE)
            {
                obs.pressure = sensor->getLastSample().getValue(0); // Pressure

                state.observations.push_back(obs);
            }

            // FORCE_TORQUE SENSOR
            else if(sensor->getScalarSensorType() == sf::ScalarSensorType::FT)
            {
                obs.force[0] = sensor->getLastSample().getValue(0); // Force X
                obs.force[1] = sensor->getLastSample().getValue(1); // Force Y
                obs.force[2] = sensor->getLastSample().getValue(2); // Force Z
                obs.torque[0] = sensor->getLastSample().getValue(3); // Torque X
                obs.torque[1] = sensor->getLastSample().getValue(4); // Torque Y
                obs.torque[2] = sensor->getLastSample().getValue(5); // Torque Z

                state.observations.push_back(obs);
            }

            // GPS SENSOR
            else if(sensor->getScalarSensorType() == sf::ScalarSensorType::GPS)
            {
                obs.gps[0] = sensor->getLastSample().getValue(0); // Latitude
                obs.gps[1] = sensor->getLastSample().getValue(1); // Longitude
                obs.gps[2] = sensor->getLastSample().getValue(2); // North
                obs.gps[3] = sensor->getLastSample().getValue(3); // East
                
                state.observations.push_back(obs);
            }
            
            // IMU_FILTER SENSOR
            else if(sensor->getScalarSensorType() == sf::ScalarSensorType::IMU)
            {
                obs.rotation[0] = sensor->getLastSample().getValue(0); // Roll
                obs.rotation[1] = sensor->getLastSample().getValue(1); // Pitch
                obs.rotation[2] = sensor->getLastSample().getValue(2); // Yaw

                obs.angular_velocity[0] = sensor->getLastSample().getValue(3); // Angular velocity X
                obs.angular_velocity[1] = sensor->getLastSample().getValue(4); // Angular velocity Y
                obs.angular_velocity[2] = sensor->getLastSample().getValue(5); // Angular velocity Z
                
                obs.linear_acceleration[0] = sensor->getLastSample().getValue(6); // Linear acceleration X
                obs.linear_acceleration[1] = sensor->getLastSample().getValue(7); // Linear acceleration Y
                obs.linear_acceleration[2] = sensor->getLastSample().getValue(8); // Linear acceleration Z

                state.observations.push_back(obs);
            }
        }   
    }

    id = 0;
    while ((actuator_ptr = getActuator(id++)) != nullptr)
    {
        std::string actuator_name = actuator_ptr->getName();
        if (ObjImportantForObs(actuator_name))
        {
            InfoObject obs;
            FillWithNanInfoObject(obs);
            obs.name = actuator_ptr->getName();
            
            // SERVO ACTUATOR
            if (actuator_ptr->getType() == sf::ActuatorType::SERVO)
            {   
                sf::Servo *servo_ptr = dynamic_cast<sf::Servo *>(actuator_ptr);
                if (!servo_ptr) continue;
                
                obs.angle = servo_ptr->getPosition();
                obs.angular_velocity[2] = servo_ptr->getVelocity();
            
                state.observations.push_back(obs);
            }

            // THRUSTER ACTUATOR
            else if (actuator_ptr->getType() == sf::ActuatorType::THRUSTER)
            {
                sf::Thruster *thruster_ptr = dynamic_cast<sf::Thruster *>(actuator_ptr);
                if (!thruster_ptr) continue;

                obs.angle = thruster_ptr->getSetpoint();

                obs.angular_velocity[2] = thruster_ptr->getThrust();

                state.observations.push_back(obs);
            }
        }
    }
    current_state_ = state;
    return state;
}


void StonefishRL::InitializeZMQ()
{
    try
    {
        socket.bind("tcp://*:5555"); // Listens on port 5555
         std::cout << "[ZMQ] REP server active on port 5555\n";
    }
    catch (const zmq::error_t &e)
    {
        std::cerr << "[ZMQ ERROR] " << e.what() << "\n";
    }
}


std::vector<StonefishRL::InfoObject> StonefishRL::ParseResetCommand(const std::string& str_command){
    
    std::vector<InfoObject> result;

    int pos = 0;

    // While there is a '{..}'
    while((pos = str_command.find("{", pos)) != std::string::npos) {
        
        // Find the end marker '}'
        int end = str_command.find("}", pos); 

        if(end == std::string::npos) break;
        
         // Take and store in a string the content inside '{..}'
        std::string object_str = str_command.substr(pos, end - pos + 1);
    
        InfoObject obj;
        // Set default values, because we will not use these values now.
        // If a different struct is created for reset values this could be removed,
        // but I think it's simpler to reuse the existing struct that already contains the needed data.
        obj.angle = 0;

        // ---- NAME ----
        // Find and grab 'name'
        size_t name_pos = object_str.find("\"name\"");
        if (name_pos != std::string::npos) {
            // Busca les ' ".." ' que contenen el valor del nom 
            size_t init_name_pos = object_str.find("\"", name_pos + 6);
            size_t end_name_pos = object_str.find("\"", init_name_pos + 1);
            obj.name = object_str.substr(init_name_pos + 1, end_name_pos - init_name_pos -1);
        }

        // ---- POSITION ----
        size_t pos_pos = str_command.find("\"position\""); 
        if (pos_pos != std::string::npos) {
            size_t bracket1 = object_str.find("[", pos_pos);
            size_t bracket2 = object_str.find("]", bracket1);
            std::string list = object_str.substr(bracket1 + 1, bracket2 - bracket1 - 1);

            // Split by commas and convert each value to float
            std::stringstream ss(list);
            std::string val;
            while (std::getline(ss, val, ',')) {
                obj.position.push_back(std::stof(val));
            }
        }

        // ---- ROTATION ----
        size_t rot_pos = object_str.find("\"rotation\"");
        if (rot_pos != std::string::npos) {
            
            size_t ini_bracket = object_str.find("[", rot_pos);
            size_t end_bracket = object_str.find("]", ini_bracket);
            
            if (ini_bracket != std::string::npos && end_bracket != std::string::npos) {
                std::string list = object_str.substr(ini_bracket + 1, end_bracket - ini_bracket - 1);

                std::stringstream ss(list);
                std::string val;
                while (std::getline(ss, val, ',')) {
                    obj.rotation.push_back(std::stof(val));
                }
            }
        }

        result.push_back(obj);
        pos = end + 1;
    }

    return result;
}


void StonefishRL::ParseCommandsAndObservations(const std::string& str)
{
    // Clean the previous values in case new ones arrive
    commands_.clear();
    relevant_obs_names_.clear();

    int obs_pos = str.find("OBS:");
    
    if (obs_pos == std::string::npos) {
        std::cerr << "[WARN] Missing 'OBS:' in the command string.\n";
        return;
    }

    std::string cmd_str = str.substr(0, obs_pos);  // Everything before "OBS:"
    std::string obs_str = str.substr(obs_pos + 4); // May be empty

    // --- Process commands (CMD:) ---
    std::stringstream ss(cmd_str);
    std::string token;
    
    // Split by ";"
    while (std::getline(ss, token, ';'))
    {
        if (token.empty()) continue;

        std::istringstream tokenStream(token);
        std::string actuator_name, action, action_value;
        
        if (std::getline(tokenStream, actuator_name, ':')
            && std::getline(tokenStream, action, ':')
            && std::getline(tokenStream, action_value)) 
        {
            try {
                float value = std::stof(action_value);
                commands_[actuator_name][action] = value;
            }
            catch (...)
            { // Catch any exception type
                std::cerr << "[ERROR] Invalid value for " << actuator_name << ":" << action << " --> '" << token << "'\n";
            }
        }
        else {
            std::cerr << "[ERROR] Invalid command format: '" << token << "'. Expected format: 'actuator_name:action:value;'.\n";
        }
    }

    // --- Process observations (OBS:) ---
    // If obs_str is empty, show all observations
    if (!obs_str.empty()) {
        std::istringstream obsStream(obs_str);
        std::string obj_name;

        while (std::getline(obsStream, obj_name, ';')) {
            if (!obj_name.empty()) {
                relevant_obs_names_.insert(obj_name);
            }
        }
    }
}


void StonefishRL::SetRobotPosition(const std::vector<InfoObject>& obj)
{
    int i = 0;
    while(i < obj.size())
    {
        sf::Robot *robot_ptr;
        bool robot_found = false;
        unsigned int id = 0;
        std::string robot_name = obj[i].name;

        if(obj[i].position.size() < 3 || obj[i].rotation.size() < 3) {
            std::cerr << "Error: Not enough values for the rotation or position of the robot '" << robot_name << "\n";
            return;
        }
        
        while ((robot_ptr = getRobot(id++)) != nullptr && !robot_found)
        {
            // Only reset the position of the robot whose name was written in the RESET command
            if(robot_ptr->getName() == robot_name)
            {     
                robot_found = true;
            
                sf::Transform tf;
                sf::Vector3 new_position(obj[i].position[0], obj[i].position[1], obj[i].position[2]);
                tf.setOrigin(new_position); // Move the robot to the indicated position
                
                
                sf::Quaternion rotation(obj[i].rotation[0], obj[i].rotation[1], obj[i].rotation[2]);
                tf.setRotation(rotation); // Rotate the robot 
                
                robot_ptr->Respawn(this, tf);
            }
        }

        if(!robot_found) std::cout << "NOT ABLE TO FIND THE ROBOT BY THE GIVEN NAME: " + robot_name << std::endl; 
        
        i++;
    }
    
    if (i==0) std::cout << "[WARN] No robots have been reseted \n";
}


void StonefishRL::ExitRequest() {
    socket.close();
    context.close();

    std::cout << "[INFO] Simulation finished." << std::endl;
    std::exit(0);
}


// Keep only the names from actuator commands sent from Python,
// because when iterating over all the actuators and sensors
// we apply only for those requested in the command filter.
bool StonefishRL::ObjImportantForObs(const std::string& objName) const {
    return relevant_obs_names_.empty() || relevant_obs_names_.count(objName) > 0;
}


std::string StonefishRL::InfoObjectToJson(const InfoObject& obj)
{
    std::ostringstream oss;
    oss << "{"
        << "\"position\": [" 
        << SafeFloat(obj.position[0]) << ", "
        << SafeFloat(obj.position[1]) << ", "
        << SafeFloat(obj.position[2]) << "], "
        << "\"rotation\": [" 
        << SafeFloat(obj.rotation[0]) << ", "
        << SafeFloat(obj.rotation[1]) << ", "
        << SafeFloat(obj.rotation[2]) << "], "
        << "\"angle\": " << SafeFloat(obj.angle) << ", " 
        << "\"angular_velocity\": ["
        << SafeFloat(obj.angular_velocity[0]) << ", "
        << SafeFloat(obj.angular_velocity[1]) << ", "
        << SafeFloat(obj.angular_velocity[2]) << "], "
        << "\"linear_velocity\": [" 
        << SafeFloat(obj.linear_velocity[0]) << ", "
        << SafeFloat(obj.linear_velocity[1]) << ", "
        << SafeFloat(obj.linear_velocity[2]) << "], "
        << "\"linear_acceleration\": [" 
        << SafeFloat(obj.linear_acceleration[0]) << ", "
        << SafeFloat(obj.linear_acceleration[1]) << ", "
        << SafeFloat(obj.linear_acceleration[2]) << "], "
        << "\"pressure\": " << SafeFloat(obj.pressure) << ", "
        << "\"force\": ["
        << SafeFloat(obj.force[0]) << ", "
        << SafeFloat(obj.force[1]) << ", "
        << SafeFloat(obj.force[2]) << "], "
        << "\"torque\": ["
        << SafeFloat(obj.torque[0]) << ", "
        << SafeFloat(obj.torque[1]) << ", "
        << SafeFloat(obj.torque[2]) << "], "
        << "\"gps\": ["
        << SafeFloat(obj.gps[0]) << ", "
        << SafeFloat(obj.gps[1]) << ", "
        << SafeFloat(obj.gps[2]) << ", "
        << SafeFloat(obj.gps[3]) << "] "
        << "}";

    return oss.str();
}


std::string StonefishRL::EscapeJson(const std::string& s) {
    std::ostringstream oss;
    oss << "\"";
    for (char c : s) {
        switch (c) {
            case '\"': oss << "\\\""; break;
            case '\\': oss << "\\\\"; break;
            default: oss << c;
        }
    }
    oss << "\"";
    return oss.str();
}


std::string StonefishRL::SerializeScene(const std::vector<InfoObject>& objs) {
    std::ostringstream oss;
    oss << "{";
    for (size_t i = 0; i < objs.size(); i++) {
        oss << EscapeJson(objs[i].name) << ":" << InfoObjectToJson(objs[i]);
        if (i < objs.size() - 1) oss << ",";
    }
    oss << "}";
    return oss.str();
}


void StonefishRL::FillWithNanInfoObject(InfoObject& obj)
{   
    obj.angle = NAN;
    obj.pressure = NAN;
 
    obj.position.resize(3, NAN);
    obj.rotation.resize(3, NAN);
    
    obj.linear_velocity.resize(3, NAN); 
    obj.linear_acceleration.resize(3, NAN);
    obj.angular_velocity.resize(3, NAN); 
    
    obj.force.resize(3, NAN);
    obj.torque.resize(3, NAN);

    obj.gps.resize(4, NAN); // Pos: [0] latitude, [1] longitude, [2] North, [3] East
}


// Because Python can interpret it as an empty value
std::string StonefishRL::SafeFloat(float val) 
{
    if (std::isnan(val)) return "null";
    else return std::to_string(val);
}



// No functional purpose, just prints values to console
void StonefishRL::PrintAll()
{
    sf::Sensor *sensor_ptr;
    sf::Robot *robot_ptr;
    sf::Actuator *actuator_ptr;

    unsigned int id = 0;
    std::cout << "\n ROBOT INFO: \n";
    while ((robot_ptr = getRobot(id++)) != nullptr)
    {
        std::cout << robot_ptr->getName() << std::endl;
        sf::Vector3 origin = robot_ptr->getTransform().getOrigin();
        std::cout << "[ROBOT INFORMATION CHANNEL] POSICIO X: " << origin.getX() << std::endl;
        std::cout << "[ROBOT INFORMATION CHANNEL] POSICIO Y: " << origin.getY() << std::endl;
        std::cout << "[ROBOT INFORMATION CHANNEL] POSICIO Z: " << origin.getZ() << std::endl;
    }

    id = 0;
    std::cout << "\n ACTUATOR INFO: \n";
    while ((actuator_ptr = getActuator(id++)) != nullptr)
    {
        std::cout << actuator_ptr->getName() << std::endl;
        if (actuator_ptr->getType() == sf::ActuatorType::SERVO)
        {   
            std::cout << "SERVO\n";
            sf::Servo *servo_ptr = dynamic_cast<sf::Servo *>(actuator_ptr);
            std::cout << "[ACTUATOR INFORMATION CHANNEL] Angle: " << servo_ptr->getPosition() << std::endl;
            std::cout << "[ACTUATOR INFORMATION CHANNEL] Angular velocity: " << servo_ptr->getVelocity() << std::endl;;
        }
        else if (actuator_ptr->getType() == sf::ActuatorType::THRUSTER)
        {   
            std::cout << "THRUSTER\n";
            sf::Thruster *thruster_ptr = dynamic_cast<sf::Thruster *>(actuator_ptr);
            std::cout << "[ACTUATOR INFORMATION CHANNEL] Is propeller right-handed? " << thruster_ptr->isPropellerRight() << std::endl;
            std::cout << "[ACTUATOR INFORMATION CHANNEL] Propeller diameter: " << thruster_ptr->getPropellerDiameter() << std::endl;
            std::cout << "[ACTUATOR INFORMATION CHANNEL] Angular Velocity: " << thruster_ptr->getOmega() << " [rad/s]" << std::endl;
            std::cout << "[ACTUATOR INFORMATION CHANNEL] Angle: " << thruster_ptr->getAngle() << " [rad]" << std::endl;
            std::cout << "[ACTUATOR INFORMATION CHANNEL] Torque: " << thruster_ptr->getTorque() << std::endl;
            std::cout << "[ACTUATOR INFORMATION CHANNEL] Thrust: " << thruster_ptr->getThrust() << std::endl;
            std::cout << "[ACTUATOR INFORMATION CHANNEL] Set Point: " << thruster_ptr->getSetpoint() << std::endl;
            std::cout << "[ACTUATOR INFORMATION CHANNEL] Set Point Limit: " << thruster_ptr->getSetpointLimit() << std::endl;
        }
    }

    id = 0;
    std::cout << "\n SENSOR INFO: \n";
    while ((sensor_ptr = getSensor(id++)) != nullptr)
    {
        if (sensor_ptr->getType() == (sf::SensorType::JOINT) || sensor_ptr->getType() == sf::SensorType::LINK) {
            std::cout << sensor_ptr->getName() << std::endl;
            sf::ScalarSensor *scalar_sensor = dynamic_cast<sf::ScalarSensor *>(sensor_ptr);
            for (unsigned int i = 0; i < scalar_sensor->getNumOfChannels(); i++)
            {
                std::string channel_name = scalar_sensor->getSensorChannelDescription(i).name;
                float value = scalar_sensor->getLastSample().getValue(i);
                std::cout << "[SENSOR INFORMATION CHANNEL] " << channel_name << ": " << value << std::endl;    
            }
        }
    }
}
