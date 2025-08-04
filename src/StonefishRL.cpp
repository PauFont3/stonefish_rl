#include "StonefishRL.h"
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <sstream> // Per poder construir el JSON de les observacions
#include <map>
#include <unordered_map>
#include <unordered_set>

#include <zmq.hpp> 

// Includes per SENSORS
#include <Stonefish/sensors/Sensor.h>
#include <Stonefish/sensors/Sample.h>
#include <Stonefish/sensors/ScalarSensor.h> // Pel getLastSample()
#include <Stonefish/sensors/VisionSensor.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/scalar/RotaryEncoder.h>
#include <Stonefish/sensors/vision/Camera.h>
#include <Stonefish/sensors/scalar/Pose.h>
#include "sensors/scalar/LinkSensor.h"
#include "sensors/scalar/JointSensor.h"

// Includes per ACTUADORS
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
#include <Stonefish/core/ScenarioParser.h> // Per carregar l'arxiu XML
#include <Stonefish/StonefishCommon.h>

#include <SDL2/SDL.h>

// Constructor de la classe StonefishRL
StonefishRL::StonefishRL(const std::string &path, double frequency)
    : sf::SimulationManager(frequency),
      scenePath(path),
      context(1),                            // Inicialitza el context de ZeroMQ amb 1 thread
      socket(context, zmq::socket_type::rep) // Crea 1 socket REP
{
    InitializeZMQ();
}


std::string StonefishRL::RecieveInstructions(sf::SimulationApp& simApp)
{
    zmq::message_t request;

    // Esperar a rebre el missatge --> Rebrà: "RESET:nom_robot;"
    auto result = socket.recv(request, zmq::recv_flags::none);

    // Convertir el missatge (que esta en un buffer) a string i mostrar-lo
    std::string cmd = request.to_string();
    //std::cout << "[ZMQ] He rebut: (" << result.value() << " bytes) en la comanda: " << cmd << std::endl;

    // Elimina el string del tipus d'instrucció que es vol fer
    int pos = cmd.find(":");              // Agafa fins al 1r ":" que troba.
    std::string prefix = cmd.substr(0, pos); // Es queda amb el que ha de fer la comanda
    cmd = cmd.substr(pos + 1);               // El +1 és per eliminar el ":"

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
        std::cout << "[ZMQ] He rebut EXIT\n";
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
    //ProvaMostrarTot();
    // Convertir les observacions a string
    std::string obs_str_json = SerializeScene(scalar_observations.observations);
    
    this->socket.send(zmq::buffer(obs_str_json), zmq::send_flags::none);
}


void StonefishRL::ApplyCommands(const std::string& str_cmds)
{
    // Assigna els nous valors a l'estructura
    // que tenim als atributs de la classe StonefishRL.
    
    ParseCommandsAndObservations(str_cmds);

    unsigned int id = 0;
    sf::Actuator *actuator_ptr;

    while ((actuator_ptr = getActuator(id++)) != nullptr)
    {
        std::string actuator_name = actuator_ptr->getName();

        // Només continuem la iteració si hi ha commands per l'actuador que estem mirant
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

                // Aplica totes les accions que hi ha a la comanda pel SERVO
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

                // Aplica totes les accions que hi ha a la comanda pel THRUSTER
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

    while ((sensor_ptr = getSensor(sensor_id++)) != nullptr)
    {
        sensors_[sensor_ptr->getName()] = sensor_ptr;
    }

    if (sensors_.empty()) std::cout << "[WARN] No sensors registered in this scenario!" << std::endl;

    sf::Actuator *actuator_ptr;
    unsigned int actuator_id = 0;
    while ((actuator_ptr = getActuator(actuator_id++)) != nullptr)
    {
        actuators_[actuator_ptr->getName()] = actuator_ptr;
    }

    if (actuators_.empty()) std::cout << "[WARN] No actuators registered in this scenario!" << std::endl;

    sf::Robot *robot_ptr;
    unsigned int robot_id = 0;
    while ((robot_ptr = getRobot(robot_id++)) != nullptr)
    {
        robots_[robot_ptr->getName()] = robot_ptr;
    }

    if (robots_.empty()) std::cout << "[WARN] No robots registered in this scenario!" << std::endl;

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

        if (ObjImportantForObs(sensor_name)) // POTSER S'HAURIA DE TREURE?
        {
            sf::ScalarSensor *sensor = dynamic_cast<sf::ScalarSensor *>(sensor_ptr); // POTSER NO CAL REPETIRHO SEMPRE
            
            // ENCODING SENSOR
            if(sensor->getScalarSensorType() == sf::ScalarSensorType::ENCODER)
            {
                if (!sensor) continue;
                
                InfoObject obs; // El poso aqui pq cada cop que es troba un objecte nou, no pugui tenir cap possible dada de l'anterior
                FillWithNanInfoObject(obs);
                obs.name = sensor_name;
                obs.angle = sensor->getLastSample().getValue(0);
                obs.angular_velocity[2] = sensor->getLastSample().getValue(1); // Posem l'unic valor que dona a l'eix Z i després al python ho tractarem

                state.observations.push_back(obs);
            }
        
            // ODOMETRY SENSOR
            if(sensor->getScalarSensorType() == sf::ScalarSensorType::ODOM)
            {
                if(!sensor) continue;

                InfoObject obs;
                FillWithNanInfoObject(obs);
                obs.name = sensor_name;
                obs.position[0] = sensor->getLastSample().getValue(0);
                obs.position[1] = sensor->getLastSample().getValue(1);
                obs.position[2] = sensor->getLastSample().getValue(2);

                obs.linear_velocity[0] = sensor->getLastSample().getValue(3);
                obs.linear_velocity[1] = sensor->getLastSample().getValue(4);
                obs.linear_velocity[2] = sensor->getLastSample().getValue(5);
                
                obs.rotation[0] = sensor->getLastSample().getValue(6);
                obs.rotation[1] = sensor->getLastSample().getValue(7); 
                obs.rotation[2] = sensor->getLastSample().getValue(8);
                float rot_w = sensor->getLastSample().getValue(9);

                obs.angular_velocity[0] = sensor->getLastSample().getValue(10);
                obs.angular_velocity[1] = sensor->getLastSample().getValue(11);
                obs.angular_velocity[2] = sensor->getLastSample().getValue(12);
                
                state.observations.push_back(obs);
            }

            // GPS SENSOR
            else if(sensor->getScalarSensorType() == sf::ScalarSensorType::GPS)
            {
                if(!sensor) continue;
            
            }
            
            // PRESSURE SENSOR
            else if(sensor->getScalarSensorType() == sf::ScalarSensorType::PRESSURE)
            {
                if(!sensor) continue;

                InfoObject obs;
                FillWithNanInfoObject(obs);
            
                obs.name = sensor_name;
                obs.pressure = sensor->getLastSample().getValue(0);

                state.observations.push_back(obs);
            }

            // FORCE_TORQUE SENSOR
            else if(sensor->getScalarSensorType() == sf::ScalarSensorType::FT)
            {
                if(!sensor) continue;

                InfoObject obs;
                FillWithNanInfoObject(obs);
            
                obs.name = sensor_name;
                obs.force[0] = sensor->getLastSample().getValue(0);
                obs.force[1] = sensor->getLastSample().getValue(1);
                obs.force[2] = sensor->getLastSample().getValue(2);
                obs.torque[0] = sensor->getLastSample().getValue(3);
                obs.torque[1] = sensor->getLastSample().getValue(4);
                obs.torque[2] = sensor->getLastSample().getValue(5);

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
            // SERVO ACTUATOR
            if (actuator_ptr->getType() == sf::ActuatorType::SERVO)
            {   
                sf::Servo *servo_ptr = dynamic_cast<sf::Servo *>(actuator_ptr);
                if (!servo_ptr) continue;
                
                InfoObject obs;
                FillWithNanInfoObject(obs);
                obs.name = actuator_ptr->getName();

                obs.angle = servo_ptr->getPosition();
                obs.angular_velocity[2] = servo_ptr->getVelocity();
            
                state.observations.push_back(obs);
            }
        /* El deixo cometat pq ara no m'interessen els valors dels thrusters
            // THRUSTER ACTUATOR
            else if (actuator_ptr->getType() == sf::ActuatorType::THRUSTER)
            {
                sf::Thruster *thruster_ptr = dynamic_cast<sf::Thruster *>(actuator_ptr);
                if (!thruster_ptr) continue;

                InfoObject obs;
                FillWithNanInfoObject(obs);
                obs.name = actuator_ptr->getName();

                obs.angle = thruster_ptr->getSetpoint();

                obs.angular_velocity[2] = thruster_ptr->getThrust();

                state.observations.push_back(obs);
            }
        */        
        }
    }
    current_state_ = state;
    return state;
}


void StonefishRL::InitializeZMQ()
{
    try
    {
        socket.bind("tcp://*:5555"); // Escolta al port "5555"
        std::cout << "[ZMQ] Servidor REP actiu al port 5555\n";
    }
    catch (const zmq::error_t &e)
    {
        std::cerr << "[ZMQ ERROR] " << e.what() << "\n";
    }
}


std::vector<StonefishRL::InfoObject> StonefishRL::ParseResetCommand(const std::string& str_command){
    
    std::vector<InfoObject> result;

    int pos = 0;

    // Mentres hi hagi '{..}'
    while((pos = str_command.find("{", pos)) != std::string::npos) {
        
        // Busca la marca de fi '}'
        int end = str_command.find("}", pos); 

        if(end == std::string::npos) break;
        
        // Agafa i guarda en un string el contingut que hi ha dins de '{..}'
        std::string object_str = str_command.substr(pos, end - pos + 1);
    
        InfoObject obj;
        // Posar aquest valors per defecte, pq no ens serveixen per res. 
        // Si es crea un altre struct pels valors del reset es podria eliminar, 
        // però he preferit reutilizar el struct que ja tenia pq té totes les dades que interessen.  
        obj.angle = 0;

        // ---- NAME ----
        // Buscar i agafar el 'name'
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

            // Divideix la llista separant per comes i convierteix cada valor a float
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
    // Cal?
    // Netejar els antics valors, per si n'hi hagues de nous.
    commands_.clear();
    relevant_obs_names_.clear();

    int obs_pos = str.find("OBS:");
    
    if (obs_pos == std::string::npos) {
        std::cerr << "[WARN] Missing 'OBS:' in the command string.\n";
        return;
    }

    std::string cmd_str = str.substr(0, obs_pos); // Agafa tot el que hi ha abans de "OBS:"
    std::string obs_str = str.substr(obs_pos + 4); // Pot ser que estigui buit

    // --- Processar les comandes (CMD:) ---
    std::stringstream ss(cmd_str);
    std::string token;
    
    // Dividir per ";"
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
            { // Agafa qualsevol excepció sense importar de quin tipus es
                std::cerr << "[ERROR] Invalid value for " << actuator_name << ":" << action << " --> '" << token << "'\n";
            }
        }
        else {
            std::cerr << "[ERROR] Invalid command format: '" << token << "'. Expected format: 'actuator_name:action:value;'.\n";
        }
    }

    // --- Processar les observacions (OBS:) ---
    // Si obs_str és buit, mostrarà totes les observacions
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
            // Aquest if només esta fent el reset de la posició al robot que es passa pel command del RESET
            if(robot_ptr->getName() == robot_name)
            {     
                robot_found = true;
            
                sf::Transform tf;
                sf::Vector3 new_position(obj[i].position[0], obj[i].position[1], obj[i].position[2]);
                tf.setOrigin(new_position); // Mou el robot a la posició indicada
                
                sf::Quaternion rotation(obj[i].rotation[0], obj[i].rotation[1], obj[i].rotation[2]);
                tf.setRotation(rotation); // Rota el robot 
                
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


// Agafa el nom dels commands dels actuadors que s'han enviat desdel Python, 
// perque després al fer el recorregut dels actuadors al while(el que itera per tots els actuadors)
// apliqui 'acció nomes als que s'han demanat al command
bool StonefishRL::ObjImportantForObs(const std::string& objName) const {
    return relevant_obs_names_.empty() || relevant_obs_names_.count(objName) > 0;
}

/*
void StonefishRL::MostrarValors() {
    std::cout << "[INFO] MOSTRAR ELS VALORS RECOLLITS:\n";
    
    for( int i = 0; i < current_state_.observations.size(); i++)
    {    
        std::cout << "  --------  " << std::endl;
        std::cout << "[INFO][" << current_state_.observations[i].name << "] \n";
        
        if(current_state_.observations[i].position.size() >= 3) 
        {
            std::cout << "[INFO] Position: X[" << current_state_.observations[i].position[0] << "],"
                      << " Y[" << current_state_.observations[i].position[1] << "],"
                      << " Z[" << current_state_.observations[i].position[2] << "]" << std::endl;
        }       

        std::cout << "[INFO] Angular Velocity: X[" << current_state_.observations[i].angular_velocity[0] << "],"
                  << " Y[" << current_state_.observations[i].angular_velocity[1]  << "],"
                  << " Z[" << current_state_.observations[i].angular_velocity[2]  << "]" << std::endl;

        std::cout << "[INFO] Linear Velocity: X[" << current_state_.observations[i].linear_velocity[0] << "],"
                  << " Y[" << current_state_.observations[i].linear_velocity[1] << "],"
                  << " Z[" << current_state_.observations[i].linear_velocity[2] << "]" << std::endl;
       
                  std::cout << "[INFO] Angle: " << current_state_.observations[i].angle << "." << std::endl;

        std::cout << "  --------  " << std::endl;
    }

    std::cout << std::endl;

    std::cout << "[INFO] MOSTRAR ELS COMMANDS APLICATS:\n";
    for(const auto& [actuator_name, actions] : commands_)
    {
        for (const auto& [action_name, value] : actions) 
        {
            std::cout << "  --------  " << std::endl;
            std::cout << "[INFO] Actuator name: " << actuator_name << std::endl;
            std::cout << "[INFO] Action: " << action_name << std::endl;
            std::cout << "[INFO] Value: " << value << std::endl;
            std::cout << "  --------  " << std::endl;
        }
    }

    std::cout << std::endl;

    for (const auto& name : relevant_obs_names_) 
    {
        if(sensors_.count(name) > 0 || actuators_.count(name) > 0) std::cout << "[INFO] " << name << std::endl;
        else if(actuators_.count(name) > 0) std::cout << "[INFO] " << name << " is an actuator in the scenario." << std::endl;
        else if(robots_.count(name) > 0) std::cout << "[INFO] " << name << " is a robot in the scenario." << std::endl;
        else std::cout << "[WARN] " << name << " is not a sensor, actuator or robot in the scenario." << std::endl;
    }
}
*/


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
        << "\"pressure\": " << SafeFloat(obj.pressure) << ", "
        << "\"force\": ["
        << SafeFloat(obj.force[0]) << ", "
        << SafeFloat(obj.force[1]) << ", "
        << SafeFloat(obj.force[2]) << "], "
        << "\"torque\": ["
        << SafeFloat(obj.torque[0]) << ", "
        << SafeFloat(obj.torque[1]) << ", "
        << SafeFloat(obj.torque[2]) << "] "
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
    obj.angular_velocity.resize(3, NAN); 
    
    obj.force.resize(3, NAN);
    obj.torque.resize(3, NAN);
}


// Pq el python ho pugui interpretar com un valor buit
std::string StonefishRL::SafeFloat(float val) 
{
    if (std::isnan(val)) return "null";
    else return std::to_string(val);
}


// No serveix per res, només mostra valors per pantalla
void StonefishRL::ProvaMostrarTot()
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

