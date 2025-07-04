#include "StonefishRL.h"
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>

// Includes per SENSORS
#include <Stonefish/sensors/Sensor.h>
#include <Stonefish/sensors/Sample.h>
#include <Stonefish/sensors/ScalarSensor.h> // Pel getLastSample()
#include <Stonefish/sensors/VisionSensor.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/scalar/RotaryEncoder.h>
#include <Stonefish/sensors/vision/Camera.h>

// Includes per ACTUADORS
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/actuators/Motor.h>
#include <Stonefish/actuators/Actuator.h>

#include <Stonefish/core/Robot.h>

#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/core/SimulationApp.h>
#include <Stonefish/core/ScenarioParser.h> // Per carregar l'arxiu XML
#include <Stonefish/StonefishCommon.h>

// Constructor de la classe StonefishRL
StonefishRL::StonefishRL(const std::string &path, double frequency)
    : sf::SimulationManager(frequency),
      scenePath(path),
      context(1),                            // Inicialitza el context de ZeroMQ amb 1 thread
      socket(context, zmq::socket_type::rep) // Crea 1 socket REP
{
    std::cout << "Constructor de StonefishRL" << std::endl;
    std::cout << "[INIT] StonefishRL created with scene: " << path << std::endl;

    InitializeZMQ();
}

// Destructor (INUTIL)
StonefishRL::~StonefishRL()
{
    std::cout << "[INFO] StonefishRL destructor called.\n";
    // En principi SimulationManager, s'encarrega de netejar els objectes que
    // s'han afegit (actuadors, sensors, ...).
    // Si s'hagues assignat alguna cosa dinamicament que NO es gestionada per
    // Stonefish ho hauríem d'eliminar aqui.
}

// (INUTIL) pq ho acabarem fent amb un setRobotPosition
void StonefishRL::Reset()
{

    std::cout << "Vaig a dormir una mica" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::cout << "pta vida ja m'han despertat" << std::endl;

    std::cout << "[StonefishRL] Resetting scenario...\n";
    sensors_.clear(); 
    actuators_.clear();

    std::cout << "Ojo q faig boom\n";
    this->RestartScenario();

    std::cout << "[StonefishRL] Scenario restarted OK\n";

    // RestartScenario ja conté:
    //      - DestroyScenario();
    //      - InitializeSolver();
    //      - InitializeScenario();
    //      - BuildScenario();
}

std::string StonefishRL::RecieveInstructions()
{

    zmq::message_t request;

    // Esperar a rebre el missatge
    auto result = socket.recv(request, zmq::recv_flags::none);

    std::string cmd_tag = "CMD:";
    std::string obs_tag = "OBS:";
/*
    if (msg.rfind(cmd_tag, 0) != 0 || msg.find(obs_tag) == std::string::npos) {
        std::cout << "[WARN] Invalid format: expected message to start with 'CMD:' and contain 'OBS:'\n";
        return;
    }
*/

    // Convertir el missatge (que esta en un buffer) a string i mostrar-lo
    std::string cmd = request.to_string();
    std::cout << "[ZMQ] He rebut: (" << result.value() << " bytes) en la comanda: " << cmd << std::endl;

    // Elimina el string del tipus d'instrucció que es vol fer
    int pos = cmd.find(":");              // Agafa fins al 1r ":" que troba.
    std::string prefix = cmd.substr(0, pos); // Es queda amb el que ha de fer la comanda
    cmd = cmd.substr(pos + 1);               // El +1 és per eliminar el ":"

    if (prefix == "RESET")
    {
        std::cout << "[ZMQ] He rebut RESET\n";
        socket.send(zmq::buffer("Preparat per rebre la posicio de RESET"), zmq::send_flags::none);

        // Rebre els valors de les posicions de reset que s'han enviat
        zmq::message_t reset_position;
        auto merda = socket.recv(reset_position, zmq::recv_flags::none);

        int n_floats = reset_position.size() / sizeof(float);
        
        if(n_floats >= 3){

            const float* data = static_cast<const float*>(reset_position.data());

            if(SetRobotPosition(cmd, data, n_floats))
            {
                socket.send(zmq::buffer("RESET OK"), zmq::send_flags::none);
            }
            else 
            {
                std::string message = "NOT ABLE TO FIND THE ROBOT BY THE GIVEN NAME: " + cmd.erase(cmd.find(";")); 
                socket.send(zmq::buffer(message), zmq::send_flags::none);
            }
            
            return "RESET";
        }
        else {
            std::cerr << "[C++] Not enough floats (at least 3), provided: " << n_floats << "\n";
            socket.send(zmq::buffer("RESET ERROR"), zmq::send_flags::none);
            return "";
        }

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
    MostrarValors();
    StateScene scalar_observations = GetStateScene();
    
    // Convertir les observacions a string
    std::string obs_str;
    for (const auto& obs : scalar_observations.observations)
    {
        obs_str += "\n  [INFO] Nom objecte: " + obs.name + " => Angle [rad]: ";
        obs_str += std::to_string(obs.angle);
        obs_str += " , Velocitat Angular [rad/s]: " + std::to_string(obs.angular_velocity); // Per separar els valors
    }

    socket.send(zmq::buffer(obs_str), zmq::send_flags::none);
}

void StonefishRL::ApplyCommands(const std::string& str_cmds)
{
    // Assigna els nous valors a l'estructura
    // que tenim als atributs de la classe StonefishRL.
    
    ParseCommandsAndObservations(str_cmds);

    unsigned int id = 0;
    sf::Actuator *actuator_ptr;

    std::cout << "\n---------------------------------------------------------------------------- \n";
    std::cout << "-                       FUNCTION APPLYCOMMANDS (ACTUATORS)                 - \n";

    while ((actuator_ptr = getActuator(id++)) != nullptr)
    {

        std::string actuator_name = actuator_ptr->getName();

        // Només continuem la iteració si hi ha commands per l'actuador que estem mirant
        if (commands_.count(actuator_name) > 0)
        {

            switch (actuator_ptr->getType())
            {
            case sf::ActuatorType::SERVO:
            {
                sf::Servo *servo = dynamic_cast<sf::Servo *>(actuator_ptr);
                if (!servo)
                {
                    std::cout << "[WARNING] Not converted " << actuator_name << " to sf::Servo.\n";
                    break;
                }

                // Ha trobat el SERVO, busca tots els parametres (VELOCITY, POSITION, ...) que té el Servo.
                for (const auto &[action, action_value] : commands_[actuator_name])
                {

                    if (action == "VELOCITY" || action == "TORQUE")
                    {
                        std::cout << "---------------------------------------------------------------------------- \n";
                        servo->setControlMode(sf::ServoControlMode::VELOCITY);
                        servo->setDesiredVelocity(action_value);
                        std::cout << "[Servo] Set VELOCITY = " << action_value << " for " << actuator_name << "\n";
                    }
                    else if (action == "POSITION") // El rang de valors va de pi a -pi
                    {
                        std::cout << "---------------------------------------------------------------------------- \n";
                        std::cout << "[Servo] Actual position of " << actuator_name << " = " << servo->getPosition() << "\n";
                        servo->setControlMode(sf::ServoControlMode::POSITION);
                        std::cout << "[Servo] New incoming POSITION = " << action_value << " for " << actuator_name << "\n";
                        // servo->setMaxVelocity(100.0); // Diria que no afecta en res. Pq si posem un valor més alt o més baix es comporta igual.
                        // servo->setDesiredVelocity(100.0); // Diria que no afecte en res. Pq si posem un valor més alt o més baix es comporta igual.
                        servo->setDesiredPosition(action_value); // Aquest es el que realment ens marca a quina posicio volem deixar l'acrobot.
                        std::cout << "[Servo] Reached POSITION = " << servo->getPosition() << " for " << actuator_name << "\n";
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

    // POTSER NO CAL FER-HO.
    sensors_.clear();
    actuators_.clear();
    

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

    std::cout << "[INFO] Scenario loaded succesfully.\n";
}

StonefishRL::StateScene StonefishRL::GetStateScene()
{
    //std::map<std::string, std::vector<float>> sensorData;
    sf::Sensor *sensor_ptr;
    sf::Robot *robot_ptr;
    sf::Actuator *actuator_ptr;
    StateScene state;
    
    std::cout << "\n---------------------------------------------------------------------------- \n";
    std::cout << "-                       FUNCTION GETSTATE                         - \n";
    std::cout << "---------------------------------------------------------------------------- \n";

    unsigned int id = 0;
    while ((robot_ptr = getRobot(id++)) != nullptr)
    {
        std::string robot_name = robot_ptr->getName();
        
        if(ObjImportantForObs(robot_name)){
            //sf::Transform position = robot_ptr->getTransform();
            //sf::Vector3 origin = position.getOrigin(); 
            sf::Vector3 origin = robot_ptr->getTransform().getOrigin();
            
            Observation obs;
            obs.name = robot_name; 
            obs.position.x = origin.getX();
            obs.position.y = origin.getY();
            obs.position.z = origin.getZ();
  
            state.observations.push_back(obs);
        } 
    }

    id = 0;
    while ((sensor_ptr = getSensor(id++)) != nullptr)
    {
        std::string sensor_name = sensor_ptr->getName();
        
        if (ObjImportantForObs(sensor_name))
        {
            if (sensor_ptr->getType() == sf::SensorType::JOINT)
            {
                sf::ScalarSensor *scalar_sensor = dynamic_cast<sf::ScalarSensor *>(sensor_ptr);
                if (!scalar_sensor) continue;

                Observation obs;
                obs.name = sensor_name;
                for (unsigned int i = 0; i < scalar_sensor->getNumOfChannels(); i++){
                    
                    float value = scalar_sensor->getLastSample().getValue(i);
                    std::string channel_name = scalar_sensor->getSensorChannelDescription(i).name;

                    if (channel_name == "Angle") obs.angle = value;
                    else if (channel_name == "Angular velocity") obs.angular_velocity = value;
                    else std::cout << "[WARN] Channel named: " << channel_name << " is not controlled.\n";
                }
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
            if (actuator_ptr->getType() == sf::ActuatorType::SERVO)
            {   
                sf::Servo *servo_ptr = dynamic_cast<sf::Servo *>(actuator_ptr);
                if (!servo_ptr) continue;
                
                Observation obs;
                obs.name = actuator_ptr->getName();

                obs.angle = servo_ptr->getPosition();
                obs.angular_velocity = servo_ptr->getVelocity();
            
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
        socket.bind("tcp://*:5555"); // Escolta al port "5555"
        std::cout << "[ZMQ] Servidor REP actiu al port 5555\n";
    }
    catch (const zmq::error_t &e)
    {
        std::cerr << "[ZMQ ERROR] " << e.what() << "\n";
    }
}
/*
StonefishRL::CommandData StonefishRL::ConvertStringToMap(const std::string str)
{

    StonefishRL::CommandData result;

    std::string input = str;
    input.erase(std::remove(input.begin(), input.end(), '\n'), input.end());

    std::istringstream iss(str);
    std::string token;

    while (std::getline(iss, token, ';'))
    {

        if (token.empty())
            continue;

        size_t pos = token.find(":"); // Busca la posicio del ":"

        if (pos == std::string::npos)
        {
            std::cerr << "[ERROR] Invalid command format: " << token << ". Expected format: 'actuator_name:value'." << std::endl;
            continue;
        }

        // Separar nom i valor
        std::string name = token.substr(0, pos);       // Agafa el que hi ha abans dels ":"
        std::string value_str = token.substr(pos + 1); // Agafa el que hi ha dps dels ":"

        float value = std::stof(value_str); // Convertir a float

        if (!(actuators_.count(name)))
        {
            std::cerr << "[ERROR] Actuator '" << name << "' not found in the scenario." << std::endl;
            std::cout << "DINS FUNCIÓ: Estic guardant al mapa amb nom: " << name << " el valor de: " << value << std::endl;

            continue; // Si l'actuador no existeix, salta al seguent
        }

        result.commands[name] = static_cast<float>(value);
        std::cout << "Estic guardant al mapa amb nom: " << name << " el valor de: " << value << std::endl;
    }

    return result;
}
*/

void StonefishRL::ParseCommandsAndObservations(const std::string& str)
{
    // Cal?
    // Netejar els antics valors, per si n'hi hagues de nous.
    commands_.clear();
    relevant_obs_names_.clear();

    // Exemple Comanda: "CMD:Robot1/Servo:POSITION:3.5;Robot2/Servo:VELOCITY:2.0;OBS:Robot1,Robot2"
    //                  "CMD:Robot1/Servo:POSITION:3.5;OBS:" --> Mostra les observacions de tots els objectes de l'escena
    std::cout << str << std::endl;
    std::size_t cmd_pos = str.find("CMD:");
    std::size_t obs_pos = str.find("OBS:");
    
    if (cmd_pos == std::string::npos || obs_pos == std::string::npos) {
        std::cerr << "[WARN] Missing 'CMD:' or 'OBS:' in the command string.\n";
        return;
    }
    
    std::string cmd_str = str.substr(cmd_pos + 4, obs_pos - (cmd_pos + 4));
    std::string obs_str = str.substr(obs_pos + 4); // Pot ser que estigui buit

    // --- Processar les comandes (CMD:) ---
    std::stringstream ss(cmd_str);
    std::string linea;
    
    while (std::getline(ss, linea, ';'))
    {
        if (linea.empty()) continue;

        std::string token;
        std::stringstream lineStream(linea);

        // 1r token = nom de l'actuador
        if (!std::getline(lineStream, token, ':')) continue;
        
        std::string actuator_name = token;
        std::cout << "  WOPWOP  " << std::endl;
        // Llegir clau(action) : valor(action_value)
        while (std::getline(lineStream, token, ':'))
        {
            std::string action = token;

            if (!std::getline(lineStream, token, ':')) break;

            try
            {
                float action_value = std::stof(token);
                 std::cout << "  --------  " << std::endl;
                std::cout << "[INFO] Actuator name: " << actuator_name << std::endl;
                std::cout << "[INFO] Action: " << action << std::endl;
                std::cout << "[INFO] Value: " << action_value << std::endl;
                std::cout << "  --------  " << std::endl;
                commands_[actuator_name][action] = action_value;
            }
            catch (...)
            { // Agafa qualsevol excepció sense importar de quin tipus es
                std::cerr << "[ERROR] Invalid value for " << actuator_name << ":" << action << " --> '" << token << "'\n";
            }
        }
    }

    // --- Processar les observacions (OBS:) ---
    // Si obs_str és buit, mostrarà totes les observacions
    if (!obs_str.empty()) {
        std::stringstream obs_ss(obs_str);
        std::string obj_name;

        while (std::getline(obs_ss, obj_name, ',')) {
            if (!obj_name.empty()) {
                relevant_obs_names_.insert(obj_name);
            }
        }
    }
}

bool StonefishRL::SetRobotPosition(std::string robot_name, const float* position_data, int n_param)
{
    sf::Robot *robot_ptr;
    bool robot_found = false;
    unsigned int id = 0;

    robot_name.erase(robot_name.find(";")); // Borra el ';'

    while ((robot_ptr = getRobot(id++)) != nullptr && robot_ptr->getName() == robot_name)
    {
        robot_found = true;
        sf::Transform tf;
        sf::Vector3 new_position(position_data[0], position_data[1], position_data[2]);
        tf.setOrigin(new_position); // Mou el robot a la posició indicada
        
        // Per si també ens passen la rotació que li volen donar
        if(n_param >= 6){
            sf::Quaternion rotation(position_data[3], position_data[4], position_data[5]);
            tf.setRotation(rotation);
        }

        robot_ptr->Respawn(this, tf);
    }
    return robot_found;
}

void StonefishRL::ExitRequest() {
    socket.close();
    context.close();

    std::cout << "[INFO] Simulation finished." << std::endl;
    std::exit(0);
}

bool StonefishRL::ObjImportantForObs(const std::string& objName) const {
    return relevant_obs_names_.empty() || relevant_obs_names_.count(objName) > 0;
}

void StonefishRL::MostrarValors() {
    std::cout << "[INFO] MOSTRAR ELS VALORS RECOLLITS:\n";
    
    for( int i = 0; i < current_state_.observations.size(); i++){
        
        std::cout << "  --------  " << std::endl;
       
        std::cout << "[INFO][" << current_state_.observations[i].name << "] "
                  << "Position: X[" << current_state_.observations[i].position.x <<"],"
                  << " Y[" << current_state_.observations[i].position.y <<"],"
                  << " Z[" << current_state_.observations[i].position.z <<"]" << std::endl;
        
        std::cout << "[INFO] Angle: " << current_state_.observations[i].angle << "." << std::endl;

        std::cout << "[INFO] Angular velocity: " << current_state_.observations[i].angular_velocity << "." << std::endl;

        std::cout << "[INFO] Linear Velocity: " << current_state_.observations[i].linear_velocity << "." << std::endl;

        std::cout << "  --------  " << std::endl;
    }

    std::cout << std::endl;

    std::cout << "[INFO] MOSTRAR ELS COMMANDS APLICATS:\n";
    for(const auto& [actuator_name, actions] : commands_){
        for (const auto& [action_name, value] : actions) {
            std::cout << "  --------  " << std::endl;
            std::cout << "[INFO] Actuator name: " << actuator_name << std::endl;
            std::cout << "[INFO] Action: " << action_name << std::endl;
            std::cout << "[INFO] Value: " << value << std::endl;
            std::cout << "  --------  " << std::endl;
        }
    }

    std::cout << std::endl;

    std::cout << "\n--- OBJECTES IN NNED TO BE OBSERVED ---" << std::endl;
    for (const auto& name : relevant_obs_names_) {
        std::cout << "Object: " << name << std::endl;
    }
}



