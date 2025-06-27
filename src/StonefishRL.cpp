#include "StonefishRL.h"
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <map>

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
StonefishRL::StonefishRL(const std::string& path, double frequency)
    : sf::SimulationManager(frequency), 
    scenePath(path),
    context(1), // Inicialitza el context de ZeroMQ amb 1 thread
    socket(context, zmq::socket_type::rep) // Crea 1 socket REP
{ 
    std::cout << "Constructor de StonefishRL" << std::endl;
    std::cout << "[INIT] StonefishRL created with scene: " << path << std::endl;
    
    InitializeZMQ(); 
}   


// Destructor
StonefishRL::~StonefishRL() {
    std::cout << "[INFO] StonefishRL destructor called.\n";
    // En principi SimulationManager, s'encarrega de netejar els objectes que 
    // s'han afegit (actuadors, sensors, ...).
    // Si s'hagues assignat alguna cosa dinamicament que NO es gestionada per
    // Stonefish ho hauríem d'eliminar aqui.
}


void StonefishRL::Reset() {

    std::cout << "Vaig a dormir una mica" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::cout << "pta vida ja m'han despertat" << std::endl;

    std::cout << "[StonefishRL] Resetting scenario...\n";
   
    obs_sensors_.clear();
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


void StonefishRL::RecieveInstructions() {

    zmq::message_t request;

    // Esperar a rebre el missatge
    auto result = socket.recv(request, zmq::recv_flags::none);

    if(!result.has_value()) {
        std::cerr << "[StonefishRL] No message received." << std::endl;
        return;
    }

    // Convertir el missatge (que esta en un buffer) a string i mostrar-lo
    std::string cmd = request.to_string();
    std::cout << "[ZMQ] He rebut: (" << result.value() << " bytes) en la comanda: " << cmd << std::endl;

    if(cmd == "RESET") {
        std::cout << "[ZMQ] He rebut RESET\n";
        socket.send(zmq::buffer("RESET OK"), zmq::send_flags::none);
    }
    else if (cmd == "EXIT"){
        std::cout << "[ZMQ] He rebut EXIT\n";
        socket.send(zmq::buffer("EXIT OK"), zmq::send_flags::none);
    }
    else {
        ApplyCommands(cmd); 
    }
}


void StonefishRL::SendObservations(){

    std::map<std::string, std::vector<float>> scalar_observations = getScalarObservations();

    // Convertir les observacions a string
    std::string obs_str;
    for (const auto& pair : scalar_observations) {
        obs_str += "\n  [INFO] Nom sensor: " + pair.first + " => Angle [rad]: ";
        for (size_t i = 0; i < pair.second.size(); i++)
        {

            obs_str += std::to_string(pair.second[i]);

            if (i < pair.second.size() - 1) {
                obs_str += " , Velocitat Angular [rad/s]: "; // Per separar els valors 
            }
        }
    }       

    socket.send(zmq::buffer(obs_str), zmq::send_flags::none);
                
}


void StonefishRL::ApplyCommands(const std::string &str_cmds) {

    /*CommandData cmds = ConvertStringToMap(str_cmds);
 
    unsigned int id = 0;
    sf::Actuator* actuator_ptr;

    while((actuator_ptr = getActuator(id++)) != nullptr) {
        
        
        // Si han escrit el nom malament, que no el detecti. Perque ara mateix els esta agafant tots igualment. 
        // T'indica que has posat el nom malament pero t'agafa tots els sensors igualment, perque ho busca per 
        // la llista de ids.
        std::string actuator_name = actuator_ptr->getName();

        if()

        if(cmds.commands.find(actuator_name) != cmds.commands.end()){

            switch (actuator_ptr->getType()) {

                case sf::ActuatorType::SERVO:
                {
                    // Mira si al mapa 
                    if(commands_.count(actuator_name) > 0 && sf::Servo* servo = dynamic_cast<sf::Servo*>(actuator_ptr);){

                        for

                        if(commands_[actuator_name][])
                        servo->setControlMode(sf::ServoControlMode::VELOCITY);
                        std::cout << "[INFO] Servo actuator found: " << servo->getName() << std::endl;
                        std::cout << "[ApplyCommands] Target for " << servo->getName() << " = " << cmds.commands[servo->getName()] << std::endl;
                        servo->setDesiredVelocity(cmds.commands[servo->getName()]);

                        std::cout << "La posicio (l'angle) a la que es troba: " << servo->getName() << " abans d'aplicar el setDesiredPosition(...) es: " << servo->getPosition() << std::endl;
                    
                        servo->setControlMode(sf::ServoControlMode::POSITION);
                        servo->setDesiredPosition(); // Va de pi a -pi el rang de possibles valors
                    
                        std::cout << "La nova posicio (l'angle) a la que es troba: " << servo->getName() << " després d'aplicar el setDesiredPosition(...) es: " << servo->getPosition() << std::endl;

                    }
                    break;
                }
                            
                default:
                    std::cout << "[WARN] Actuator type not supported.\n";
                    break;
            
            }
        }
    }*/

    // Assigna els nous valors a l'estructura 
    // que tenim als atributs de la classe StonefishRL.
    ConvertStringToUnorderedMap(str_cmds);

    unsigned int id = 0;
    sf::Actuator* actuator_ptr;

    while ((actuator_ptr = getActuator(id++)) != nullptr) {
    
        std::string actuator_name = actuator_ptr->getName();

        // Només continuem la iteració si hi ha commands per l'actuador que estem mirant 
        if (commands_.count(actuator_name) > 0){
        
            switch (actuator_ptr->getType())
            {
                case sf::ActuatorType::SERVO: 
                {
                    sf::Servo* servo = dynamic_cast<sf::Servo*>(actuator_ptr);
                    if (!servo) {
                        std::cout << "[WARNING] Not converted " << actuator_name << " to sf::Servo.\n";
                        break;
                    }

                    // Ha trobat el SERVO, busca tots els parametres (VELOCITY, POSITION, ...) que té el Servo.
                    for(const auto& [action, action_value] : commands_[actuator_name])
                    {
                        std::cout << "[INFO] Nom de l'actuador al qual intentem aplicar la velocitat " << actuator_name << std::endl;
                        if(action == "VELOCITY") 
                        {
                            servo->setControlMode(sf::ServoControlMode::VELOCITY);
                            servo->setDesiredVelocity(action_value);
                            std::cout << "[Servo] Set VELOCITY = " << action_value << " from " << actuator_name << "\n";
                        }
                        else if (action == "POSITION") // El rang de valors va de pi a -pi
                        {  
                            std::cout << "[Servo] Actual position of " << actuator_name << " = " << servo->getPosition() << "\n";
                            servo->setControlMode(sf::ServoControlMode::POSITION);
                            std::cout << "[Servo] New incoming POSITION = " << action_value << " for " << actuator_name << "\n";
                            //servo->setMaxVelocity(100.0); // Diria que no afecta en res. Pq si posem un valor més alt o més baix es comporta igual.
                            //servo->setDesiredVelocity(100.0); // Diria que no afecte en res. Pq si posem un valor més alt o més baix es comporta igual.
                            servo->setDesiredPosition(action_value); // Aquest es el que realment ens marca a quina posicio volem deixar l'acrobot.
                            std::cout << "[Servo] Reached POSITION = " << servo->getPosition() << " for " << actuator_name << "\n";
                        }
                        else if(action == "TORQUE") 
                        {
                            servo->setControlMode(sf::ServoControlMode::TORQUE);
                            servo->setMaxTorque(action_value);
                            std::cout << "[Servo] Set MAX TORQUE = " << action_value << " from " << actuator_name << "\n";
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


void StonefishRL::BuildScenario() {

    std::cout << "[INFO] Building scenario from: " << scenePath << std::endl;
    sf::ScenarioParser parser(this);
    
    if(!parser.Parse(scenePath)){
        std::cerr << "[ERROR] Error charging the scenario: " << scenePath << "\n";
        for (const auto& msg : parser.getLog()) {
            std::cerr << "[ScenarioParser Log] " << msg.text << "\n";
        }
        return;
    }

    std::cout << "Ha entrat al BuildScenario()" << std::endl; 


    // POTSER NO CAL FER-HO.
    obs_sensors_.clear();
    actuators_.clear();


    sf::Sensor* sensor_ptr;
    unsigned int sensor_id = 0;

    while((sensor_ptr = getSensor(sensor_id++)) != nullptr) 
    {
        obs_sensors_[sensor_ptr->getName()] = sensor_ptr;
    }  
    
    if(obs_sensors_.empty()) std::cout << "[WARN] No sensors registered in this scenario!" << std::endl;
    else {
        std::cout << "Contingut del mapa de sensors:" << std::endl;
        for (const auto& pair : obs_sensors_) {
            std::cout << "  Clau: '" << pair.first 
                      << "\n        Valor UpdatedFrequency: " << pair.second->getUpdateFrequency() 
                      << "\n        Valor Nom Sensor: " << pair.second->getName() << std::endl;
        }
    }

    
    sf::Actuator* actuator_ptr;
    unsigned int actuator_id = 0;
    while((actuator_ptr = getActuator(actuator_id++)) != nullptr)
    {
        actuators_[actuator_ptr->getName()] = actuator_ptr;
    } 

    if (actuators_.empty()) std::cout << "[WARN] No actuators registered in this scenario!" << std::endl;
    else {
        std::cout << "  Contingut del mapa d'actuadors:" << std::endl;
        for (const auto& pair : actuators_) {
            std::cout << "      Clau: '" << pair.first << "', punter: " << pair.second << std::endl;
        }
    }

    std::cout << "[INFO] Scenario loaded succesfully.\n";
}


std::map<std::string, std::vector<float>> StonefishRL::getScalarObservations() {

    std::map<std::string, std::vector<float>> sensorData;
    sf::Sensor* sensor_ptr;
    sf::Robot* robot_ptr;
    sf::Actuator* actuator_ptr;


    unsigned int id = 0;
    std::cout << "------------ ROBOTS ------------" << std::endl;
    while((robot_ptr = getRobot(id++)) != nullptr){
        
        sf::Transform position = robot_ptr->getTransform();
        sf::Vector3 origin = position.getOrigin();
        float eix_x = origin.getX();
        float eix_y = origin.getY();
        float eix_z = origin.getZ();

        std::cout << "[COORDENADES] Posició del robot " << robot_ptr->getName() << " en el frame 'World' del robot : EIX X: " << eix_x << " EIX Y: " << eix_y << " EIX Z: " << eix_z << std::endl;
    }


    id = 0;
    std::cout << "------------ SENSORS ------------" << std::endl;
    while((sensor_ptr = getSensor(id++)) != nullptr) 
    {
        if(!sensor_ptr->isNewDataAvailable()) {
            std::cout << "[WARN] No new data available for sensor: " << sensor_ptr->getName() << std::endl;
            continue;
        }

        if(sensor_ptr->getType() == sf::SensorType::JOINT) {

            sf::ScalarSensor* scalar_sensor = dynamic_cast<sf::ScalarSensor*>(sensor_ptr);
            
            if(!scalar_sensor) continue;

            sf::Sample lastSample = scalar_sensor->getLastSample();
            
            std::vector<float> sensor_values;
            sensor_values.reserve(scalar_sensor->getNumOfChannels());

            for(unsigned int i = 0; i < scalar_sensor->getNumOfChannels(); i++) {
                sensor_values.push_back(static_cast<float>(lastSample.getValue(i)));
                std::cout << "[CHANNEL] Nom del canal "<< i << ": " << scalar_sensor->getSensorChannelDescription(i).name << std::endl;
            }
            sensorData[sensor_ptr->getName()] = std::move(sensor_values);
        }

        sensor_ptr->MarkDataOld(); // Marcar dades com antigues per evitar duplicats, no se si realment està fent
    }
    
    id = 0;
    std::cout << "------------ ACTUATORS ------------" << std::endl;
    while((actuator_ptr = getActuator(id++)) != nullptr) {
        if(actuator_ptr->getType() == sf::ActuatorType::SERVO){
            sf::Servo* servo_ptr = dynamic_cast<sf::Servo*>(actuator_ptr);
            std::cout << "[INFO] ACTUATOR: " << actuator_ptr->getName() << " is in POSITION: " << servo_ptr->getPosition() << std::endl;
            std::cout << "[INFO] ACTUATOR: " << actuator_ptr->getName() << " is with VELOCITY: " << servo_ptr->getVelocity() << std::endl;
        }
    }

    return sensorData;
}


void StonefishRL::InitializeZMQ() {
    try {
        socket.bind("tcp://*:5555");  // Escolta al port "5555"
        std::cout << "[ZMQ] Servidor REP actiu al port 5555\n";
    }
    catch(const zmq::error_t& e) {
        std::cerr << "[ZMQ ERROR] " << e.what() << "\n";
    }
}


StonefishRL::CommandData StonefishRL::ConvertStringToMap(const std::string str){
    
    StonefishRL::CommandData result;

    std::string input = str;
    input.erase(std::remove(input.begin(), input.end(), '\n'), input.end());

    std::istringstream iss(str);
    std::string token;
    
    while (std::getline(iss, token, ';')) {

        if(token.empty()) continue;

        size_t pos = token.find(":"); // Busca la posicio del ":"

        if(pos == std::string::npos) {
            std::cerr << "[ERROR] Invalid command format: " << token << ". Expected format: 'actuator_name:value'." << std::endl;
            continue; 
        }

        // Separar nom i valor 
        std::string name = token.substr(0, pos); // Agafa el que hi ha abans dels ":"
        std::string value_str = token.substr(pos + 1); // Agafa el que hi ha dps dels ":"

        float value = std::stof(value_str); // Convertir a float

        if(!(actuators_.count(name))) {
            std::cerr << "[ERROR] Actuator '" << name << "' not found in the scenario." << std::endl;
            std::cout << "DINS FUNCIÓ: Estic guardant al mapa amb nom: " << name <<  " el valor de: " << value << std::endl; 
            
            continue; // Si l'actuador no existeix, salta al seguent
        }

        result.commands[name] = static_cast<float>(value);
        std::cout << "Estic guardant al mapa amb nom: " << name <<  " el valor de: " << value << std::endl; 
    }

    return result;
}



void StonefishRL::ConvertStringToUnorderedMap(std::string str){

    commands_.clear(); // Netejar els antics valors, per si n'hi hagues de nous.

    std::string linea;
    std::stringstream ss(str);

    while(std::getline(ss, linea, ';')){
        
        if(linea.empty()) continue;

        std::string token;
        std::stringstream lineStream(linea);

        // 1r token = nom de l'actuador
        if (!std::getline(lineStream, token, ':')) continue;
        std::string actuator_name = token;


        // Llegir clau(action) : valor(action_value)
        while (std::getline(lineStream, token, ':')) {
            
            std::string action = token;
            
            if (!std::getline(lineStream, token, ':')) break;
            
            try {
                float action_value = std::stof(token);
                commands_[actuator_name][action] = action_value;
            } 
            catch (...) { // Agafa qualsevol excepció sense importar de quin tipus es
                std::cerr << "[ERROR] Invalid value for " << actuator_name << ":" << action << " --> '" << token << "'\n";
            }
        }
    }
}
