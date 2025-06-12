#include "StonefishRL.h"
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <cmath> // Per fer servir std::sin

// Includes per SENSORS
#include <Stonefish/sensors/Sensor.h> // Per la clase base Sensor i el getName()
#include <Stonefish/sensors/Sample.h> // Per poder fer sf:Sample
#include <Stonefish/sensors/ScalarSensor.h> // Per la clase base ScalarSensor i el getLastSample()
#include <Stonefish/sensors/VisionSensor.h>
#include <Stonefish/sensors/scalar/IMU.h> // IMU es vectorial, pero esta en aquesta ruta
#include <Stonefish/sensors/scalar/RotaryEncoder.h> // Per fer sf::RotaryEncoder
#include <Stonefish/sensors/vision/Camera.h>

// Includes per ACTUADORS
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/actuators/Motor.h>
#include <Stonefish/actuators/Actuator.h>

#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/core/ScenarioParser.h> // Per carregar l'arxiu XML
#include <Stonefish/StonefishCommon.h>



// Constructor de la classe StonefishRL
StonefishRL::StonefishRL(const std::string& path, double frequency)
    : sf::SimulationManager(frequency), scenePath(path)
{ 
    std::cout << "Constructor de StonefishRL" << std::endl;
    std::cout << "[INIT] StonefishRL created with scene: " 
              << path << std::endl;
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
    sf::SimulationManager::RestartScenario();
    // RestartScenario ja conté:
    //      - DestroyScenario();
    //      - InitializeSolver();
    //      - InitializeScenario();
    //      - BuildScenario();
    std::cout << "[StonefishRL] Scenario restarted OK\n";
}

void StonefishRL::Step(sf::Scalar ts) {
    sf::SimulationManager::StepSimulation(ts);
    std::cout << "[StonefishRL] StepSimulation OK";
}

void StonefishRL::ApplyCommands(const CommandData& cmds) {
 
    for(auto const& [name, val] : cmds.commands) {
        std::cout << "NOM ACTUADOR: " << name << ". Valors: " << val << "." << std::endl;
        
        auto iT = actuators_.find("Robot/Servo");
        
        if(iT == actuators_.end()) {
            std::cout << "[TESTS] Actuator name:" << name << " not registered" << std::endl;
            //continue;
        }

        auto act = iT->second;
        if(act->getType() == sf::ActuatorType::SERVO){
            std::cout << "He entrat" << std::endl;
            sf::Servo* servo = (sf::Servo*)(act);
            servo->setControlMode(sf::ServoControlMode::POSITION);
            servo->setDesiredPosition(9.5);
        }
    }   

// DIU QUE NO TROBA ELS ACTUADORS
/*    for(const auto& pair : cmds.commands) {
        const std::string& actuator_name = pair.first;
        float command_value = pair.second;
    
        // Buscar actuador per nom
        auto iT = actuators_.find(actuator_name);
        if(iT == actuators_.end()) {
            std::cerr << "[WARN] Actuator '" << actuator_name << "' not found in simulation." << std::endl;
            continue;
        }

        sf::Actuator* generic_actuator = iT->second;

        if(auto* servo = dynamic_cast<sf::Servo*>(generic_actuator)) {
            servo->setControlMode(sf::ServoControlMode::POSITION);
            servo->setMaxVelocity(0.1f);
            servo->setDesiredPosition(command_value);
            // ...
        }

     // Per afegir més tipus 
     // else if(auto* motor = dynamic_cast<sf::Motor*>(actuator)) {
     //     motor->setIntensity(value);
     //
     // }

        else {
            std::cerr << "[WARN] Actuator '" << actuator_name << "' type not supported in ApplyCommands." << std::endl;
        }
    }

*/
/*   
    float t = getSimulationTime();
    float pos = 0.5 * std::sin(t);


    for(const auto& pair : cmds.commands) 
    {
        const std::string& actuator_name = pair.first;
        const float command_value = pair.second;
    
    
        auto iT = actuators_.find(actuator_name);
        if(iT == actuators_.end())
        {
            std::cerr << "[WARN] Actuator " << actuator_name << " not found in simulation." << std::endl;
        }

        sf::Actuator* generic_actuator = iT->second;

        if(auto motor = dynamic_cast<sf::Motor*>(generic_actuator)){
            
        }
        else if(auto servo = dynamic_cast<sf::Servo*>(generic_actuator)){
            
        }
    }
*/
}


StonefishRL::ObsData StonefishRL::GetObservations() {
    
    ObsData data;

    if (obs_sensors_.empty()) {
        return data;
    }

    for(const auto& [sensor_name, sensorPtr] : obs_sensors_) {

        if(!sensorPtr) {
            std::cerr << "[WARN] Sensor " << sensor_name << " is nullptr!" << std::endl;
            continue;
        }
        
        // ScalarSensor
        if(auto scalar_sensor = dynamic_cast<sf::ScalarSensor*>(sensorPtr))
        {
            //std::vector<float> sensor_values = scalar_sensor->getLastSample().getData();
            sensor_values_[sensor_name] = scalar_sensor->getLastSample().getData();
            std::cout << "Tamany del vector:" << sensor_values_[sensor_name].size() << std::endl;

            for(unsigned int i = 0; i < sensor_values_[sensor_name].size(); i++) {
                std::cout << "Valors del sensors: " << sensor_values_[sensor_name][i] << std::endl;
            }

        // ------------------ PROBABLEMENT OPCIONAL ------------------
            sf::Sample last_sample = scalar_sensor->getLastSample();
            std::cout << "[INFO] LastSample ID: " << last_sample.getId() << std::endl 
                      << "[INFO] Sensor Name: " << sensor_name << std::endl 
                      << "[INFO] Num of channels: " << scalar_sensor->getNumOfChannels() << std::endl;
       
            for (size_t i = 0; i < last_sample.getNumOfDimensions(); i++) {
                std::cout << " Dim " << i << ": " << last_sample.getValue(i) << std::endl;
            }
         // ------------------------------------------------------------
       
            if (scalar_sensor->getNumOfChannels() == 0) {
                std::cerr << "[WARN] Sensor " << sensor_name << " has zero channels." << std::endl;
                continue;
            }

            if (sensor_values_.empty()) {
                std::cerr << "[WARN] No observations catched! (Check if sensors are defined in XML)" << std::endl;
            }
        }
    }

/*
    // Iterar sobre els sensors que s'han guardat durant BuildScenario(). 
    for(auto const& [name, sensor] : obs_sensors_) 
    {
        // Falta controlar errors del nullptr per exemple
        if(auto* scalarSensor = dynamic_cast<sf::ScalarSensor*>(sensor))
        {
            // Agafar la ultima mostra del ScalarSensor
            sf::Sample lastSample = scalarSensor->getLastSample();

            // Crear un vector per guardar els valors dels canals d'aquest sensor en el que estem
            std::vector<float> sensor_values;
            sensor_values.reserve(scalarSensor->getNumOfChannels());

            // Iterar sobre tots els canals del sensor i obtenir els seus valors
            for(unsigned int i = 0; i < scalarSensor->getNumOfChannels(); i++) 
            {
                sensor_values.push_back(static_cast<float>(lastSample.getValue(i)));
            }

            // Guardar el vector al map d'observacions
            data.num_observations[name] = sensor_values;
        }
        else 
        {
            std::cout << "Sensor: " << sensor->getName() << " not detected" << std::endl;
        }

        // else if (vision sensor)

    }
*/
    return data;
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
/*
    sf::Robot* robot_ptr;
    unsigned int id = 0;
    while((robot_ptr = getRobot(id++)) != nullptr){
        std::cout << "Nom robot: " << robot_ptr->getName() << std::endl;
    }   
*/  

    obs_sensors_.clear();
    actuators_.clear();

    sf::Sensor* sensor_ptr;
    unsigned int sensor_id = 0;

    while((sensor_ptr = getSensor(sensor_id++)) != nullptr) {
        obs_sensors_[sensor_ptr->getName()] = sensor_ptr;
    }  

    
    if(obs_sensors_.empty()) std::cout << "[WARN] No sensors registered in this scenario!" << std::endl;
    else {
        std::cout << "Contingut del mapa de sensors:" << std::endl;
        for (const auto& pair : obs_sensors_) {
            std::cout << "  Clau: '" << pair.first 
                      << "\n        Valor UpdatedFrequency: " << pair.second->getUpdateFrequency() 
                      << "\n        Valor Nom Sensor: " << pair.second->getName()
                      << std::endl;
        }
    }
    
    sf::Actuator* actuator_ptr;
    unsigned int actuator_id = 0;
    while((actuator_ptr = getActuator(actuator_id++)) != nullptr){
       
        actuators_[actuator_ptr->getName()] = actuator_ptr;
        
    } 

    if (actuators_.empty()) std::cout << "[WARN] No actuators registered in this scenario!" << std::endl;
    else {
        std::cout << "  Contingut del mapa d'actuadors:" << std::endl;
        for (const auto& pair : actuators_) {
            std::cout << "      Clau: '" << pair.first << "', punter: " << pair.second << std::endl;
        }
    }

    ObsData ab = GetObservations();
    
    std::cout << "[INFO] Scenario loaded succesfully.\n";
}
