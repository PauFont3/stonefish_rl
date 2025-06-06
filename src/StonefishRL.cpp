#include "StonefishRL.h"
#include <iostream>

#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/core/ScenarioParser.h> // Per carregar l'arxiu XML
#include <Stonefish/sensors/scalar/RotaryEncoder.h> // Per fer sf::RotaryEncoder
#include <Stonefish/actuators/Motor.h> // Per fer sf::Motor

#include <Stonefish/sensors/Sensor.h> // Per la clase base Sensor i el getName()
#include <Stonefish/sensors/Sample.h> // Per poder fer sf:Sample
#include <Stonefish/sensors/ScalarSensor.h> // Per la clase base ScalarSensor i el getLastSample()




// Constructor de la classe StonefishRL
StonefishRL::StonefishRL(double frequency) 
    : sf::SimulationManager(frequency) //Frequencia simulacio en Hz
{ 
    std::cout << "[INIT] StonefishRL created with scene: " << scenePath << std::endl;
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

void StonefishRL::Step(double ts) {
    sf::SimulationManager::StepSimulation(ts);
    std::cout << "[StonefishRL] StepSimulation OK";
}

void StonefishRL::ApplyCommands(const CommandData& cmds) {
    
}

StonefishRL::ObsData StonefishRL::GetObservations() {
    ObsData data; // Per guardar les dades

    // Iterar sobre els sensors que s'han guardat durant BuildScenario(). 
    for(auto const& [name, sensor] : obs_sensors_) 
    {
        // Falta controlar errors del nullptr per exemple
        if(auto* scalarSensor = dynamic_cast<sf::ScalarSensor*>(sensor)){
            
            // Agafar la ultima mostra del ScalarSensor
            sf::Sample lastSample = scalarSensor->getLastSample();

            // Crear un vector per guardar els valors dels canals d'aquest sensor en el que estem
            std::vector<float> sensor_values;
            sensor_values.reserve(scalarSensor->getNumOfChannels());

            // Iterar sobre tots els canals del sensor i obtenir els seus valors
            // "Unsigned int" pq aixi es com ho té en Patrykc
            for(unsigned int i = 0; i < scalarSensor->getNumOfChannels(); i++) 
            {
                sensor_values.push_back(static_cast<float>(lastSample.getValue(i)));
            }

            // Guardar el vector al map d'observacions
            data.num_observations[name] = sensor_values;
        }
        else {
            std::cout << "Sensor: " << sensor->getName() << " not detected" << std::endl;
        }

        // else if (sensor Imatge)

    }

    return data;
}

void StonefishRL::BuildScenario() {

    std::cout << "[INFO] Building scenario from: " << scenePath << std::endl;

    sf::ScenarioParser parser(this);
    
    if (parser.Parse("minimal_scene.xml")) 
    {

        sf::Robot* robot;
        unsigned int id = 0;
        while((robot = getRobot(id++)) != nullptr){
            std::cout << "Num robots: " << id << std::endl;
        }   

        std::cout << "[INFO] Scenario loaded succesfully.\n";
    } 
    else 
    {
        std::cerr << "[ERROR] Error charging the scenario: " << scenePath << "\n";
        for (const auto& msg : parser.getLog()) {
            std::cerr << "[ScenarioParser] Mensaje en log del parser.\n";
        }
    }


/*    sf::ScenarioParser parser(this);
    bool success = parser.Parse(scenePath);


    if(!success)
        std::cerr << "Failed to parse scenario: " << scenePath << std::endl;
    
    
   
    return result;
 */   


    /*
    if(parser.Parse(scenePath)){


        sf::Sensor* sensor;
        unsigned int id = 0;
        while((sensor = getSensor(id++)) != nullptr){
            obs_sensors_[sensor->getName()] = sensor;
        }  

        sf::Actuator* actuator;
        id = 0;
        while((sensor = getActuator(id++)) != nullptr){
            actuators_[actuator->getName()] = actuator;
        }  


        std::cout << "[INFO] Found " << obs_sensors_.size() << " sensors and " << actuators_.size() << " actuators." << std::endl;
        std::cout << "[INFO] Scene loaded successfully." << std::endl;
    }

    else {
        std::cout << "La funcio 'Parse(scenePath)' no ha funcionat be" << std::endl;
    }
        */
    
}
