#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/actuators/Actuator.h>
#include <Stonefish/sensors/Sensor.h> 

#include <zmq.hpp> // Per la comunicació amb Python
 

class StonefishRL : public sf::SimulationManager {
public:

    // Conté les comandes a aplicar
    struct CommandData {
        std::map<std::string, float> commands; // Nom de l'actuador i el seu valor (float)
    };

    // Constructor
    StonefishRL(const std::string& path, double frequency);

    // Destructor
    ~StonefishRL(); 

    // Metodes per la interface de RL
    void Reset();
    void RecieveInstructions();
    void SendObservations();
    void ApplyCommands(const std::string &str_cmds);
    std::map<std::string, std::vector<float>> getScalarObservations();

   CommandData ConvertStringToMap(std::string str);

protected:
    // Override del mètode de BuildScenario de sf::SimulationManager
    void BuildScenario() override;


private:
    std::string scenePath;

    //Guardar punters i actuadors pel seu nom
    std::map<std::string, sf::Sensor*> obs_sensors_;
    std::map<std::string, sf::Actuator*> actuators_;
    
    std::map<std::string, std::vector<double>> sensor_values_; // [nom sensor] -> [valors]

    void InitializeZMQ();

    zmq::context_t context; // Context per ZeroMQ
    zmq::socket_t socket; // Socket per ZeroMQ
};