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
        std::map<std::string, float> commands; // Nom de l'actuador i el seu valor (float). Aquest llegira els que estiguin ben escrits desde python.
                                               // Si hi ha algun error en el nom no l'agafarà
    };

    // Constructor
    StonefishRL(const std::string& path, double frequency);

    // Destructor
    ~StonefishRL(); 

    // Metodes per la interface de RL
    void Reset();
    std::string RecieveInstructions();
    void SendObservations();
    void ApplyCommands(const std::string &str_cmds);
    std::map<std::string, std::vector<float>> getScalarObservations();

    CommandData ConvertStringToMap(std::string str);
    void ConvertStringToUnorderedMap(std::string str);

    void SetRobotPosition(const std::string &cmd, sf::Vector3 new_pos);

protected:
    // Override del mètode de BuildScenario de sf::SimulationManager
    void BuildScenario() override;


private:
    
    void InitializeZMQ();
    
    std::string scenePath;

    //Guardar punters i actuadors pel seu nom
    std::map<std::string, sf::Sensor*> obs_sensors_;
    std::map<std::string, sf::Actuator*> actuators_; // Agafa tots els actuadors, perque es crida al BuildScenario.
    
    std::map<std::string, std::vector<double>> sensor_values_; // [nom sensor] -> [valors]
    std::unordered_map<std::string, std::unordered_map<std::string, float>> commands_; // Valors que s'aplicaran als actuadors.

    zmq::context_t context; // Context per ZeroMQ
    zmq::socket_t socket; // Socket per ZeroMQ
};