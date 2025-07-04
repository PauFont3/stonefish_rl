#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/actuators/Actuator.h>
#include <Stonefish/sensors/Sensor.h> 

#include <zmq.hpp> // Per la comunicació amb Python


class StonefishRL : public sf::SimulationManager {
public:

    struct Vec3
    {
        float x = NAN, y = NAN, z = NAN;
    };

    struct Observation
    {
        std::string name;
        Vec3 position;
        float angle = NAN;
        float linear_velocity = NAN;
        float angular_velocity = NAN;
    };

    struct StateScene
    {
        std::vector<Observation> observations;
    };


    // Constructor
    StonefishRL(const std::string& path, double frequency);

    // Destructor
    ~StonefishRL(); 

    // Metodes per la interface de RL
    void Reset();
    std::string RecieveInstructions();
    void SendObservations();
    void ApplyCommands(const std::string& str_cmds);
    StateScene GetStateScene();
    std::vector<float> StateToVector(const StateScene& state);

    //CommandData ConvertStringToMap(std::string& str);
    void ParseCommandsAndObservations(const std::string& str);

    bool SetRobotPosition(std::string cmd, const float* position_data, int n_param);
    
    void ExitRequest();

    bool ObjImportantForObs(const std::string& objName) const;

    void MostrarValors();


protected:
    // Override del mètode de BuildScenario de sf::SimulationManager
    void BuildScenario() override;

private:
    
    void InitializeZMQ();

    std::string scenePath;

    //Guardar punters i actuadors pel seu nom
    std::unordered_map<std::string, sf::Sensor*> sensors_;      // Agafa tots els actuadors que hi ha a l'escena, perque es crida al BuildScenario.
    std::unordered_map<std::string, sf::Actuator*> actuators_; // Agafa tots els actuadors que hi ha a l'escena, perque es crida al BuildScenario.
    
    std::unordered_set<std::string> relevant_obs_names_;

    std::map<std::string, std::map<std::string, float>> commands_; // Valors que s'aplicaran als actuadors.
    StateScene current_state_; // Representa l'ultim estat observat de l'entorn
    
    // std::vector<std::string> ordered_names_; // Ens pot servir per quan utilitzem "Box"

    zmq::context_t context; // Context per ZeroMQ
    zmq::socket_t socket; // Socket per ZeroMQ
};