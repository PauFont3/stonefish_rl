#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/core/SimulationApp.h>

#include <Stonefish/actuators/Actuator.h>
#include <Stonefish/actuators/Thruster.h>

#include <Stonefish/sensors/Sensor.h> 

#include <zmq.hpp> // Per la comunicació amb Python

class StonefishRL : public sf::SimulationManager {
public:

// IMPORTANT: Whenever a new struct and/or parameter is added to send to Python
// (e.g. a new sensor whose readings you want to include), you must also update
// the functions 'InfoObjectToJson', 'FillWithNanInfoObject', and 'GetStateScene',
// because those are responsible for sending data to Python.
    
    // Contains all information for one scene object
    struct InfoObject 
    {
        std::string name;
    
        float angle;
        float pressure;
    
        std::vector<float> position;
        std::vector<float> rotation;
        
        std::vector<float> linear_velocity;
        std::vector<float> linear_acceleration;
        std::vector<float> angular_velocity;
    
        std::vector<float> force; 
        std::vector<float> torque; 

        std::vector<float> gps; // Indices: [0] latitude, [1] longitude, [2] North, [3] East
    };

    // Holds all objects in the scene
    struct StateScene
    {
        std::vector<InfoObject> observations;
    };

    // Constructor
    StonefishRL(const std::string& path, double frequency);

    // Methods for the RL interface
    std::string RecieveInstructions(sf::SimulationApp& simApp);
    
    void SendObservations();
    
    void ApplyCommands(const std::string& str_cmds);
    
    StateScene GetStateScene();

    std::vector<InfoObject> ParseResetCommand(const std::string& str);

    void ApplyReset(const std::vector<InfoObject>& objects);

    void ParseCommandsAndObservations(const std::string& str);

    void SetRobotPosition(const std::vector<InfoObject>& obj);
    
    void ExitRequest();

    bool ObjImportantForObs(const std::string& objName) const;

    void FillWithNanInfoObject(InfoObject& pose);

    void PrintAll();

    // Convert to JSON to send to Python
    std::string InfoObjectToJson(const InfoObject& pose); 
    std::string EscapeJson(const std::string& s);
    std::string SerializeScene(const std::vector<InfoObject>& poses);
    std::string SafeFloat(float val);

protected:
    // Override of the method 'sf::SimulationManager::BuildScenario'
    void BuildScenario() override;

private:
    
    void InitializeZMQ();

    std::string scenePath;

    std::unordered_set<std::string> relevant_obs_names_;

    std::map<std::string, std::map<std::string, float>> commands_; // Values to apply to actuators
    StateScene current_state_; // Represents the latest observed state of the environment
    
    zmq::context_t context; // ZeroMQ context
    zmq::socket_t socket;   // ZeroMQ socket
};
