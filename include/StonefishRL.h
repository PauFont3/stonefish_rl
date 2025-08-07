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

// IMPORTANT: Sempre que s'afegueixi un nou struct i/o paràmetre per enviar cap 
// al Python, perque per exemle hi ha un sensor nou i volem enviar els nous valors
// que recull que aquest sensor, els parametres o structs que s'afegeixin, també s'hauran
// d'afegir a les funcions: 'InfoObjectToJson', 'FillWithNanInfoObject' i 'GetStateScene'
// pq son les que envien les dades cap al python.

    
    // Conté tota l'informació d'un objecte de l'escena
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

        std::vector<float> gps; // Pos: [0] latitude, [1] longitude, [2] North, [3] East
    };

    // Conté tots els obejctes de l'escena
    struct StateScene
    {
        std::vector<InfoObject> observations;
    };

    // Constructor
    StonefishRL(const std::string& path, double frequency);

    // Metodes per la interface de RL
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

    void MostrarValors();

    void FillWithNanInfoObject(InfoObject& pose);

    void ProvaMostrarTot();

    // Converteix a JSON per enviar al Python
    std::string InfoObjectToJson(const InfoObject& pose); 
    std::string EscapeJson(const std::string& s);
    std::string SerializeScene(const std::vector<InfoObject>& poses);
    std::string SafeFloat(float val);

protected:
    // Override del mètode de BuildScenario de sf::SimulationManager
    void BuildScenario() override;

private:
    
    void InitializeZMQ();

    std::string scenePath;

    //Guardar punters i actuadors pel seu nom
    std::unordered_map<std::string, sf::Sensor*> sensors_;      // Agafa tots els actuadors que hi ha a l'escena, perque es crida al BuildScenario.
    std::unordered_map<std::string, sf::Actuator*> actuators_; // Agafa tots els actuadors que hi ha a l'escena, perque es crida al BuildScenario.
    std::unordered_map<std::string, sf::Robot*> robots_;      // Agafa tots els robots que hi ha a l'escena, perque es crida al BuildScenario.
    
    std::unordered_set<std::string> relevant_obs_names_; // Segurament no caldra, pq agafem totes les observacions, no només les que ens han indicat.

    std::map<std::string, std::map<std::string, float>> commands_; // Valors que s'aplicaran als actuadors.
    StateScene current_state_; // Representa l'ultim estat observat de l'entorn
    
    zmq::context_t context; // Context per ZeroMQ
    zmq::socket_t socket; // Socket per ZeroMQ
};


/* 
ACCIONS QUE ES PODEN APLICAR:
    
    - SERVO:
        - VELOCITY i TORQUE (els dos aplicaran velocitat)
        - POSITION 
    
    - THRUSTER:
        - TORQUE (Dona una velocitat de rotació a les helices del thrusters)
*/

/*
El sensor de OdoGripper per saber la posició del braç (pinça del girona500)
i si s'està acostant gaire al objecte, està acoplat al link "ECAEndEffector"
*/