#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/actuators/Actuator.h> // Incloure per sf::Actuador*
#include <Stonefish/sensors/Sensor.h> // Incloure per sf::Sensor*
 
class StonefishRL : public sf::SimulationManager {
public:

    // Conté les dades d'observació numeriques
    struct ObsData {
        //std::map<std::string, std::vector<double>> num_observations;
        
        // Estructura per les dades d'una imatge
        struct ImageData {
            std::vector<unsigned char> data; // Pixels raw
            unsigned int width;
            unsigned int height;
            unsigned int channels;
        };

        std::map<std::string, ImageData> img_observations;
    };

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
    void Step(double dt);
    void ApplyCommands(std::map<std::string, float> cmds);
    ObsData GetObservations();
    std::map<std::string, std::vector<float>> getScalarObservations();

protected:
    // Override del mètode de BuildScenario de sf::SimulationManager
    void BuildScenario() override;


private:
    std::string scenePath;

    //Guardar punters i actuadors pel seu nom
    std::map<std::string, sf::Sensor*> obs_sensors_;
    std::map<std::string, sf::Actuator*> actuators_;
    
    std::map<std::string, std::vector<double>> sensor_values_; // [nom sensor] -> [valors]

    //sf::Robot* robot;
    //std::vector<std::string> actuators;   
};

