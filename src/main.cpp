#include "StonefishRL.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/core/SimulationApp.h>
#include <Stonefish/core/SimulationManager.h>


int main(int argc, char **argv) {

 
/*
    auto rSettings = sf::GraphicalSimulationApp::getRenderSettings(); // ???

    StonefishRL sim("../scenes/minimal_scene.xml");

    if(sim.StartSimulation()){

        std::cout << "Simulation Running, entering to the loop.\n";
        
        for (int i = 0; i < 500; i++) {

            sim.Step(0.01);

            StonefishRL::ObsData observations = sim.GetObservations();

            // Mostrar observacions (Nomes per la fase de test)
            for (const auto& pair : observations.num_observations) {
                std::cout << "Observed sensor '" << pair.first << "': ";
                for (float val : pair.second) {
                    std::cout << val << " ";
                }
                std::cout << "\n";
            }

        }

        std::cout << "Simulation finished.\n";
    }
    else 
        std::cout << "The function 'StartSimulation' is not working.\n"; 
    

    return 0;

*/

    sf::RenderSettings r;
    sf::HelperSettings s;

    return 0;

}
