#include "StonefishRL.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/core/SimulationApp.h>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/sensors/scalar/RotaryEncoder.h>
#include <Stonefish/actuators/Motor.h>
#include <Stonefish/sensors/Sample.h>


int main(int argc, char **argv) {

 
/*
    auto rSettings = sf::GraphicalSimulationApp::getRenderSettings(); // ???

    StonefishRL sim("../scenes/minimal_scene.xml");

    sim.StartSimulation()

        std::cout << "Simulation Running, entering to the loop.\n";
        
        std::cout << "Simulation finished.\n";
    }

    return 0;
*/

    double frequency = 1000.0;

    sf::RenderSettings r;
    sf::HelperSettings h;

    StonefishRL* simulationManager = new StonefishRL(frequency);

    sf::GraphicalSimulationApp app("DEMO STONEFISH RL", "../scenes/minimal_learning.xml", r, h, simulationManager);
    
    app.Run(false, false, sf::Scalar(0.1));

    return 0;

    // Clean-up
    //int status {0};
    //SDL_WaitThread(learningThread, &status);
    //return status;

}
