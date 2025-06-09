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


    double frequency = 1000.0;
    std::string dataDirPath = "scenes/minimal_learning.xml";//"../scenes/minimal_learning.xml";
    if (argc > 1) {
        dataDirPath = argv[1]; // Permet carregar l'escena des d'un argument en la linea de comandes 
    }
    
    sf::RenderSettings r;
    sf::HelperSettings h;

    StonefishRL* simulationManager = new StonefishRL(frequency, dataDirPath);
    sf::GraphicalSimulationApp app("DEMO STONEFISH RL", dataDirPath, r, h, simulationManager);
    
    //std::cout << sf::GetDataPath() << std::endl;
    

    app.Run(false, false, /*1.0/frequency/*/sf::Scalar(0.1));

    return 0;

    // Clean-up
    //int status {0};
    //SDL_WaitThread(learningThread, &status);
    //return status;

}
