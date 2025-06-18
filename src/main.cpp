#include "StonefishRL.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/core/SimulationApp.h>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/sensors/scalar/RotaryEncoder.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/ScalarSensor.h>
#include <Stonefish/sensors/Sample.h>
#include <Stonefish/actuators/Motor.h>
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/StonefishCommon.h>



struct LearningThreadData
{
    sf::SimulationApp& sim;
};


int learning(void* data) {
    sf::SimulationApp& simApp = static_cast<LearningThreadData*>(data)->sim;
    sf::SimulationManager* simManager = simApp.getSimulationManager();
    StonefishRL* myManager = static_cast<StonefishRL*>(simManager);

    while (simApp.getState() == sf::SimulationState::NOT_READY)
    {
        SDL_Delay(10);
    }
    
    // Start the simulation (includes building the scenario)
    simApp.StartSimulation();
    int contador = 0;

    while(simApp.getState() != sf::SimulationState::FINISHED && contador < 50 /*|| simApp.getState() != sf::SimulationState::STOPPED*/)
    {
        if(simApp.getState() == sf::SimulationState::STOPPED) {
            myManager->RestartScenario();
            simApp.StartSimulation();
            std::cout << "[INFO] Scenario restarted." << std::endl;
        }

        myManager->RecieveInstructions();

        simApp.StepSimulation();
    
        std::cout << std::endl << "----------------  STARTING STEP: " << contador << "  ----------------" << std::endl; 
        std::cout << "------------------------------------------------------" << std::endl;

        myManager->SendObservations();
      
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        contador++;

        std::cout << "[INFO] Simulation time: " << simManager->getSimulationTime() << " seconds." << std::endl;
        std::cout << "[INFO] Step " << contador << " completed." << std::endl;
    }
    simApp.StopSimulation();
    std::cout << "[INFO] Learning thread finished after " << contador << " steps." << std::endl;

    return 0;
}


int main(int argc, char **argv) {

    double frequency = 1000.0;
    
    if (argc < 2) {
        std::cerr << "[ERROR] You may need 1 arguments at least." << std::endl;
        return 1;
    }

    std::string scene_path = argv[1]; 

    sf::HelperSettings h;
    sf::RenderSettings r;
    r.windowW = 1200;
    r.windowH = 900;
   

    StonefishRL* simManager = new StonefishRL(scene_path, frequency);

    sf::GraphicalSimulationApp app("DEMO STONEFISH RL", scene_path, r, h, simManager);

    LearningThreadData data {app};
    SDL_Thread* learningThread = SDL_CreateThread(learning, "learningThread", &data);
    
    app.Run(false, false, sf::Scalar(1/frequency));
    std::cout << "[INFO] Simulation finished." << std::endl;

    int status {0};
    SDL_WaitThread(learningThread, &status);
    return status;
}
