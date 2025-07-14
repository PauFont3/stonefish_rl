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
    std::string nextStepSim;

    while(nextStepSim != "EXIT")
    {
        std::cout << "\n\n---------------------------------------------------------------------------- \n";
        std::cout << "-----------------------  STARTING STEP: " << contador << "  ------------------------------" << std::endl; 
        std::cout << "---------------------------------------------------------------------------- \n";
        
        nextStepSim = myManager->RecieveInstructions();
        
        float time0 = simManager->getSimulationTime();
        
        if(nextStepSim == "CMD" || nextStepSim == "RESET"){
            simApp.StepSimulation();
            if(nextStepSim == "CMD"){
                myManager->SendObservations();
                contador++;
            }
            else contador = 0;

        }
        
        //std::this_thread::sleep_for(std::chrono::milliseconds(1));  // Si el treiem (comentem aquesta linea) va tremendament ràpid.
                                                                    // Si el posem a 1, va una mica ràpid, però acceptable.
                                                                    // Si el deixem a 10, potser és un xic lent, però veus bé les trajectories que fa.
        float time1 = simManager->getSimulationTime();
        
        float delta_time = time1 - time0;
        float steps_per_second = 1.0 / delta_time;
        
        std::cout << "[INFO] Delta Time = " << delta_time << " segons" << std::endl;
        std::cout << "[INFO] Steps per second: " << steps_per_second << std::endl;
    }

    std::cout << "[INFO] Learning thread finished after " << contador << " steps." << std::endl;
    myManager->ExitRequest();
    return 0;
}


int main(int argc, char **argv) {

    double frequency = 1000.0;
    
    if (argc < 2) {
        std::cerr << "[ERROR] You may need 1 argument at least." << std::endl;
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

    SDL_WaitThread(learningThread, nullptr);
    return 0;
}