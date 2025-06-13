#include "StonefishRL.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath> // Per fer std::cos(..)

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

    while (simApp.getState() == sf::SimulationState::NOT_READY)
    {
        SDL_Delay(10);
    }
    

    simApp.StartSimulation();
    int contador = 0;

    while(simApp.getState() != sf::SimulationState::FINISHED && contador < 10) 
    {
        
        //simManager->StepSimulation(0.001); // Avançar 1ms
        //std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Esperar 100ms per veure els resultats


        sf::Servo* srv1 = (sf::Servo*)simManager->getActuator("Robot/Servo");
        srv1->setControlMode(sf::ServoControlMode::POSITION);
        float command1 = -0.5 * std::cos(simManager->getSimulationTime());
        srv1->setDesiredPosition(command1);
        
        
        sf::Servo* srv2 = (sf::Servo*)simManager->getActuator("Robot/Servo2");
        srv2->setControlMode(sf::ServoControlMode::POSITION);
        float command2 = 0.5 * std::cos(simManager->getSimulationTime());
        srv2->setDesiredPosition(command2);

    // ----- DIRIA QUE NO ESTA FENT RES (NO AFECTA A LA VELOCTAT) -----
        srv1->setControlMode(sf::ServoControlMode::VELOCITY);
        srv1->setMaxVelocity(0.1f); 
        
        srv2->setControlMode(sf::ServoControlMode::VELOCITY);
        srv2->setMaxVelocity(100.0f);     
    // ------------------------------------------------------------------
     

        simApp.StepSimulation();
        
    
        std::cout << std::endl;
        std::cout << "----------------  STARTING STEP: " << contador << "  ----------------" << std::endl; 

        auto* enc1 = static_cast<sf::RotaryEncoder*>(simManager->getSensor("Robot/Encoder"));
        if(enc1) {
            double angle1 = enc1->getLastSample().getValue(0);
            std::cout << "Encoder1 angle: " << angle1 << std::endl;
        } 
        else {
            std::cout << "[ERROR] Encoder1 not found." << std::endl;
        }

        auto* enc2 = static_cast<sf::RotaryEncoder*>(simManager->getSensor("Robot/Encoder2"));
        if(enc2) {
            double angle2 = enc2->getLastSample().getValue(0);
            std::cout << "Encoder2 angle: " << angle2 << std::endl;
        } 
        else {
            std::cout << "[ERROR] Encoder2 not found." << std::endl;
        }


       
        //static_cast<sf::Motor*>(simManager->getActuator("Robot/Motor"))->setIntensity(0.75);

        //sf::Scalar command1 = btCos(angle1) * 15;
        //sf::Scalar command2 = btCos(angle1) * 30;


        

        //static_cast<sf::Actuator*>(simManager->getActuator("Robot/Servo"))->setIntensity(command1); // Fa petar el programa
        //static_cast<sf::Motor*>(simManager->getActuator("Motor"))->setIntensity(command1);
        
        
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        contador++;
        std::cout << "[INFO] Simulation time: " << simManager->getSimulationTime() << " seconds." << std::endl;
        std::cout << "[INFO] Step " << contador << " completed." << std::endl;
        
    }

    return 0;
}


int main(int argc, char **argv) {

    double frequency = 1000.0;
    
    if (argc < 2) {
        std::cerr << "[ERROR] You may need 1 arguments at least." << std::endl;
        std::cerr << "Utilitzat: " << argv[0] << " <Resources/type_scene/scene.xml>\n";
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
/*
    app.StartSimulation();

    unsigned int comptador = 0;

    while(app.getState() != sf::SimulationState::FINISHED && comptador< 10)
    {

        app.StepSimulation(); // Fa donar un segmentation fault

        float t = simManager->getSimulationTime();
        std::cout << "[TEMPS SIMULACIO] " << t << std::endl;
        StonefishRL::CommandData cmd;
        
        cmd.commands["Servo"] = 0.5 * std::cos(t);
        cmd.commands["Servo2"] = -0.5 * std::cos(t);
    
     
        simManager->ApplyCommands(cmd);
        
        StonefishRL::ObsData obs = simManager->GetObservations();

        if(auto* enc = static_cast<sf::RotaryEncoder*>(simManager->getSensor("Encoder"))){
            double angle = enc->getLastSample().getValue(0);
            std::cout << "Encoder angle: " << angle << std::endl;
        }
        comptador++;
   
        simManager->Step(0.001); // Avançar 1ms
        std::cout << "Fins aqui arriba" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Esperar 100ms per veure els resultats
    }



    /*
    while(app.getState() != sf::SimulationState::FINISHED){
        

        app.StepSimulation();

        // Get Observations
        sf::Scalar angle1 = static_cast<sf::RotaryEncoder*>(simManager->getSensor("Encoder1"))->getLastSample().getValue(0);


        static_cast<sf::Motor*>(simManager->getActuator("Servo"))->setIntensity(0.75);
    }

        if(auto *enc = static_cast<sf::RotaryEncoder*>(simManager->getSensor("Encoder"))){
            double angle = enc->getLastSample().getValue(0);
            std::cout << "Encoder angle: " << angle << std::endl;
        }      
        
      
        if(auto *imu = static_cast<sf::IMU*>(simManager->getSensor("IMU"))){
            auto sample = imu->getLastSample();
            std::cout << "IMU ang vel: "
                      << sample.angular_velocity.transpose()
                      << "; lin acc: "
                      << sample.linear_acceleration.transpose()
                      << std::endl;
        }
    

        double t = simManager->getSimulationTime();

        auto* s1 = static_cast<sf::Servo*>(simManager->getActuator("Servo"));
        if (s1) {
            s1->setControlMode(sf::ServoControlMode::POSITION);
            s1->setMaxVelocity(0.1f);             // Max velocitat (rad/s)
            s1->setDesiredPosition(0.5 * sin(t)); // Posicio objetiu
        }

        if(auto* servo2 = static_cast<sf::Servo*>(simManager->getActuator("Servo2"))){
            servo2->setControlMode(sf::ServoControlMode::POSITION);
            servo2->setMaxVelocity(0.1f);             // Max velocitat (rad/s)
            servo2->setDesiredPosition(0.5 * sin(t)); // Posicio objetiu
        }

    }
*/
    
    app.Run(false, false, sf::Scalar(0.1));
    std::cout << "[INFO] Simulation finished." << std::endl;

    int status {0};
    SDL_WaitThread(learningThread, &status);
    return status;
}
