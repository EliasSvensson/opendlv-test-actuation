/*
Microservice for the kiwi car created by Adrian Lundell, Fredrik Lindelöw and Elias Svensson.
This microservice was created for a project in the course Autonomous Robots given at Chalmers UoT.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include <stdlib.h>  
#include <mutex>

#include <algorithm>
#include <list>

// Main function
int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") ||
      0 == commandlineArguments.count("freq")) {
    std::cerr << "Bad arguments" << std::endl;
    retCode = 1;
  } else {
    float const FREQ{static_cast<float>(std::stof(commandlineArguments["freq"]))};   
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))}; 
        
    std::list<float> recSteering(10, 0.f);
    std::list<float> recPedal(10, 0.f);
    std::list<float> testSteering(10, 0.f);
    std::list<float> testPedal(10, 0.f);

    std::mutex recSteeringMutex{};
    std::mutex recPedalMutex{};
    std::mutex testSteeringMutex{};
    std::mutex testPedalMutex{};

    auto onGroundSteeringRequest{[&recSteering, &recSteeringMutex, &testSteering, &testSteeringMutex](
          cluon::data::Envelope &&envelope)
        {
          uint32_t const senderStamp = envelope.senderStamp();
          auto steeringRequest = 
            cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(
                std::move(envelope));
            
          if (senderStamp == 0) {
            std::lock_guard<std::mutex> const lock(recSteeringMutex);
            recSteering.push_back(steeringRequest.groundSteering());
            if (recSteering.size() > 10){
              recSteering.pop_front();
            }
          } else if (senderStamp == 1) {
            std::lock_guard<std::mutex> const lock(testSteeringMutex);
            testSteering.push_back(steeringRequest.groundSteering());
            if (testSteering.size() > 10){
              testSteering.pop_front();
            }
          }
        }};

    auto onPedalPositionRequest{[&od4, &recPedal, &recPedalMutex, &testPedal, &testPedalMutex](
          cluon::data::Envelope &&envelope)
        {
          uint32_t const senderStamp = envelope.senderStamp();
          auto pedalPositionRequest = 
            cluon::extractMessage<opendlv::proxy::PedalPositionRequest>(
                std::move(envelope));
            
          if (senderStamp == 0) {
            std::lock_guard<std::mutex> const lock(recPedalMutex);
            recPedal.push_back(pedalPositionRequest.position());
            if (recPedal.size() > 10){
              recPedal.pop_front();
            }
          } else if (senderStamp == 1) {
            std::lock_guard<std::mutex> const lock(testPedalMutex);
            testPedal.push_back(pedalPositionRequest.position());
            if (testPedal.size() > 10){
              testPedal.pop_front();
            }
          }
        }};

    auto startTime = cluon::time::toMicroseconds(cluon::time::now());
    std::cout << "Time since start,recorded Steering Avg,model Steering Avg,recorded Pedal Avg,model Pedal Avg" << std::endl; 

    auto onFrequency{[&recSteering, &recPedal, &testSteering, &testPedal, &recSteeringMutex, &recPedalMutex, 
        &testSteeringMutex, &testPedalMutex, &startTime]()
        {
        
          std::vector<float> recSteeringCopy, recPedalCopy, testSteeringCopy, testPedalCopy;
          {
            std::lock_guard<std::mutex> const lock1(recSteeringMutex);
            std::lock_guard<std::mutex> const lock2(recPedalMutex);
            std::lock_guard<std::mutex> const lock3(testSteeringMutex);
            std::lock_guard<std::mutex> const lock4(testPedalMutex);

            recSteeringCopy = {std::begin(recSteering), std::end(recSteering)};
            recPedalCopy = {std::begin(recPedal), std::end(recPedal)};
            testSteeringCopy = {std::begin(testSteering), std::end(testSteering)};
            testPedalCopy = {std::begin(testPedal), std::end(testPedal)};     
          }
          
          float recSteeringAvg{0.f};
          for (uint32_t i=0; i<recSteeringCopy.size(); i++) {
            recSteeringAvg += recSteeringCopy[i];
          }
          recSteeringAvg = recSteeringAvg/(float)recSteeringCopy.size();
          
          float testSteeringAvg{0.f};
          for (uint32_t i=0; i<testSteeringCopy.size(); i++) {
            testSteeringAvg += testSteeringCopy[i];
          }
          testSteeringAvg = testSteeringAvg/(float)testSteeringCopy.size();

          float recPedalAvg{0.f};
          for (uint32_t i=0; i<recPedalCopy.size(); i++) {
            recPedalAvg += recPedalCopy[i];
          }
          recPedalAvg = recPedalAvg/(float)recPedalCopy.size();

          float testPedalAvg{0.f};
          for (uint32_t i=0; i<testPedalCopy.size(); i++) {
            testPedalAvg += testPedalCopy[i];
          }
          testPedalAvg = testPedalAvg/(float)testPedalCopy.size();

          auto timeNow = cluon::time::toMicroseconds(cluon::time::now());
          auto timePassed = timeNow - startTime;

          std::cout << timePassed << "," << recSteeringAvg << "," << testSteeringAvg << "," << recPedalAvg << "," << testPedalAvg << std::endl; 
          return true;
        }};

    od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);
    od4.dataTrigger(opendlv::proxy::PedalPositionRequest::ID(), onPedalPositionRequest);
    od4.timeTrigger(FREQ, onFrequency);
  }
  return retCode;
}


