#include "systemLoopHandler.h"

void ControlHandler::fesLoop(const systemLoopHandler<int> *obj)
{
    while(true)
    {
        while(!obj->startIterator){std::this_thread::sleep_for(std::chrono::milliseconds(100));}

        std::cout << "Fes: " << obj->mode << "\n\n";
    }
}

void ControlHandler::controllerLoop(const systemLoopHandler<int> *obj)
{
    while(true)
    {
        while(!obj->startIterator){std::this_thread::sleep_for(std::chrono::milliseconds(100));}

        std::cout << "Controller: "  << obj->mode << "\n\n";
    }
}

void ControlHandler::observationLoop(const systemLoopHandler<int> *obj)
{
    while(true)
    {
        while(!obj->startIterator){std::this_thread::sleep_for(std::chrono::milliseconds(100));}

        std::cout << "Observation: "  << obj->mode << "\n\n";
    }
}

void ControlHandler::advancedControllerLoop(const systemLoopHandler<int> *obj)
{
    while(true)
    {
        while(!obj->startIterator){std::this_thread::sleep_for(std::chrono::milliseconds(100));}

        std::cout << "AdvancedController: "  << obj->mode << "\n\n";
    }
}
