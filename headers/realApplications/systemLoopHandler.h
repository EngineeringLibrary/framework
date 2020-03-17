#ifndef systemLoopHandler_H
#define systemLoopHandler_H
#include <list>
#include <thread>
#include <algorithm>
#include <mutex>
#include <condition_variable>
#include <framework/headers/primitiveLibs/LinAlg/linalg.h>
#include <framework/headers/controlLibs/pidTuning.h>
#include <framework/headers/modelLibs/arx.h>
#include <framework/headers/optimizationLibs/recursiveleastsquare.h>
#include "sensor.h"

namespace ControlHandler{
    template <typename Type>
    struct observationHandler
    {
        sensorHandler::Sensor *sensors;

    };

    template <typename Type>
    struct fesHandler
    {
        //adicionar o eletroestimulador e os sensores;
        sensorHandler::Sensor *sensors;
        volatile Type channel, *in, *out, input;
        volatile uint16_t iterator, maxIterator, operationalPointIterator;
        volatile uint8_t tuningMethod, controller;
        volatile float TIMER_SCALE, TIMER_FINE_ADJ, TIMER_INTERVAL0_SEC;
    };

    template <typename Type>
    struct controllerHandler
    {
        sensorHandler::Sensor *sensors;

        volatile Type input, operationalInput, operationalOutput;
        Type controllerReference;
        ControlHandler::GeneralController<Type> **controller;
        LinAlg::Matrix<Type> controlReferences;
        volatile uint16_t iterator, maxIterator;
    };

    template <typename Type>
    struct impedanceEstimatonHandler
    {
        volatile Type relayMinLimit, relayMaxLimit, relayTolerance, relayOperationalInput,
                      relayOperationalOutput, relayReference, error, controllerSensibility,
                      *relayIn, *relayOut,
                      relayOperationalPointIterator;
        volatile uint16_t relayIterator, relayMaxIterator;

        ModelHandler::ARX<Type> *boost;
        OptimizationHandler::RecursiveLeastSquare<Type> *rls;
    };

    template <typename Type>
    struct advancedControllerHandler
    {
        observationHandler<Type> **observation;
        fesHandler<Type> **fes;

        volatile Type input, operationalInput, operationalOutput;
        Type controllerReference;
        ControlHandler::GeneralController<Type> *advancedController;
        LinAlg::Matrix<Type> controlReferences;
        volatile uint16_t iterator, maxIterator;
    };

    typedef enum {
        OBSERVATION = 0,
        IMPEDANCE_ESTIMATION = 1,
        CLASSIC_FES = 2,
        REGULATORY_CONTROL = 3,
        ADVANCED_CONTROL = 4,
        LOOPMODE_MAX,
    } loopMode_t;

    template <typename Type>
    struct systemLoopHandler
    {
        fesHandler<Type> fes;
        observationHandler<Type> observer;
        controllerHandler<Type>  controller;
        impedanceEstimatonHandler<Type> impedance;

        loopMode_t mode;
        bool startIterator;

        std::thread *routine;
    };

    static ::std::mutex loopMutex;
    static ::std::condition_variable cv_;

    void fesLoop(const systemLoopHandler<int> *obj);
    void controllerLoop(const systemLoopHandler<int> *obj);
    void observationLoop(const systemLoopHandler<int> *obj);
    void advancedControllerLoop(const systemLoopHandler<int> *obj);
}

#include "systemloophandler.hpp"
#endif // systemLoopHandler_H
