#ifndef SYSTEMLOOP_H
#define SYSTEMLOOP_H

#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "adxl345.h"
#include "bioSignalGenerator.h"
#include "arx.h"
#include "pid.h"
#include "pidTuning.h"
#include "wifi/wifista.h"

namespace ControlHandler{
    typedef enum {
        SIMPLELOOP = 0, /*!<Hw timer group 0*/
        RELAY = 1, /*!<Hw timer group 1*/
        CONTROL = 2,
        ANGLECONTROL = 3,
        LOOP_MAX,
    } loopHandler_t;


    template <typename Type>
    struct systemLoopHandler{

        systemLoopHandler();
        
        void startLoop(void (*loopFunction2Call)(void*));
        void stopLoop();
        void pauseLoop();
        void resumeLoop();

        volatile int16_t inputSignal, relayMinLimit, relayMaxLimit, channel, tolerance, operationalInput, operationalOutput, reactionCurveReference, simpleLoopValue;
        Type controllerReference, relayReference, error, controllerSensibility;
        adxl345 accel;
        ControlHandler::PID<Type> **pid;
        ModelHandler::ARX<Type> **boost;
        LinAlg::Matrix<Type> controlReferences;
        ElectroStimulation::bioSignalController **signal;
        OptimizationHandler::RecursiveLeastSquare<Type> **rls;
        volatile timer_group_t timer_group;
        volatile timer_idx_t timer_idx;
        timer_config_t config;
        volatile float TIMER_SCALE, TIMER_FINE_ADJ, TIMER_INTERVAL0_SEC;
        double *in, *out;
        volatile uint16_t iterator, maxIterator, operationalPointIterator;
        bool startIterator;
        volatile uint8_t tuningMethod, controller; 
        Communication::WifiSTA wifi;
        TaskHandle_t *xHandle;
        loopHandler_t loopHandler;
    };
    

    static void squaredWaveExitationLoop(void*);

    static void relayExitationLoop(void*);

    static void LimiarTest(void*);

    static void angleControlNormal(void*);
    
    //template <typename Type>
    void IRAM_ATTR systemLoop(void *para);

    template <typename Type>
    inline void systemControlLoop(systemLoopHandler<Type> *idStructure);

    template <typename Type>
    inline void simpleLoop(systemLoopHandler<Type> *idStructure);

    template <typename Type>
    inline void systemExitationforRelayLoop(systemLoopHandler<Type> *idStructure);
    
    template <typename Type>
    inline void normalController (systemLoopHandler<Type> *idStructure, uint_fast8_t repetition);

    template <typename Type>
    inline void closedLoopNormalController (systemLoopHandler<Type> *idStructure, uint_fast8_t repetition);

    template <typename Type>
    inline void wifiSend(systemLoopHandler<Type> *idStructure);

    template <typename Type>
    inline void reTune(systemLoopHandler<Type> *idStructure);

    template <typename Type>
    inline void angleControlLoop(systemLoopHandler<Type> *idStructure);

}

#include "systemLoop.hpp"
#endif // SYSTEMLOOP_H