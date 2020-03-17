#ifndef BIOSIGNALGENERATOR_H
#define BIOSIGNALGENERATOR_H

#include <iostream>
#include <time.h>
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include "pid.h"
#include "arx.h"
#include "recursiveLeastSquare.h"
#include "esp_system.h"
#include "rom/ets_sys.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "map"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "soc/sens_reg.h"
#include <soc/sens_struct.h>
#include "soc/rtc.h"
#include "soc/syscon_struct.h"

namespace ElectroStimulation{
    class bioSignalController
    {
    public:
        bioSignalController(){output = 0;}
        void powerControllerInit(const gpio_num_t &pin, const adc1_channel_t &feedbackPin, const uint32_t &freq, const ledc_channel_t &channel, const ledc_timer_t &timer);
        void setPowerLevel(const double &powerLevel);
        void setOutputHandlerDirectPin(const gpio_num_t &outputHandlerDirectPin);
        void setOutputHandlerReversePin(const gpio_num_t &outputHandlerReversePin);
        gpio_num_t getOutputHandlerDirectPin () const {return outputHandlerDirectPin;}
        gpio_num_t getOutputHandlerReversePin () const {return outputHandlerReversePin;}

        void addSignalBehavior(const std::string &signalBehaviorName, const double &signalBehavior);
        void removeSignalBehavior(const std::string &signalBehaviorName);
        double getSignalBehavior(const std::string &signalBehavior) const;
        uint32_t getFeedbackForPowerControl() {SENS.sar_meas_start1.sar1_en_pad = (1 << ADC1_CHANNEL_0); // only one channel is selected
                                             while (SENS.sar_slave_addr1.meas_status != 0);
                                                    SENS.sar_meas_start1.meas1_start_sar = 0;
                                                    SENS.sar_meas_start1.meas1_start_sar = 1;
                                                    while (SENS.sar_meas_start1.meas1_done_sar == 0);
                                                    output +=  0.2*(SENS.sar_meas_start1.meas1_data_sar-output);
                                                    output2 +=  0.2*(output-output2);
                                                    return output2;}

    private:

        ledc_channel_config_t ledc_channel;
        ledc_timer_config_t ledc_timer;
        long double output, output2;
        gpio_num_t outputHandlerDirectPin, outputHandlerReversePin; adc1_channel_t feedbackPin;
        std::map<std::string, double> signalBehaviorHandler;
        void adc_i2s_init(void);
    };

    static void circuitTransferFunctionIdentification(void*);
    static void burstController(void*);
    static void openLoopNormalController(void*);
    static void closedLoopNormalController(void*);
    static void twoFaseNormalController(void*);
    static void closedLoopTwoFaseNormalController(void*);
    static void modulationController(void*);
    static void sd1Controller(void*);
    static void sd2Controller(void*);
    void IRAM_ATTR controlLoop(void*);
}

#include "bioSignalGenerator.hpp"
#endif // BIOSIGNALGENERATOR_H
