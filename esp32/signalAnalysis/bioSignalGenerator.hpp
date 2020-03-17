#include "bioSignalGenerator.h"

void ElectroStimulation::bioSignalController::powerControllerInit(const gpio_num_t &pin, const adc1_channel_t &feedbackPin, const uint32_t &freq, const ledc_channel_t &channel, const ledc_timer_t &timer)
{
    gpio_pad_select_gpio((gpio_num_t)pin);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT); 

    adc1_config_width(ADC_WIDTH_BIT_9);
    adc1_config_channel_atten(feedbackPin,ADC_ATTEN_DB_0);
    this->feedbackPin = feedbackPin;
    adc_set_clk_div(2);
    adc1_get_raw(feedbackPin);
    
    SENS.sar_read_ctrl.sar1_dig_force = false;
    SENS.sar_meas_start1.meas1_start_force = false;
    SENS.sar_meas_start1.sar1_en_pad_force = false;
    SENS.sar_touch_ctrl1.xpd_hall_force = false;
    SENS.sar_touch_ctrl1.hall_phase_force = false;
    
    //adc_value = adc_convert( ADC_UNIT_1, channel );

    ledc_timer.duty_resolution = LEDC_TIMER_10_BIT;
    ledc_timer.freq_hz = freq;
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num = timer;
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    ledc_channel.hpoint     = 0;
	ledc_channel.duty       = 0; 
	ledc_channel.channel    = channel;
	ledc_channel.gpio_num   = pin;
	ledc_channel.timer_sel  = timer;
	ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel_config(&ledc_channel);
}

void ElectroStimulation::bioSignalController::setPowerLevel(const double &powerLevel)
{
    ledc_channel.duty       = (uint16_t)((powerLevel)*1024/100); 
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, ledc_channel.duty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}

void ElectroStimulation::bioSignalController::setOutputHandlerDirectPin(const gpio_num_t &outputHandlerDirectPin)
{
    this->outputHandlerDirectPin = outputHandlerDirectPin;
    gpio_pad_select_gpio(this->outputHandlerDirectPin);
    gpio_set_direction(this->outputHandlerDirectPin, GPIO_MODE_OUTPUT); 
}

void ElectroStimulation::bioSignalController::setOutputHandlerReversePin(const gpio_num_t &outputHandlerReversePin)
{
    this->outputHandlerReversePin = outputHandlerReversePin;
    gpio_pad_select_gpio(this->outputHandlerReversePin);
    gpio_set_direction(this->outputHandlerReversePin, GPIO_MODE_OUTPUT); 
}

void ElectroStimulation::bioSignalController::addSignalBehavior(const std::string &signalBehaviorName, const double &signalBehavior)
{
    this->signalBehaviorHandler.emplace(signalBehaviorName, signalBehavior);
}

void ElectroStimulation::bioSignalController::removeSignalBehavior(const std::string &signalBehaviorName)
{
    this->signalBehaviorHandler.erase (signalBehaviorName);
}

double ElectroStimulation::bioSignalController::getSignalBehavior(const std::string &signalBehavior) const
{
    return this->signalBehaviorHandler.find(signalBehavior)->second;                        
}

void ElectroStimulation::circuitTransferFunctionIdentification(void* pvParameter)
{
    bioSignalController signalHandler = *((bioSignalController*) pvParameter);
    double minLimit = 40, maxLimit = 55;
    uint32_t time = 100;
    //ModelHandler::ARX<double> boost(1,1);
    //OptimizationHandler::RecursiveLeastSquare<double> rls(&boost); 
    
    while(1){
        for(unsigned i = 0; i < 100000; ++i)
        {
            for(unsigned i = 0; i < 10; ++i)
            {
                //signalHandler.setPowerLevel(minLimit);
                ets_delay_us(time);
                //rls.optimize(minLimit, signalHandler.getFeedbackForPowerControl());
            }
            for(unsigned i = 0; i < 10; ++i)
            {
                //signalHandler.setPowerLevel(maxLimit);
                ets_delay_us(time);
                //rls.optimize(minLimit, signalHandler.getFeedbackForPowerControl());
            }
        }
       // std::cout << rls.print();
    }
}

void ElectroStimulation::burstController(void* pvParameter)
{
    bioSignalController signalHandler = *((bioSignalController*) pvParameter);  
    ControlHandler::PID<double> pid("1.0,0.33,0.0");
    pid.setLimits(60.0,0.0);
    double reference = signalHandler.getSignalBehavior("ccLevel");

    while(1){
        int y = 60*signalHandler.getFeedbackForPowerControl()/0.9;
        signalHandler.setPowerLevel(pid.OutputControl(reference,y));
        //signalHandler.setPowerLevel(signalHandler.getSignalBehavior("ccLevel"));
        for(int i = 0; i<10; i++){ 
            gpio_set_level(signalHandler.getOutputHandlerDirectPin(), 0);
            ets_delay_us(signalHandler.getSignalBehavior("period"));
            gpio_set_level(signalHandler.getOutputHandlerDirectPin(), 1);
            ets_delay_us(10000-signalHandler.getSignalBehavior("period"));
        }
        ets_delay_us((1000000/signalHandler.getSignalBehavior("freq"))-100000);
    }
}

void ElectroStimulation::openLoopNormalController(void* pvParameter)
{
    bioSignalController *signalHandler = ((bioSignalController*) pvParameter);
    double reference = signalHandler->getSignalBehavior("ccLevel");
    uint32_t time = (1000000/signalHandler->getSignalBehavior("freq"))-signalHandler->getSignalBehavior("period");
    
    while(1){
        signalHandler->setPowerLevel(reference);
        for(uint_fast8_t i = 0; i < 10; ++i)
        {
            gpio_set_level(signalHandler->getOutputHandlerDirectPin(), 0);
            ets_delay_us(signalHandler->getSignalBehavior("period"));
            gpio_set_level(signalHandler->getOutputHandlerDirectPin(), 1);
            ets_delay_us(time);
        }
    }
}

void ElectroStimulation::closedLoopNormalController(void* pvParameter)//verificar a estabilidade para vários canais, se pvParameter é diferente
{
    bioSignalController signalHandler = *((bioSignalController*) pvParameter);
    ControlHandler::PID<double> pid("1.00,0.13,0.0");
    pid.setLimits(0.0,60.0);
    double reference = signalHandler.getSignalBehavior("ccLevel");
    uint32_t time = (1000000/signalHandler.getSignalBehavior("freq"))-signalHandler.getSignalBehavior("period");
    
    while(1){
        double y = 60*signalHandler.getFeedbackForPowerControl();
        double u = pid.OutputControl(reference,y);
        signalHandler.setPowerLevel(u);
        signalHandler.setPowerLevel(reference);
        for(uint_fast8_t i = 0; i < 10; ++i)
        {
            gpio_set_level(signalHandler.getOutputHandlerDirectPin(), 0);
            ets_delay_us(signalHandler.getSignalBehavior("period"));
            gpio_set_level(signalHandler.getOutputHandlerDirectPin(), 1);
            ets_delay_us(time);
        }
        //std::cout << reference << ", " << y << ", " << u << std::endl;
    }
}

void ElectroStimulation::twoFaseNormalController(void* pvParameter)
{
    bioSignalController signalHandler = *((bioSignalController*) pvParameter);
    double reference = signalHandler.getSignalBehavior("ccLevel");
    uint32_t time = (1000000/signalHandler.getSignalBehavior("freq"))-signalHandler.getSignalBehavior("period");
    gpio_set_level(signalHandler.getOutputHandlerDirectPin(),  0);
    gpio_set_level(signalHandler.getOutputHandlerReversePin(), 0);

    while(1){
        signalHandler.setPowerLevel(reference);
        for(uint_fast8_t i = 0; i < 10; ++i)
        {
            gpio_set_level(signalHandler.getOutputHandlerReversePin(), 0);
            ets_delay_us(signalHandler.getSignalBehavior("period"));
            gpio_set_level(signalHandler.getOutputHandlerDirectPin(),  1);
            ets_delay_us(time);

            gpio_set_level(signalHandler.getOutputHandlerDirectPin(),  0);
            ets_delay_us(signalHandler.getSignalBehavior("period"));
            gpio_set_level(signalHandler.getOutputHandlerReversePin(), 1);
            ets_delay_us(time);
        }
    }
}

void ElectroStimulation::closedLoopTwoFaseNormalController(void* pvParameter)
{
    bioSignalController signalHandler = *((bioSignalController*) pvParameter);
    ControlHandler::PID<double> pid("1.00,0.13,0.0");
    pid.setLimits(0.0,60.0);
    double reference = signalHandler.getSignalBehavior("ccLevel");
    uint32_t time = (1000000/signalHandler.getSignalBehavior("freq"))-signalHandler.getSignalBehavior("period");
    gpio_set_level(signalHandler.getOutputHandlerDirectPin(),  0);
    gpio_set_level(signalHandler.getOutputHandlerReversePin(), 0);

    while(1){
        double y = 60*signalHandler.getFeedbackForPowerControl();
        double u = pid.OutputControl(reference,y);
        signalHandler.setPowerLevel(u);
        signalHandler.setPowerLevel(reference);
        for(uint_fast8_t i = 0; i < 10; ++i)
        {
            gpio_set_level(signalHandler.getOutputHandlerReversePin(), 0);
            ets_delay_us(signalHandler.getSignalBehavior("period"));
            gpio_set_level(signalHandler.getOutputHandlerDirectPin(),  1);
            ets_delay_us(time);

            gpio_set_level(signalHandler.getOutputHandlerDirectPin(),  0);
            ets_delay_us(signalHandler.getSignalBehavior("period"));
            gpio_set_level(signalHandler.getOutputHandlerReversePin(), 1);
            ets_delay_us(time);
        }
        //std::cout << reference << ", " << y << ", " << u << std::endl;
    }
}

void ElectroStimulation::modulationController(void* pvParameter)
{
    bioSignalController signalHandler = *((bioSignalController*) pvParameter);
    
    uint16_t time;
    
    while(1){
        signalHandler.setPowerLevel(signalHandler.getSignalBehavior("ccLevel"));
        time = (1000000/signalHandler.getSignalBehavior("freq"));
        for(int i = 0; i < (500000/time); i++ ){
            gpio_set_level(signalHandler.getOutputHandlerDirectPin(), 0);
            ets_delay_us(time/2); 
            gpio_set_level(signalHandler.getOutputHandlerDirectPin(), 1);
            ets_delay_us(time/2);
        }
        time = (1000000/(signalHandler.getSignalBehavior("freq")/2));
        for(int i = 0; i < (500000/time); i++ ){
            gpio_set_level(signalHandler.getOutputHandlerDirectPin(), 0);
            ets_delay_us(time/2); 
            gpio_set_level(signalHandler.getOutputHandlerDirectPin(), 1);
            ets_delay_us(time/2); 
        }
    }
}

void ElectroStimulation::sd1Controller(void* pvParameter)
{
    bioSignalController signalHandler = *((bioSignalController*) pvParameter);

    uint16_t time, value, valDecay;
    double K = 4/50;
    
    while(1){
        time = (1000000/signalHandler.getSignalBehavior("freq"));
        // duty = signalHandler.getSignalBehavior("period");
        value = signalHandler.getSignalBehavior("ccLevel");
        valDecay = K*(signalHandler.getSignalBehavior("ccLevel")/signalHandler.getSignalBehavior("freq"));
        // dutyDecay = K*(signalHandler.getSignalBehavior("period")/signalHandler.getSignalBehavior("freq"));
        gpio_set_level(signalHandler.getOutputHandlerDirectPin(), 0);
        signalHandler.setPowerLevel(value);
        for(uint32_t i = 0; i < (uint32_t)(5000000/time); ++i){
            //gpio_set_level((gpio_num_t) signalHandler.getOutputHandlerPin(), 0);
            //ets_delay_us(duty += dutyDecay);
            //gpio_set_level((gpio_num_t) signalHandler.getOutputHandlerPin(), 1);
            //ets_delay_us(time-duty);
            signalHandler.setPowerLevel(value -= valDecay);
            ets_delay_us(time);
        }
        for(uint32_t i = 0; i < (uint32_t)(5000000/time); ++i){
            //gpio_set_level((gpio_num_t) signalHandler.getOutputHandlerPin(), 0);
            //ets_delay_us(duty -= dutyDecay);
            //gpio_set_level((gpio_num_t) signalHandler.getOutputHandlerPin(), 1);
            //ets_delay_us(time-duty);
            signalHandler.setPowerLevel(value += valDecay);
            ets_delay_us(time);
        }
    }
}

void ElectroStimulation::sd2Controller(void* pvParameter)
{
    bioSignalController signalHandler = *((bioSignalController*) pvParameter);

    uint16_t time, duty, value, valDecay, dutyDecay;
    
    while(1){
        time = (1000000/signalHandler.getSignalBehavior("freq"));
        duty = signalHandler.getSignalBehavior("period");
        value = signalHandler.getSignalBehavior("ccLevel");
        valDecay = (7*signalHandler.getSignalBehavior("ccLevel"))/(50*signalHandler.getSignalBehavior("freq"));
        dutyDecay = (7*signalHandler.getSignalBehavior("period"))/(50*signalHandler.getSignalBehavior("freq"));

        signalHandler.setPowerLevel(value);
        for(int i = 0; i < (5000000/time); i++){
            gpio_set_level(signalHandler.getOutputHandlerDirectPin(), 0);
            ets_delay_us(duty);
            gpio_set_level(signalHandler.getOutputHandlerDirectPin(), 1);
            ets_delay_us(time-duty);
            duty += dutyDecay;
            value -= valDecay;
            signalHandler.setPowerLevel(value);
        }
        for(int i = 0; i < (5000000/time); i++){
            gpio_set_level(signalHandler.getOutputHandlerDirectPin(), 0);
            ets_delay_us(duty);
            gpio_set_level(signalHandler.getOutputHandlerDirectPin(), 1);
            ets_delay_us(time-duty);
            duty -= dutyDecay;
            value += valDecay;
            signalHandler.setPowerLevel(value);
        }
    }
}