#include "systemLoop.h"

template <typename Type>
ControlHandler::systemLoopHandler<Type>::systemLoopHandler(){
    iterator = 0;
    in = NULL;
    out = NULL;
    reactionCurveReference = 0;
    controllerReference    = 0;
    simpleLoopValue     = 0;
    relayReference      = 0;
    timer_idx           = TIMER_0;
    timer_group         = TIMER_GROUP_0; /*!< Test on timer group 0 */
    config.alarm_en     = 1;
    config.auto_reload  = 1;
    config.counter_dir  = TIMER_COUNT_UP;
    //config.divider      = 4000;/*!< Hardware timer clock divider, 80 to get 1MHz clock to timer */
    config.divider      = 4000;/*!< Hardware timer clock divider, 80 to get 1MHz clock to timer */
    config.intr_type    = TIMER_INTR_LEVEL; /*!< Timer level interrupt */
    config.counter_en   = TIMER_PAUSE;
    TIMER_SCALE         = (TIMER_BASE_CLK / config.divider);  /*!< used to calculate counter value */
    TIMER_FINE_ADJ      =  (0*(TIMER_BASE_CLK / config.divider)/1000000); /*!< used to compensate alarm value */
    TIMER_INTERVAL0_SEC = (0.0001);   /*!< test interval for timer 0 */
    maxIterator = 900;
    operationalInput  = 0;
    operationalOutput = 0;
    startIterator = true;
}

template <typename Type>
void ControlHandler::systemLoopHandler<Type>::startLoop(void (*loopFunction2Call)(void*)){
    /*Configure timer*/
    timer_init(timer_group, timer_idx, &config);
    /*Stop timer this*/
    timer_pause(timer_group, timer_idx);
    /*Load counter value */
    timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
    /*Set alarm value*/
    timer_set_alarm_value(timer_group, timer_idx, (TIMER_INTERVAL0_SEC * TIMER_SCALE) - TIMER_FINE_ADJ);
    /*Enable timer interrupt*/
    timer_enable_intr(timer_group, timer_idx);
    /*Set ISR handler*/
    timer_isr_register(timer_group, timer_idx, loopFunction2Call, (void*) this, ESP_INTR_FLAG_IRAM, NULL);
    /*Start timer counter*/
    timer_start(timer_group, timer_idx);
}

template <typename Type>
void ControlHandler::systemLoopHandler<Type>::stopLoop(){
    timer_pause(timer_group, timer_idx);
    timer_group_intr_disable( timer_group, TIMG_T0_INT_ENA_M);
    timer_disable_intr(timer_group, timer_idx);
}

template <typename Type>
void ControlHandler::systemLoopHandler<Type>::pauseLoop(){
    // timer_pause(timer_group, timer_idx);
    startIterator = false;
}

template <typename Type>
void ControlHandler::systemLoopHandler<Type>::resumeLoop(){
    // timer_start(timer_group, timer_idx);
    startIterator = true;
}

void IRAM_ATTR ControlHandler::systemLoop(void *para){// timer group 0, ISR
    ControlHandler::systemLoopHandler<double> *idStructure = ((ControlHandler::systemLoopHandler<double>*) para);
    timer_idx_t timer_idx = idStructure->timer_idx;
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    if((intr_status & BIT(timer_idx)) ) {
        TIMERG0.hw_timer[timer_idx].update = 1;
        TIMERG0.int_clr_timers.t0 = 1;
        TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;

        if(idStructure->startIterator)
            switch (idStructure->loopHandler)
            {
            case SIMPLELOOP:
                ControlHandler::simpleLoop(idStructure);
                break;

            case RELAY:
                ControlHandler::systemExitationforRelayLoop(idStructure);
                break;

            case CONTROL:
                ControlHandler::systemControlLoop(idStructure);
                break;

            case ANGLECONTROL:
                ControlHandler::angleControlLoop(idStructure);

            default:
                break;
            }
    }
}

template <typename Type>
void ControlHandler::simpleLoop(ControlHandler::systemLoopHandler<Type> *idStructure){// timer group 0, ISR
    if(idStructure->iterator < idStructure->maxIterator){
        idStructure->signal[idStructure->channel]->setPowerLevel(idStructure->simpleLoopValue);
        idStructure->in[idStructure->iterator]  = idStructure->simpleLoopValue;
        idStructure->out[idStructure->iterator] = idStructure->signal[idStructure->channel]->getFeedbackForPowerControl();
    
        idStructure->iterator++;
    }     
}

template <typename Type>
void ControlHandler::systemExitationforRelayLoop(ControlHandler::systemLoopHandler<Type> *idStructure){// timer group 0, ISR
    if(idStructure->iterator < idStructure->maxIterator){
        idStructure->signal[idStructure->channel]->setPowerLevel(idStructure->inputSignal);
        idStructure->in [idStructure->iterator] = idStructure->inputSignal - idStructure->operationalInput;
        idStructure->out[idStructure->iterator] = idStructure->signal[idStructure->channel]->getFeedbackForPowerControl() - idStructure->operationalOutput;
        idStructure->error = idStructure->relayReference - idStructure->operationalOutput - idStructure->out[idStructure->iterator];

        if(abs(idStructure->error) > idStructure->tolerance && idStructure->error > 0) idStructure->inputSignal = idStructure->relayMaxLimit;
        if(abs(idStructure->error) > idStructure->tolerance && idStructure->error < 0) idStructure->inputSignal = idStructure->relayMinLimit;
        if(abs(idStructure->error) < idStructure->tolerance      && idStructure->inputSignal == idStructure->relayMaxLimit) idStructure->inputSignal = idStructure->relayMaxLimit;
        else if(abs(idStructure->error) < idStructure->tolerance && idStructure->inputSignal == idStructure->relayMinLimit) idStructure->inputSignal = idStructure->relayMinLimit;
        if(idStructure->error == idStructure->tolerance)  idStructure->inputSignal = idStructure->relayMaxLimit;
        if(idStructure->error == -idStructure->tolerance) idStructure->inputSignal = idStructure->relayMinLimit;
    
        idStructure->iterator++;
    }     
}

template <typename Type>
void ControlHandler::systemControlLoop(ControlHandler::systemLoopHandler<Type> *idStructure){// timer group 0, ISR
    if(idStructure->iterator < idStructure->maxIterator){
        idStructure->out[idStructure->iterator] = idStructure->signal[0]->getFeedbackForPowerControl() - idStructure->operationalOutput;
        idStructure->in[idStructure->iterator]  = idStructure->pid[0]->OutputControl( idStructure->controllerReference, idStructure->out[idStructure->iterator]);
        idStructure->signal[0]->setPowerLevel( idStructure->in[idStructure->iterator] + idStructure->operationalInput);
    
        idStructure->iterator++;
    }  
}

template <typename Type>
void ControlHandler::normalController(ControlHandler::systemLoopHandler<Type> *idStructure, uint_fast8_t repetition)//verificar a estabilidade para vários canais, se pvParameter é diferente
{
    //uint32_t time = (10000/idStructure->signal[idStructure->channel]->getSignalBehavior("freq")) - idStructure->signal[idStructure->channel]->getSignalBehavior("period");
    uint32_t time1 = idStructure->signal[idStructure->channel]->getSignalBehavior("period"), time2 = idStructure->signal[idStructure->channel]->getSignalBehavior("freq");
    //std::cout << "entrou Normal\n" << "Tempo 1: " << time1 << "  Tempo 2: "<< time2 << "\n";
    for(uint_fast8_t i = 0; i < repetition; ++i)
    {
        gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerReversePin(), 0);
        ets_delay_us(time1);
        gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerDirectPin(),  1);
        ets_delay_us(time2);

        gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerDirectPin(),  0);
        ets_delay_us(time1);
        gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerReversePin(), 1);
        ets_delay_us(time2);
    }
    //std::cout << "Saiu Normal\n" << "Iterador = " << idStructure->iterator << "\n";
}

template <typename Type>
void ControlHandler::wifiSend(ControlHandler::systemLoopHandler<Type> *idStructure)//verificar a estabilidade para vários canais, se pvParameter é diferente
{
    uint16_t counter = 0;

    for(uint16_t k = 0; k < idStructure->iterator/50; ++k){
        std::stringstream ss;
        for(uint16_t j = 0; j < 50; ++j){
            if(counter > idStructure->maxIterator){
                k = idStructure->maxIterator/50; break;
            }
            ss << idStructure->in[counter] << ",   " << idStructure->out[counter] << ";\n"; 
            counter++;
        }
        idStructure->wifi << ss.str();
        ets_delay_us(500000);
    }
    idStructure->iterator = 0;
}

template <typename Type>
void ControlHandler::reTune(systemLoopHandler<Type> *idStructure)
{
    //std::cout << "Dados de Treinamento \n" << idStructure->operationalPointIterator << "  " << idStructure->iterator << "  \n";
    for(unsigned j = idStructure->operationalPointIterator; j < idStructure->iterator; ++j){
        idStructure->rls[idStructure->channel]->optimize(idStructure->in[j], idStructure->out[j]);
    }

    LinAlg::Matrix<double> FOP = d2cConversion(idStructure->boost[idStructure->channel][0]); //etapa de sintonia 
    double sampleTime = idStructure->pid[idStructure->channel]->getSampleTime();
    FOP(0,2) = sampleTime*idStructure->controllerSensibility;
    switch (idStructure->controller)
    {
    case 0:
        idStructure->pid[idStructure->channel][0] = ControlHandler::controllerTuning(FOP,"P", ControlHandler::tune[idStructure->tuningMethod]); idStructure->pid[idStructure->channel]->setSampleTime(sampleTime);
        break;
    case 1:
        idStructure->pid[idStructure->channel][0] = ControlHandler::controllerTuning(FOP,"PI", ControlHandler::tune[idStructure->tuningMethod]); idStructure->pid[idStructure->channel]->setSampleTime(sampleTime);
        break;
    case 2:
        idStructure->pid[idStructure->channel][0] = ControlHandler::controllerTuning(FOP,"PID", ControlHandler::tune[idStructure->tuningMethod]); idStructure->pid[idStructure->channel]->setSampleTime(sampleTime);
        break;
    
    default:
        break;
    }
    
    std::cout << "Parametros PID:  " << idStructure->pid[idStructure->channel]->getParams() << " Periodo de Amostragem: " <<  idStructure->pid[idStructure->channel]->getSampleTime()  << std::endl;
    std::cout << "Parametors do Modelo: " << idStructure->boost[idStructure->channel]->print() << std::endl;
    std::cout << "Parametros da Funcao Continua: "<< (double)FOP(0,0) << " " << (double)FOP(0,1) << " " << (double)FOP(0,2) << std::endl;

    //idStructure->wifi << idStructure->boost[idStructure->channel]->print();
    // std::stringstream ss; ss << std::setw(2*5+1) << std::setprecision(5) << std::fixed;
    //ss << "Parametros da Funcao Continua: "<< (double)FOP(0,0) << " " << (double)FOP(0,1) << " " << (double)FOP(0,2) << std::endl;
    //idStructure->wifi << ss.str();
}


void ControlHandler::relayExitationLoop(void* pvParameter)
{
    ControlHandler::systemLoopHandler<double> *idStructure = ((ControlHandler::systemLoopHandler<double>*) pvParameter);
    idStructure->wifi << "N";
    //std::stringstream ss;
    // idStructure->in = new int16_t[idStructure->maxIterator]; idStructure->out = new int16_t[idStructure->maxIterator];
    idStructure->in = new double[idStructure->maxIterator]; idStructure->out = new double[idStructure->maxIterator];
    idStructure->iterator = 0;
    
    idStructure->simpleLoopValue = idStructure->reactionCurveReference;
    idStructure->loopHandler = SIMPLELOOP;
    idStructure->startLoop(ControlHandler::systemLoop); // esse primeiro loop objetiva estabilizar a saída no ponto de operação
    adc1_get_raw((adc1_channel_t)idStructure->channel);
    ControlHandler::normalController(idStructure,60);
    idStructure->operationalPointIterator = idStructure->iterator - 1;
    idStructure->operationalInput  =  idStructure->in [idStructure->operationalPointIterator]; 
    idStructure->operationalOutput =  idStructure->out[idStructure->operationalPointIterator]; 

    std::cout << "Ponto de Operacao: \n Entrada/Saida" << idStructure->operationalInput << "  "<< idStructure->operationalOutput << std::endl;

    idStructure->loopHandler = RELAY;
    //idStructure->relayReference = idStructure->signal[0]->getSignalBehavior("ccLevel") ;
    ControlHandler::normalController(idStructure,40);
    idStructure->signal[idStructure->channel]->setPowerLevel(0);
    gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerDirectPin(),  0);  gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerReversePin(), 0);
    idStructure->pauseLoop();

    ControlHandler::reTune(idStructure);
    ControlHandler::wifiSend(idStructure);
    
    idStructure->loopHandler = CONTROL;
    idStructure->resumeLoop(); // Etapa de controle
    adc1_get_raw((adc1_channel_t)idStructure->channel); idStructure->controllerReference = idStructure->signal[idStructure->channel]->getSignalBehavior("ccLevel") - idStructure->operationalOutput;
    ControlHandler::normalController(idStructure,120);
    idStructure->stopLoop();
    idStructure->signal[idStructure->channel]->setPowerLevel(0);
    gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerDirectPin(),  0);
    gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerReversePin(), 0);

    ControlHandler::wifiSend(idStructure);
     
                                                                       
    if(idStructure->xHandle[idStructure->channel] != NULL){
        std::cout << "EntrouS 1\n";
        delete idStructure->in; delete idStructure->out; 
        std::cout << "EntrouS 2\n";
        idStructure->in = NULL; idStructure->out = NULL;
        std::cout << "EntrouS 3\n";
		vTaskDelete(idStructure->xHandle[idStructure->channel]);
        std::cout << "EntrouS 4\n";
    }
}

void ControlHandler::squaredWaveExitationLoop(void* pvParameter)
{
    ControlHandler::systemLoopHandler<uint32_t> *idStructure = ((ControlHandler::systemLoopHandler<uint32_t>*) pvParameter);
    uint32_t minLimit = idStructure->relayMinLimit, maxLimit = idStructure->relayMaxLimit;
    idStructure->iterator = 0;
    std::cout << "Entrou 1\n";
    idStructure->loopHandler = SIMPLELOOP;
    idStructure->startLoop(ControlHandler::systemLoop);
    adc1_get_raw((adc1_channel_t)idStructure->channel); // Criar função ADCReconfigure dentro do biosignalgenerator
    for(uint_fast8_t j = 0; j < 10; ++j){
        idStructure->simpleLoopValue = minLimit;
        ets_delay_us(30000);

        idStructure->simpleLoopValue = maxLimit;
        ets_delay_us(30000);
    }
    idStructure->stopLoop();
    

    std::cout << "entrou 2\n";
    std::stringstream ss; ss << std::setw(2*5+1) << std::setprecision(5) << std::fixed << "\nEntrada | Saida \n";
    std::cout << "entrou 3\n";
    
    std::cout << "entrou 5\n";

    for(unsigned j = 0; j < idStructure->iterator; ++j){
        idStructure->rls[idStructure->channel]->optimize(idStructure->in[j], idStructure->out[j]);
    }
    ControlHandler::wifiSend(idStructure);
    idStructure->wifi << idStructure->boost[idStructure->channel]->print();

    if(idStructure->xHandle[idStructure->channel] != NULL){
        std::cout << "EntrouS 1\n";
        delete idStructure->in; delete idStructure->out; 
        std::cout << "EntrouS 2\n";
        idStructure->in = NULL; idStructure->out = NULL;
        std::cout << "EntrouS 3\n";
		vTaskDelete(idStructure->xHandle[idStructure->channel]);
        std::cout << "EntrouS 4\n";
    }
}
uint16_t iteratorGambiarra = 0;
template <typename Type>
void ControlHandler::angleControlLoop(ControlHandler::systemLoopHandler<Type> *idStructure){// timer group 0, ISR
    iteratorGambiarra++;
    if(iteratorGambiarra > 2000){
        idStructure->accel.read();
        idStructure->out[idStructure->iterator] = idStructure->accel.get_filtered_x();
        idStructure->in [idStructure->iterator] = idStructure->pid[0]->OutputControl(
                                                                            idStructure->controllerReference, 
                                                                            idStructure->out[idStructure->iterator]);
        idStructure->signal[idStructure->channel]->setPowerLevel(idStructure->in[idStructure->iterator]);

        iteratorGambiarra = 0;
    }
}

void ControlHandler::LimiarTest(void* pvParameter)
{
    ControlHandler::systemLoopHandler<double> *idStructure = ((ControlHandler::systemLoopHandler<double>*) pvParameter);
    
    idStructure->signal[idStructure->channel]->setPowerLevel(idStructure->signal[idStructure->channel]->getSignalBehavior("ccLevel"));
    while(true){
        if(idStructure->signal[idStructure->channel]->getSignalBehavior("ccLevel") != 0)
            ControlHandler::normalController(idStructure,100);
        else{
            gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerDirectPin(),  0);
            gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerReversePin(), 0); 
        }
    }
}

template <typename Type>
void ControlHandler::closedLoopNormalController(ControlHandler::systemLoopHandler<Type> *idStructure, uint_fast8_t repetition)//verificar a estabilidade para vários canais, se pvParameter é diferente
{
    uint32_t time1 = idStructure->signal[idStructure->channel]->getSignalBehavior("period"), time2 = idStructure->signal[idStructure->channel]->getSignalBehavior("freq");
    gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerDirectPin(),  0);
    gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerReversePin(), 0);

    for(uint_fast16_t j = 0; j < idStructure->maxIterator; ++j){
        idStructure->accel.read();
        idStructure->out[j] = idStructure->accel.get_filtered_x();
        idStructure->in [j] = idStructure->pid[idStructure->channel]->OutputControl(idStructure->controllerReference,idStructure->out[j]);
        idStructure->signal[idStructure->channel]->setPowerLevel(idStructure->in [j]);
        //std::cout << "R: " << idStructure->controllerReference << "O: " << idStructure->out[j] << "I: " << idStructure->in [j] <<"\n";
        gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerReversePin(), 0);
        ets_delay_us(time1);
        gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerDirectPin(),  1);
        ets_delay_us(time2);

        gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerDirectPin(),  0);
        ets_delay_us(time1);
        gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerReversePin(), 1);
        ets_delay_us(time2);
    }
}

void ControlHandler::angleControlNormal(void* pvParameter)
{
    ControlHandler::systemLoopHandler<double> *idStructure = ((ControlHandler::systemLoopHandler<double>*) pvParameter);
    idStructure->wifi << "N";
    idStructure->in = new double[idStructure->maxIterator]; idStructure->out = new double[idStructure->maxIterator];

    ControlHandler::closedLoopNormalController(idStructure,40);
    idStructure->signal[idStructure->channel]->setPowerLevel(0);
    gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerDirectPin(),  0);
    gpio_set_level(idStructure->signal[idStructure->channel]->getOutputHandlerReversePin(), 0);
    
    idStructure->iterator = idStructure->maxIterator;
    ControlHandler::wifiSend(idStructure);
     
                                                                       
    if(idStructure->xHandle[idStructure->channel] != NULL){
        delete idStructure->in; delete idStructure->out; 
        idStructure->in = NULL; idStructure->out = NULL;
		vTaskDelete(idStructure->xHandle[idStructure->channel]);
    }
}