#ifndef ACTUATOR_H
#define ACTUATOR_H
#include "framework/headers/primitiveLibs/LinAlg/linalg.h"

namespace actuatorHandler {
    class Actuator
    {
    public:
        Actuator(){}
        virtual void init() = 0;
        virtual void setNewBehaviour(LinAlg::Matrix<uint16_t> behaviourParameters) = 0;
    };
}
#endif // ACTUATOR_H
