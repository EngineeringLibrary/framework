#ifndef SENSOR_H
#define SENSOR_H
#include <framework/headers/primitiveLibs/LinAlg/linalg.h>

namespace sensorHandler {
    class Sensor
    {
    public:
        Sensor(){}
        virtual void init() = 0;
        virtual void read() = 0;
        virtual LinAlg::Matrix<uint16_t> getValues() = 0;
    };
}
#endif // SENSOR_H
