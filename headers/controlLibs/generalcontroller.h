#ifndef GENERALCONTROLLER_H
#define GENERALCONTROLLER_H
#include "framework/headers/primitiveLibs/LinAlg/linalg.h"

namespace ControlHandler {
    template <typename Type>
    class GeneralController
    {
    public:
        GeneralController(){}

        LinAlg::Matrix<Type> getLimits() const = 0;
        LinAlg::Matrix<Type> getParams() const = 0;
        Type getSampleTime() const {return this->sampleTime;}

        void setLimits(const LinAlg::Matrix<Type> &upperLimits, const LinAlg::Matrix<Type> &lowerLimits) = 0;
        void setTuneParameters(const LinAlg::Matrix<Type> &tuneParameters) = 0;
        void setSampleTime(Type sampletime){this->sampleTime = sampletime;}

        LinAlg::Matrix<Type> getOutputControlAction(const LinAlg::Matrix<Type> &controllerInput,
                                                    const LinAlg::Matrix<Type> &systemOutput,
                                                    const LinAlg::Matrix<Type> &systemStates = 0 ) = 0;

        std::string print() = 0;
    private:
        Type sampleTime;
        LinAlg::Matrix<Type> controllerOutput;
    };
}

#endif // GENERALCONTROLLER_H
