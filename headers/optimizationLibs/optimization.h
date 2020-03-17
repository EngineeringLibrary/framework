#ifndef __OPTIMIZATION_H_INCLUDED
#define __OPTIMIZATION_H_INCLUDED

#ifdef testModel
    #include "../../../headers/modelLibs/model.h"
    #include "../../../headers/modelLibs/arx.h"
#else
    #include "framework/headers/modelLibs/model.h"
    #include "framework/headers/modelLibs/arx.h"
#endif

//Biblioteca incompleta, ainda nao funcional.
namespace OptimizationHandler {
    template <class Type>
    class Optimization
    {
    public:
        Optimization();

        virtual void Optimize()=0;
        virtual void Optimize(LinAlg::Matrix<Type> Input,
                              LinAlg::Matrix<Type> Output)=0;

    protected:
        ModelHandler::Model<Type> *model;
    };
}

#ifdef testModel
    #include "../../../src/optimizationLibs/optimization.hpp"
#else
    #include "framework/src/optimizationLibs/optimization.hpp"
#endif

#endif // OPTIMIZATION_H
