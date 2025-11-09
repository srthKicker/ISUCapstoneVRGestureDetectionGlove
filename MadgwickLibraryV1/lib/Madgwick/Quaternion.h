#ifndef QUATERNION_H
#define QUATERNION_H
#include <Arduino.h>

//A quaternion stores 4 ints (wxyz) and allows user to normalize and multiply
class Quaternion { 
    public:
        float w;
        float x;
        float y;
        float z;
        //By default, starts facing forward, will change later
        Quaternion() : w(1), x(0), y(0), z(0) {} 

        Quaternion(float Winit, float Xinit, float Yinit, float Zinit)
        : w(Winit), x(Xinit), y(Yinit), z(Zinit) {}
        void qMultiply(const Quaternion& deltaQuaternion);
        void normalize();
};
#endif