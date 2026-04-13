#include "Quat.h"
#include "math.h"

//Ensures the quaternion's magnitude is 1.
void Quat::normalize() {
    float mag = sqrt(w*w + x*x + y*y + z*z);
    w = w/mag;
    x = x/mag;
    y = y/mag;
    z = z/mag;
}

//Takes this quaternion and multiplies it by a new quaternion
void Quat::qMultiply(const Quat& dQ) {
    float newW = w*dQ.w - x*dQ.x - y*dQ.y - z*dQ.z;
    float newX = w*dQ.x + x*dQ.w + y*dQ.z - z*dQ.y;
    float newY = w*dQ.y - x*dQ.z + y*dQ.w + z*dQ.x;
    float newZ = w*dQ.z + x*dQ.y - y*dQ.x + z*dQ.w;

    w = newW;
    x = newX;
    y = newY;
    z = newZ; //trust me bro its gotta be this way
    /*
    As an explaination, multiplying 2 quaternions applies rotations in order
    Think of quaternions as rotations.
    So, q1 * q2 rotates from origin by q1, then rotates from that point by q2.
    Not commutative!
    */
}

//Scales the quat by a scalar
void Quat::qScale(const float scaleFactor) {
    w = scaleFactor * w;
    x = scaleFactor * x;
    y = scaleFactor * y;
    z = scaleFactor * z;

}

//It's just vector addition but for quaternions
void Quat::qAdd(const Quat& deltaQuaternion) {
    w = w + deltaQuaternion.w;
    x = x + deltaQuaternion.x;
    y = y + deltaQuaternion.y;
    z = z + deltaQuaternion.z;
}

void orientQuaternion(const Quat & wristQuaternion, const Quat & fingerQuaternion) {

    
}
