#include "Quaternion.h"

//Ensures the quaternion's magnitude is 1.
void Quaternion::normalize() {
    float mag = sqrt(w*w + x*x + y*y + z*z);
    w = w/mag;
    x = x/mag;
    y = y/mag;
    z = z/mag;
}

//Takes this quaternion and multiplies it by a new quaternion
void Quaternion::qMultiply(const Quaternion& dQ) {
    float newW = dQ.w*w - dQ.x*x - dQ.y*y - dQ.z*z;
    float newX = dQ.w*x + dQ.x*w + dQ.y*z - dQ.z*y;
    float newY = dQ.w*y - dQ.x*z + dQ.y*w + dQ.z*x;
    float newZ = dQ.w*z + dQ.x*y - dQ.y*x + dQ.z*w;

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