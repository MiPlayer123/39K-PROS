#include "PurePursuit/util/mathUtil.hpp"

#include "okapi/api/util/mathUtil.hpp"

#include <cmath>

double constrainAnglePi(double radians) {
    return radians - 2 * okapi::pi * floor((radians + okapi::pi) / (2 * okapi::pi));
}

double constrainAngle2Pi(double radians) {
    return radians - 2 * okapi::pi * floor(radians / (2 * okapi::pi));
}

double constrainAngle180(double degrees) {
    return degrees - 360 * floor((degrees + 180) / 360);
}

double constrainAngle360(double degrees) {
    return degrees - 360 * floor(degrees / 360);
}