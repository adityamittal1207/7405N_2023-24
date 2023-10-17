#include "TBH.h"

TBH::TBH(double _gain) {
    gain = _gain;
}

double TBH::get_value(double error) {
    output += gain * error;

    if(output > 12000) output = 12000;
    else if (output < 0) output = 0;

    if (std::signbit(error) != std::signbit(prev_error)) {
        output = 0.5 * (output + tbh);
        tbh = output;
    }

    prev_error = error;
    return output;
}

void TBH::reset() {
    tbh = 0;
    prev_error = 0;
}