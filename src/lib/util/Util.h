#pragma once


namespace util {
    static double to_rad(double n) {
        return n * M_PI / 180;
    }

    static double to_deg(double n) {
        return n * 180 / M_PI;
    }

    static int sign(double val) {
        return val > 0 ? 1 : -1;
    }

    static double clamp(double val, double min, double max) {
        if(std::abs(val) < min) {
            return sign(val) * min;
        } else if (std::abs(val) > max) {
            return sign(val) * max;
        }
        return val;
    }

    static int dampen(int input) {
        double s = 40;
        double a = .60;
        double v = (127*a-127)/(-s*s+254*s-16129);
        double c = a - 2*v*s;
        double output;
        if (abs(input) < abs(s)) {
            output = a * input;
        }
        else {
            double x = abs(input);
            double y = -(s - x) * (c + v * (s + x)) + a * s;
            output = y * input / abs(input);
        }

        return (int)std::round(output);
    }
}


