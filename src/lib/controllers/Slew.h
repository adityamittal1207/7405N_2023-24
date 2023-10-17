//
// Created by Zayn Rekhi on 10/10/22.
//

#ifndef INC_7405N_2022_2023_SLEW_H
#define INC_7405N_2022_2023_SLEW_H


class Slew {
    private:
        double accel_limit_;
        double decel_limit_;
        double prev_val = 0;

    public:
        Slew(double accel_limit, double decel_limit);

        double limit(double val);
};


#endif //INC_7405N_2022_2023_SLEW_H
