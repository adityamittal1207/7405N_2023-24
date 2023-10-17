#include <cmath>

class TBH {
    private:
        double output, gain, tbh, prev_error;

    public:
        TBH(double gain);

        double get_value(double error);
        void reset();
};


