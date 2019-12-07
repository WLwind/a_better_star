#pragma once

#include<vector>
#include<a_better_star/potential_calculator.h>

namespace a_better_star
{

class Traceback {
    public:
        Traceback(PotentialCalculator* p_calc) : p_calc_(p_calc) {}

        virtual bool getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path) = 0;
        virtual void setSize(int xs, int ys) {
            xs_ = xs;
            ys_ = ys;
        }
        inline int getIndex(int x, int y) {
            return x + y * xs_;
        }
        void setLethalCost(unsigned char lethal_cost) {
            lethal_cost_ = lethal_cost;
        }
    protected:
        int xs_, ys_;
        unsigned char lethal_cost_;
        PotentialCalculator* p_calc_;
};

} //end namespace
