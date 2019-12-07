#pragma once

#include<vector>
#include<a_better_star/potential_calculator.h>

namespace a_better_star {

class QuadraticCalculator : public PotentialCalculator {
    public:
        QuadraticCalculator(int nx, int ny): PotentialCalculator(nx,ny) {}

        float calculatePotential(float* potential, unsigned char cost, int n, float prev_potential);
};

} //end namespace 
