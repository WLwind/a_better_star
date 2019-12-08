#pragma once

#include <a_better_star/planner_core.h>
#include <a_better_star/expander.h>
#include <vector>
#include <algorithm>
#include <a_better_star/fast_euclidean_distance.h>

namespace a_better_star
{
class Index {
    public:
        Index(int a, float b) {
            i = a;
            cost = b;
        }
        int i;
        float cost;
};

struct greater1 {
        bool operator()(const Index& a, const Index& b) const {
            return a.cost > b.cost;
        }
};

class AStarExpansion : public Expander {
    public:
        AStarExpansion(PotentialCalculator* p_calc, int nx, int ny);
        bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles, float* potential);
    private:
        void add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x, int end_y);
        std::vector<Index> queue_;//f(n)
        FastEuclideanDistance m_fed{128};//to calculate Euclidean distance
};

} //end namespace
