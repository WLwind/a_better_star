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
        int i;//index
        float cost;//f(n)
};

struct greater1 {
        bool operator()(const Index& a, const Index& b) const {
            return a.cost > b.cost;
        }
};

class AStarExpansion : public Expander {
    public:
        AStarExpansion(PotentialCalculator* p_calc, int nx, int ny);
        bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles, float* potential) override;
    private:
        void add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x, int end_y);
        /**
        * @brief Discriminate whether the point is in the open list
        * @param index The index of the point in the grid map
        * @param no The index of the point in queue_ (the open list instance)
        * @return True if the point is in the open list, otherwise false
        */
        bool isInOpen(int index,size_t& no);
        std::vector<Index> queue_;//f(n) of the open list
        std::unique_ptr<a_better_star::FastEuclideanDistance> m_fed_ptr;//to calculate Euclidean distance
        int m_fed_resolution{64};//resolution of fast Euclidean distance
};

} //end namespace
