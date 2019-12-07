#include<a_better_star/astar.h>
#include<costmap_2d/cost_values.h>
#include<cmath>

namespace a_better_star
{

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys):Expander(p_calc, xs, ys)
{
}

bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles, float* potential)
{
    queue_.clear();
    int start_i = toIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));

    std::fill(potential, potential + ns_, POT_HIGH);
    potential[start_i] = 0;

    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;

    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];//get the lowest f(n)
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        queue_.pop_back();//put is into the close list

        int i = top.i;
        if (i == goal_i)
            return true;

        add(costs, potential, potential[i], i + 1, end_x, end_y);//4-connected grid squares
        add(costs, potential, potential[i], i - 1, end_x, end_y);
        add(costs, potential, potential[i], i + nx_, end_x, end_y);
        add(costs, potential, potential[i], i - nx_, end_x, end_y);

        cycle++;
    }

    return false;
}

void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x, int end_y)
{
    if (next_i < 0 || next_i >= ns_)//out of map
        return;

    if (potential[next_i] < POT_HIGH)//in open list or close list
        return;

    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))//obstacles
        return;

    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);//g(n)=costs[next_i]+neutral_cost_+prev_potential
    int x = next_i % nx_, y = next_i / nx_;
    float distance = hypot(abs(end_x - x),abs(end_y - y));//Heuristics h(n)

    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));//f(n)=g(n)+h(n)
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}

} //end namespace
