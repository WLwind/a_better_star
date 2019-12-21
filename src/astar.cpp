#include <a_better_star/astar.h>
#include <costmap_2d/cost_values.h>
#include <cmath>

namespace a_better_star
{

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys):Expander(p_calc, xs, ys)
{
    ros::NodeHandle private_nh("~/FastEuclideanDistance");
    private_nh.param("fast_euclidean_resolution",m_fed_resolution,64);
    m_fed_ptr.reset(new a_better_star::FastEuclideanDistance(m_fed_resolution));
}

bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles, float* potential)
{
    queue_.clear();
    int start_i = toIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));

    std::fill(potential, potential + ns_, POT_HIGH);//initialize all potentials with the biggest number
    potential[start_i] = 0;

    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;

    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];//get the lowest f(n)
        std::pop_heap(queue_.begin(), queue_.end(), greater1());//put the lowest f(n) to the last place and find the second lowest f(n) and put it to the first place
        queue_.pop_back();//remove the last point from the open list (put it into the close list)

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

    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))//obstacles
        return;

    bool new_point_in_open_list=false;
    bool need_to_update=false;
    size_t no;
    if(potential[next_i]<POT_HIGH)//is in open or closed
    {
        if(isInOpen(next_i,no))//is in open
        {
            float new_potential = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);//g(n)=costs[next_i]+neutral_cost_+prev_potential
            if(new_potential<potential[next_i])//new g(n) is lower than the previous one
            {
                need_to_update=true;
                potential[next_i]=new_potential;//update g(n)
            }
        }
    }
    else
    {
        new_point_in_open_list=true;
        potential[next_i]=p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
    }
    if(need_to_update==false&&new_point_in_open_list==false)//no need to calculate h(n) and f(n)
        return;
    int x = next_i % nx_, y = next_i / nx_;
//    float distance = hypot(abs(end_x - x),abs(end_y - y));//Heuristics h(n)=Euclidean distance
    float distance = m_fed_ptr->calculateEuclideanDistance(abs(end_x - x),abs(end_y - y));
    if(distance<0)
        distance=0;
    if(new_point_in_open_list)//add a new element to the open list
    {
        queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));//f(n)=g(n)+h(n)
        std::push_heap(queue_.begin(), queue_.end(), greater1());//add the new point, find the lowest f(n) and put it to the first place
    }
    else if(need_to_update)//update f(n) of an exsted element
    {
        queue_[no].cost=potential[next_i] + distance * neutral_cost_;
        if(!std::is_heap(queue_.begin(), queue_.end(), greater1()))//need to remake the heap
        {
            std::make_heap(queue_.begin(), queue_.end(), greater1());//find the lowest f(n) and put it to the first place
        }
    }
    return;
}

bool AStarExpansion::isInOpen(int index,size_t& no)
{
    size_t size_of_queue=queue_.size();
    for(size_t i=0;i<size_of_queue;i++)
    {
        if(queue_[i].i==index)
        {
            no=i;
            return true;
        }
    }
    return false;
}

} //end namespace
