#pragma once

#include<vector>
#include<a_better_star/traceback.h>

namespace a_better_star
{

class GridPath : public Traceback {
    public:
        GridPath(PotentialCalculator* p_calc): Traceback(p_calc){}
        bool getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path);
};

} //end namespace
