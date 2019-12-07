#include <a_better_star/grid_path.h>
#include <algorithm>
#include <stdio.h>
namespace a_better_star
{

bool GridPath::getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path) {
    std::pair<float, float> current;
    current.first = end_x;
    current.second = end_y;

    int start_index = getIndex(start_x, start_y);

    path.push_back(current);
    int c = 0;
    int ns = xs_ * ys_;
    
    while (getIndex(current.first, current.second) != start_index) {
        float min_val = 1e10;
        int min_x = 0, min_y = 0;
        for (int xd = -1; xd <= 1; xd++) {
            for (int yd = -1; yd <= 1; yd++) {
                if (xd == 0 && yd == 0)
                    continue;
                int x = current.first + xd, y = current.second + yd;
                int index = getIndex(x, y);
                if (potential[index] < min_val) {
                    min_val = potential[index];
                    min_x = x;
                    min_y = y;
                }
            }
        }
        if (min_x == 0 && min_y == 0)
            return false;
        current.first = min_x;
        current.second = min_y;
        path.push_back(current);
        
        if(c++>ns*4){
            return false;
        }

    }
    return true;
}

} //end namespace
