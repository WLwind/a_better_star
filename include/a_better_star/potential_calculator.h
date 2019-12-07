#pragma once

namespace a_better_star
{

class PotentialCalculator {
    public:
        PotentialCalculator(int nx, int ny) {
            setSize(nx, ny);
        }

        virtual float calculatePotential(float* potential, unsigned char cost, int n, float prev_potential=-1){
            if(prev_potential < 0){
                // get min of neighbors
                float min_h = std::min( potential[n - 1], potential[n + 1] ),
                      min_v = std::min( potential[n - nx_], potential[n + nx_]);
                prev_potential = std::min(min_h, min_v);
            }

            return prev_potential + cost;
        }

        /**
         * @brief  Sets or resets the size of the map
         * @param nx The x size of the map
         * @param ny The y size of the map
         */
        virtual void setSize(int nx, int ny) {
            nx_ = nx;
            ny_ = ny;
            ns_ = nx * ny;
        } /**< sets or resets the size of the map */

    protected:
        inline int toIndex(int x, int y) {
            return x + nx_ * y;
        }

        int nx_, ny_, ns_; /**< size of grid, in pixels */
};

} //end namespace
