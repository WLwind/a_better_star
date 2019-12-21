#include <a_better_star/fast_euclidean_distance.h>

namespace a_better_star
{
FastEuclideanDistance::FastEuclideanDistance(int resolution):m_resolution(resolution)
{
    std::cout<<"Constructing FastEuclideanDistance with resolution of "<<m_resolution<<std::endl;
    for (size_t i=0;i<m_resolution;i++)
    {
        m_proportion_lookup.insert(std::pair<double,double>(1.0/m_resolution*i,hypot(1.0/m_resolution*i,1.0)));
    }    
}
}//end namespace
