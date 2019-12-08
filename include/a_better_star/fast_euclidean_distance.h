#pragma once

#include <map>
#include <iostream>
#include <cmath>

namespace a_better_star
{
class FastEuclideanDistance
{
public:
    /**
    * @brief Constructor
    * @param resolution Size of the map for looking up the approximate Euclidean distance
    */
    FastEuclideanDistance(int resolution=128);
    /**
    * @brief Destructor
    */
    virtual ~FastEuclideanDistance(){};
    /**
    * @brief Calculate approximate Euclidean distance
    * @param x Coordinate x
    * @param y Coordinate y
    * @return Approximate Euclidean distance
    */
    template <typename T>
    double calculateEuclideanDistance(T x,T y)
    {
        if(x<0||y<0)
        {
            std::cout<<"x or y is negative."<<std::endl;
            return -3.0;//negative
        }
        double app_prop;//approximate proportion
        bool y_greater_than_x;
        if(y>=x&&y!=0)
        {
            y_greater_than_x=true;
            app_prop=1.0/m_resolution*static_cast<int>(static_cast<double>(x*m_resolution)/y);//1.0/m_resolution*int(x/y*(1.0/m_resolution))
        }
        else if(x>y&&x!=0)
        {
            y_greater_than_x=false;
            app_prop=1.0/m_resolution*static_cast<int>(static_cast<double>(y*m_resolution)/x);
        }
        else if(x==0&&y==0)
        {
            return 0.0;
        }
        else
        {
            std::cout<<"Cannot devide zero!"<<std::endl;
            return -2.0;//divide zero
        }
        if(m_proportion_lookup.count(app_prop)>0)//whether app_prop is in the map
        {
            return m_proportion_lookup[app_prop]*(y_greater_than_x?y:x);
        }
        else if(x==y)
        {
            return y*m_sqr2;
        }
        else
        {
            std::cout<<"The proportion is not correct!"<<std::endl;
            return -1.0;//error
        }
        std::cout<<"Unknown error!"<<std::endl;
        return -4.0;
    }

private:
    std::map<double,double> m_proportion_lookup;//{x/y : y/(x^2+y^2)^0.5}
    int m_resolution;//Size of the std::map
    double m_sqr2{std::sqrt(2.0)};//2^0.5
};

}//end namespace
