#include <iostream>
#include <chrono>

#include <pcl/point_types.h>


struct Pcloud 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts_;

};


class ROSA_main {
public:
    void init();
    void main();    

    Pcloud P;

private: 
    /* Functions */

    /* Params */

    /* Data */

    /* Utils */
    


};