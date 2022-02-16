#include <math.h>
#include <iostream>
#include "navsat_conversions.h"

#define PI 3.1415926

using namespace RobotLocalization;
using namespace NavsatConversions;

int main()
{
    int radius = 5;
    int agent_number = 5;
    double theta = 0;
    double offset_angle = 10/180.0;
    printf("offset angle is %f\n", offset_angle);

    for(int i=1;i<5;i++){
        theta = 2.0 * PI * (i-1)/(agent_number-1) + offset_angle;
        double x = radius * cos(theta);
        double y = radius * sin(theta);
        printf("x position is %f ", x);
        printf("y position is %f ", y);
        printf("theta is %f\n", theta);
    }

    double lat0 = 47.3977418;
    double long0 = 8.5455935;

    double e0 = 0.0; double n0 = 0.0;
    double e1 = 0.0; double n1 = 0.0;

    double lat1 = 47.3977444;
    double long1 = 8.5456599;

    std::string zone;
    double gamma;

    //UTM(lat0, long0, &e0, &n0);
    LLtoUTM(lat0, long0, n0, e0, zone);
    //UTM(lat1, long1, &e1, &n1);
    LLtoUTM(lat1, long1, n1, e1, zone);

    printf("uav0 position x is %f ", e0); // this is actually y but it should be x in mavros
    printf("uav0 position y is %f \n", n0); // this is actually x but it should be y in mavros

    printf("uav1 position x is %f ", e1);
    printf("uav1 position y is %f \n", n1);

    double relative_x = e1 - e0;
    double relative_y = n1 - n0;

    printf("relative x is %f ", relative_x);
    printf("relative y is %f \n", relative_y);

    double true_x = 4.992286;
    double true_y = 0.277635;

    double e1_computed = e0 + true_x;
    double n1_computed = n0 + true_y;

    double lat1_computed;
    double long1_computed;

    UTMtoLL(n1_computed, e1_computed, zone, lat1_computed,long1_computed);

    printf("computed lat is %f ", lat1_computed);
    printf("computed long is %f \n", long1_computed);    


}