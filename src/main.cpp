
#include "formation.h"
#include "csv.h"

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

class fc_node_class
{
    public:
    fc_node_class(ros::NodeHandle &nodeHandle)
    {
    
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_controller");
    ros::NodeHandle nh("~");
    // The setpoint publishing rate MUST be faster than 2Hz
    fc_node_class fc_node_class(nh);
    ros::spin();
    return 0;
}

