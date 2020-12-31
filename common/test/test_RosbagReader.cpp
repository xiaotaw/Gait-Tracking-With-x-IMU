#include "RosbagReader.hpp"

int main(int argc, char** argv){

    std::string bag_path = "test.bag";
    if (argc >= 2){
        bag_path = argv[1];
    }

    RosbagReader reader(bag_path);

    std::vector<geometry_msgs::PoseStamped> pose;
    reader.readTopic("/imu/data", pose);

    return 0;

}