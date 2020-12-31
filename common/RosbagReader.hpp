#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm> // std::transform
#include <iterator> // std::back_inserter

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>

class RosbagReader{
public:
    // 
    enum kMsgs{
        kPoseStamped = 0,
        kPointCloud2 = 1,
        kImu = 2
    };

    // 
    using TopicNameType = std::unordered_map<std::string, kMsgs>;
    using TopicMsgs = std::unordered_map<std::string, std::vector<void*>>;

    RosbagReader(const std::string& bag_path) : bag_path_(bag_path) {}

    // read pose stamped data
    void readPoseStamped(const std::string& topic_name, std::vector<geometry_msgs::PoseStamped>& msgs){
        rosbag::Bag bag;
        bag.open(bag_path_, rosbag::bagmode::Read);

        std::vector<std::string> topic_names = {topic_name};
        rosbag::View view(bag, rosbag::TopicQuery(topic_names));

        for(rosbag::MessageInstance const msg : view){
            geometry_msgs::PoseStamped::ConstPtr msg_ptr = msg.instantiate<geometry_msgs::PoseStamped>();
            if (msg_ptr != nullptr){
                msgs.push_back(*msg_ptr);
            }else{
                std::cerr << "[Error]: " << __FILE__ << "Instantiate msg type error" << std::endl;
            }
        }
        ROS_INFO_STREAM("[INFO]: Read " << msgs.size() << " PoseStamped messages");
        bag.close();
    }


    // read a topic
    template<class MsgType>
    void readTopic(const std::string& topic_name, std::vector<MsgType>& msgs);

    // Read many topics
    // unfinished
    void readTopic(const TopicNameType topic_name_type, TopicMsgs& topic_msgs){
        rosbag::Bag bag;
        bag.open(bag_path_, rosbag::bagmode::Read);

        // get Map's Keys into a Vector
        std::vector<std::string> topic_names;
        std::transform(
            topic_name_type.begin(), 
            topic_name_type.end(), 
            std::back_inserter(topic_names),
            [](auto& kv) { return kv.first; }
        );
            
        rosbag::View view(bag, rosbag::TopicQuery(topic_names));

        for(rosbag::MessageInstance const msg : view){
            std::string topic_name = msg.getTopic();


        }
    }

private:
    std::string bag_path_;
    // 



};

