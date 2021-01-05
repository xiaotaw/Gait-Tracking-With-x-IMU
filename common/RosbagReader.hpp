#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm> // std::transform
#include <iterator> // std::back_inserter

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

class RosbagReader{
public:
    // ctor
    RosbagReader(const std::string& bag_path) : bag_path_(bag_path) {}

    // read pose stamped
    int readPoseStamped(const std::string& topic_name, std::vector<geometry_msgs::PoseStamped>& msgs){
        return readTopic<geometry_msgs::PoseStamped>(topic_name, msgs);
    }

    // read pointcloud2
    int readPointCloud2(const std::string& topic_name, std::vector<sensor_msgs::PointCloud2>& msgs){
        return readTopic<sensor_msgs::PointCloud2>(topic_name, msgs);
    }

    // read a topic
    template<class MsgType>
    int readTopic(const std::string& topic_name, std::vector<MsgType>& msgs){
        rosbag::Bag bag;
        bag.open(bag_path_, rosbag::bagmode::Read);
        std::vector<std::string> topic_names = {topic_name};
        rosbag::View view(bag, rosbag::TopicQuery(topic_names));

        msgs.clear();

        for(rosbag::MessageInstance const msg : view){
            typename MsgType::ConstPtr msg_ptr = msg.instantiate<MsgType>();
            if(msg_ptr != nullptr){
                msgs.push_back(*msg_ptr);
            }else{
                std::cerr << "[Error]: " << __FILE__ << "Instantiate msg type error" << std::endl;
            }
        }
        ROS_INFO_STREAM("[INFO]: Read " << msgs.size() << " messages");
        bag.close();
        return msgs.size();
    }

    // read pose stamped data (debug test)
    int readPoseStamped_debug(const std::string& topic_name, std::vector<geometry_msgs::PoseStamped>& msgs){
        rosbag::Bag bag;
        bag.open(bag_path_, rosbag::bagmode::Read);

        std::vector<std::string> topic_names = {topic_name};
        rosbag::View view(bag, rosbag::TopicQuery(topic_names));

        msgs.clear();

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
        return msgs.size();
    }

    // trying to read many topic at one time
    enum kMsgs{
        kPoseStamped = 0,
        kPointCloud2 = 1,
        kImu = 2
    };

    using TopicNameType = std::unordered_map<std::string, kMsgs>;
    using TopicMsgs = std::unordered_map<std::string, std::vector<void*>>;

    // Read many topics (unfinished)
    int readTopic(const TopicNameType topic_name_type, TopicMsgs& topic_msgs){
        std::cerr << "[FATAL]: Not Implemented yet!" << std::endl;
        return 0;

        rosbag::Bag bag;
        bag.open(bag_path_, rosbag::bagmode::Read);

        // get Map's Keys into a Vector
        std::vector<std::string> topic_names;
        std::transform(
            topic_name_type.begin(), 
            topic_name_type.end(), 
            std::back_inserter(topic_names),
            [](const std::pair<std::string, kMsgs>& kv) { return kv.first; } /* need c++14 for 'auto' in lambda */
        );
            
        rosbag::View view(bag, rosbag::TopicQuery(topic_names));

        for(rosbag::MessageInstance const msg : view){
            std::string topic_name = msg.getTopic();

        }
        return 0;
    }

private:
    std::string bag_path_;

};

