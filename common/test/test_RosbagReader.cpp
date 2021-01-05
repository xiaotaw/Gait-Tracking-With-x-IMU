// for converting sensor_msgs::PointCloud2 to pcl::PointCloud
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// for converting geometry_msgs::Pose to Eigen::Affine3d
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include "RosbagReader.hpp"

static std::string itos(int i){
    char str[256] = {0};
    sprintf(str, "%d", i);
    return std::string(str);
}

int main(int argc, char **argv)
{
    
    Eigen::Matrix4d Tcb;
    Tcb << -0.00518964, 0.99994908, -0.00865516, -0.030,
        -0.99996105, -0.00525111, -0.0070945, 0.00809529,
        -0.00713959, 0.008618, 0.99993738, 0.010,
        0, 0, 0, 1;

    // P_b = Tbc * P_c
    Eigen::Matrix4d Tbc = Tcb.inverse();

    std::cout << Tbc << std::endl;


    std::string bag_path = "/data/DATASETS/IMU_CAM/1_room_DeskFast_output.bag";
    if (argc >= 2)
    {
        bag_path = argv[1];
    }

    RosbagReader reader(bag_path);

    std::vector<geometry_msgs::PoseStamped> pose_vec;
    reader.readPoseStamped("/estimated/pose", pose_vec);

    std::vector<sensor_msgs::PointCloud2> ros_pc2_vec;
    reader.readPointCloud2("/camera_argus100/publish_point_cloud", ros_pc2_vec);

    // convert ros_pc2 to pcl_pc

    assert(pose_vec.size() == ros_pc2_vec.size());

    geometry_msgs::PoseStamped pose;
    sensor_msgs::PointCloud2 ros_pc2;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointCloud<pcl::PointXYZ> pcl_pc, pcl_pc_transformed;
    for (int i = 0; i < pose_vec.size(); ++i)
    {
        ros_pc2 = ros_pc2_vec[i];
        pose = pose_vec[i];

        Eigen::Affine3d transform, transform_trans;
        tf::poseMsgToEigen(pose.pose, transform);
        std::cout << transform.matrix() << std::endl;
        Eigen::Matrix4d transform_mat = transform.matrix() * Tbc;
        std::cout << transform_mat << std::endl << std::endl;

        pcl_conversions::toPCL(ros_pc2, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, pcl_pc);
        pcl::io::savePCDFileASCII("ori/" + itos(i) + ".pcd", pcl_pc);


        pcl::transformPointCloud(pcl_pc, pcl_pc_transformed, transform_mat);
        pcl::io::savePCDFileASCII("trans/" + itos(i) + ".pcd", pcl_pc_transformed);
    }

    return 0;
}