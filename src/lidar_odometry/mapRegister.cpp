#include "utility.h"
#include "lvi_sam/cloud_info.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <unordered_map>


// 定义栅格的键值
struct GridKey {
    int x, y, z;
    bool operator==(const GridKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// 定义哈希函数
struct GridKeyHash {
    std::size_t operator()(const GridKey& key) const {
        return std::hash<int>()(key.x) ^ std::hash<int>()(key.y) ^ std::hash<int>()(key.z);
    }
};

class MapRegister : public ParamServer
{
public:
    MapRegister(){  
        // 八叉树分辨率为 0.25
        // 订阅点云话题
        ROS_INFO("begin build MapRegister\n");
        octree_=std::make_shared<octomap::OcTree>(0.25);

        pointcloud_sub_ = nh.subscribe(PROJECT_NAME+"/pointcloud_topic", 5, &MapRegister::pointCloudCallback, this,
        ros::TransportHints().tcpNoDelay());
        ROS_INFO("build MapRegister finish\n");
    }
private:
    void pointCloudCallback(const lvi_sam::cloud_infoConstPtr& msgIn) {
        // 将 PointCloud2 转换为 PCL 点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(msgIn->cloud_deskewed, *cloud);

        // 遍历点云构造八叉树和栅格地图
        for (const auto& point : cloud->points) {
            // 添加点到八叉树
            octree_->updateNode(octomap::point3d(point.x, point.y, point.z), true);

            // 构造哈希栅格地图
            GridKey key = {static_cast<int>(point.x / 0.1),
                           static_cast<int>(point.y / 0.1),
                           static_cast<int>(point.z / 0.1)};
            grid_map_[key] = true;
        }

        // 可选：压缩八叉树以节省存储
        octree_->updateInnerOccupancy();

        // 输出八叉树和栅格地图信息
        ROS_INFO("Octree size: %zu, Grid map size: %zu", octree_->size(), grid_map_.size());
    }
    std::shared_ptr<octomap::OcTree> octree_;
    std::unordered_map<GridKey, bool, GridKeyHash> grid_map_; // 哈希栅格地图
    ros::Subscriber pointcloud_sub_;
};

int main(int argc,char ** argv)
{
    ros::init(argc, argv, "map");

    MapRegister MR;

    ROS_INFO("\033[1;32m----> Map Register Started.\033[0m");
   
    ros::spin();

    return 0;
}