#include "utility.h"
#include "lvi_sam/cloud_info.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <unordered_map>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
struct GridCell
{
    bool occupied;              // 是否被占用
    uint8_t r, g, b;            // 颜色 (RGB)
    float intensity;            // 光强
    std::string semantic_label; // 语义标签
    float semantic_confidence;  // 语义置信度

    GridCell()
        : occupied(true), r(0), g(0), b(0), intensity(0.0f), semantic_confidence(0.0f), semantic_label("") {}

    // 设置颜色
    void setColor(uint8_t red, uint8_t green, uint8_t blue)
    {
        r = red;
        g = green;
        b = blue;
    }

    // 设置光强
    void setIntensity(float value)
    {
        intensity = value;
    }

    // 设置语义信息
    void setSemanticInfo(const std::string &label, float confidence)
    {
        semantic_label = label;
        semantic_confidence = confidence;
    }
};

class HashGridMap
{
private:
    float resolution;                                // 栅格分辨率
    std::unordered_map<size_t, GridCell> grid_cells; // 哈希表：键是栅格位置，值是GridCell

    // 根据三维坐标计算哈希键
    size_t computeHashKey(int x, int y, int z) const
    {
        return std::hash<int>()(int(x / resolution)) ^ (std::hash<int>()(int(y / resolution)) << 1) ^ (std::hash<int>()(int(z / resolution)) << 2);
    }

public:
    HashGridMap(float res) : resolution(res) {}

    // 更新栅格信息
    void updateGridCell(const float x, const float y, const float z, const bool occupied,
                        const uint8_t r, const uint8_t g, const uint8_t b,
                        const float intensity, const std::string &label, const float confidence)
    {
        size_t key = computeHashKey(x, y, z);
        GridCell &cell = grid_cells[key];
        cell.occupied = occupied;
        cell.setColor(r, g, b);
        cell.setIntensity(intensity);
        cell.setSemanticInfo(label, confidence);
    }

    // 查询栅格信息
    bool getGridCell(int x, int y, int z, GridCell &out_cell) const
    {
        size_t key = computeHashKey(x, y, z);
        auto it = grid_cells.find(key);
        if (it != grid_cells.end())
        {
            out_cell = it->second;
            return true;
        }
        return false;
    }
    bool getGridCell(int x, int y, int z) const
    {
        size_t key = computeHashKey(x, y, z);
        auto it = grid_cells.find(key);
        if (it != grid_cells.end())
        {
            return true;
        }
        return false;
    }

    // 打印栅格信息
    void printGridCellInfo(int x, int y, int z) const
    {
        GridCell cell;
        if (getGridCell(x, y, z, cell))
        {
            std::cout << "GridCell at (" << x << ", " << y << ", " << z << "):\n";
            std::cout << "Occupied: " << cell.occupied << "\n";
            std::cout << "Color: (" << (int)cell.r << ", " << (int)cell.g << ", " << (int)cell.b << ")\n";
            std::cout << "Intensity: " << cell.intensity << "\n";
            std::cout << "Semantic Label: " << cell.semantic_label << "\n";
            std::cout << "Semantic Confidence: " << cell.semantic_confidence << "\n";
        }
        else
        {
            std::cout << "No data for grid cell at (" << x << ", " << y << ", " << z << ").\n";
        }
    }
};
class MapRegister : public ParamServer
{
public:
    MapRegister()
    {
        // 八叉树分辨率为 0.25
        const float resolution = 0.25f;
        ROS_INFO("begin build MapRegister\n");
        octree_ = std::make_shared<octomap::OcTree>(resolution);
        grid_map_ = std::make_shared<HashGridMap>(resolution);
        // 订阅点云话题
        ROS_INFO("%s/lidar/feature/cloud_info",PROJECT_NAME.c_str());
        pointcloud_sub_ = nh.subscribe(PROJECT_NAME + "/lidar/feature/cloud_info", 5, &MapRegister::pointCloudCallback, this,
                                       ros::TransportHints().tcpNoDelay());
        octomap_pub_ = nh.advertise<octomap_msgs::Octomap>("octomap", 1);
        ROS_INFO("build MapRegister finish\n");
    }

private:
    void pointCloudCallback(const lvi_sam::cloud_infoConstPtr &msgIn)
    {
        auto start_time = ros::Time::now();
        static int cloud_callback_count=0;
        cloud_callback_count+=1;
        // 将 PointCloud2 转换为 PCL 点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(msgIn->cloud_deskewed, *cloud);
        ROS_INFO("pointCloudCallback %d %d\n",cloud_callback_count,cloud->size());
        const octomap::point3d start_point(msgIn->odomX,msgIn->odomY, msgIn->odomZ);
        const uint8_t color_red = 125;
        const uint8_t color_green = 125;
        const uint8_t color_blue = 125;
        const float intensity = 10;
        const string label = "ground";
        const float label_confidence = 0.9;
        // 遍历点云构造八叉树和栅格地图
        for (const auto &point : cloud->points)
        {
            bool find_flag=grid_map_->getGridCell(point.x, point.y, point.z);
            if(find_flag){
                continue;
            }
            // 添加点到八叉树
            const octomap::point3d end_point(point.x, point.y, point.z);
            auto node = octree_->updateNode(end_point, true);
            if (node->getOccupancy() > 0.85)
            {
                // 构造哈希栅格地图
                grid_map_->updateGridCell(point.x, point.y, point.z, true, color_red,
                                          color_green, color_blue, intensity, label, label_confidence);
            }

            octomap::KeyRay key_ray; // 用于存储射线路径的 Key
            // if (octree_->computeRayKeys(start_point, end_point, key_ray))
            // {
            //     // 遍历射线上的所有体素并设置为未占用
            //     for (const auto &key : key_ray)
            //     {
            //         octree_->updateNode(key, false); // 标记为未占用
            //     }
            // }
        }

        // 可选：压缩八叉树以节省存储
        octree_->updateInnerOccupancy();

        if(cloud_callback_count%1==0){
            octomap_msgs::Octomap map_msg;
            map_msg.header = msgIn->header;
            if (octomap_msgs::fullMapToMsg(*octree_, map_msg)) {
                octomap_pub_.publish(map_msg); // 发布消息
            } else {
                ROS_ERROR("Failed to convert OctoMap to message.");
            }
        }
        auto end_time = ros::Time::now();

        // 计算时间差（单位：毫秒）
        ros::Duration elapsed_time = end_time - start_time;
        ROS_INFO("count Elapsed time: %f ms octree size :%d",elapsed_time.toSec()*1000,octree_->size());
        // 输出八叉树和栅格地图信息
        //ROS_INFO("Octree size: %zu, Grid map size: %zu", octree_->size(), grid_map_->size());
    }
    std::shared_ptr<HashGridMap> grid_map_;
    std::shared_ptr<octomap::OcTree> octree_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher octomap_pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map");

    MapRegister MR;

    ROS_INFO("\033[1;32m----> Map Register Started.\033[0m");

    ros::spin();

    return 0;
}