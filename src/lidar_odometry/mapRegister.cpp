#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <unordered_map>
#include <vector>
#include <map>
#include <tuple>

// 自定义点类型：包含 xyzrgb
// struct PointXYZRGB : public pcl::PointXYZRGB {
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// };
// POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGB,
//                                   (float, x, x)
//                                   (float, y, y)
//                                   (float, z, z)
//                                   (uint32_t, rgba, rgba))

// 栅格单元：存储颜色和语义信息
struct GridCell {
    bool occupied;
    uint8_t r, g, b;
    int semantic_class;

    GridCell()
        : occupied(true), r(0), g(0), b(0), semantic_class(-1) {}
};

// RGB 到语义类别的映射 Key
typedef std::tuple<uint8_t, uint8_t, uint8_t> ColorKey;

struct ColorKeyHash {
    size_t operator()(const ColorKey &key) const {
        return std::get<0>(key) + std::get<1>(key) * 256 + std::get<2>(key) * 256 * 256;
    }
};

// 颜色映射表（对应 Python 的 class_colors）
const std::vector<std::vector<uint8_t>> class_colors = {
    {0, 0, 0},         // 背景 - 黑色
    {255, 0, 0},       // 红色
    {0, 255, 0},       // 绿色
    {0, 0, 255},       // 蓝色
    {255, 255, 0},     // 黄色
    {255, 0, 255},     // 品红
    {0, 255, 255},     // 青色
    {128, 0, 0},       // 深红
    {0, 128, 0},       // 深绿
    {0, 0, 128},       // 深蓝
    {128, 128, 0},     // 橄榄
    {128, 0, 128},     // 紫色
    {0, 128, 128},     // 青绿
    {255, 165, 0},     // 橙色
    {255, 192, 203},   // 粉红
    {75, 192, 192},    // 钢蓝
    {128, 128, 128},   // 灰色
    {255, 255, 255},   // 白色
    {192, 192, 192},   // 浅灰
    {255, 223, 0},     // 金色
    {173, 216, 230}    // 浅蓝
};

// HashGridMap 类：用于存储栅格的语义信息
class HashGridMap {
private:
    float resolution_;
    std::unordered_map<size_t, GridCell> grid_cells_;
    std::unordered_map<ColorKey, int, ColorKeyHash> color_to_class_;

    size_t computeKey(float x, float y, float z) const {
        int xi = static_cast<int>(x / resolution_);
        int yi = static_cast<int>(y / resolution_);
        int zi = static_cast<int>(z / resolution_);
        return std::hash<int>()(xi) ^ (std::hash<int>()(yi) << 1) ^ (std::hash<int>()(zi) << 2);
    }

public:
    explicit HashGridMap(float res) : resolution_(res) {
        // 初始化：从 class_colors 构建 color_to_class_
        for (size_t i = 0; i < class_colors.size(); ++i) {
            const auto &color = class_colors[i];
            color_to_class_[ColorKey(color[0], color[1], color[2])] = static_cast<int>(i);
        }
    }

    void updateCell(float x, float y, float z, bool occupied,
                    uint8_t r, uint8_t g, uint8_t b) {
        size_t key = computeKey(x, y, z);
        GridCell &cell = grid_cells_[key];
        cell.occupied = occupied;
        cell.r = r;
        cell.g = g;
        cell.b = b;
        cell.semantic_class = getSemanticClass(r, g, b);
    }

    int getSemanticClass(uint8_t r, uint8_t g, uint8_t b) const {
        auto it = color_to_class_.find(ColorKey(r, g, b));
        if (it != color_to_class_.end()) {
            return it->second;
        }
        return -1; // 未知类别
    }

    bool getCell(float x, float y, float z, GridCell &out_cell) const {
        size_t key = computeKey(x, y, z);
        auto it = grid_cells_.find(key);
        if (it != grid_cells_.end()) {
            out_cell = it->second;
            return true;
        }
        return false;
    }
    bool isOccupied(float x, float y, float z) const {
        size_t key = computeKey(x, y, z);
        auto it = grid_cells_.find(key);
        if (it != grid_cells_.end()) {
            return true;
        }
        return false; // 如果没有这个格子，默认认为未被占用
    }
};

class MapRegister {
private:
    ros::NodeHandle nh_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher octomap_pub_;
    std::shared_ptr<octomap::OcTree> octree_;
    std::shared_ptr<HashGridMap> grid_map_;
public:
    MapRegister() : nh_("~") {
        const float resolution = 0.25f;
        octree_ = std::make_shared<octomap::OcTree>(resolution);
        grid_map_ = std::make_shared<HashGridMap>(resolution);

        pointcloud_sub_ = nh_.subscribe("/rgb_seg_cloud", 5, &MapRegister::pointCloudCallback, this);
        octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap", 1);

        ROS_INFO("MapRegister node started.");
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *cloud);

        //octomap::point3d sensor_origin(0, 0, 0); // 可替换为真实传感器位姿

        for (const auto &pt : cloud->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                continue;

            octomap::point3d point(pt.x, pt.y, pt.z);

            // 提取 RGB
            uint32_t rgba = pt.r;

            uint8_t r = pt.r;
            uint8_t g = pt.g;
            uint8_t b = pt.b;

            if(grid_map_->isOccupied(pt.x,pt.y,pt.z)==false){
                // 更新 OctoMap
                auto node=octree_->updateNode(point, true);
                if (node->getOccupancy() > 0.85){
                    // 更新 HashGridMap
                    grid_map_->updateCell(pt.x, pt.y, pt.z, true, r, g, b);
                    // 射线追踪标记自由空间
                    // octomap::KeyRay ray;
                    // if (octree_->computeRayKeys(sensor_origin, point, ray)) {
                    //     for (const octomap::OcTreeKey &key : ray) {
                    //         octree_->updateNode(key, false);
                    //     }
                    // }
                }
            }
            
            
            

            
        }

        octree_->updateInnerOccupancy();
        octree_->prune();

        // 发布 OctoMap
        octomap_msgs::Octomap map_msg;
        map_msg.header.frame_id = "odom";
        map_msg.header.stamp = ros::Time::now();
        map_msg.resolution = 0.25f;
        //map_msg.id = OCTOMAP_MSG_ID;

        if (octomap_msgs::fullMapToMsg(*octree_, map_msg)) {
            octomap_pub_.publish(map_msg);
        } else {
            ROS_ERROR("Failed to convert OctoMap to message.");
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_register");
    MapRegister mr;
    ros::spin();
    return 0;
}