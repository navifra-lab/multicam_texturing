#pragma once

#include <iostream>
#include <optional>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>

namespace color_point_cloud
{

struct Point
{
    float x;
    float y;
    float z;
    float intensity;

    void transform(const geometry_msgs::TransformStamped& t_transform)
    {
        const tf2::Quaternion rotation{
            t_transform.transform.rotation.x,
            t_transform.transform.rotation.y,
            t_transform.transform.rotation.z,
            t_transform.transform.rotation.w
        };
        const tf2::Vector3 translation{
            t_transform.transform.translation.x,
            t_transform.transform.translation.y,
            t_transform.transform.translation.z
        };
        const tf2::Transform transform{ rotation, translation };

        const tf2::Vector3 old_point{ x, y, z };
        const tf2::Vector3 new_point{ transform(old_point) };

        x = new_point.getX();
        y = new_point.getY();
        z = new_point.getZ();
    }
};

/**
 * @brief Read-only iterator wrapper over an existing const sensor_msgs::PointCloud2.
 */
class PointCloudConst
{
public:
    explicit PointCloudConst(const sensor_msgs::PointCloud2& t_pointcloud2)
        : m_pointcloud2{ t_pointcloud2 }
        , m_point_count{ static_cast<unsigned int>(m_pointcloud2.height * m_pointcloud2.width) }
        , m_current_index{ 0 }
        , m_iter_x{ m_pointcloud2, "x" }
        , m_iter_y{ m_pointcloud2, "y" }
        , m_iter_z{ m_pointcloud2, "z" }
        , m_iter_intensity{ m_pointcloud2, "intensity" }
    {}

    unsigned int getPointCount() const { return m_point_count; }

    Point getCurrentPoint() const
    {
        const Point point{ *m_iter_x, *m_iter_y, *m_iter_z, *m_iter_intensity };
        return point;
    }

    void nextPoint()
    {
        if (m_current_index < m_point_count)
        {
            ++m_current_index;
            ++m_iter_x;
            ++m_iter_y;
            ++m_iter_z;
            ++m_iter_intensity;
        }
    }

    size_t getCurrentIndex() const { return m_current_index; }

private:
    // Reference to existing PointCloud2
    const sensor_msgs::PointCloud2& m_pointcloud2;

    // Number of points
    const unsigned int m_point_count;

    // Current index
    size_t m_current_index;

    // Iterators
    sensor_msgs::PointCloud2ConstIterator<float> m_iter_x;
    sensor_msgs::PointCloud2ConstIterator<float> m_iter_y;
    sensor_msgs::PointCloud2ConstIterator<float> m_iter_z;
    sensor_msgs::PointCloud2ConstIterator<float> m_iter_intensity;
};

/**
 * @brief Mutable iterator wrapper over sensor_msgs::PointCloud2, with ability to
 *        allocate/resize and set fields.
 */
class PointCloud
{
public:
    explicit PointCloud(sensor_msgs::PointCloud2& t_pointcloud2)
        : m_pointcloud2{ t_pointcloud2 }
        , m_point_count{ static_cast<unsigned int>(m_pointcloud2.height * m_pointcloud2.width) }
        , m_current_index{ 0 }
    {}

    unsigned int getPointCount() const { return m_point_count; }

    Point getCurrentPoint() const
    {
        // Iterators must be emplaced before calling this
        const Point point{ *(*m_iter_x), *(*m_iter_y), *(*m_iter_z), *(*m_iter_intensity) };
        return point;
    }

    void nextPoint()
    {
        if (m_current_index < m_point_count)
        {
            ++m_current_index;
            ++(*m_iter_x);
            ++(*m_iter_y);
            ++(*m_iter_z);
            ++(*m_iter_intensity);
        }
    }

    /**
     * @brief Define fields (x,y,z,intensity as float32) and resize the cloud.
     *        Also (re)initialize the iterators.
     */
    void setFieldsAndResize(const size_t& t_point_count)
    {
        m_point_count = static_cast<unsigned int>(t_point_count);

        sensor_msgs::PointCloud2Modifier modifier{ m_pointcloud2 };
        modifier.setPointCloud2Fields(
            4,
            "x",         1, sensor_msgs::PointField::FLOAT32,
            "y",         1, sensor_msgs::PointField::FLOAT32,
            "z",         1, sensor_msgs::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::PointField::FLOAT32
        );
        modifier.resize(m_point_count);

        // Initialize writable iterators
        m_iter_x.emplace(m_pointcloud2, "x");
        m_iter_y.emplace(m_pointcloud2, "y");
        m_iter_z.emplace(m_pointcloud2, "z");
        m_iter_intensity.emplace(m_pointcloud2, "intensity");

        // Optional: set default frame_id here (override outside if needed)
        m_pointcloud2.header.frame_id = "base_link";

        // Reset write index
        m_current_index = 0;
    }

    /**
     * @brief Write current point values into the cloud.
     */
    void setCurrentPoint(const Point& t_point)
    {
        *(*m_iter_x) = t_point.x;
        *(*m_iter_y) = t_point.y;
        *(*m_iter_z) = t_point.z;
        *(*m_iter_intensity) = t_point.intensity;
    }

    /**
     * @brief Append (write then advance).
     */
    void append(const Point& t_point)
    {
        setCurrentPoint(t_point);
        nextPoint();
    }

private:
    // Reference to existing PointCloud2
    sensor_msgs::PointCloud2& m_pointcloud2;

    // Number of points
    unsigned int m_point_count;

    // Current index
    size_t m_current_index;

    // Writable iterators (lazily emplaced after setFieldsAndResize)
    std::optional<sensor_msgs::PointCloud2Iterator<float>> m_iter_x;
    std::optional<sensor_msgs::PointCloud2Iterator<float>> m_iter_y;
    std::optional<sensor_msgs::PointCloud2Iterator<float>> m_iter_z;
    std::optional<sensor_msgs::PointCloud2Iterator<float>> m_iter_intensity;
};

}  // namespace color_point_cloud
