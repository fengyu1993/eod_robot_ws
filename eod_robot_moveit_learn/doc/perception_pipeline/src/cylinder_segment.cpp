#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

class CylinderSegment
{
public:
    CylinderSegment()
    {
        ros::NodeHandle nh;
        ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, &CylinderSegment::cloudCB, this);
        ros::spin();
    }

    void addCylinder()
    {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "camera_rgb_optical_frame";
        collision_object.id = "cylinder";

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);

        primitive.dimensions[0] = cylinder_params->height;
        primitive.dimensions[1] = cylinder_params->radius;

        geometry_msgs::Pose cylinder_pose;

        Eigen::Vector3d cylinder_z_direction(cylinder_params->direction_vec[0], cylinder_params->direction_vec[1],
                                         cylinder_params->direction_vec[2]);
                        
        Eigen::Vector3d origin_z_direction(0., 0., 1.);
        Eigen::Vector3d axis;
        axis = origin_z_direction.cross(cylinder_z_direction);
        axis.normalize();
        double angle = acos(cylinder_z_direction.dot(origin_z_direction));
        cylinder_pose.orientation.x = axis.x() * sin(angle / 2);
        cylinder_pose.orientation.y = axis.y() * sin(angle / 2);
        cylinder_pose.orientation.z = axis.z() * sin(angle / 2);
        cylinder_pose.orientation.w = cos(angle / 2);

        // Setting the position of cylinder.
        cylinder_pose.position.x = cylinder_params->center_pt[0];
        cylinder_pose.position.y = cylinder_params->center_pt[1];
        cylinder_pose.position.z = cylinder_params->center_pt[2];

        // Add cylinder as collision object
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(cylinder_pose);
        collision_object.operation = collision_object.ADD;
        planning_scene_interface.applyCollisionObject(collision_object);

    }

  void extractLocationHeight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    double max_angle_y = 0.0;
    double min_angle_y = std::numeric_limits<double>::infinity();

    double lowest_point[3];
    double highest_point[3];

    for (auto const point : cloud->points)
    {
      /* Find the coordinates of the highest point */
      if (atan2(point.z, point.y) < min_angle_y)
      {
        min_angle_y = atan2(point.z, point.y);
        lowest_point[0] = point.x;
        lowest_point[1] = point.y;
        lowest_point[2] = point.z;
      }
      /* Find the coordinates of the lowest point */
      else if (atan2(point.z, point.y) > max_angle_y)
      {
        max_angle_y = atan2(point.z, point.y);
        highest_point[0] = point.x;
        highest_point[1] = point.y;
        highest_point[2] = point.z;
      }
    }
    /* Store the center point of cylinder */
    cylinder_params->center_pt[0] = (highest_point[0] + lowest_point[0]) / 2;
    cylinder_params->center_pt[1] = (highest_point[1] + lowest_point[1]) / 2;
    cylinder_params->center_pt[2] = (highest_point[2] + lowest_point[2]) / 2;
    /* Store the height of cylinder */
    cylinder_params->height =
        sqrt(pow((lowest_point[0] - highest_point[0]), 2) + pow((lowest_point[1] - highest_point[1]), 2) +
             pow((lowest_point[2] - highest_point[2]), 2));
    // END_SUB_TUTORIAL
  }

  void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    // min and max values in z axis to keep
    pass.setFilterLimits(0.3, 1.1);
    pass.filter(*cloud);
  }

  void computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
  {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    // Set the number of k nearest neighbors to use for the feature estimation.
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
  }

  void extractNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_plane)
  {
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals);
  }

  void removePlaneSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers_plane)
  {
    // create a SAC segmenter without using normals
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    /* run at max 1000 iterations before giving up */
    segmentor.setMaxIterations(1000);
    /* tolerance for variation from model */
    segmentor.setDistanceThreshold(0.01);
    segmentor.setInputCloud(cloud);
    /* Create the segmentation object for the planar model and set all the parameters */
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    segmentor.segment(*inliers_plane, *coefficients_plane);
    /* Extract the planar inliers from the input cloud */
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers_plane);
    /* Remove the planar inliers, extract the rest */
    extract_indices.setNegative(true);
    extract_indices.filter(*cloud);
  }

  void extractCylinder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients_cylinder,
                       pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
  {
    // Create the segmentation object for cylinder segmentation and set all the parameters
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_CYLINDER);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    // Set the normal angular distance weight
    segmentor.setNormalDistanceWeight(0.1);
    // run at max 1000 iterations before giving up
    segmentor.setMaxIterations(10000);
    // tolerance for variation from model
    segmentor.setDistanceThreshold(0.05);
    // min max values of radius in meters to consider
    segmentor.setRadiusLimits(0, 1);
    segmentor.setInputCloud(cloud);
    segmentor.setInputNormals(cloud_normals);

    // Obtain the cylinder inliers and coefficients
    segmentor.segment(*inliers_cylinder, *coefficients_cylinder);

    // Extract the cylinder inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cloud);
  }


    void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*input, *cloud);

        passThroughFilter(cloud);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        computeNormals(cloud, cloud_normals);

        pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

        removePlaneSurface(cloud, inliers_plane);

        extractNormals(cloud_normals, inliers_plane);

        pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);

        extractCylinder(cloud, coefficients_cylinder, cloud_normals);

        if (cloud->points.empty())
        {
            ROS_ERROR_STREAM_NAMED("cylinder_segment", "Can't find the cylindrical component.");
            return;
        }
        if (points_not_found)
        {
            cylinder_params->radius = coefficients_cylinder->values[6];
            cylinder_params->direction_vec[0] = coefficients_cylinder->values[3];
            cylinder_params->direction_vec[1] = coefficients_cylinder->values[4];
            cylinder_params->direction_vec[2] = coefficients_cylinder->values[5];

            extractLocationHeight(cloud);

            addCylinder();
            points_not_found = false;
        }
    }

private:
    struct AddCylinderParams
    {
        double radius;
        double direction_vec[3];
        double center_pt[3];
        double height;
    };
    AddCylinderParams* cylinder_params;
    
    bool points_not_found = true;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "cylinder_segment");
    CylinderSegment();
}