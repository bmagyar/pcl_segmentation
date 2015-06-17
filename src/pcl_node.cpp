#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);



  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);

  // Get the plane model, if present.
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::SACSegmentation<pcl::PointXYZ> segmentation;
      segmentation.setInputCloud(msg);
      segmentation.setModelType(pcl::SACMODEL_PLANE);
      segmentation.setMethodType(pcl::SAC_RANSAC);
      segmentation.setDistanceThreshold(0.01);
      segmentation.setOptimizeCoefficients(true);
      pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
      segmentation.segment(*planeIndices, *coefficients);

      if (planeIndices->indices.size() == 0)
          std::cout << "Could not find a plane in the scene." << std::endl;
      else
      {
          // Copy the points of the plane to a new cloud.
          pcl::ExtractIndices<pcl::PointXYZ> extract;
          extract.setInputCloud(msg);
          extract.setIndices(planeIndices);
          extract.filter(*plane);

          // Retrieve the convex hull.
          pcl::ConvexHull<pcl::PointXYZ> hull;
          hull.setInputCloud(plane);
          // Make sure that the resulting hull is bidimensional.
          hull.setDimension(2);
          hull.reconstruct(*convexHull);

          // Redundant check.
          if (hull.getDimension() == 2)
          {
              // Prism object.
              pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
              prism.setInputCloud(msg);
              prism.setInputPlanarHull(convexHull);
              // First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
              // Second parameter: maximum Z value, set to 10cm. Tune it according to the height of the objects you expect.
              prism.setHeightLimits(0.0f, 0.1f);
              pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

              prism.segment(*objectIndices);

              // Get and show all points retrieved by the hull.
              extract.setIndices(objectIndices);
              extract.filter(*objects);
              pcl::visualization::CloudViewer viewerObjects("Objects on table");
              viewerObjects.showCloud(objects);
              while (!viewerObjects.wasStopped())
              {
                  // Do nothing but wait.
              }
          }
          else std::cout << "The chosen hull is not planar." << std::endl;
      }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("points", 1, callback);
  ros::spin();
}

