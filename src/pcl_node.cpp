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
#include <pcl/segmentation/extract_clusters.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>


#include <iostream>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
ros::Publisher segmented_objects;
ros::Publisher segmented_plane;
ros::Publisher segmented_convexHull;
ros::Publisher clustering1;
ros::Publisher clustering2;
ros::Publisher clustering3;
ros::Publisher clustering4;
ros::Publisher masking;

boost::shared_ptr<sensor_msgs::CameraInfo const> CameraInfo;

void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGB>);

  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(CameraInfo);

  // Get the plane model, if present.
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
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
          pcl::ExtractIndices<pcl::PointXYZRGB> extract;
          extract.setInputCloud(msg);
          extract.setIndices(planeIndices);
          extract.filter(*plane);

          // Retrieve the convex hull.
          pcl::ConvexHull<pcl::PointXYZRGB> hull;
          hull.setInputCloud(plane);
          // Make sure that the resulting hull is bidimensional.
          hull.setDimension(2);
          hull.reconstruct(*convexHull);

          // Redundant check.
          if (hull.getDimension() == 2)
          {
              // Prism object.
              pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
              prism.setInputCloud(msg);
              prism.setInputPlanarHull(convexHull);
              // First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
              // Second parameter: maximum Z value, set to 10cm. Tune it according to the height of the objects you expect.
              prism.setHeightLimits(0.02, 0.2);
              pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

              prism.segment(*objectIndices);

              // Get and show all points retrieved by the hull.
              extract.setIndices(objectIndices);
              extract.filter(*objects);
              segmented_objects.publish(objects);
              segmented_plane.publish(plane);
              segmented_convexHull.publish(convexHull);

              // run clustering extraction on "objects" to get several pointclouds
              pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
              pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
              std::vector<pcl::PointIndices> cluster_indices;
              ec.setClusterTolerance (0.05);
              ec.setMinClusterSize (1000);
              ec.setMaxClusterSize (10000000);
              ec.setSearchMethod (tree);
              ec.setInputCloud (objects);
              ec.extract (cluster_indices);

              cv::Mat mask_image = cv::Mat::zeros(CameraInfo->height, CameraInfo->width, CV_8UC1);

              pcl::ExtractIndices<pcl::PointXYZRGB> extract_object_indices;
              std::vector<pcl::PointCloud<pcl::PointXYZRGB> > objectf;
              for(int i = 0; i<cluster_indices.size(); ++i)
              {
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                  //pcl::PointCloud<pcl::PointXYZRGB> object_cloud;
                  extract_object_indices.setInputCloud(objects);
                  extract_object_indices.setIndices(boost::make_shared<const pcl::PointIndices>(cluster_indices[i]));
                  extract_object_indices.filter(*object_cloud);
                  objectf.push_back(*object_cloud);

                  //cv::Point3d pt_cv(object_cloud->at(0).x, object_cloud->at(0).y, object_cloud->at(0).z);//init Point3d
                  //cv::Point2d uv = cam_model.project3dToPixel(pt_cv); // project 3d point to 2d point
                  //mask_image.at<uchar>(uv.x,uv.y) = 255;

                  for (int j = 0; j < object_cloud->points.size(); j++) {
                    pcl::PointXYZRGB p = object_cloud->points[j];
                    cv::Point2d uv = cam_model.project3dToPixel(cv::Point3d(p.x, p.y, p.z));
                    mask_image.at<uint8_t>(uv.y,uv.x) = 255;
                    //printf ("\t(%f, %f)\n", uv.x, uv.y);
                  }

                  /*
                  bool in_mask = false;
                  for (int j = 0; j < object_cloud->points.size(); j++) {
                    pcl::PointXYZRGB p = object_cloud->points[j];
                    cv::Point2d uv = model.project3dToPixel(cv::Point3d(p.x, p.y, p.z));
                    if (uv.x > 0 && uv.x < mask_image.cols && uv.y > 0 && uv.y < mask_image.rows) {
                      if (mask_image.at<uchar>(uv.y, uv.x) == 255) {
                        in_mask = true;
                        break;
                      }
                    }
                  }
                  if (in_mask) {
                    cluster_indices->size();
                    for(size_t j=0; j < cluster_indices->size(); j++)
                      {
                        indices.indices.push_back((*cluster_indices)[j]);
                      }
                  }
                }
                mask.publish(indices); */
              }

              clustering1.publish(objectf[0]);
              clustering2.publish(objectf[1]);
              clustering3.publish(objectf[2]);
              clustering4.publish(objectf[3]);
              masking.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8", mask_image).toImageMsg());

              ROS_INFO_STREAM("Clusters: " << objectf.size());
          }
          else std::cout << "The chosen hull is not planar." << std::endl;
      }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("points", 1, callback);
  segmented_objects = nh.advertise<PointCloud> ("segmented_objects",1);
  segmented_plane = nh.advertise<PointCloud> ("segmented_plane",1);
  segmented_convexHull = nh.advertise<PointCloud> ("segmented_convexHull",1);
  clustering1 = nh.advertise<PointCloud> ("cluster1",1);
  clustering2 = nh.advertise<PointCloud> ("cluster2",1);
  clustering3 = nh.advertise<PointCloud> ("cluster3",1);
  clustering4 = nh.advertise<PointCloud> ("cluster4",1);
  CameraInfo  = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera/rgb/camera_info");
  masking = nh.advertise<sensor_msgs::Image> ("mask_image",1);

  ros::spin();
}

