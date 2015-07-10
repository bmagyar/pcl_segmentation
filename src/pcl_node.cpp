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
#include <pcl_segmentation/ObjectCluster.h>
#include <pcl_segmentation/ObjectClusters.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>

pcl_segmentation::ObjectCluster obj;
//obj.mask =
//obj.object_id =
pcl_segmentation::ObjectClusters objs;
//objs.objects.push_back(obj);

#include <iostream>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
ros::Publisher segmented_objects;
ros::Publisher segmented_plane;
ros::Publisher clustering0;
ros::Publisher clustering1;
ros::Publisher clustering2;
ros::Publisher clustering3;
ros::Publisher clustering4;
ros::Publisher masking;
ros::Publisher color;

boost::shared_ptr<sensor_msgs::CameraInfo const> CameraInfo;
cv::Mat rgb_image;
int img_counter = 0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        rgb_image = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void pointCallback(const PointCloud::ConstPtr& msg)
{
    //printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
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

            // Get and show all points retrieved by the hull.
            prism.segment(*objectIndices);
            extract.setIndices(objectIndices);
            extract.filter(*objects);
            segmented_objects.publish(objects);
            segmented_plane.publish(plane);

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
                //cv::Mat mask_image = cv::Mat::zeros(CameraInfo->height, CameraInfo->width, CV_8UC1);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                extract_object_indices.setInputCloud(objects);
                extract_object_indices.setIndices(boost::make_shared<const pcl::PointIndices>(cluster_indices[i]));
                extract_object_indices.filter(*object_cloud);
                objectf.push_back(*object_cloud);

                for (int j = 0; j < object_cloud->points.size(); j++) {
                    pcl::PointXYZRGB p = object_cloud->points[j];
                    cv::Point2d uv = cam_model.project3dToPixel(cv::Point3d(p.x, p.y, p.z));
                    mask_image.at<uint8_t>(uv.y,uv.x) = (i+1)*(255/cluster_indices.size());
                    //printf ("\t(%f, %f)\n", uv.x, uv.y);
                }
                masking.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8", mask_image).toImageMsg());
            }
            std::stringstream ss;
            ss << "/home/jordi/catkin_ws/src/pcl_segmentation/pictures/Gray_Image" << img_counter << ".png";
            imwrite(ss.str(), mask_image);
            std::stringstream ss2;
            ss2 << "/home/jordi/catkin_ws/src/pcl_segmentation/pictures/RGB_Image" << img_counter << ".png";
            imwrite(ss2.str(), rgb_image);
            img_counter = img_counter + 1 ;
            clustering0.publish(objectf[0]);
            clustering1.publish(objectf[1]);
            clustering2.publish(objectf[2]);
            clustering3.publish(objectf[3]);
            clustering4.publish(objectf[4]);

            ROS_INFO_STREAM("Clusters: " << objectf.size());
        }
        else std::cout << "The chosen hull is not planar." << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("points", 1, pointCallback);
    ros::Subscriber sub2 = nh.subscribe<sensor_msgs::Image>("images", 1, imageCallback);
    segmented_objects = nh.advertise<PointCloud> ("segmented_objects",1);
    segmented_plane = nh.advertise<PointCloud> ("segmented_plane",1);
    clustering0 = nh.advertise<PointCloud> ("cluster0",1);
    clustering1 = nh.advertise<PointCloud> ("cluster1",1);
    clustering2 = nh.advertise<PointCloud> ("cluster2",1);
    clustering3 = nh.advertise<PointCloud> ("cluster3",1);
    clustering4 = nh.advertise<PointCloud> ("cluster4",1);
    CameraInfo  = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera/rgb/camera_info");
    masking = nh.advertise<sensor_msgs::Image> ("mask_image",1);

    ros::spin();
}
