#ifndef __DYNAMIC_OBJECT__H
#define __DYNAMIC_OBJECT__H

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/correspondence.h>
#include <pcl/features/vfh.h>

#include <geometry_msgs/Transform.h>
#include <tf/transform_datatypes.h>

class DynamicObject  {
public:

    typedef pcl::PointXYZRGB PointType;
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

    typedef pcl::search::KdTree<PointType> Tree;
    typedef Tree::Ptr TreePtr;

    typedef pcl::Normal NormalType;
    typedef pcl::PointCloud<NormalType> NormalCloud;
    typedef typename NormalCloud::Ptr NormalCloudPtr;

    typedef boost::shared_ptr<DynamicObject> Ptr;

    struct ObjectTrack{
        tf::Transform pose;
        CloudPtr cloud;

        ObjectTrack()
        {
            cloud = CloudPtr(new Cloud());
        }
    };

	DynamicObject(bool verbose = false);
   ~DynamicObject();

   void setTime(boost::posix_time::ptime);
   void setCloud(CloudPtr);
   void setLabel(std::string label_);

   std::vector<tf::StampedTransform> getAdditionalViewTransforms();
   void addAdditionalViewTransform(tf::StampedTransform);
   std::vector<CloudPtr> getAdditionalViews();
   void addAdditionalView(CloudPtr view);
   void addAdditionalViewMask(cv::Mat& mask_image, std::vector<int> mask_indices);
   void clearAdditionalViewMasks();

   void setAdditionalViewsToObservationTransform(tf::StampedTransform);
   void addAdditionalViewTransformRegistered(tf::StampedTransform);

   void clearAdditionalViews();

   void updateAllData();
   void computeCentroid();
   void computeBbox();
   void computeNormals(double, int noThreads = 4);
   void computeVFHFeatures();
   void addObjectTrack(tf::Transform pose, CloudPtr cloud);

   bool operator==(const DynamicObject& rhs); // equality operator -> deep comparison of all fields
   bool operator!=(const DynamicObject& rhs); // equality operator -> deep comparison of all fields

   static void computeFitness(CloudPtr src, CloudPtr tgt, double& fitnessScore, int& noPoints, double max_range);



private:

public:
   std::string                      m_roomLogString;
   std::string                      m_roomStringId; // waypoint_#
   int                              m_roomRunNumber; // room_#

   CloudPtr                          m_points;
   std::vector<CloudPtr>             m_vAdditionalViews;
   std::vector<tf::StampedTransform> m_vAdditionalViewsTransforms;
   std::vector<tf::StampedTransform> m_vAdditionalViewsTransformsRegistered;
   std::vector<cv::Mat>              m_vAdditionalViewsMasks;
   tf::StampedTransform              m_AdditionalViewsTransformToObservation;
   std::vector<std::vector<int>>     m_vAdditionalViewMaskIndices;
   std::vector<cv::Mat>              m_vAdditionalViewMaskImages;

   std::vector<ObjectTrack>          m_vObjectTracks;
   int                               m_noAdditionalViews;
   NormalCloudPtr                    m_normals;
   std::string                       m_label;
   PointType                         m_bboxHighest, m_bboxLowest;
   Eigen::Vector4f                   m_centroid;
   boost::posix_time::ptime          m_time;
   std::vector<float>                m_vfh;
   bool                              m_bVerbose;



};

#endif // __DYNAMIC_OBJECT__H
