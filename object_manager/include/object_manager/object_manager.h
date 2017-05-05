#ifndef __SEMANTIC_MAP_PUBLISHER__H
#define __SEMANTIC_MAP_PUBLISHER__H

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>


// ROS includes
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

// Services
#include <object_manager_msgs/DynamicObjectsService.h>
#include <object_manager_msgs/GetDynamicObjectService.h>
#include <object_manager_msgs/ProcessDynamicObjectService.h>

// Registration service
#include <observation_registration_services/ObjectAdditionalViewRegistrationService.h>

// Additional view mask service
#include <object_manager_msgs/DynamicObjectComputeMaskService.h>

// Custom messages
#include <object_manager_msgs/DynamicObjectTracks.h>
#include <object_manager_msgs/DynamicObjectTrackingData.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/registration/distances.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <geometry_msgs/Point.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/frustum_culling.h>
#include <semantic_map/room_xml_parser.h>
#include <semantic_map/reg_transforms.h>
#include <semantic_map/reg_features.h>
#include <cv_bridge/cv_bridge.h>

// application includes
#include <metaroom_xml_parser/load_utilities.h>
#include "object_manager/dynamic_object.h"
#include "object_manager/dynamic_object_xml_parser.h"
#include "object_manager/dynamic_object_utilities.h"
#include "object_manager/dynamic_object_mongodb_interface.h"

template <class PointType>
class ObjectManager {
public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef semantic_map_load_utilties::DynamicObjectData<PointType> ObjectData;

    typedef typename object_manager_msgs::DynamicObjectsService::Request DynamicObjectsServiceRequest;
    typedef typename object_manager_msgs::DynamicObjectsService::Response DynamicObjectsServiceResponse;

    typedef typename object_manager_msgs::GetDynamicObjectService::Request GetDynamicObjectServiceRequest;
    typedef typename object_manager_msgs::GetDynamicObjectService::Response GetDynamicObjectServiceResponse;

    typedef typename object_manager_msgs::ProcessDynamicObjectService::Request ProcessDynamicObjectServiceRequest;
    typedef typename object_manager_msgs::ProcessDynamicObjectService::Response ProcessDynamicObjectServiceResponse;

    struct GetObjStruct
    {
        CloudPtr object_cloud;
        std::vector<int> object_indices;
        tf::StampedTransform transform_to_map;
        cv::Mat object_mask;
        int pan_angle, tilt_angle;
    };

    ObjectManager(ros::NodeHandle nh);
    ~ObjectManager();

    bool dynamicObjectsServiceCallback(DynamicObjectsServiceRequest &req, DynamicObjectsServiceResponse &res);
    bool getDynamicObjectServiceCallback(GetDynamicObjectServiceRequest &req, GetDynamicObjectServiceResponse &res);
    bool processDynamicObjectServiceCallback(ProcessDynamicObjectServiceRequest &req, ProcessDynamicObjectServiceResponse &res);
    std::vector<DynamicObject::Ptr>  loadDynamicObjectsFromObservation(std::string obs_file);
    bool getDynamicObject(std::string waypoint, std::string object_id, DynamicObject::Ptr& object, std::string& object_observation);
    bool returnObjectMask(std::string waypoint, std::string object_id, std::string observation_xml, GetObjStruct& returned_object);
    void additionalViewsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void additionalViewsStatusCallback(const std_msgs::String& msg);
    void dynamicObjectTracksCallback(const object_manager_msgs::DynamicObjectTracksConstPtr& msg);

    static CloudPtr filterGroundClusters(CloudPtr dynamic, double min_height)
    {
        CloudPtr filtered(new Cloud());


        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud (dynamic);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-1.0, min_height);
        pass.setFilterLimitsNegative (true);
        pass.filter (*filtered);

        return filtered;
    }


private:

    bool updateObjectsAtWaypoint(std::string waypoint_id);

    ros::NodeHandle                                                             m_NodeHandle;
    ros::Publisher                                                              m_PublisherDynamicClusters;
    ros::Publisher                                                              m_PublisherRequestedObjectCloud;
    ros::Publisher                                                              m_PublisherRequestedObjectImage;
    ros::Publisher                                                              m_PublisherLearnedObjectXml;
    ros::Publisher                                                              m_PublisherLearnedObjectTrackingData;
    ros::Publisher                                                              m_PublisherLearnedObjectModel;
    ros::Subscriber                                                             m_SubscriberDynamicObjectTracks;
    ros::Subscriber                                                             m_SubscriberAdditionalObjectViews;
    ros::Subscriber                                                             m_SubscriberAdditionalObjectViewsStatus;
    ros::ServiceServer                                                          m_DynamicObjectsServiceServer;
    ros::ServiceServer                                                          m_GetDynamicObjectServiceServer;
    ros::ServiceServer                                                          m_ProcessDynamicObjectServiceServer;
    tf::TransformListener                                                       m_TransformListener; // for additional views

    std::string                                                                 m_additionalViewsTopic;
    std::string                                                                 m_additionalViewsStatusTopic;
    std::string                                                                 m_dataFolder;
    std::map<std::string, std::vector<DynamicObject::Ptr>>                      m_waypointToObjMap;
    std::map<std::string, std::string>                                          m_waypointToSweepFileMap;
    bool                                                                        m_bLogToDB;
    bool                                                                        m_bTrackingStarted;
    bool                                                                        m_bSaveMask;
    DynamicObject::Ptr                                                          m_objectTracked;
    std::string                                                                 m_objectTrackedObservation; // for saving
    int										m_MinClusterSize;
};

template <class PointType>
ObjectManager<PointType>::ObjectManager(ros::NodeHandle nh) : m_TransformListener(nh,ros::Duration(1000))
{
    ROS_INFO_STREAM("ObjectManager node initialized");

    m_NodeHandle = nh;

    passwd* pw = getpwuid(getuid());
    std::string default_folder(pw->pw_dir);
    default_folder+="/.semanticMap/";

    m_NodeHandle.param<std::string>("object_folder",m_dataFolder,default_folder);
    ROS_INFO_STREAM("Reading dynamic object from "<<m_dataFolder);

    m_PublisherDynamicClusters = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/object_manager/objects", 1, true);

    m_PublisherRequestedObjectCloud = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/object_manager/requested_object", 1, true);
    m_PublisherRequestedObjectImage = m_NodeHandle.advertise<sensor_msgs::Image>("/object_manager/requested_object_mask", 1, true);
    m_PublisherLearnedObjectXml = m_NodeHandle.advertise<std_msgs::String>("/object_learning/learned_object_xml", 1, false);
    m_PublisherLearnedObjectTrackingData = m_NodeHandle.advertise<object_manager_msgs::DynamicObjectTrackingData>("/object_learning/learned_object_tracking_data", 1, false);
    m_PublisherLearnedObjectModel = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/object_learning/learned_object_model", 1, false);

    m_DynamicObjectsServiceServer = m_NodeHandle.advertiseService("ObjectManager/DynamicObjectsService", &ObjectManager::dynamicObjectsServiceCallback, this);
    m_GetDynamicObjectServiceServer = m_NodeHandle.advertiseService("ObjectManager/GetDynamicObjectService", &ObjectManager::getDynamicObjectServiceCallback, this);
    m_ProcessDynamicObjectServiceServer = m_NodeHandle.advertiseService("ObjectManager/ProcessDynamicObjectService", &ObjectManager::processDynamicObjectServiceCallback, this);


    m_NodeHandle.param<bool>("log_objects_to_db",m_bLogToDB,true);
    if (m_bLogToDB)
    {
        ROS_INFO_STREAM("The dynamic objects will be logged to the database.");
    } else {
        ROS_INFO_STREAM("The dynamic objects will NOT be logged to the database.");
    }
    m_NodeHandle.param<bool>("save_mask",m_bSaveMask,true);
    if (m_bLogToDB)
    {
        ROS_INFO_STREAM("The dynamic object mask (image and indices) will be saved on the disk.");
    } else {
        ROS_INFO_STREAM("The dynamic object mask (image and indices) will NOT be saved on the disk.");
    }



    m_NodeHandle.param<int>("min_object_size",m_MinClusterSize,500);
    ROS_INFO_STREAM("ObjectManager:: min object size set to "<<m_MinClusterSize);
    
    m_NodeHandle.param<std::string>("additional_views_topic",m_additionalViewsTopic,"/object_learning/object_view");
    ROS_INFO_STREAM("The additional views topic is "<<m_additionalViewsTopic);
    m_SubscriberAdditionalObjectViews = m_NodeHandle.subscribe(m_additionalViewsTopic,1, &ObjectManager<PointType>::additionalViewsCallback,this);

    m_NodeHandle.param<std::string>("additional_views_status_topic",m_additionalViewsStatusTopic,"/object_learning/status");
    ROS_INFO_STREAM("The additional views status topic is "<<m_additionalViewsStatusTopic);
    m_SubscriberAdditionalObjectViewsStatus = m_NodeHandle.subscribe(m_additionalViewsStatusTopic,1, &ObjectManager<PointType>::additionalViewsStatusCallback,this);
    m_objectTracked = NULL;
    m_objectTrackedObservation = "";
    m_bTrackingStarted = false;

    m_SubscriberDynamicObjectTracks = m_NodeHandle.subscribe("/object_learning/dynamic_object_tracks",1, &ObjectManager::dynamicObjectTracksCallback,this);
}



template <class PointType>
ObjectManager<PointType>::~ObjectManager()
{

}

template <class PointType>
void ObjectManager<PointType>::additionalViewsStatusCallback(const std_msgs::String& controlString)
{
    if (m_objectTracked == NULL)
    {
        ROS_ERROR_STREAM("Received an additional views status message but a dynamic object hasn't been selected yet");
        return;
    }

    if (controlString.data == "start_viewing")
    {
        ROS_INFO_STREAM("Starting viewing of object "<<m_objectTracked->m_label);
        m_bTrackingStarted = true;
    }

    // save object
    unsigned found = m_objectTrackedObservation.find_last_of("/");
    std::string obs_folder = m_objectTrackedObservation.substr(0,found+1);
    DynamicObjectXMLParser parser(obs_folder, true);
    std::string xml_file = parser.saveAsXML(m_objectTracked);
    ROS_INFO_STREAM("Object saved at "<<xml_file);

    if (controlString.data == "stop_viewing")
    {
        ROS_INFO_STREAM("Stopping viewing of object "<<m_objectTracked->m_label);
        m_bTrackingStarted = false;
        // register object
        if (m_objectTracked->m_vAdditionalViews.size()){
            ros::ServiceClient client = m_NodeHandle.serviceClient<observation_registration_services::ObjectAdditionalViewRegistrationService>("/object_additional_view_registration_server");
            observation_registration_services::ObjectAdditionalViewRegistrationService srv;
            srv.request.observation_xml = m_objectTrackedObservation;
            srv.request.object_xml = xml_file;

            std::vector<tf::Transform> registered_transforms;

            ROS_INFO_STREAM("Using object_additional_view_registration_server service");
            if (client.call(srv))
            {
                int total_constraints = 0;
                std::for_each(srv.response.additional_view_correspondences.begin(), srv.response.additional_view_correspondences.end(), [&] (int n) {
                    total_constraints += n;
                });

                ROS_INFO_STREAM("Registration done. Number of additional view registration constraints "<<total_constraints<<". Number of additional view transforms "<<srv.response.additional_view_transforms.size());
                if ((total_constraints <= 0) && (m_objectTracked->m_vAdditionalViews.size() != 1)){
                    ROS_INFO_STREAM("Additional view Registration unsuccessful due to insufficient constraints.");
                } else {
                    m_objectTracked->m_vAdditionalViewsTransformsRegistered.clear();
                    for (auto tr : srv.response.additional_view_transforms){
                        tf::Transform transform;
                        tf::transformMsgToTF(tr, transform);
                        tf::StampedTransform stamped_transform(transform, ros::Time::now(), "", "");
                        m_objectTracked->m_vAdditionalViewsTransformsRegistered.push_back(stamped_transform);
                    }

                    tf::Transform obs_transform;
                    tf::transformMsgToTF(srv.response.observation_transform, obs_transform);
                    tf::StampedTransform obs_stamped_transform(obs_transform, ros::Time::now(), "", "");
                    m_objectTracked->m_AdditionalViewsTransformToObservation = obs_stamped_transform;
                }
            } else {
                ROS_ERROR_STREAM("Could not call object_additional_view_registration_server to register object views");
            }
        }


        if (m_objectTracked->m_vAdditionalViewsTransformsRegistered.size()){
            // we got some registered transforms -> save object again
            parser.saveAsXML(m_objectTracked);
            ROS_INFO_STREAM("Object saved after registering additional views at "<<xml_file);
        }


        if (m_bLogToDB)
        {
            DynamicObjectMongodbInterface mongo_inteface(m_NodeHandle, true);
            mongo_inteface.logToDB(m_objectTracked, xml_file);
        }

        // create additional view masks
        ros::ServiceClient client_masks = m_NodeHandle.serviceClient<object_manager_msgs::DynamicObjectComputeMaskService>("/dynamic_object_compute_mask_server");
        object_manager_msgs::DynamicObjectComputeMaskService srv_masks;
        srv_masks.request.observation_xml = m_objectTrackedObservation;
        srv_masks.request.object_xml = xml_file;

        ROS_INFO_STREAM("Using dynamic_object_compute_mask_server service");
        if (client_masks.call(srv_masks))
        {
            ROS_INFO_STREAM("Service dynamic_object_compute_mask_server finished. Masks computed (hopefully).");
        } else {
            ROS_ERROR_STREAM("Could not call dynamic_object_compute_mask_server to create masks for the object views");
        }

        // publish object xml
        std_msgs::String object_xml_msg;
        object_xml_msg.data = xml_file;
        m_PublisherLearnedObjectXml.publish(object_xml_msg);

        // reset object
        m_objectTracked = NULL;

        // re-load object and publish tracks data
        ObjectData object = semantic_map_load_utilties::loadDynamicObjectFromSingleSweep<PointType>(xml_file);
        object_manager_msgs::DynamicObjectTrackingData tracking_data_msg;

        // views
        for (auto obj_view : object.vAdditionalViews){
            sensor_msgs::PointCloud2 view_msg;
            pcl::toROSMsg(*obj_view, view_msg);
            tracking_data_msg.additional_views.push_back(view_msg);
        }

        // transforms
        for (auto reg_tf : object.vAdditionalViewsTransformsRegistered){
            geometry_msgs::Transform registered_transform_msg;
            tf::transformTFToMsg(reg_tf, registered_transform_msg);
            tracking_data_msg.additional_view_transforms.push_back(registered_transform_msg);
        }

        // mask
        if (object.vAdditionalViews.size()){
            tracking_data_msg.object_mask.assign(object.vAdditionalViewMaskIndices[0].begin(), object.vAdditionalViewMaskIndices[0].end());
        }

        sensor_msgs::PointCloud2 learned_object_model = srv_masks.response.segmented_object;
        learned_object_model.header.frame_id = "/map";
        m_PublisherLearnedObjectModel.publish(learned_object_model);
        m_PublisherLearnedObjectTrackingData.publish(tracking_data_msg);

        // also save data to the disk
        int index = m_objectTrackedObservation.find_last_of("/");
        std::string sweep_folder = m_objectTrackedObservation.substr(0,index+1);
//        std::string poses_file = sweep_folder+"/poses.txt";


        for (size_t i=0; i < object.vAdditionalViewsTransformsRegistered.size(); i++){
            tf::Transform tr = object.vAdditionalViewsTransformsRegistered[i];
            std::stringstream ss;ss<<i;
            std::string poses_file = sweep_folder + "/pose_" + object.objectLabel + "_additional_view_"+ss.str()+".txt";
            ofstream out(poses_file, ios::out | ios::trunc);
            ROS_INFO_STREAM("Saving additional view pose "<<i<<" at "<<poses_file);

            Eigen::Affine3d eigen_affine; tf::transformTFToEigen(tr, eigen_affine);
            Eigen::Matrix4f eigen_tr(eigen_affine.matrix().cast<float>());
            Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "", "", "", "");
            std::cout<<eigen_tr.format(CommaInitFmt)<<std::endl;
            out<<eigen_tr.format(CommaInitFmt)<<std::endl;
            out.close();
        }
    }
}

template <class PointType>
void ObjectManager<PointType>::additionalViewsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (m_objectTracked == NULL)
    {
        ROS_ERROR_STREAM("Received an additional views message but a dynamic object hasn't been selected yet");
        return;
    }

    if (m_bTrackingStarted)
    {
        CloudPtr new_cloud(new Cloud());
        pcl::fromROSMsg(*msg, *new_cloud);
        new_cloud->header = pcl_conversions::toPCL(msg->header);

        try {
            // get additional views pose
            ROS_INFO_STREAM("Additional views cloud is in frame ["<<new_cloud->header.frame_id<<"] requesting and storing transform to map frame. ");
            tf::StampedTransform transform;
            m_TransformListener.waitForTransform("/map", new_cloud->header.frame_id,msg->header.stamp, ros::Duration(20.0) );
            m_TransformListener.lookupTransform("/map", new_cloud->header.frame_id,
                                                msg->header.stamp, transform);

            m_objectTracked->addAdditionalViewTransform(transform);
            m_objectTracked->addAdditionalView(new_cloud);
        }         catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }


    } else {
        ROS_ERROR_STREAM("Received an additional view when we're not viewing an object.");
    }


}

template <class PointType>
void ObjectManager<PointType>::dynamicObjectTracksCallback(const object_manager_msgs::DynamicObjectTracksConstPtr& msg)
{
    if (m_objectTracked == NULL)
    {
        ROS_ERROR_STREAM("Received a dynamic object tracks message but a dynamic object hasn't been selected yet");
        return;
    }

    if (m_bTrackingStarted)
    {
        ROS_INFO_STREAM("Received a dynamic object tracks message. Object tracked id: "<<m_objectTracked->m_label);
        ROS_INFO_STREAM("Dynamic object tracks message contains "<<msg->poses.size()<<" poses and  "<<msg->clouds.size()<<" clouds.");
        if (msg->poses.size() != msg->clouds.size())
        {
            ROS_ERROR_STREAM("The dynamic object tracks message contains an unequal number of poses and clouds. Aborting");
            return;
        }

        // TESTING
        m_objectTracked->m_bVerbose = true;
        //        SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(m_objectTrackedObservation,true);
        //        auto clouds = aRoom.getIntermediateClouds();
        //        auto transforms = aRoom.getIntermediateCloudTransforms();
        //        for (size_t i=0; i<clouds.size(); i++)
        //        {
        //            m_objectTracked->addObjectTrack(transforms[i], clouds[i]);
        //        }

        for (size_t i=0; i<msg->clouds.size(); i++)
        {
            // cloud message
            CloudPtr new_cloud(new Cloud());
            pcl::fromROSMsg(msg->clouds[i], *new_cloud);
            new_cloud->header = pcl_conversions::toPCL(msg->clouds[i].header);
            // pose message
            tf::Transform pose;
            tf::transformMsgToTF(msg->poses[i],pose);

            if (new_cloud->points.size() == 0)
            {
                ROS_ERROR_STREAM("Provided point cloud contains 0 points. Skipping");
                continue;
            } else {
                m_objectTracked->addObjectTrack(pose, new_cloud);
            }
        }
    } else {
        ROS_ERROR_STREAM("Received a dynamic object tracks message when we're not viewing an object.");
    }

}

template <class PointType>
bool ObjectManager<PointType>::dynamicObjectsServiceCallback(DynamicObjectsServiceRequest &req, DynamicObjectsServiceResponse &res)
{
    ROS_INFO_STREAM("Received a dynamic clusters request for waypoint "<<req.waypoint_id);

    using namespace std;

    std::vector<DynamicObject::Ptr> currentObjects;
    bool objects_found = updateObjectsAtWaypoint(req.waypoint_id);
    if (!objects_found)
    {
        return true;
    }
    currentObjects = m_waypointToObjMap[req.waypoint_id];
    ROS_INFO_STREAM("Found "<<currentObjects.size() <<" objects at "<<req.waypoint_id);

    // publish objects
    CloudPtr allObjects(new Cloud());

    for (auto cloudObj : currentObjects)
    {
        *allObjects += *(cloudObj->m_points);
    }

    sensor_msgs::PointCloud2 msg_objects;
    pcl::toROSMsg(*allObjects, msg_objects);
    msg_objects.header.frame_id="/map";

    m_PublisherDynamicClusters.publish(msg_objects);

    std::vector<sensor_msgs::PointCloud2> clouds;
    std::vector<std::string> id;
    std::vector<geometry_msgs::Point> centroids;

    for (auto cloudObj : currentObjects)
    {
        sensor_msgs::PointCloud2 msg_objects;
        pcl::toROSMsg(*cloudObj->m_points, msg_objects);
        msg_objects.header.frame_id="/map";

        clouds.push_back(msg_objects);
        id.push_back(cloudObj->m_label);


        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloudObj->m_points, centroid);

        geometry_msgs::Point ros_centroid; ros_centroid.x = centroid[0];ros_centroid.y = centroid[1];ros_centroid.z = centroid[2];
        centroids.push_back(ros_centroid);
    }

    res.objects = clouds;
    res.object_id = id;
    res.centroids = centroids;

    return true;


}

template <class PointType>
bool ObjectManager<PointType>::getDynamicObject(std::string waypoint, std::string object_id, DynamicObject::Ptr& object, std::string& object_observation)
{
	printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);
    auto it =  m_waypointToObjMap.find(waypoint);
    if (it == m_waypointToObjMap.end() )
    {
        ROS_ERROR_STREAM("No objects loaded for waypoint "+waypoint);
        return false;
    }
	printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);
    std::vector<DynamicObject::Ptr> objects = m_waypointToObjMap[waypoint];
    printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);
    bool found = false;
    for (auto objectStruct : objects)
    {
        if (objectStruct->m_label == object_id)
        {
            found = true;
            object = objectStruct;
            object_observation = m_waypointToSweepFileMap[waypoint];
            break;
        }
    }
    	printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);
    if(!found)
    {
        ROS_ERROR_STREAM("Object "<<object_id<<" at waypoint "<<waypoint<<" could not be found.");
    }
    	printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);
    return true;
}

template <class PointType>
bool ObjectManager<PointType>::processDynamicObjectServiceCallback(ProcessDynamicObjectServiceRequest &req, ProcessDynamicObjectServiceResponse &res)
{
    using namespace std;
    ROS_INFO("Received a process dynamic object service request");
    ROS_INFO_STREAM("Object " << req.object_xml);
    ROS_INFO_STREAM("Observation "<<req.observation_xml);

    unsigned found = req.observation_xml.find_last_of("/");
    std::string obs_folder = req.observation_xml.substr(0,found+1);
    DynamicObjectXMLParser parser(obs_folder, true);
    DynamicObject::Ptr object = parser.loadFromXML(req.object_xml,true);

    if ((!object->m_points) || (!object->m_points->points.size())){
        // could not load object
        ROS_ERROR_STREAM("processDynamicObjectServiceCallback: could not load object from " << req.object_xml);
        throw std::runtime_error(
                    std::string("processDynamicObjectServiceCallback: could not load object. \n"));
        return true;
    }

    // register object
    if (object->m_vAdditionalViews.size()){
        ros::ServiceClient client = m_NodeHandle.serviceClient<observation_registration_services::ObjectAdditionalViewRegistrationService>("/object_additional_view_registration_server");
        observation_registration_services::ObjectAdditionalViewRegistrationService srv;
        srv.request.observation_xml = req.observation_xml;
        srv.request.object_xml = req.object_xml;

        std::vector<tf::Transform> registered_transforms;

        ROS_INFO_STREAM("Using object_additional_view_registration_server service");
        if (client.call(srv))
        {
            int total_constraints = 0;
            std::for_each(srv.response.additional_view_correspondences.begin(), srv.response.additional_view_correspondences.end(), [&] (int n) {
                total_constraints += n;
            });

            ROS_INFO_STREAM("Registration done. Number of additional view registration constraints "<<total_constraints<<". Number of additional view transforms "<<srv.response.additional_view_transforms.size());
            if ((total_constraints <= 0) && (object->m_vAdditionalViews.size() != 1)){
                ROS_INFO_STREAM("Additional view Registration unsuccessful due to insufficient constraints.");
            } else {
                object->m_vAdditionalViewsTransformsRegistered.clear();
                for (auto tr : srv.response.additional_view_transforms){
                    tf::Transform transform;
                    tf::transformMsgToTF(tr, transform);
                    tf::StampedTransform stamped_transform(transform, ros::Time::now(), "", "");
                    object->m_vAdditionalViewsTransformsRegistered.push_back(stamped_transform);
                }

                tf::Transform obs_transform;
                tf::transformMsgToTF(srv.response.observation_transform, obs_transform);
                tf::StampedTransform obs_stamped_transform(obs_transform, ros::Time::now(), "", "");
                object->m_AdditionalViewsTransformToObservation = obs_stamped_transform;
            }
        } else {
            ROS_ERROR_STREAM("Could not call object_additional_view_registration_server to register object views");
        }
    }


    if (object->m_vAdditionalViewsTransformsRegistered.size()){
        // we got some registered transforms -> save object again
        ROS_INFO_STREAM("Saving object");
        string saved_xml = parser.saveAsXML(object);
        ROS_INFO_STREAM("Object saved after registering additional views at "<<saved_xml);
    }

    // no need for logging here
//    if (m_bLogToDB)
//    {
//        DynamicObjectMongodbInterface mongo_inteface(m_NodeHandle, true);
//        mongo_inteface.logToDB(m_objectTracked, xml_file);
//    }

    // create additional view masks
    ros::ServiceClient client_masks = m_NodeHandle.serviceClient<object_manager_msgs::DynamicObjectComputeMaskService>("/dynamic_object_compute_mask_server");
    object_manager_msgs::DynamicObjectComputeMaskService srv_masks;
    srv_masks.request.observation_xml = req.observation_xml;
    srv_masks.request.object_xml = req.object_xml;

    ROS_INFO_STREAM("Using dynamic_object_compute_mask_server service");
    if (client_masks.call(srv_masks))
    {
        ROS_INFO_STREAM("Service dynamic_object_compute_mask_server finished. Masks computed (hopefully).");
    } else {
        ROS_ERROR_STREAM("Could not call dynamic_object_compute_mask_server to create masks for the object views");
    }

    // re-load object and publish tracks data
    ObjectData processed_object = semantic_map_load_utilties::loadDynamicObjectFromSingleSweep<PointType>(req.object_xml);
    object_manager_msgs::DynamicObjectTrackingData tracking_data_msg;

    // views
    for (auto obj_view : processed_object.vAdditionalViews){
        sensor_msgs::PointCloud2 view_msg;
        pcl::toROSMsg(*obj_view, view_msg);
        tracking_data_msg.additional_views.push_back(view_msg);
    }

    // transforms
    for (auto reg_tf : processed_object.vAdditionalViewsTransformsRegistered){
        geometry_msgs::Transform registered_transform_msg;
        tf::transformTFToMsg(reg_tf, registered_transform_msg);
        tracking_data_msg.additional_view_transforms.push_back(registered_transform_msg);
    }

    // mask
    if (processed_object.vAdditionalViews.size()){
        tracking_data_msg.object_mask.assign(processed_object.vAdditionalViewMaskIndices[0].begin(), processed_object.vAdditionalViewMaskIndices[0].end());
    }

    sensor_msgs::PointCloud2 learned_object_model = srv_masks.response.segmented_object;
    learned_object_model.header.frame_id = "/map";
    m_PublisherLearnedObjectModel.publish(learned_object_model);
    m_PublisherLearnedObjectTrackingData.publish(tracking_data_msg);

//    // also save data to the disk
//    int index = m_objectTrackedObservation.find_last_of("/");
//    std::string sweep_folder = m_objectTrackedObservation.substr(0,index+1);
////        std::string poses_file = sweep_folder+"/poses.txt";


//    for (size_t i=0; i < object.vAdditionalViewsTransformsRegistered.size(); i++){
//        tf::Transform tr = object.vAdditionalViewsTransformsRegistered[i];
//        std::stringstream ss;ss<<i;
//        std::string poses_file = sweep_folder + "/pose_" + object.objectLabel + "_additional_view_"+ss.str()+".txt";
//        ofstream out(poses_file, ios::out | ios::trunc);
//        ROS_INFO_STREAM("Saving additional view pose "<<i<<" at "<<poses_file);

//        Eigen::Affine3d eigen_affine; tf::transformTFToEigen(tr, eigen_affine);
//        Eigen::Matrix4f eigen_tr(eigen_affine.matrix().cast<float>());
//        Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "", "", "", "");
//        std::cout<<eigen_tr.format(CommaInitFmt)<<std::endl;
//        out<<eigen_tr.format(CommaInitFmt)<<std::endl;
//        out.close();
//    }

    return true;
}

template <class PointType>
bool ObjectManager<PointType>::getDynamicObjectServiceCallback(GetDynamicObjectServiceRequest &req, GetDynamicObjectServiceResponse &res)
{
    ROS_INFO_STREAM("Received a get dynamic cluster request for waypoint "<<req.waypoint_id);

    using namespace std;

    std::vector<DynamicObject::Ptr> currentObjects;
    bool objects_found = updateObjectsAtWaypoint(req.waypoint_id);
    if (!objects_found)
    {
        return true;
    }

	printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);
    GetObjStruct object;
    bool found =returnObjectMask(req.waypoint_id, req.object_id,m_waypointToSweepFileMap[req.waypoint_id], object);
    if (!found)
    {
        ROS_ERROR_STREAM("Could not compute mask for object id "<<req.object_id);
        return true;
    }
	printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);
    res.object_mask = object.object_indices;
    tf::transformTFToMsg(object.transform_to_map, res.transform_to_map);
    pcl::toROSMsg(*object.object_cloud, res.object_cloud);
    res.object_cloud.header.frame_id="/map";
    res.pan_angle = -object.pan_angle;
    res.tilt_angle = -object.tilt_angle;
	printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);
    m_PublisherRequestedObjectCloud.publish(res.object_cloud);
	printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);
    // convert to sensor_msgs::Image
    cv_bridge::CvImage aBridgeImage;
    aBridgeImage.image = object.object_mask;
    aBridgeImage.encoding = "bgr8";
    sensor_msgs::ImagePtr rosImage = aBridgeImage.toImageMsg();
    m_PublisherRequestedObjectImage.publish(rosImage);
	printf("%s::%i\n",__PRETTY_FUNCTION__,__LINE__);
    // update tracked object
    bool trackedUpdated = getDynamicObject(req.waypoint_id, req.object_id, m_objectTracked, m_objectTrackedObservation);
    if (!trackedUpdated)
    {
        ROS_ERROR_STREAM("Could not find object for viewing");
    } else {
        // check if this object doesn't have any additional views -> if not, add the intermediate cloud & its transform
        //        if (!m_objectTracked->m_vAdditionalViews.size()){
        //            m_objectTracked->addAdditionalView(object.object_cloud);
        //            m_objectTracked->addAdditionalViewTransform(object.transform_to_map);
        //        }
    }

    return true;
}

template <class PointType>
bool ObjectManager<PointType>::updateObjectsAtWaypoint(std::string waypoint_id)
{
	printf("START bool ObjectManager<PointType>::updateObjectsAtWaypoint(std::string waypoint_id)\n");
    using namespace std;
    std::vector<std::string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(m_dataFolder, waypoint_id);
    if (matchingObservations.size() == 0)
    {
        ROS_INFO_STREAM("No observations for this waypoint "<<waypoint_id);
        return false;
    }

    ROS_INFO_STREAM("Found "<<matchingObservations.size()<<" matching observations.");

    //    sort(matchingObservations.begin(), matchingObservations.end());


    reverse(matchingObservations.begin(), matchingObservations.end());
    string latest = matchingObservations[0];

    ROS_INFO_STREAM("Latest observation "<<latest);

    auto it = m_waypointToSweepFileMap.find(waypoint_id);
    if (it == m_waypointToSweepFileMap.end())
    {
        // no dynamic clusters loaded for this waypoint
        // -> load them
        std::vector<DynamicObject::Ptr> dynamicObjects = loadDynamicObjectsFromObservation(latest);
        if (dynamicObjects.size() == 0)
        {
            ROS_INFO_STREAM("No objects detected after clustering.");
            return false;
        }

        m_waypointToSweepFileMap[waypoint_id] = latest;
        m_waypointToObjMap[waypoint_id] = dynamicObjects;
    } else {
        if (m_waypointToSweepFileMap[waypoint_id] != latest)
        {
            ROS_INFO_STREAM("Older point cloud loaded in memory. Loading objects from latest observation ...");
            std::vector<DynamicObject::Ptr> dynamicObjects = loadDynamicObjectsFromObservation(latest);
            if (dynamicObjects.size() == 0)
            {
                ROS_INFO_STREAM("No objects detected after clustering.");
                return false;
            }

            m_waypointToSweepFileMap[waypoint_id] = latest;
            m_waypointToObjMap[waypoint_id] = dynamicObjects;
        } else {
            ROS_INFO_STREAM("Objects loaded in memory");
            auto it2 =  m_waypointToObjMap.find(waypoint_id);
            if (it2 == m_waypointToObjMap.end())
            {
                ROS_ERROR_STREAM("Object map is empty. Reload.");
                std::vector<DynamicObject::Ptr> dynamicObjects = loadDynamicObjectsFromObservation(latest);
                if (dynamicObjects.size() == 0)
                {
                    ROS_INFO_STREAM("No objects detected after clustering.");
                    return false;
                }

                m_waypointToSweepFileMap[waypoint_id] = latest;
                m_waypointToObjMap[waypoint_id] = dynamicObjects;
            }
        }
    }

    return true;
}

template <class PointType>
std::vector<DynamicObject::Ptr>  ObjectManager<PointType>::loadDynamicObjectsFromObservation(std::string obs_file)
{
	printf("START std::vector<DynamicObject::Ptr>  ObjectManager<PointType>::loadDynamicObjectsFromObservation(std::string obs_file)\n");
    std::vector<DynamicObject::Ptr> dynamicObjects;

    // check if the objects have been computed already
    {
        unsigned found = obs_file.find_last_of("/");
        std::string obs_folder = obs_file.substr(0,found+1);
        dynamicObjects = dynamic_object_utilities::loadDynamicObjects(obs_folder);
        if (dynamicObjects.size() != 0)
        {
            ROS_INFO_STREAM("Dyanmic objects previously saved. Loading them from disk. Loaded "<<dynamicObjects.size()<<" objects.");
            return dynamicObjects;
        }
    }

    auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(obs_file, std::vector<std::string>{"RoomDynamicClusters"},false);
    if (sweep.dynamicClusterCloud->points.size() == 0)
    {
        // no clusters in the observation
        return dynamicObjects;
    }

    sweep.dynamicClusterCloud = filterGroundClusters(sweep.dynamicClusterCloud, 0.2);

    double tolerance = 0.03;
    int min_cluster_size = m_MinClusterSize;
    int max_cluster_size = 10000;
    // split into clusters
    typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (sweep.dynamicClusterCloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (sweep.dynamicClusterCloud);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        CloudPtr cloud_cluster (new Cloud());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (sweep.dynamicClusterCloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::stringstream ss;ss<<"object_";ss<<j;
        //        ObjStruct object;
        //        object.cloud = cloud_cluster;
        //        object.id = ss.str();



        // create and save dynamic object
        DynamicObject::Ptr roomObject(new DynamicObject());
        roomObject->setCloud(cloud_cluster);
        roomObject->setTime(sweep.roomLogStartTime);
        roomObject->m_roomLogString = sweep.roomLogName;
        roomObject->m_roomStringId = sweep.roomWaypointId;
        roomObject->m_roomRunNumber = sweep.roomRunNumber;
        // create label from room log time; could be useful later on, and would resolve ambiguities
        std::stringstream ss_obj;
        ss_obj<<boost::posix_time::to_simple_string(sweep.roomLogStartTime);
        ss_obj<<"_object_";ss_obj<<j;
        roomObject->m_label = ss_obj.str();
        // save dynamic object
        unsigned found = obs_file.find_last_of("/");
        std::string obs_folder = obs_file.substr(0,found+1);
        DynamicObjectXMLParser parser(obs_folder, true);
        std::string xml_file = parser.saveAsXML(roomObject);

        if (m_bLogToDB)
        {
            DynamicObjectMongodbInterface mongo_inteface(m_NodeHandle, true);
            mongo_inteface.logToDB(roomObject, xml_file);
        }

        dynamicObjects.push_back(roomObject);
        j++;
    }



    return dynamicObjects;
}

template <class PointType>
bool ObjectManager<PointType>::returnObjectMask(std::string waypoint, std::string object_id, std::string observation_xml, GetObjStruct& returned_object)
{
    auto it =  m_waypointToObjMap.find(waypoint);
    if (it == m_waypointToObjMap.end() )
    {
        ROS_ERROR_STREAM("No objects loaded for waypoint "+waypoint);
        return false;
    }

    std::vector<DynamicObject::Ptr> objects = m_waypointToObjMap[waypoint];
    DynamicObject::Ptr object;
    bool found = false;
    for (auto objectStruct : objects)
    {
        if (objectStruct->m_label == object_id)
        {
            found = true;
            object = objectStruct;
            break;
        }
    }

    if (!found)
    {
        ROS_ERROR_STREAM("Cannot find object "+object_id+" at waypoint "+waypoint);
        return false;
    }


    // load observation and individual clouds
    SemanticRoom<PointType> observation = SemanticRoomXMLParser<PointType>::loadRoomFromXML(observation_xml,true);

    //    int argc = 0;
    //    char** argv;
    //    pcl::visualization::PCLVisualizer* p = new pcl::visualization::PCLVisualizer (argc, argv, "object manager");
    //    p->addCoordinateSystem();
    CloudPtr object_cloud(new Cloud());
    *object_cloud = *object->m_points;

    // transform into the camera frame of ref (to compare with the individual clusters)
    Eigen::Matrix4f roomTransform = observation.getRoomTransform();
    if (observation.getIntermediateCloudTransforms().size() ==0)
    {
        ROS_ERROR_STREAM("Cannot get transform to origin.");
        return false;
    }

    tf::StampedTransform transformToOrigin = observation.getIntermediateCloudTransforms()[0];

    std::vector<tf::StampedTransform> allTransforms = observation.getIntermediateCloudTransformsRegistered();
    std::vector<CloudPtr> allClouds = observation.getIntermediateClouds();
    // frustrum for centroid projection

    pcl::FrustumCulling<PointType> fc;
    fc.setInputCloud (object_cloud);
    fc.setVerticalFOV (45);
    fc.setHorizontalFOV (55);
    fc.setNearPlaneDistance (.2);
    fc.setFarPlaneDistance (4.0);

    int max_overlap = 0;
    int best_index = -1;
    Eigen::Matrix4f best_transform;

    for (int j=0; j<allTransforms.size();j++)
    {

        Eigen::Matrix4f eigen_combined;
        pcl_ros::transformAsMatrix (transformToOrigin*allTransforms[j], eigen_combined);

        Eigen::Matrix4f cam2robot;
        cam2robot << 0, 0, 1, 0, 0,-1, 0, 0,1, 0, 0, 0, 0, 0, 0, 1;
        Eigen::Matrix4f pose_new = roomTransform * eigen_combined ;

        // set pose to frustum
        fc.setCameraPose (pose_new * cam2robot);
        pcl::PointCloud <PointType> target;
        fc.filter (target);

        //        ROS_INFO_STREAM("Overlap "<<target.points.size() <<" max overlap"<<max_overlap);

        if (target.points.size() > max_overlap)
        {
            //            ROS_INFO_STREAM("Intermediate cloud "<<j<<" contains the current dynamic cluster. No Points "<<target.points.size()<<"  total points "<<object_cloud->points.size());
            max_overlap = target.points.size();
            best_index = j;
            best_transform = pose_new;
        }

    }

    if (max_overlap > 0)
    {
        ROS_INFO_STREAM("Intermediate cloud "<<best_index<<" contains the current dynamic cluster. No Points "<<max_overlap<<"  total points "<<object_cloud->points.size());
        CloudPtr transformedCloud(new Cloud());
        pcl::transformPointCloud (*allClouds[best_index], *transformedCloud, best_transform);


        // filter out the cluster from the intermediate cloud
        CloudPtr filteredFromIntCloudCluster(new Cloud());
        std::vector<int> nn_indices (1);
        std::vector<float> nn_distances (1);
        std::vector<int> src_indices;
        pcl::SegmentDifferences<PointType> segment;
        typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
        tree->setInputCloud (object_cloud);

        // Iterate through the source data set
        for (int i = 0; i < static_cast<int> (transformedCloud->points.size ()); ++i)
        {
            if (!isFinite (transformedCloud->points[i]))
                continue;
            // Search for the closest point in the target data set (number of neighbors to find = 1)
            if (!tree->nearestKSearch (transformedCloud->points[i], 1, nn_indices, nn_distances))
            {
                PCL_WARN ("No neighbor found for point %zu (%f %f %f)!\n", i, transformedCloud->points[i].x, transformedCloud->points[i].y, transformedCloud->points[i].z);
                continue;
            }

            if (nn_distances[0] < 0.001)
            {
                src_indices.push_back (i);
                filteredFromIntCloudCluster->push_back(transformedCloud->points[i]);
            }
        }

        ROS_INFO_STREAM("Filtered cluster from int cloud "<<filteredFromIntCloudCluster->points.size()<<"  inliers "<<src_indices.size());

        if (filteredFromIntCloudCluster->points.size() == 0)
        {
            ROS_ERROR("No points found in cluster from intermediate clouds. Aborting.");
            return false;
        }

        // reproject intermediate cloud using original camera parameters
        auto camParamOrig = observation.getIntermediateCloudCameraParameters();
        double fx = camParamOrig[0].fx();
        double fy = camParamOrig[0].fy();
        double center_x = camParamOrig[0].cx();
        double center_y = camParamOrig[0].cy();
        double cx = 0.001 / fx;
        double cy = 0.001 / fy;
        ROS_INFO_STREAM("ObjectManager :: Camera parameters "<<fx<<" "<<fy<<" "<<center_x<<" "<<center_y);

        CloudPtr reprojected_cloud(new Cloud);
        RegistrationFeatures reg;

        std::pair<cv::Mat,cv::Mat> rgbAndDepth = reg.createRGBandDepthFromPC(transformedCloud);
        /*
        pcl::PointXYZRGB point;
        for (size_t y = 0; y < rgbAndDepth.first.rows; ++y) {
           for (size_t x = 0; x < rgbAndDepth.first.cols; ++x) {

              uint16_t depth = rgbAndDepth.second.at<u_int16_t>(y, x);
              if (!(depth != 0))
              {
                  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
              } else {
                 point.x = (x - center_x) * depth * cx;
                 point.y = (y - center_y) * depth * cy;
                 point.z = depth * 0.001f;
              }
              uint32_t rgb = ((uint8_t)rgbAndDepth.first.at<cv::Vec3b>(y, x)[2] << 16 | rgbAndDepth.first.at<cv::Vec3b>(y, x)[1] << 8 | rgbAndDepth.first.at<cv::Vec3b>(y, x)[0]);
              point.rgb = *reinterpret_cast<float*>(&rgb);

              reprojected_cloud->points[y*rgbAndDepth.first.cols + x] = point;
           }
        }
        *transformedCloud = *reprojected_cloud;
*/

        cv::Mat cluster_image = cv::Mat::zeros(480, 640, CV_8UC3);
        int top_y = -1, bottom_y = 640, top_x = -1, bottom_x = 640;
        for (int index : src_indices)
        {
            pcl::PointXYZRGB point = transformedCloud->points[index];
            int y = index / cluster_image.cols;
            int x = index % cluster_image.cols;
            cluster_image.at<cv::Vec3b>(y, x)[0] = point.b;
            cluster_image.at<cv::Vec3b>(y, x)[1] = point.g;
            cluster_image.at<cv::Vec3b>(y, x)[2] = point.r;

            if (y > top_y)
                top_y = y;
            if (x > top_x)
                top_x = x;
            if (y < bottom_y)
                bottom_y = y;
            if (x < bottom_x)
                bottom_x = x;

        }

        ROS_INFO_STREAM("Image ROI "<<bottom_y<<" "<<bottom_x<<" "<<top_y - bottom_y<<" "<<top_x - bottom_x);

        cv::Rect rec(bottom_x,bottom_y,top_x - bottom_x, top_y - bottom_y);
        cv::Mat tmp = cluster_image(rec);

        //        cv::imwrite("image.jpg", cluster_image);
        //        cv::imshow( "Display window", cluster_image );                   // Show our image inside it.


        // transform back into room complete cloud frame
        best_transform = roomTransform.inverse() * best_transform;
        const Eigen::Affine3d eigenTr(best_transform.cast<double>());
        tf::transformEigenToTF(eigenTr,returned_object.transform_to_map);
        returned_object.object_cloud = CloudPtr(new Cloud());
        *returned_object.object_cloud = *allClouds[best_index];
        //        pcl::transformPointCloud (*returned_object.object_cloud, *returned_object.object_cloud, best_transform); // transform to map frame

        returned_object.object_indices.insert(returned_object.object_indices.begin(), src_indices.begin(),src_indices.end());
        returned_object.object_mask = cluster_image;

        // save mask and indices
        if (m_bSaveMask) {
            // find observation folder
            int slash_pos = observation_xml.find_last_of("/");
            std::string observation_folder = observation_xml.substr(0, slash_pos);
            std::string mask_image = observation_folder + "/" + object_id + "_mask.jpg";
            cv::imwrite(mask_image, cluster_image);
            std::string mask_indices = observation_folder + "/" + object_id + "_mask.txt";
            ofstream mask_indices_os;
            mask_indices_os.open(mask_indices);
            for (int index : returned_object.object_indices){
                mask_indices_os << index<<" ";
            }
            mask_indices_os.close();
            ROS_INFO_STREAM("Object mask saved at: "<<mask_image);
            ROS_INFO_STREAM("Object mask indices saved at: "<<mask_indices);
        }

        int pan_angle = 0, tilt_angle = 0;
        semantic_map_registration_transforms::getPtuAnglesForIntPosition(observation.m_SweepParameters.m_pan_start, observation.m_SweepParameters.m_pan_step, observation.m_SweepParameters.m_pan_end,
                                                                         observation.m_SweepParameters.m_tilt_start, observation.m_SweepParameters.m_tilt_step, observation.m_SweepParameters.m_tilt_end,
                                                                         best_index, pan_angle, tilt_angle);

        returned_object.pan_angle = pan_angle;
        returned_object.tilt_angle = tilt_angle;

        // visualization
        //        cv::waitKey(0);                                          // Wait for a keystroke in the window

        //        pcl::visualization::PointCloudColorHandlerCustom<PointType> cluster_handler (object_cloud, 0, 255, 0);
        //        pcl::visualization::PointCloudColorHandlerCustom<PointType> int_handler (transformedCloud, 255, 255, 0);
        //        p->addPointCloud(object_cloud,cluster_handler,"cluster");
        //        p->addPointCloud(transformedCloud,int_handler,"int");
        //        p->spin();
        //        p->removeAllPointClouds();

        return true;
    } else {
        return false;
    }



}

#endif
