#ifndef __SEMANTIC_MAP_NODE__H
#define __SEMANTIC_MAP_NODE__H

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>

// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>

#include <mongodb_store/message_store.h>

#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/image_encodings.h>

// Services
#include <semantic_map_msgs/ClearMetaroomService.h>
#include <observation_registration_services/ObservationRegistrationService.h>
//#include <semantic_map/DynamicClusterService.h>
//#include <semantic_map/ObservationService.h>


//#include "object_manager/dynamic_object.h"

#include <semantic_map_msgs/RoomObservation.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <metaroom_xml_parser/load_utilities.h>
#include <strands_sweep_registration/RobotContainer.h>

#include "room_xml_parser.h"
#include "metaroom_xml_parser.h"
#include "semantic_map_summary_parser.h"
#include "reg_features.h"
#include "reg_transforms.h"
#include "room_utilities.h"

#include "quasimodo_msgs/model.h"
#include "quasimodo_msgs/rgbd_frame.h"
#include "quasimodo_msgs/model_from_frame.h"
#include "quasimodo_msgs/index_frame.h"
#include "quasimodo_msgs/fuse_models.h"
#include "quasimodo_msgs/get_model.h"
#include "quasimodo_msgs/segment_model.h"
#include "quasimodo_msgs/metaroom_pair.h"
#include <cv_bridge/cv_bridge.h>



template <class PointType>
class SemanticMapNode {
public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename SemanticMapSummaryParser::EntityStruct Entities;
    typedef pcl::search::KdTree<PointType> Tree;

    typedef typename std::map<std::string, boost::shared_ptr<MetaRoom<PointType> > >::iterator WaypointMetaroomMapIterator;
    typedef typename std::map<std::string, SemanticRoom<PointType> >::iterator WaypointRoomMapIterator;
    typedef typename std::map<std::string, CloudPtr>::iterator WaypointPointCloudMapIterator;

    typedef typename semantic_map_msgs::ClearMetaroomService::Request ClearMetaroomServiceRequest;
    typedef typename semantic_map_msgs::ClearMetaroomService::Response ClearMetaroomServiceResponse;

    SemanticMapNode(ros::NodeHandle nh);
    ~SemanticMapNode();

    void processRoomObservation(std::string xml_file_name);

    void roomObservationCallback(const semantic_map_msgs::RoomObservationConstPtr& obs_msg);
    bool clearMetaroomServiceCallback(ClearMetaroomServiceRequest &req, ClearMetaroomServiceResponse &res);

    ros::Subscriber                                                             m_SubscriberRoomObservation;
    ros::Publisher                                                              m_PublisherMetaroom;
    ros::Publisher                                                              m_PublisherObservation;
    ros::Publisher                                                              m_PublisherDynamicClusters;
    ros::Publisher                                                              m_PublisherStatus;
    ros::ServiceServer                                                          m_ClearMetaroomServiceServer;
    ros::ServiceClient                                                          m_segmentation_client;

private:

    ros::NodeHandle                                                             m_NodeHandle;
    SemanticMapSummaryParser                                         m_SummaryParser;
    bool                                                                        m_bSaveIntermediateData;
    std::vector<boost::shared_ptr<MetaRoom<PointType> > >                       m_vLoadedMetarooms;
    //mongodb_store::MessageStoreProxy                                           m_messageStore;
    bool                                                                        m_bLogToDB;
    std::map<std::string, boost::shared_ptr<MetaRoom<PointType> > >             m_WaypointToMetaroomMap;
    std::map<std::string, SemanticRoom<PointType> >                             m_WaypointToRoomMap;
    std::map<std::string, CloudPtr>                                             m_WaypointToDynamicClusterMap;
    bool                                                                        m_bUpdateMetaroom;
    bool                                                                        m_bNewestClusters;
    bool                                                                        m_bUseNDTRegistration;
	int									m_MinObjectSize;
    int segmentationtype;

};

template <class PointType>
SemanticMapNode<PointType>::SemanticMapNode(ros::NodeHandle nh)// : m_messageStore(nh)
{
    segmentationtype = 1;
    ROS_INFO_STREAM("Semantic map node initialized");
    m_NodeHandle = nh;

    m_segmentation_client = nh.serviceClient<quasimodo_msgs::metaroom_pair>("segment_metaroom");
    m_SubscriberRoomObservation = m_NodeHandle.subscribe("/local_metric_map/room_observations",1, &SemanticMapNode::roomObservationCallback,this);

    m_PublisherMetaroom = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/local_metric_map/metaroom", 1, true);
    m_PublisherDynamicClusters = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/local_metric_map/dynamic_clusters", 1, true);
    m_PublisherObservation = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/local_metric_map/merged_point_cloud_downsampled", 1, true);
    m_ClearMetaroomServiceServer = m_NodeHandle.advertiseService("/local_metric_map/ClearMetaroomService", &SemanticMapNode::clearMetaroomServiceCallback, this);

    m_NodeHandle.param<bool>("save_intermediate",m_bSaveIntermediateData,false);
    if (!m_bSaveIntermediateData)
    {
        ROS_INFO_STREAM("NOT saving intermediate data.");
    } else {
        ROS_INFO_STREAM("Saving intermediate data.");
    }

    m_NodeHandle.param<bool>("log_to_db",m_bLogToDB,false);
    if (m_bLogToDB)
    {
        //m_messageStore = mongodb_store::MessageStoreProxy(nh);
        ROS_INFO_STREAM("Logging dynamic clusters to the database.");
    } else {
        ROS_INFO_STREAM("NOT logging dynamic clusters to the database.");
    }

    m_NodeHandle.param<bool>("update_metaroom",m_bUpdateMetaroom,true);
    if (m_bUpdateMetaroom)
    {
        ROS_INFO_STREAM("The metarooms will be updated with new room observations.");
    } else {
        ROS_INFO_STREAM("The metarooms will NOT be updated with new room observations.");
    }

    m_NodeHandle.param<bool>("newest_dynamic_clusters",m_bNewestClusters,false);
    if (m_bNewestClusters)
    {
        ROS_INFO_STREAM("Only the latest dynamic clusters will be computed (comapring with previous observations).");
    } else {
        ROS_INFO_STREAM("All the dynamic clusters will be computer (comparing with the metaroom).");
    }

//    m_NodeHandle.param<bool>("use_NDT_registration",m_bUseNDTRegistration,true);
//    if (m_bUseNDTRegistration)
//    {
        m_bUseNDTRegistration = true;
        ROS_INFO_STREAM("The default registration method between sweeps is NDT.");
//    } else {
//        ROS_INFO_STREAM("The default registration between sweeps will use the computed image features (and RANSAC), if available. If not available defaulting to NDT registration.");
//    }


	m_NodeHandle.param<int>("min_object_size",m_MinObjectSize,500);
    ROS_INFO_STREAM("Min object size set to"<<m_MinObjectSize);

    std::string statusTopic;
    m_NodeHandle.param<std::string>("status_topic",statusTopic,"/local_metric_map/status");
    ROS_INFO_STREAM("The status topic is "<<statusTopic);
    m_PublisherStatus = m_NodeHandle.advertise<std_msgs::String>(statusTopic, 1000);
}

template <class PointType>
SemanticMapNode<PointType>::~SemanticMapNode()
{

}


template <class PointType>
void SemanticMapNode<PointType>::processRoomObservation(std::string xml_file_name)
{
    std::cout<<"File name "<<xml_file_name<<std::endl;

	printf("//////////////////////////////////////////////////////////\n");
	printf("//////////////////////////////////////////////////////////\n");
	printf("//////////////////////////////////////////////////////////\n");
	printf("%i\n",__LINE__);
	printf("//////////////////////////////////////////////////////////\n");
	printf("//////////////////////////////////////////////////////////\n");
	printf("//////////////////////////////////////////////////////////\n");



    if ( ! boost::filesystem::exists( xml_file_name ) )
    {
        ROS_ERROR_STREAM("Xml file does not exist. Aborting.");
        std_msgs::String msg;
        msg.data = "error_processing_observation";
        m_PublisherStatus.publish(msg);
        return;
    }

    SemanticRoomXMLParser<PointType> parser;
    SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(xml_file_name,true);
    aRoom.resetRoomTransform();
    CloudPtr room_complete_cloud = aRoom.getCompleteRoomCloud();

    // update summary xml
    m_SummaryParser.createSummaryXML<PointType>();

    // update list of rooms & metarooms
    m_SummaryParser.refresh();

    ROS_INFO_STREAM("Summary XML created. Looking for metaroom for room with id: "<<aRoom.getRoomStringId());

    boost::shared_ptr<MetaRoom<PointType> > metaroom;
    bool found = false;

    std::string matchingMetaroomXML = "";
    // first check if the matching metaroom has already been loaded
    for (size_t i=0; i<m_vLoadedMetarooms.size(); i++)
    {
        // compare by waypoint, if set
        if (aRoom.getRoomStringId() != "")
        {
            if (aRoom.getRoomStringId() == m_vLoadedMetarooms[i]->m_sMetaroomStringId)
            {
                metaroom = m_vLoadedMetarooms[i];
                found = true;
                break;
            }
        } else {
            // if not set, compare by centroid
            double centroidDistance = pcl::distances::l2(m_vLoadedMetarooms[i]->getCentroid(),aRoom.getCentroid());
            if (! (centroidDistance < ROOM_CENTROID_DISTANCE) )
            {
                continue;
            } else {
                ROS_INFO_STREAM("Matching metaroom already loaded.");
                metaroom = m_vLoadedMetarooms[i];
                found = true;
                break;
            }
        }
    }

    // if not loaded already, look through already saved metarooms
    if (!found)
    {
        std::vector<Entities> allMetarooms = m_SummaryParser.getMetaRooms();
        ROS_INFO_STREAM("Loaded "<<allMetarooms.size()<<" metarooms.");
        for (size_t i=0; i<allMetarooms.size();i++)
        {
            // compare by waypoint, if set
            if (aRoom.getRoomStringId() != "")
            {
                if (aRoom.getRoomStringId() == allMetarooms[i].stringId)
                {
                    matchingMetaroomXML = allMetarooms[i].roomXmlFile;
                    break;
                }
            } else {
                // if not set, compare by centroid
                double centroidDistance = pcl::distances::l2(allMetarooms[i].centroid,aRoom.getCentroid());
                if (! (centroidDistance < ROOM_CENTROID_DISTANCE) )
                {
                    continue;
                } else {
                    matchingMetaroomXML = allMetarooms[i].roomXmlFile;
                    break;
                }
            }
        }

        if (matchingMetaroomXML == "")
        {
            ROS_INFO_STREAM("No matching metaroom found. Create new metaroom.");
            metaroom =  boost::shared_ptr<MetaRoom<PointType> >(new MetaRoom<PointType>());
            metaroom->m_sMetaroomStringId = aRoom.getRoomStringId(); // set waypoint, to figure out where to save

        } else {
            ROS_INFO_STREAM("Matching metaroom found. XML file: "<<matchingMetaroomXML);
            metaroom =  MetaRoomXMLParser<PointType>::loadMetaRoomFromXML(matchingMetaroomXML,true);
            found = true;
        }
    } else {
        MetaRoomXMLParser<PointType> meta_parser;
        matchingMetaroomXML= meta_parser.findMetaRoomLocation(metaroom.get()).toStdString();
        matchingMetaroomXML+="/metaroom.xml";
    }

    if (!found)
    {
        ROS_INFO_STREAM("Initializing metaroom.");
        metaroom->setUpdateMetaroom(m_bUpdateMetaroom);
        metaroom->setSaveIntermediateSteps(m_bSaveIntermediateData);
        // also update the metaroom here
        auto updateIteration = metaroom->updateMetaRoom(aRoom,"",true);
        // save metaroom
        ROS_INFO_STREAM("Saving metaroom.");
        MetaRoomXMLParser<PointType> meta_parser;
        meta_parser.saveMetaRoomAsXML(*metaroom);
    }

	printf("//////////////////////////////////////////////////////////\n");
	printf("//////////////////////////////////////////////////////////\n");
	printf("//////////////////////////////////////////////////////////\n");
	printf("%i\n",__LINE__);
	printf("//////////////////////////////////////////////////////////\n");
	printf("//////////////////////////////////////////////////////////\n");
	printf("//////////////////////////////////////////////////////////\n");
	usleep(10*1000*1000);

	CloudPtr dynamicClusters(new Cloud());
	CloudPtr difference(new Cloud());
    if(segmentationtype == 1){
        std::string previousObservationXml;
        passwd* pw = getpwuid(getuid());
        std::string dataPath(pw->pw_dir);
        dataPath+="/.semanticMap";

        std::vector<std::string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(dataPath, aRoom.getRoomStringId());
        if (matchingObservations.size() == 0) // no observations -> first observation
        {
            ROS_INFO_STREAM("No observations for this waypoint saved on the disk "+aRoom.getRoomStringId()+" cannot compare with previous observation.");
            std_msgs::String msg;
            msg.data = "finished_processing_observation";
            m_PublisherStatus.publish(msg);
            return;
        }
        if (matchingObservations.size() == 1) // first observation at this waypoint
        {
            ROS_INFO_STREAM("No dynamic clusters.");
            std_msgs::String msg;
            msg.data = "finished_processing_observation";
            m_PublisherStatus.publish(msg);
            return;
        }

        int stopind = 0;

        for(unsigned int i = 0; i < matchingObservations.size(); i++){
            if(matchingObservations[i].compare(xml_file_name) == 0){
                break;
            }else{
                stopind = i;//latest = matchingObservations[i];
            }
        }

        if(stopind == 0) // first observation at this waypoint
        {
            ROS_INFO_STREAM("No dynamic clusters.");
            std_msgs::String msg;
            msg.data = "finished_processing_observation";
            m_PublisherStatus.publish(msg);
            return;
        }
        previousObservationXml = matchingObservations[stopind];
        printf("prev: %s\n",previousObservationXml.c_str());

		quasimodo_msgs::metaroom_pair sm;
		sm.request.background = previousObservationXml;
		sm.request.foreground = xml_file_name;

		printf("//////////////////////////////////////////////////////////\n");
		printf("//////////////////////////////////////////////////////////\n");
		printf("//////////////////////////////////////////////////////////\n");
		printf("%i\n",__LINE__);
		printf("//////////////////////////////////////////////////////////\n");
		printf("//////////////////////////////////////////////////////////\n");
		printf("//////////////////////////////////////////////////////////\n");

		usleep(10*1000*1000);
		if (m_segmentation_client.call(sm)){
			CloudPtr dynamiccloud(new Cloud());
			std::vector<cv::Mat> dynamicmasks;
			for(unsigned int i = 0; i < sm.response.dynamicmasks.size(); i++){
				cv_bridge::CvImagePtr			img_ptr;
				try{							img_ptr = cv_bridge::toCvCopy(sm.response.dynamicmasks[i], sensor_msgs::image_encodings::MONO8);}
				catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what());}
				cv::Mat mask = img_ptr->image;
				dynamicmasks.push_back(mask);

				CloudPtr currentcloud = aRoom.getIntermediateClouds()[i];
				CloudPtr currentcloudTMP(new Cloud());
				aRoom.getIntermediateCloudTransformsRegistered()[i];
				//pcl::transformPointCloud (*currentcloud, *currentcloudTMP, aRoom.m_vIntermediateRoomCloudTransformsRegistered[i]);

				pcl_ros::transformPointCloud(*currentcloud, *currentcloudTMP,aRoom.getIntermediateCloudTransformsRegistered()[i]);
				unsigned char * maskdata = mask.data;
				for(unsigned int j = 0; j < currentcloudTMP->points.size(); j++){
					if(maskdata[j] > 0){
						//difference->points.push_back(currentcloudTMP->points[j]);
						dynamicClusters->points.push_back(currentcloudTMP->points[j]);
					}
				}
			}


//			std::vector<CloudPtr> vClusters = MetaRoom<PointType>::clusterPointCloud(dynamiccloud,0.03,100,1000000);
//			ROS_INFO_STREAM("Clustered differences. "<<vClusters.size()<<" different clusters.");
		}else{
			ROS_ERROR("Failed to call service segment_model");
			std_msgs::String msg;
			msg.data = "error_processing_observation";
			m_PublisherStatus.publish(msg);
			return;
		}

		printf("//////////////////////////////////////////////////////////\n");
		printf("//////////////////////////////////////////////////////////\n");
		printf("//////////////////////////////////////////////////////////\n");
		printf("%i\n",__LINE__);
		printf("//////////////////////////////////////////////////////////\n");
		printf("//////////////////////////////////////////////////////////\n");
		printf("//////////////////////////////////////////////////////////\n");
		usleep(10*1000*1000);

    }else{

	// this stores the difference between a) current obs and previous obs or b) current obs and metaroom (depending on the node parameters)
    if (m_bNewestClusters){
         // computing the latest clusters by comparison with the previous observation -> register with the previous observation
        // look for the previous observation in ~/.semanticMap/
        std::string previousObservationXml;
        passwd* pw = getpwuid(getuid());
        std::string dataPath(pw->pw_dir);
        dataPath+="/.semanticMap";

        std::vector<std::string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(dataPath, aRoom.getRoomStringId());
        if (matchingObservations.size() == 0) // no observations -> first observation
        {
            ROS_INFO_STREAM("No observations for this waypoint saved on the disk "+aRoom.getRoomStringId()+" cannot compare with previous observation.");
            std_msgs::String msg;
            msg.data = "finished_processing_observation";
            m_PublisherStatus.publish(msg);
            return;
        }
        if (matchingObservations.size() == 1) // first observation at this waypoint
        {
            ROS_INFO_STREAM("No dynamic clusters.");
            std_msgs::String msg;
            msg.data = "finished_processing_observation";
            m_PublisherStatus.publish(msg);
            return;
        }

        // multiple observations -> find the previous one
        reverse(matchingObservations.begin(), matchingObservations.end());
        std::string latest = matchingObservations[0];
        if (latest != xml_file_name)
        {
            ROS_WARN_STREAM("The xml file for the latest observations "+latest+" is different from the one provided "+xml_file_name+" Aborting");
        }
        previousObservationXml = matchingObservations[1];

        // load previous observation
        SemanticRoom<PointType> prevRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(previousObservationXml,true);
        CloudPtr prevCompleteCloud = prevRoom.getCompleteRoomCloud();
        CloudPtr prevCloud = MetaRoom<PointType>::downsampleCloud(prevCompleteCloud->makeShared());


        // register with the previous observation
        ros::ServiceClient client = m_NodeHandle.serviceClient<observation_registration_services::ObservationRegistrationService>("/observation_registration_server");
        observation_registration_services::ObservationRegistrationService srv;
        srv.request.source_observation_xml = xml_file_name; // source
        srv.request.target_observation_xml = previousObservationXml; //target
        Eigen::Matrix4f registered_transform;
        if (client.call(srv))
        {
            ROS_INFO_STREAM("Registration done using the observation_registration_server service. Number of constraints "<<srv.response.total_correspondences);
            if (srv.response.total_correspondences <= 0){
                ROS_ERROR_STREAM("Registration unsuccessful due to insufficient constraints. Will use the default NDT registration");
                CloudPtr transformedRoomCloud = NdtRegistration<PointType>::registerClouds(room_complete_cloud, prevCloud,registered_transform);
            } else {
                // registration successfull -> get registered transform
                tf::Transform tf_registered_transform;
               tf::transformMsgToTF(srv.response.transform, tf_registered_transform);
               pcl_ros::transformAsMatrix(tf_registered_transform, registered_transform);
            }
        }

        // apply transform to the observation cloud
        CloudPtr room_interior_cloud = MetaRoom<PointType>::downsampleCloud(room_complete_cloud->makeShared());
        pcl::transformPointCloud(*room_interior_cloud, *room_interior_cloud,registered_transform);
        aRoom.setDeNoisedRoomCloud(room_interior_cloud);
        aRoom.setInteriorRoomCloud(room_interior_cloud);

        aRoom.setRoomTransform(registered_transform);
        // save updated room
        parser.saveRoomAsXML(aRoom);

        // compute difference
        ROS_INFO_STREAM("Comparing with "<<previousObservationXml);

        // compute differences and check occlusions
        CloudPtr differenceRoomToPrevRoom(new Cloud);
        CloudPtr differencePrevRoomToRoom(new Cloud);

        // compute the differences
        pcl::SegmentDifferences<PointType> segment;
        segment.setInputCloud(room_interior_cloud);
        segment.setTargetCloud(prevCloud);
        segment.setDistanceThreshold(0.001);
        typename Tree::Ptr tree (new pcl::search::KdTree<PointType>);
        tree->setInputCloud (prevCloud);
        segment.setSearchMethod(tree);
        segment.segment(*differenceRoomToPrevRoom);

        segment.setInputCloud(prevCloud);
        segment.setTargetCloud(room_interior_cloud);
        tree->setInputCloud(room_interior_cloud);

        segment.segment(*differencePrevRoomToRoom);

        CloudPtr toBeAdded(new Cloud());
        CloudPtr toBeRemoved(new Cloud());

        OcclusionChecker<PointType> occlusionChecker;
//        occlusionChecker.setSensorOrigin(metaroom->getSensorOrigin()); // since it's already transformed in the metaroom frame of ref
        occlusionChecker.setSensorOrigin(prevRoom.getIntermediateCloudTransforms()[0].getOrigin()); // since it's already transformed in the metaroom frame of ref
        auto occlusions = occlusionChecker.checkOcclusions(differenceRoomToPrevRoom,differencePrevRoomToRoom, 720 );
        *difference = *occlusions.toBeRemoved;

        ROS_INFO_STREAM("Computed differences " << difference->points.size());
        ROS_INFO_STREAM("Comparison cloud "<<prevCloud->points.size()<<"  room cloud "<<room_interior_cloud->points.size());

    } else {
        // computing all the clusters, by comparison with the metaroom -> register the observation with the metaroom
        bool do_registration = true;
        auto updateIteration = metaroom->updateMetaRoom(aRoom,"",do_registration);

        if (updateIteration.roomRunNumber == -1) // no update performed
        {
            ROS_ERROR_STREAM("Could not update metaroom with the room instance.");
            std_msgs::String msg;
            msg.data = "error_processing_observation";
            m_PublisherStatus.publish(msg);
            return;
        }

        CloudPtr metaroomCloud = metaroom->getInteriorRoomCloud();

        // publish data
        sensor_msgs::PointCloud2 msg_metaroom;
        pcl::toROSMsg(*metaroomCloud, msg_metaroom);
        msg_metaroom.header.frame_id="/map";
        m_PublisherMetaroom.publish(msg_metaroom);
    //    m_vLoadedMetarooms.push_back(metaroom); // don't keep data in memory
        ROS_INFO_STREAM("Published metaroom");

        CloudPtr room_interior_cloud = aRoom.getInteriorRoomCloud();
        ROS_INFO_STREAM("Computing differences");

        // compute differences
            pcl::SegmentDifferences<PointType> segment;
            segment.setInputCloud(room_interior_cloud);
            segment.setTargetCloud(metaroomCloud);
            segment.setDistanceThreshold(0.001);
            typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
            tree->setInputCloud (metaroomCloud);
            segment.setSearchMethod(tree);
            segment.segment(*difference);
            ROS_INFO_STREAM("Computed differences " << difference->points.size());
            ROS_INFO_STREAM("Comparison cloud "<<metaroomCloud->points.size()<<"  room cloud "<<room_interior_cloud->points.size());

    }
//}
    ROS_INFO_STREAM("Raw difference "<<difference->points.size());

/*
    if (difference->points.size() == 0)
    {
        // -> no dynamic clusters can be computed
        ROS_INFO_STREAM("No dynamic clusters.");
        std_msgs::String msg;
        msg.data = "finished_processing_observation";
        m_PublisherStatus.publish(msg);
        return;
    }
*/

    std::vector<CloudPtr> vClusters = MetaRoom<PointType>::clusterPointCloud(difference,0.03,m_MinObjectSize,100000);
    ROS_INFO_STREAM("Clustered differences. "<<vClusters.size()<<" different clusters.");
	MetaRoom<PointType>::filterClustersBasedOnDistance(aRoom.getIntermediateCloudTransforms()[0].getOrigin(), vClusters,4.0);
	ROS_INFO_STREAM(vClusters.size()<<" different clusters after max distance filtering.");

    ROS_INFO_STREAM("Clustered differences. "<<vClusters.size()<<" different clusters.");

    // Check cluster planarity and discard the absolutely planar ones (usually parts of walls, floor, ceiling).
	//CloudPtr dynamicClusters(new Cloud());
	/*
	for (size_t i=0; i<vClusters.size(); i++)
	{
		pcl::SACSegmentation<PointType> seg;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.02);

		seg.setInputCloud (vClusters[i]);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () > 0.9 * vClusters[i]->points.size())
		{
			ROS_INFO_STREAM("Discarding planar dynamic cluster");
		} else {
			*dynamicClusters += *vClusters[i];
		}
	}
*/

	for (size_t i=0; i<vClusters.size(); i++)
	{
		*dynamicClusters += *vClusters[i];
	}
}

	printf("dynamicClusters->points.size() = %i\n",dynamicClusters->points.size());
    // publish dynamic clusters
    // transform back into the sweep frame of ref (before metaroom registration, to align with the previously published sweep point cloud)
    Eigen::Matrix4f inverseRoomTransform = aRoom.getRoomTransform().inverse();
    CloudPtr tempClusters(new Cloud()); *tempClusters = *dynamicClusters;
    pcl::transformPointCloud (*tempClusters, *tempClusters, inverseRoomTransform);
    sensor_msgs::PointCloud2 msg_clusters;
    pcl::toROSMsg(*tempClusters, msg_clusters);
    msg_clusters.header.frame_id="/map";
    m_PublisherDynamicClusters.publish(msg_clusters);
    ROS_INFO_STREAM("Published differences "<<dynamicClusters->points.size());

    aRoom.setDynamicClustersCloud(dynamicClusters);
    // save updated room
    parser.saveRoomAsXML(aRoom);

    if (aRoom.getRoomStringId() != "") // waypoint id set, update the map
    {
        m_WaypointToDynamicClusterMap[aRoom.getRoomStringId()] = dynamicClusters;
        ROS_INFO_STREAM("Updated map with new dynamic clusters for waypoint "<<aRoom.getRoomStringId());
    }

//    if (m_bLogToDB)
//    {
//        QString databaseName = QString(aRoom.getRoomLogName().c_str()) + QString("/room_")+ QString::number(aRoom.getRoomRunNumber()) +QString("/dynamic_clusters");
//        std::string id(m_messageStore.insertNamed(databaseName.toStdString(), msg_clusters));
//        ROS_INFO_STREAM("Dynamic clusters \""<<databaseName.toStdString()<<"\" inserted with id "<<id);

//        std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > results;
//        if(m_messageStore.queryNamed<sensor_msgs::PointCloud2>(databaseName.toStdString(), results)) {

//            BOOST_FOREACH( boost::shared_ptr<sensor_msgs::PointCloud2> p,  results)
//            {
//                CloudPtr databaseCloud(new Cloud());
//                pcl::fromROSMsg(*p,*databaseCloud);
//                ROS_INFO_STREAM("Got pointcloud by name. No points: " << databaseCloud->points.size());
//            }
//        }
//    }


    std_msgs::String msg;
    msg.data = "finished_processing_observation";
    m_PublisherStatus.publish(msg);

    return;
}

template <class PointType>
void SemanticMapNode<PointType>::roomObservationCallback(const semantic_map_msgs::RoomObservationConstPtr& obs_msg)
{
    std::cout<<"Room obs message received"<<std::endl;
    this->processRoomObservation(obs_msg->xml_file_name);
}


template <class PointType>
bool SemanticMapNode<PointType>::clearMetaroomServiceCallback(ClearMetaroomServiceRequest &req, ClearMetaroomServiceResponse &res)
{
    ROS_INFO_STREAM("Received a clear metaroom service request ");

    using namespace std;
    std::vector<Entities> allMetarooms = m_SummaryParser.getMetaRooms();
    ROS_INFO_STREAM("Loaded "<<allMetarooms.size()<<" metarooms.");

    for (size_t i=0; i<allMetarooms.size(); i++)
    {
        bool clear_mr = false;
        if (req.waypoint_id.size() == 0)
        {
            clear_mr = true;
        }
        for_each(req.waypoint_id.begin(), req.waypoint_id.end(), [&clear_mr, &allMetarooms, &i](string way)
        {
            if (way == allMetarooms[i].stringId) clear_mr = true;
        } );

        if (clear_mr)
        {
            int index = QString(allMetarooms[i].roomXmlFile.c_str()).lastIndexOf('/');
            QString metaroomFolder = QString(allMetarooms[i].roomXmlFile.c_str()).left(index);

            ROS_INFO_STREAM("Removing metaroom corresponding to waypoint ["<< allMetarooms[i].stringId<< "]  saved at "<<metaroomFolder.toStdString());
//            QDir home = QDir::homePath();
//            home.removeRecursively(metaroomFolder);
            boost::filesystem::remove_all(metaroomFolder.toStdString().c_str());
        }

    }

    // clear metarooms loaded in memory
    m_vLoadedMetarooms.clear();

    // check if to reinitialize the metarooms
    if (req.initialize)
    {
        auto sweep_xmls = m_SummaryParser.getRooms();

        // first construct waypoint id to sweep xml map
        std::map<std::string, std::vector<std::string>> waypointToSweepsMap;
        for (size_t i=0; i<sweep_xmls.size(); i++)
        {
            auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweep_xmls[i].roomXmlFile, std::vector<std::string>(), false);
            waypointToSweepsMap[sweep.roomWaypointId].push_back(sweep_xmls[i].roomXmlFile);
        }

        for (auto it = waypointToSweepsMap.begin(); it!= waypointToSweepsMap.end(); it++)
        {
            bool initialize_metaroom = false;
            if (req.waypoint_id.size() == 0)
            {
                initialize_metaroom = true;
            }
            for_each(req.waypoint_id.begin(), req.waypoint_id.end(), [&initialize_metaroom, &it](string way)
            {
                if (way == it->first) initialize_metaroom = true;
            } );

            if (initialize_metaroom)
            {
                vector<string> waypoint_observations = it->second;
                sort(waypoint_observations.begin(), waypoint_observations.end(),
                     [](const std::string& a, const std::string& b )
                {
                    std::string patrol_string = "patrol_run_";
                    std::string room_string = "room_";
                    size_t p_pos_1 = a.find(patrol_string);
                    size_t r_pos_1 = a.find(room_string) - 1; // remove the / before the room_
                    size_t sl_pos_1 = a.find("/",r_pos_1+1);

                    size_t p_pos_2 = b.find(patrol_string);
                    size_t r_pos_2 = b.find(room_string) - 1; // remove the / before the room_
                    size_t sl_pos_2 = b.find("/",r_pos_2+1);

                    // just in case we have some different folder structure (shouldn't happen)
                    if ((p_pos_1 == std::string::npos) || (r_pos_1 == std::string::npos) || (sl_pos_1 == std::string::npos) ||
                            (p_pos_2 == std::string::npos) || (r_pos_2 == std::string::npos) || (sl_pos_2 == std::string::npos))
                    {
                        return a<b;
                    }

                    std::string p_1 = a.substr(p_pos_1 + patrol_string.length(), r_pos_1 - (p_pos_1 + patrol_string.length()));
                    std::string r_1 = a.substr(r_pos_1 + 1 + room_string.length(), sl_pos_1 - (r_pos_1 + 1 + room_string.length()));
        //            std::cout<<"Patrol 1: "<<p_1<<"  room 1: "<<r_1<<std::endl;
                    std::string p_2 = b.substr(p_pos_2 + patrol_string.length(), r_pos_2 - (p_pos_2 + patrol_string.length()));
                    std::string r_2 = b.substr(r_pos_2 + 1 + room_string.length(), sl_pos_2 - (r_pos_2 + 1 + room_string.length()));
        //            std::cout<<"Patrol 2: "<<p_2<<"  room 2: "<<r_2<<std::endl;

                    if (stoi(p_1) == stoi(p_2)){
                        if (stoi(r_1) == stoi(r_2)){
                            return a<b;
                        } else {
                            return stoi(r_1) < stoi(r_2);
                        }
                    } else {
                        return stoi(p_1) < stoi(p_2);
                    }
                }
                     );
                reverse(waypoint_observations.begin(), waypoint_observations.end());
                ROS_INFO_STREAM("Initializing metaroom for waypoint ["<<it->first<<"] with observation "<<waypoint_observations[0]);

                processRoomObservation(waypoint_observations[0]);

                ROS_INFO_STREAM("Finished initializing metaroom for waypoint ["<<it->first<<"]");
            }
        }
    }

    return true;
}

#endif

