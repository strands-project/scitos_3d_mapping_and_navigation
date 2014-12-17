#ifndef __SIMPLE_XML_PARSER__H
#define __SIMPLE_XML_PARSER__H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "ros/time.h"
#include "ros/serialization.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include "tf/tf.h"

#include <QFile>
#include <QDir>
#include <QXmlStreamWriter>

#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


template <class PointType>
class SimpleXMLParser {
public:

//    typedef pcl::PointCloud<PointType> Cloud;
//    typedef typename Cloud::Ptr CloudPtr;


    struct IntermediateCloudData{
        std::string                     filename;
        tf::StampedTransform            transform;
        sensor_msgs::CameraInfo         camInfo;
    };

    struct IntermediatePositionImages
    {
        std::vector<cv::Mat>                vIntermediateDepthImages;
        std::vector<cv::Mat>                vIntermediateRGBImages;
        tf::StampedTransform                intermediateDepthTransform;
        tf::StampedTransform                intermediateRGBTransform;
        image_geometry::PinholeCameraModel  intermediateRGBCamParams;
        image_geometry::PinholeCameraModel  intermediateDepthCamParams;
    };

//    template <class PointType>
    struct RoomData
    {

        std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>                            vIntermediateRoomClouds;
        std::vector<tf::StampedTransform>                vIntermediateRoomCloudTransforms;
        std::vector<image_geometry::PinholeCameraModel>  vIntermediateRoomCloudCamParams;
        std::vector<cv::Mat>                             vIntermediateRGBImages; // type CV_8UC3
        std::vector<cv::Mat>                             vIntermediateDepthImages; // type CV_16UC1
        boost::shared_ptr<pcl::PointCloud<PointType>>                                         completeRoomCloud;
        std::string                                      roomWaypointId;
        std::vector<IntermediatePositionImages>          vIntermediatePositionImages;

        RoomData(){
            completeRoomCloud = boost::shared_ptr<pcl::PointCloud<PointType>>(new pcl::PointCloud<PointType>());
        }
    };



    SimpleXMLParser(){}

    ~SimpleXMLParser()
    {}


//    template <class PointType>
    static std::pair<cv::Mat, cv::Mat> createRGBandDepthFromPC(boost::shared_ptr<pcl::PointCloud<PointType>> cloud)
    {
        std::pair<cv::Mat, cv::Mat> toRet;
        toRet.first = cv::Mat::zeros(480, 640, CV_8UC3); // RGB image
        toRet.second = cv::Mat::zeros(480, 640, CV_16UC1); // Depth image
        pcl::PointXYZRGB point;
        for (size_t y = 0; y < toRet.first.rows; ++y) {
            for (size_t x = 0; x < toRet.first.cols; ++x) {
                point = cloud->points[y*toRet.first.cols + x];
                // RGB
                toRet.first.at<cv::Vec3b>(y, x)[0] = point.b;
                toRet.first.at<cv::Vec3b>(y, x)[1] = point.g;
                toRet.first.at<cv::Vec3b>(y, x)[2] = point.r;
                // Depth
                toRet.second.at<u_int16_t>(y, x) = point.z*1000; // convert to uint 16 from meters

            }
        }

        return toRet;
    }

//    template <class PointType>
    static RoomData loadRoomFromXML(const std::string& xmlFile)
    {
        RoomData aRoom;


        QFile file(xmlFile.c_str());

        if (!file.exists())
        {
            std::cerr<<"Could not open file "<<xmlFile<<" to load room."<<std::endl;
            return aRoom;
        }

        QString xmlFileQS(xmlFile.c_str());
        int index = xmlFileQS.lastIndexOf('/');
        QString roomFolder = xmlFileQS.left(index);


        file.open(QIODevice::ReadOnly);

        QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);
        Eigen::Vector4f centroid(0.0,0.0,0.0,0.0);

        while (!xmlReader->atEnd() && !xmlReader->hasError())
        {
            QXmlStreamReader::TokenType token = xmlReader->readNext();
            if (token == QXmlStreamReader::StartDocument)
                continue;

            if (xmlReader->hasError())
            {
                std::cout << "XML error: " << xmlReader->errorString().toStdString() << std::endl;
                return aRoom;
            }

            QString elementName = xmlReader->name().toString();

            if (token == QXmlStreamReader::StartElement)
            {
                if (xmlReader->name() == "RoomCompleteCloud")
                {
                    QXmlStreamAttributes attributes = xmlReader->attributes();
                    if (attributes.hasAttribute("filename"))
                    {
                        QString roomCompleteCloudFile = attributes.value("filename").toString();
                        int sl_index = roomCompleteCloudFile.indexOf('/');
                        if (sl_index == -1)
                        {
                            roomCompleteCloudFile=roomFolder + "/" + roomCompleteCloudFile;
                        }

                        std::cout<<"Loading complete cloud file name "<<roomCompleteCloudFile.toStdString()<<std::endl;
                        pcl::PCDReader reader;
                        boost::shared_ptr<pcl::PointCloud<PointType>> cloud (new pcl::PointCloud<PointType>);
                        reader.read (roomCompleteCloudFile.toStdString(), *cloud);
                        *aRoom.completeRoomCloud = *cloud;
                    } else {
                        std::cerr<<"RoomCompleteCloud xml node does not have filename attribute. Aborting."<<std::endl;
                        return aRoom;
                    }
                }
                if (xmlReader->name() == "RoomStringId")
                {
                    QString roomWaypointId = xmlReader->readElementText();
                    aRoom.roomWaypointId = roomWaypointId.toStdString();
                }

                if (xmlReader->name() == "RoomIntermediateCloud")
                {
                    IntermediateCloudData intermediateCloudData = parseRoomIntermediateCloudNode(*xmlReader);
                    image_geometry::PinholeCameraModel aCameraModel;
                    aCameraModel.fromCameraInfo(intermediateCloudData.camInfo);

                    QString cloudFileName = intermediateCloudData.filename.c_str();
                    int sl_index = cloudFileName.indexOf('/');
                    if (sl_index == -1)
                    {
                        cloudFileName=roomFolder + "/" + cloudFileName;
                    }

                    std::ifstream file(cloudFileName.toStdString().c_str());
                    if (file)
                    {
                        std::cout<<"Loading intermediate cloud file name "<<cloudFileName.toStdString()<<std::endl;
                        pcl::PCDReader reader;
                        boost::shared_ptr<pcl::PointCloud<PointType>> cloud (new pcl::PointCloud<PointType>);
                        reader.read (cloudFileName.toStdString(), *cloud);
                        aRoom.vIntermediateRoomClouds.push_back(cloud);
                        aRoom.vIntermediateRoomCloudTransforms.push_back(intermediateCloudData.transform);
                        aRoom.vIntermediateRoomCloudCamParams.push_back(aCameraModel);
                        std::pair<cv::Mat,cv::Mat> rgbAndDepth = SimpleXMLParser::createRGBandDepthFromPC(cloud);
                        aRoom.vIntermediateRGBImages.push_back(rgbAndDepth.first);
                        aRoom.vIntermediateDepthImages.push_back(rgbAndDepth.second);
                    }
                }

                if (xmlReader->name() == "IntermediatePosition")
                {
                    auto positionImages = parseIntermediatePositionImages(xmlReader, roomFolder.toStdString());

                    aRoom.vIntermediatePositionImages.push_back(positionImages);
                }
            }
        }

        delete xmlReader;


        return aRoom;
    }

private:

    static IntermediateCloudData parseRoomIntermediateCloudNode(QXmlStreamReader& xmlReader)
    {
        tf::StampedTransform transform;
        geometry_msgs::TransformStamped tfmsg;
        sensor_msgs::CameraInfo camInfo;
        bool camInfoError = false;

        IntermediateCloudData       structToRet;
        //        toRet.first = CloudPtr(new Cloud);
        QString intermediateParentNode("");

        if (xmlReader.name()!="RoomIntermediateCloud")
        {
            std::cerr<<"Cannot parse RoomIntermediateCloud node, it has a different name: "<<xmlReader.name().toString().toStdString()<<std::endl;
        }
        QXmlStreamAttributes attributes = xmlReader.attributes();
        if (attributes.hasAttribute("filename"))
        {
            QString roomIntermediateCloudFile = attributes.value("filename").toString();
            structToRet.filename = roomIntermediateCloudFile.toStdString();
            //            pcl::PCDReader reader;
            //            reader.read (roomIntermediateCloudFile.toStdString(), *toRet.first);

        } else {
            std::cerr<<"RoomIntermediateCloud xml node does not have filename attribute. Aborting."<<std::endl;
            return structToRet;
        }
        QXmlStreamReader::TokenType token = xmlReader.readNext();

        while(!((token == QXmlStreamReader::EndElement) && (xmlReader.name() == "RoomIntermediateCloud")) )
        {

            if (token == QXmlStreamReader::StartElement)
            {
                if (xmlReader.name() == "sec")
                {
                    int sec = xmlReader.readElementText().toInt();
                    tfmsg.header.stamp.sec = sec;
                }
                if (xmlReader.name() == "nsec")
                {
                    int nsec = xmlReader.readElementText().toInt();
                    tfmsg.header.stamp.nsec = nsec;
                }
                if (xmlReader.name() == "Translation")
                {
                    intermediateParentNode = xmlReader.name().toString();
                }
                if (xmlReader.name() == "Rotation")
                {
                    intermediateParentNode = xmlReader.name().toString();
                }
                if (xmlReader.name() == "w")
                {
                    double w = xmlReader.readElementText().toDouble();
                    tfmsg.transform.rotation.w = w;
                }
                if (xmlReader.name() == "x")
                {
                    double x = xmlReader.readElementText().toDouble();
                    if (intermediateParentNode == "Rotation")
                    {
                        tfmsg.transform.rotation.x = x;
                    }
                    if (intermediateParentNode == "Translation")
                    {
                        tfmsg.transform.translation.x = x;
                    }
                }
                if (xmlReader.name() == "y")
                {
                    double y = xmlReader.readElementText().toDouble();
                    if (intermediateParentNode == "Rotation")
                    {
                        tfmsg.transform.rotation.y = y;
                    }
                    if (intermediateParentNode == "Translation")
                    {
                        tfmsg.transform.translation.y = y;
                    }
                }
                if (xmlReader.name() == "z")
                {
                    double z = xmlReader.readElementText().toDouble();
                    if (intermediateParentNode == "Rotation")
                    {
                        tfmsg.transform.rotation.z = z;
                    }
                    if (intermediateParentNode == "Translation")
                    {
                        tfmsg.transform.translation.z = z;
                    }
                }

                // camera parameters
                if (xmlReader.name() == "RoomIntermediateCameraParameters")
                {
                    QXmlStreamAttributes paramAttributes = xmlReader.attributes();

                    if (paramAttributes.hasAttribute("height"))
                    {
                        QString val = paramAttributes.value("height").toString();
                        camInfo.height = val.toInt();

                    } else {
                        ROS_ERROR("RoomIntermediateCameraParameters xml node does not have the height attribute. Cannot construct camera parameters object.");
                        camInfoError = true;
                    }

                    if (paramAttributes.hasAttribute("width"))
                    {
                        QString val = paramAttributes.value("width").toString();
                        camInfo.width = val.toInt();

                    } else {
                        ROS_ERROR("RoomIntermediateCameraParameters xml node does not have the width attribute. Cannot construct camera parameters object.");
                        camInfoError = true;
                    }

                    if (paramAttributes.hasAttribute("distortion_model"))
                    {
                        QString val = paramAttributes.value("distortion_model").toString();
                        camInfo.distortion_model = val.toStdString();

                    } else {
                        ROS_ERROR("RoomIntermediateCameraParameters xml node does not have the distortion_model attribute. Cannot construct camera parameters object.");
                        camInfoError = true;
                    }

                    if (paramAttributes.hasAttribute("frame_id"))
                    {
                        QString val = paramAttributes.value("frame_id").toString();
                        camInfo.header.frame_id = val.toStdString();

                    } else {
                        ROS_ERROR("RoomIntermediateCameraParameters xml node does not have the frame_id attribute. Cannot construct camera parameters object.");
                        camInfoError = true;
                    }

                    if (paramAttributes.hasAttribute("K"))
                    {
                        QString val = paramAttributes.value("K").toString();
                        QStringList val_list = val.split(",",QString::SkipEmptyParts);
                        if (val_list.size() != 9) // wrong number of parameters
                        {
                            ROS_ERROR("RoomIntermediateCameraParameters K attribute has more than 9 parameters. Cannot construct camera parameters object.");
                            camInfoError = true;
                        } else {
                            for (size_t j=0; j< val_list.size();j++)
                            {
                                camInfo.K[j] = val_list[j].toDouble();
                            }
                        }
                    } else {
                        ROS_ERROR("RoomIntermediateCameraParameters xml node does not have the K attribute. Cannot construct camera parameters object.");
                        camInfoError = true;
                    }

                    if (paramAttributes.hasAttribute("D"))
                    {
                        QString val = paramAttributes.value("D").toString();
                        QStringList val_list = val.split(",",QString::SkipEmptyParts);
                        if (val_list.size() != 5) // wrong number of parameters
                        {
                            ROS_ERROR("RoomIntermediateCameraParameters D attribute has more than 5 parameters. Cannot construct camera parameters object.");
                            camInfoError = true;
                        } else {
                            for (size_t j=0; j< val_list.size();j++)
                            {
                                camInfo.D.push_back(val_list[j].toDouble());
                            }
                        }
                    } else {
                        ROS_ERROR("RoomIntermediateCameraParameters xml node does not have the D attribute. Cannot construct camera parameters object.");
                        camInfoError = true;
                    }

                    if (paramAttributes.hasAttribute("R"))
                    {
                        QString val = paramAttributes.value("R").toString();
                        QStringList val_list = val.split(",",QString::SkipEmptyParts);
                        if (val_list.size() != 9) // wrong number of parameters
                        {
                            ROS_ERROR("RoomIntermediateCameraParameters R attribute has more than 9 parameters. Cannot construct camera parameters object.");
                            camInfoError = true;
                        } else {
                            for (size_t j=0; j< val_list.size();j++)
                            {
                                camInfo.R[j] = val_list[j].toDouble();
                            }
                        }
                    } else {
                        ROS_ERROR("RoomIntermediateCameraParameters xml node does not have the R attribute. Cannot construct camera parameters object.");
                        camInfoError = true;
                    }

                    if (paramAttributes.hasAttribute("P"))
                    {
                        QString val = paramAttributes.value("P").toString();
                        QStringList val_list = val.split(",",QString::SkipEmptyParts);
                        if (val_list.size() != 12) // wrong number of parameters
                        {
                            ROS_ERROR("RoomIntermediateCameraParameters P attribute has more than 12 parameters. Cannot construct camera parameters object.");
                            camInfoError = true;
                        } else {
                            for (size_t j=0; j< val_list.size();j++)
                            {
                                camInfo.P[j] = val_list[j].toDouble();
                            }
                        }
                    } else {
                        ROS_ERROR("RoomIntermediateCameraParameters xml node does not have the P attribute. Cannot construct camera parameters object.");
                        camInfoError = true;
                    }


                }

            }
            token = xmlReader.readNext();
        }

        if (!camInfoError)
        {
            structToRet.camInfo = camInfo;
        }
        tf::transformStampedMsgToTF(tfmsg, transform);
        structToRet.transform = transform;
        return structToRet;
    }

    static IntermediatePositionImages parseIntermediatePositionImages(QXmlStreamReader* xmlReader, std::string roomFolder)
    {

        static int intermediatePoisitionCounter = 0;

        IntermediatePositionImages toRet;
        tf::StampedTransform transform;
        geometry_msgs::TransformStamped tfmsg;
        sensor_msgs::CameraInfo camInfo;
        bool camInfoError = false;

        if (xmlReader->name()!="IntermediatePosition")
        {
            ROS_ERROR("Cannot parse IntermediatePosition node, it has a different name: %s",xmlReader->name().toString().toStdString().c_str());
            return toRet;
        }
        QXmlStreamAttributes attributes = xmlReader->attributes();

        // read in number of depth and rgb images
        int numRGB, numDepth;
        if (attributes.hasAttribute("RGB_Images"))
        {
            numRGB = attributes.value("RGB_Images").toString().toInt();
        }

        if (attributes.hasAttribute("Depth_Images"))
        {
            numDepth = attributes.value("Depth_Images").toString().toInt();
        }

        // read in the images

        for (int i=0; i<numRGB;i++)
        {
            std::stringstream ss; ss<<roomFolder;ss<<"/rgb_image"; ss<<"_"<<std::setfill('0')<<std::setw(4)<<intermediatePoisitionCounter<<"_"<<std::setfill('0')<<std::setw(4)<<i<<".png";
            cv::Mat image = cv::imread(ss.str().c_str(), CV_LOAD_IMAGE_COLOR);
            ROS_INFO_STREAM("Loading intermediate image: "<<ss.str().c_str());
            toRet.vIntermediateRGBImages.push_back(image);
        };

        for (int i=0; i<numDepth;i++)
        {
            std::stringstream ss; ss<<roomFolder;ss<<"/depth_image"; ss<<"_"<<std::setfill('0')<<std::setw(4)<<intermediatePoisitionCounter<<"_"<<std::setfill('0')<<std::setw(4)<<i<<".png";
            cv::Mat image = cv::imread(ss.str().c_str(), CV_LOAD_IMAGE_ANYDEPTH);
            ROS_INFO_STREAM("Loading intermediate image: "<<ss.str().c_str());
            toRet.vIntermediateDepthImages.push_back(image);
        };

        QXmlStreamReader::TokenType token = xmlReader->readNext();

        while(!((token == QXmlStreamReader::EndElement) && (xmlReader->name() == "IntermediatePosition")) )
        {

            if ((xmlReader->name() == "RGBTransform") && !(token == QXmlStreamReader::EndElement))
            {
                bool errorReading = false;
                toRet.intermediateRGBTransform = readTfStampedTransformFromXml(xmlReader, "RGBTransform", errorReading);
            }

            if ((xmlReader->name() == "DepthTransform") && !(token == QXmlStreamReader::EndElement))
            {
                bool errorReading = false;
                toRet.intermediateDepthTransform = readTfStampedTransformFromXml(xmlReader, "DepthTransform", errorReading);
            }

            if ((xmlReader->name() == "RGBCameraParameters") && !(token == QXmlStreamReader::EndElement))
            {
                bool errorReading = false;
                toRet.intermediateRGBCamParams = readCamParamsFromXml(xmlReader, "RGBCameraParameters", errorReading);
            }

            if ((xmlReader->name() == "DepthCameraParameters") && !(token == QXmlStreamReader::EndElement))
            {
                bool errorReading = false;
                toRet.intermediateDepthCamParams = readCamParamsFromXml(xmlReader, "DepthCameraParameters", errorReading);
            }

            token = xmlReader->readNext();
        }
        intermediatePoisitionCounter++;

        return toRet;

    }

    // Reads in a image_geometry::PinholeCameraModel from the current xml node
    // Does not advance the xml reader!!

    static image_geometry::PinholeCameraModel readCamParamsFromXml(QXmlStreamReader* xmlReader, std::string nodeName, bool& errorReading)
    {
        image_geometry::PinholeCameraModel toRet;
        sensor_msgs::CameraInfo camInfo;
        bool camInfoError = false;

        // camera parameters
        if (xmlReader->name() == nodeName.c_str())
        {
            QXmlStreamAttributes paramAttributes = xmlReader->attributes();

            if (paramAttributes.hasAttribute("height"))
            {
                QString val = paramAttributes.value("height").toString();
                camInfo.height = val.toInt();

            } else {
                ROS_ERROR("%s xml node does not have the height attribute. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
                camInfoError = true;
            }

            if (paramAttributes.hasAttribute("width"))
            {
                QString val = paramAttributes.value("width").toString();
                camInfo.width = val.toInt();

            } else {
                ROS_ERROR("%s xml node does not have the width attribute. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
                camInfoError = true;
            }

            if (paramAttributes.hasAttribute("distortion_model"))
            {
                QString val = paramAttributes.value("distortion_model").toString();
                camInfo.distortion_model = val.toStdString();

            } else {
                ROS_ERROR("%s xml node does not have the distortion_model attribute. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
                camInfoError = true;
            }

            if (paramAttributes.hasAttribute("frame_id"))
            {
                QString val = paramAttributes.value("frame_id").toString();
                camInfo.header.frame_id = val.toStdString();

            } else {
                ROS_ERROR("%s xml node does not have the frame_id attribute. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
                camInfoError = true;
            }

            if (paramAttributes.hasAttribute("K"))
            {
                QString val = paramAttributes.value("K").toString();
                QStringList val_list = val.split(",",QString::SkipEmptyParts);
                if (val_list.size() != 9) // wrong number of parameters
                {
                    ROS_ERROR("%s K attribute has more than 9 parameters. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
                    camInfoError = true;
                } else {
                    for (size_t j=0; j< val_list.size();j++)
                    {
                        camInfo.K[j] = val_list[j].toDouble();
                    }
                }
            } else {
                ROS_ERROR("%s xml node does not have the K attribute. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
                camInfoError = true;
            }

            if (paramAttributes.hasAttribute("D"))
            {
                QString val = paramAttributes.value("D").toString();
                QStringList val_list = val.split(",",QString::SkipEmptyParts);
                if (val_list.size() != 5) // wrong number of parameters
                {
                    ROS_ERROR("%s D attribute has more than 5 parameters. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
                    camInfoError = true;
                } else {
                    for (size_t j=0; j< val_list.size();j++)
                    {
                        camInfo.D.push_back(val_list[j].toDouble());
                    }
                }
            } else {
                ROS_ERROR("%s xml node does not have the D attribute. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
                camInfoError = true;
            }

            if (paramAttributes.hasAttribute("R"))
            {
                QString val = paramAttributes.value("R").toString();
                QStringList val_list = val.split(",",QString::SkipEmptyParts);
                if (val_list.size() != 9) // wrong number of parameters
                {
                    ROS_ERROR("%s R attribute has more than 9 parameters. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
                    camInfoError = true;
                } else {
                    for (size_t j=0; j< val_list.size();j++)
                    {
                        camInfo.R[j] = val_list[j].toDouble();
                    }
                }
            } else {
                ROS_ERROR("%s xml node does not have the R attribute. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
                camInfoError = true;
            }

            if (paramAttributes.hasAttribute("P"))
            {
                QString val = paramAttributes.value("P").toString();
                QStringList val_list = val.split(",",QString::SkipEmptyParts);
                if (val_list.size() != 12) // wrong number of parameters
                {
                    ROS_ERROR("%s P attribute has more than 12 parameters. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
                    camInfoError = true;
                } else {
                    for (size_t j=0; j< val_list.size();j++)
                    {
                        camInfo.P[j] = val_list[j].toDouble();
                    }
                }
            } else {
                ROS_ERROR("%s xml node does not have the P attribute. Cannot construct camera parameters object.",xmlReader->name().toString().toStdString().c_str());
                camInfoError = true;
            }
        } else {
            ROS_ERROR("Error while attempting to construct camera parameters object from node name %s  Node expected %s",xmlReader->name().toString().toStdString().c_str(), nodeName.c_str());
            camInfoError = true;
        }

        if (!camInfoError)
        {
            toRet.fromCameraInfo(camInfo);
        }

        return toRet;


    }

    // Reads in a TfStampedTransform from the current xml node
    // Does not advance the xml reader!!

    static tf::StampedTransform readTfStampedTransformFromXml(QXmlStreamReader* xmlReader, std::string nodeName, bool& errorReading)
    {

        geometry_msgs::TransformStamped tfmsg;
        tf::StampedTransform transform;

        if (xmlReader->name() == nodeName.c_str())
        {
            errorReading = false;
            QXmlStreamAttributes paramAttributes = xmlReader->attributes();

            if (paramAttributes.hasAttribute("Stamp_sec"))
            {
                QString val = paramAttributes.value("Stamp_sec").toString();
                tfmsg.header.stamp.sec = val.toInt();

            } else {
                ROS_ERROR("%s xml node does not have the Stamp_sec attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
                errorReading = true;
            }
            if (paramAttributes.hasAttribute("Stamp_nsec"))
            {
                QString val = paramAttributes.value("Stamp_nsec").toString();
                tfmsg.header.stamp.nsec = val.toInt();

            } else {
                ROS_ERROR("%s xml node does not have the Stamp_nsec attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
                errorReading = true;
            }
            if (paramAttributes.hasAttribute("FrameId"))
            {
                QString val = paramAttributes.value("FrameId").toString();
                tfmsg.header.frame_id = val.toStdString();

            } else {
                ROS_ERROR("%s xml node does not have the FrameId attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
                errorReading = true;
            }
            if (paramAttributes.hasAttribute("ChildFrameId"))
            {
                QString val = paramAttributes.value("ChildFrameId").toString();
                tfmsg.child_frame_id = val.toStdString();

            } else {
                ROS_ERROR("%s xml node does not have the ChildFrameId attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
                errorReading = true;
            }
            if (paramAttributes.hasAttribute("Trans_x"))
            {
                QString val = paramAttributes.value("Trans_x").toString();
                tfmsg.transform.translation.x = val.toDouble();

            } else {
                ROS_ERROR("%s xml node does not have the Trans_x attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
                errorReading = true;
            }
            if (paramAttributes.hasAttribute("Trans_y"))
            {
                QString val = paramAttributes.value("Trans_y").toString();
                tfmsg.transform.translation.y = val.toDouble();

            } else {
                ROS_ERROR("%s xml node does not have the Trans_y attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
                errorReading = true;
            }
            if (paramAttributes.hasAttribute("Trans_z"))
            {
                QString val = paramAttributes.value("Trans_z").toString();
                tfmsg.transform.translation.z = val.toDouble();

            } else {
                ROS_ERROR("%s xml node does not have the Trans_z attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
                errorReading = true;
            }
            if (paramAttributes.hasAttribute("Rot_w"))
            {
                QString val = paramAttributes.value("Rot_w").toString();
                tfmsg.transform.rotation.w = val.toDouble();

            } else {
                ROS_ERROR("%s xml node does not have the Rot_w attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
                errorReading = true;
            }
            if (paramAttributes.hasAttribute("Rot_x"))
            {
                QString val = paramAttributes.value("Rot_x").toString();
                tfmsg.transform.rotation.x = val.toDouble();

            } else {
                ROS_ERROR("%s xml node does not have the Rot_x attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
                errorReading = true;
            }
            if (paramAttributes.hasAttribute("Rot_y"))
            {
                QString val = paramAttributes.value("Rot_y").toString();
                tfmsg.transform.rotation.y = val.toDouble();

            } else {
                ROS_ERROR("%s xml node does not have the Rot_y attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
                errorReading = true;
            }
            if (paramAttributes.hasAttribute("Rot_z"))
            {
                QString val = paramAttributes.value("Rot_z").toString();
                tfmsg.transform.rotation.z = val.toDouble();

            } else {
                ROS_ERROR("%s xml node does not have the Rot_z attribute. Cannot construct tf::StampedTransform.", xmlReader->name().toString().toStdString().c_str());
                errorReading = true;
            }

        } else {
            ROS_ERROR("Error while attempting to construct tf::StampedTransform from node name %s  Node expected %s",xmlReader->name().toString().toStdString().c_str(), nodeName.c_str());
            errorReading = true;
        }

        if (!errorReading)
        {
//            ROS_INFO_STREAM("No error while parsing node "<<xmlReader->name().toString().toStdString()<<"  constructing tf object ");
            tf::transformStampedMsgToTF(tfmsg, transform);
        }

        return transform;

    }

};
#endif
