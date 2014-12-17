#include "simpleXMLparser.h"
#include "simpleSummaryParser.h"

typedef pcl::PointXYZRGB PointType;

typedef typename SimpleSummaryParser::EntityStruct Entities;

using namespace std;

int main(int argc, char** argv)
{

    if (argc < 2)
    {
        cout<<"Please provide folder as input."<<endl;
        return -1;
    }

    /*

    string folderPath = argv[1];
    string summaryXMLPath = folderPath + "/index.xml";

    SimpleSummaryParser summary_parser(summaryXMLPath);
    summary_parser.createSummaryXML(folderPath);

    SimpleXMLParser simple_parser;
    SimpleXMLParser::RoomData<PointType> roomData;

    std::vector<Entities> allSweeps = summary_parser.getRooms();

    for (size_t i=0; i<allSweeps.size(); i++)
    {
        cout<<"Parsing "<<allSweeps[i].roomXmlFile<<endl;

        roomData = simple_parser.loadRoomFromXML<PointType>(allSweeps[i].roomXmlFile);
        cout<<"Complete cloud size "<<roomData.completeRoomCloud->points.size()<<endl;
        cout<<"Room waypoint id "<<roomData.roomWaypointId<<std::endl;

        for (size_t i=0; i<roomData.vIntermediateRoomClouds.size(); i++)
        {
            cout<<"Intermediate cloud size "<<roomData.vIntermediateRoomClouds[i]->points.size()<<endl;
            cout<<"Fx: "<<roomData.vIntermediateRoomCloudCamParams[i].fx()<<" Fy: "<<roomData.vIntermediateRoomCloudCamParams[i].fy()<<endl;
        }
    }
    */
}
