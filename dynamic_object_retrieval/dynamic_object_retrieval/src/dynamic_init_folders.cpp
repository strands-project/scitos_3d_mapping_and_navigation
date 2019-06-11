#include <metaroom_xml_parser/simple_summary_parser.h>
#include <metaroom_xml_parser/simple_xml_parser.h>
#include <metaroom_xml_parser/simple_dynamic_object_parser.h>
#include <metaroom_xml_parser/load_utilities.h>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <metaroom_xml_parser/simple_summary_parser.h>
#include <metaroom_xml_parser/simple_xml_parser.h>
#include <metaroom_xml_parser/simple_dynamic_object_parser.h>
#include <metaroom_xml_parser/load_utilities.h>

#include <metaroom_xml_parser/load_utilities.h>

#include "dynamic_object_retrieval/summary_types.h"

using namespace std;
using namespace dynamic_object_retrieval;
using PointT = pcl::PointXYZRGB;

typedef pcl::PointXYZRGB PointType;

typedef pcl::PointXYZRGB PointType;
typedef semantic_map_load_utilties::DynamicObjectData<PointType> ObjectData;

typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;

using namespace std;

int main(int argc, char** argv)
{
    string folder;


    if (argc > 1){
        folder = argv[1];
    } else {
        cout<<"Please specify the folder from where to load the data PFE BILAL OUSSAMA."<<endl;
        return -1;
    }

    cout<<"Looking for sweeps..."<<endl;
    
    
    boost::filesystem::path data_path(argv[1]);

    //vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(),True);
        vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    cout<<folder_xmls.size()<<" sweeps found."<<endl;



    
    data_summary summary;
    summary.nbr_sweeps = folder_xmls.size();

    summary.save(data_path);

    for (const string& xml : folder_xmls) {
        boost::filesystem::path xml_path(xml);
        boost::filesystem::path convex_path = xml_path.parent_path() / "convex_segments";
        boost::filesystem::path subsegment_path = xml_path.parent_path() / "subsegments";
        boost::filesystem::create_directory(convex_path);
        boost::filesystem::create_directory(subsegment_path);

        sweep_summary convex_summary;
        if (!boost::filesystem::exists(convex_path / "segments.json")) {
            convex_summary.save(convex_path);
        }
        sweep_summary subsegment_summary;
        if (!boost::filesystem::exists(subsegment_path / "segments.json")) {
            subsegment_summary.save(subsegment_path);
        }
    }

    for (string sweep : folder_xmls){

        cout<<"Sweep "<<sweep<<endl;

        vector<ObjectData> objects = semantic_map_load_utilties::loadAllDynamicObjectsFromSingleSweep<PointType>(sweep);
        cout<<"Found "<<objects.size()<<" objects"<<endl;
        for (size_t k=0; k<objects.size(); k++){
            auto object = objects[k];
            cout<<"Object "<<k<<" has "<<object.vAdditionalViews.size()<<" additional views"<<endl;
        }
    }
}
