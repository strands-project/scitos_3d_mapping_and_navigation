#include "registration/MassRegistration.h"

namespace reglib
{

bool MassRegistration::okVal(double v){return !std::isnan(v) && !(v == std::numeric_limits<double>::infinity());}

bool MassRegistration::isValidPoint(pcl::PointXYZRGBNormal p){
	return	okVal (p.x)         && okVal (p.y)			&& okVal (p.z) &&			//No nans or inf in position
			okVal (p.normal_x)  && okVal (p.normal_y)	&& okVal (p.normal_z) &&	//No nans or inf in normal
			!(p.x == 0			&& p.y == 0				&& p.z == 0 ) &&						//not a zero point
			!(p.normal_x == 0	&& p.normal_y == 0		&& p.normal_z == 0);					//not a zero normal
}

MassRegistration::MassRegistration(){
	visualizationLvl = 0;
	nomask = true;
	maskstep = 1;
	nomaskstep = 100000;
	timeout = 60;//1 minute timeout
}
MassRegistration::~MassRegistration(){}

void MassRegistration::setData(std::vector<RGBDFrame*> frames_,std::vector<ModelMask *> mmasks_){
	frames = frames_;
	mmasks = mmasks_;
}

void MassRegistration::setData(std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > all_clouds){}

void MassRegistration::setVisualizationLvl(unsigned int lvl){
	visualizationLvl = lvl;
}

MassFusionResults MassRegistration::getTransforms(std::vector<Eigen::Matrix4d> guess){
	return MassFusionResults(guess,0);
}

void MassRegistration::clearData(){}

void MassRegistration::addModelData(Model * model, bool submodels){
	if(submodels){
		for(unsigned int i = 0; i < model->submodels.size(); i++){
			addData(model->submodels[i]->getPCLnormalcloud(1,false));
		}
	}else{

	}
}

void MassRegistration::addData(RGBDFrame* frame, ModelMask * mmask){}
void MassRegistration::addData(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud){}

void MassRegistration::show(Eigen::MatrixXd X, Eigen::MatrixXd Y){
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	unsigned int s_nr_data = X.cols();
	unsigned int d_nr_data = Y.cols();

	scloud->points.clear();
	dcloud->points.clear();

	for(unsigned int i = 0; i < s_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = X(0,i);p.y = X(1,i);p.z = X(2,i);p.b = 0;p.g = 255;p.r = 0;scloud->points.push_back(p);}
	for(unsigned int i = 0; i < d_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = Y(0,i);p.y = Y(1,i);p.z = Y(2,i);p.b = 0;p.g = 0;p.r = 255;dcloud->points.push_back(p);}

	viewer->removeAllPointClouds();
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
	viewer->spin();

}

Eigen::MatrixXd MassRegistration::getMat(int rows, int cols, double * datas){
	Eigen::MatrixXd mat (rows,cols);
	for(int i = 0; i < rows; i++){
		for(int j = 0; j < cols; j++){
			mat(i,j) = i;//datas[cols*i+j];
		}
	}
	return mat;
}


void MassRegistration::show(std::vector<Eigen::MatrixXd> Xv, bool save, std::string filename, bool stop){
	printf("show\n");
	//for(unsigned int a = 0; a < Xv.size(); a++){
	viewer->removeAllPointClouds();

	srand(0);
	for(unsigned int xi = 0; xi < Xv.size(); xi++){

		Eigen::MatrixXd X = Xv[xi];
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		int r,g,b;

		r = 256*(1+(rand()%4))/4 - 1;//255*((xi+1) & 1);
		g = 256*(1+(rand()%4))/4 - 1;//255*((xi+1) & 1);
		b = 256*(1+(rand()%4))/4 - 1;//255*(xi & 1);

		unsigned int nr_data = X.cols();
		cloud->points.clear();
		for(unsigned int i = 0; i < nr_data; i++){
			pcl::PointXYZRGBNormal p;
			p.x = X(0,i);
			p.y = X(1,i);
			p.z = X(2,i);
			p.b = r;
			p.g = g;
			p.r = b;
			cloud->points.push_back(p);
		}

		//printf("cloud->points: %i\n",cloud->points.size());

		char buf [1024];
		sprintf(buf,"cloud%i",xi);
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(cloud), buf);

	}



	if(!save){
		viewer->spin();
	}else{
		viewer->spinOnce();
	}
	//void pcl::visualization::PCLVisualizerInteractorStyle::saveScreenshot	(	const std::string & 	file	)
	if(save){
		printf("saving: %s\n",filename.c_str());
		viewer->saveScreenshot(filename);
	}
	viewer->removeAllPointClouds();
	//}
}

void MassRegistration::savePCD(std::vector<Eigen::MatrixXd> Xv, std::string path){

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	srand(0);
	for(unsigned int xi = 0; xi < Xv.size(); xi++){
		Eigen::MatrixXd X = Xv[xi];
		int r,g,b;
		r = 256*(1+(rand()%4))/4 - 1;//255*((xi+1) & 1);
		g = 256*(1+(rand()%4))/4 - 1;//255*((xi+1) & 1);
		b = 256*(1+(rand()%4))/4 - 1;//255*(xi & 1);

		unsigned int nr_data = X.cols();
		for(unsigned int i = 0; i < nr_data; i++){
			pcl::PointXYZRGBNormal p;
			p.x = X(0,i);
			p.y = X(1,i);
			p.z = X(2,i);
			p.b = r;
			p.g = g;
			p.r = b;
			cloud->points.push_back(p);
		}
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;

	//pcl::io::savePCDFileBinaryCompressed (cloud,path);
	pcl::io::savePCDFileBinaryCompressed (path, *cloud);
}
/*
void Registration::show(Eigen::MatrixXd X, Eigen::MatrixXd Y){

	//printf("start show\n");

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	unsigned int s_nr_data = X.cols();
	unsigned int d_nr_data = Y.cols();
	//printf("nr datas: %i %i\n",s_nr_data,d_nr_data);
	scloud->points.clear();
	dcloud->points.clear();
	for(unsigned int i = 0; i < s_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = X(0,i);p.y = X(1,i);p.z = X(2,i);p.b = 0;p.g = 255;p.r = 0;scloud->points.push_back(p);}
	for(unsigned int i = 0; i < d_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = Y(0,i);p.y = Y(1,i);p.z = Y(2,i);p.b = 0;p.g = 0;p.r = 255;dcloud->points.push_back(p);}
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
	//printf("pre\n");
	viewer->spin();
	//printf("post\n");
	viewer->removeAllPointClouds();

	//printf("stop show\n");
}

void Registration::show(Eigen::MatrixXd X, Eigen::MatrixXd Xn, Eigen::MatrixXd Y, Eigen::MatrixXd Yn){

	//printf("start show\n");

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	pcl::PointCloud<pcl::Normal>::Ptr sNcloud (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr dNcloud (new pcl::PointCloud<pcl::Normal>);
	unsigned int s_nr_data = X.cols();
	unsigned int d_nr_data = Y.cols();
	//printf("nr datas: %i %i\n",s_nr_data,d_nr_data);
	scloud->points.clear();
	dcloud->points.clear();

	sNcloud->points.clear();
	dNcloud->points.clear();

	for(unsigned int i = 0; i < s_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = X(0,i);p.y = X(1,i);p.z = X(2,i);p.b = 0;p.g = 255;p.r = 0;scloud->points.push_back(p);}
	for(unsigned int i = 0; i < d_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = Y(0,i);p.y = Y(1,i);p.z = Y(2,i);p.b = 0;p.g = 0;p.r = 255;dcloud->points.push_back(p);}

	for(unsigned int i = 0; i < s_nr_data; i++){pcl::Normal p;p.normal_x = Xn(0,i);p.normal_y = Xn(1,i);p.normal_z = Xn(2,i);sNcloud->points.push_back(p);}
	for(unsigned int i = 0; i < d_nr_data; i++){pcl::Normal p;p.normal_x = Yn(0,i);p.normal_y = Yn(1,i);p.normal_z = Yn(2,i);dNcloud->points.push_back(p);}
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
	viewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::Normal> (scloud, sNcloud, 100, 0.2, "sNcloud");

	viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
	viewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::Normal> (dcloud, dNcloud, 100, 0.2, "dNcloud");
	//printf("pre\n");
	viewer->spin();
	//printf("post\n");
	viewer->removeAllPointClouds();

	//printf("stop show\n");
}

void Registration::setVisualizationLvl(unsigned int lvl){visualizationLvl = lvl;}

void Registration::show(Eigen::MatrixXd X, Eigen::MatrixXd Y, Eigen::VectorXd W){
	show(X,Y);
	double mw = W.maxCoeff();
	W = W/mw;
	//std::cout << W << std::endl;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	unsigned int s_nr_data = X.cols();
	unsigned int d_nr_data = Y.cols();

	//printf("nr datas: %i %i\n",s_nr_data,d_nr_data);
	scloud->points.clear();
	dcloud->points.clear();
	//for(unsigned int i = 0; i < s_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = X(0,i);p.y = X(1,i);p.z = X(2,i);p.b = 0;p.g = 255;p.r = 0;scloud->points.push_back(p);}
	//for(unsigned int i = 0; i < d_nr_data; i++){pcl::PointXYZRGBNormal p;p.x = Y(0,i);p.y = Y(1,i);p.z = Y(2,i);p.b = 0;p.g = 0;p.r = 255;dcloud->points.push_back(p);}
	for(unsigned int i = 0; i < s_nr_data; i++){
		pcl::PointXYZRGBNormal p;p.x = X(0,i);p.y = X(1,i);p.z = X(2,i);p.b = 255*W(i);	p.g = 255*W(i);	p.r = 255*W(i);	scloud->points.push_back(p);
		//if(W(i) > 0.001){	pcl::PointXYZRGBNormal p;p.x = X(0,i);p.y = X(1,i);p.z = X(2,i);p.b = 0;p.g = 255;	p.r = 0;	scloud->points.push_back(p);}
		//else{				pcl::PointXYZRGBNormal p;p.x = X(0,i);p.y = X(1,i);p.z = X(2,i);p.b = 0;p.g = 0;	p.r = 255;	scloud->points.push_back(p);}
	}
	for(unsigned int i = 0; i < d_nr_data; i+=1){pcl::PointXYZRGBNormal p;p.x = Y(0,i);p.y = Y(1,i);p.z = Y(2,i);p.b = 255;p.g = 0;p.r = 0;dcloud->points.push_back(p);}
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (scloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(scloud), "scloud");
	viewer->addPointCloud<pcl::PointXYZRGBNormal> (dcloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(dcloud), "dcloud");
	//printf("pre spin\n");
	viewer->spin();
	//printf("post spin\n");
	viewer->removeAllPointClouds();
}
*/
}
