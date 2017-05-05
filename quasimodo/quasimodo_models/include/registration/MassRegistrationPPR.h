#ifndef MassRegistrationPPR_H
#define MassRegistrationPPR_H

#include "MassRegistration.h"
#include <time.h>

namespace reglib
{

	class MassRegistrationPPR : public MassRegistration
	{
		public:

		MatchType type;

		bool fast_opt;
		bool use_PPR_weight;
		bool use_features;
		bool normalize_matchweights;

		double stopval;
		unsigned steps;

		double rematch_time;
		double residuals_time;
		double opt_time;
		double computeModel_time;
		double setup_matches_time;
		double setup_equation_time;
		double setup_equation_time2;
		double solve_equation_time;
		double total_time_start;

		const unsigned int maxcount = 1000000;
		double * Qp_arr;
		double * Qn_arr;
		double * Xp_arr;
		double * Xn_arr;
		double * rangeW_arr;

		std::vector<int> nr_datas;

		std::vector< bool > is_ok;
		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > points;
		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > colors;
		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > normals;
		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > transformed_points;
		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > transformed_normals;
		std::vector< Eigen::VectorXd > informations;

		std::vector< int > nr_arraypoints;
		std::vector< double * > arraypoints;
		std::vector< double * > arraynormals;
		std::vector< double * > arraycolors;
		std::vector< double * > arrayinformations;


		std::vector< Tree3d * > trees3d;
		std::vector< ArrayData3D<double> * > a3dv;

		std::vector<int> nr_matches;
		std::vector< std::vector< std::vector<int> > > matchids;

		std::vector<int> sweepids;
		std::vector<int> background_nr_datas;
//		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > background_points;
//		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > background_colors;
//		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > background_normals;
//		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > background_transformed_points;
//		std::vector< Eigen::Matrix<double, 3, Eigen::Dynamic> > background_transformed_normals;
//		std::vector< Eigen::VectorXd > background_informations;
//		std::vector< nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple> * > background_trees;
//		std::vector<int> background_nr_matches;
//		std::vector< std::vector< std::vector<int> > > background_matchids;


//		std::vector<int> feature_start;//Dimension of data a specific feature starts, if the feature is RGB this should be 3
//		std::vector<int> feature_end;//Dimension of data a specific feature ends, if the feature is RGB this should be 5
//		std::vector< DistanceWeightFunction2 * > feature_func;

		DistanceWeightFunction2PPR2 * func;

		MassRegistrationPPR(double startreg = 0.05, bool visualize = false);
		~MassRegistrationPPR();
		
		void addModelData(Model * model, bool submodels = true);

		void clearData();
		void addData(RGBDFrame* frame, ModelMask * mmask);
		void addData(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

		MassFusionResults getTransforms(std::vector<Eigen::Matrix4d> guess);
		void setData(std::vector<RGBDFrame*> frames_, std::vector<ModelMask *> mmasks);
		void setData(std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > all_clouds);

		void rematch(std::vector<Eigen::Matrix4d> poses, std::vector<Eigen::Matrix4d> prev_poses, bool first);
		Eigen::MatrixXd getAllResiduals(std::vector<Eigen::Matrix4d> poses);
		std::vector<Eigen::Matrix4d> optimize(std::vector<Eigen::Matrix4d> poses);
	};

}

#endif
