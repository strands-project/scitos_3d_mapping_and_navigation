#ifndef DistanceWeightFunction2test_H
#define DistanceWeightFunction2test_H

#include <vector>
#include <stdio.h>
#include <stdlib.h> 
//#include "model2.h"
//#include "inputframe.h"
#include <Eigen/Dense>

using namespace Eigen;
namespace reglib
{

enum Function {
    PNORM,
    TUKEY,
    FAIR,
    LOGISTIC,
    TRIMMED,
	THRESHOLD,
    NONE
};

void uniform_weight(Eigen::VectorXd& r);
void threshold_weight(Eigen::VectorXd& r, double p);
void pnorm_weight(Eigen::VectorXd& r, double p, double reg=1e-8);

void tukey_weight(Eigen::VectorXd& r, double p);
void fair_weight(Eigen::VectorXd& r, double p);
void logistic_weight(Eigen::VectorXd& r, double p);
void trimmed_weight(Eigen::VectorXd& r, double p);
void robust_weight(Function f, Eigen::VectorXd& r, double p);

class DistanceWeightFunction2
{
public:
	Function f;     // robust function type
	double p;       // paramter of the robust function
    double regularization;
	double convergence_threshold;
	bool debugg_print;

	std::string savePath;
	std::stringstream saveData;

	DistanceWeightFunction2();
	~DistanceWeightFunction2();

	MatrixXd getMat(std::vector<double> & vec);

	virtual void computeModel(std::vector<double> & vec);
	virtual void computeModel(MatrixXd mat);
	virtual VectorXd getProbs(std::vector<double> & vec);
	virtual VectorXd getProbs(MatrixXd mat);

	virtual double getProbInfront(double d, bool debugg = false);
	virtual double getProb(double d, bool debugg = false);
	virtual double getNoise();
	virtual double getConvergenceThreshold();
	virtual bool update();
	virtual void reset();
	virtual std::string getString();
};

}

#include "DistanceWeightFunction2PPR.h"
#include "DistanceWeightFunction2PPR2.h"
#include "DistanceWeightFunction2PPR3.h"
#endif // DistanceWeightFunction2_H
