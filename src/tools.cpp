#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  //rmse delcaration
  VectorXd rmse(4);
  rmse << 0,0,0,0;
    
  //check data consistency
  if(estimations.size() == 0){
    cout<< "Estimation vector size should not be zero" << endl;
  }
  if(estimations.size() != ground_truth.size()){
    cout<< "Estimation vector size should equal ground truth vector size" << endl;
  }
    
  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }
    
  //calculate the mean
  rmse /= estimations.size();
  
  //calculate the squared root
  rmse = rmse.array().sqrt();
    
  //return the result
  return rmse;
    
}