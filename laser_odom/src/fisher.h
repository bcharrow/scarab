#ifndef FISHER_H
#define FISHER_H

#include <ros/ros.h>

#include <iostream>
#include "armadillo"
#include "type.h"

using namespace arma;
using namespace std;

void cov_fisher(vector<scanData> data, double* cov);

#endif
