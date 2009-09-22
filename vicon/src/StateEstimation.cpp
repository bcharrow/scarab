#include <iostream>
#include <list>
#include <math.h>

#include "StateEstimation.h"

#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif


State::State()
{
	TX = 0;
	TY = 0;
	TZ = 0;
}

State::State(const State &copy)
{
	TX = copy.TX;
	TY = copy.TY;
	TZ = copy.TZ;

	Q = copy.Q;

	marker_quality = copy.marker_quality;
}

State::State(const BodyData &data)
{
	TX = data.TX;
	TY = data.TY;
	TZ = data.TZ;

	Q = CQuat(data.QX, data.QY, data.QZ, data.QW);
}

State::State(const BodyData &data, double rx, double ry, double rz)
{
	TX = data.TX;
	TY = data.TY;
	TZ = data.TZ;

	double radians = sqrt(rx*rx + ry*ry + rz*rz);
	Q.SetAxis(radians, rx/radians, ry/radians, rz/radians);
}

State::State(double x, double y, double z, double qw, double qx, double qy, double qz)
{
	TX = x;
	TY = y;
	TZ = z;

	Q = CQuat(qx, qy, qz, qw);
}

std::ostream& operator<< (std::ostream& os, const State &foo)
{
	os << foo.TX << ", " << foo.TY << ", " << foo.TZ << " :: " << foo.Q.x << ", " << foo.Q.y << ", " << foo.Q.z << ", " << foo.Q.w;

	return os;
}

StateEstimate::StateEstimate()
{
	this->N = 1;
	this->alpha = 0.0;
	this->t = 1.0;

	this->dX = this->dY = this->dZ = 0.0;

  this->tracking = false;
}
StateEstimate::StateEstimate(int N, double alpha, double t)
{
	this->N = N;
	this->alpha = alpha;
	this->t = t;
  this->tracking = false;
}

void StateEstimate::SetOffset(double dX, double dY, double dZ, CQuat dQ)
{
	this->dX = dX;
	this->dY = dY;
	this->dZ = dZ;
	this->dQ = dQ;
}

void StateEstimate::SetParam(int N, double alpha, double t)
{
	this->N = N;
	this->alpha = alpha;
	this->t = t;
}

void StateEstimate::AddStateMeasurement(State &data, double quality)
{
  this->tracking = true;
	history.push_front(data);

	estimate.marker_quality = quality;

	// keep history to N items
	while((int)history.size() > N)
		history.pop_back();

	UpdateStateEstimate();
}

void StateEstimate::UpdateStateEstimate()
{
	// Do moving average on position data
	double x = 0;
	double y = 0;
	double z = 0;

	double idx = 0;
	double den = 0;
	double scale = 1;
	for(std::list<State>::iterator i = history.begin();
		i != history.end();
		++i) {

		scale = pow(1-alpha, idx);
		x += i->TX*scale;
		y += i->TY*scale;
		z += i->TZ*scale;
		den += scale;
		idx += 1.0;
	}
	x /= den;
	y /= den;
	z /= den;

	estimate.TX = x;
	estimate.TY = y;
	estimate.TZ = z;

	// Do SLERP on orientation data
	// Slerp(q0, q1; t) = (q1*q0^-1)^t*q0

	std::list<State>::iterator now, last;
	now = history.begin();
	last = now; last++;

	if( last != history.end() ) 
		//estimate.Q.Slerp(last->Q, now->Q, t);
		estimate.Q.Slerp(estimate.Q, now->Q, t);
	else
		estimate.Q = now->Q;

	// No SLERP for now!
	if(this->t == 1.0)
		estimate.Q = now->Q;

	// Convert dX,dY,dZ into viconBodyFrame
	robotBodyEstimate.Q = estimate.Q*dQ;

	double dXR = dX;
	double dYR = dY;
	double dZR = dZ;

	robotBodyEstimate.Q.RotateVector(dXR, dYR, dZR);

	robotBodyEstimate.TX = estimate.TX + dXR;
	robotBodyEstimate.TY = estimate.TY + dYR;
	robotBodyEstimate.TZ = estimate.TZ + dZR;

	//std::cout << "robot " << robotBodyEstimate << std::endl;
}
