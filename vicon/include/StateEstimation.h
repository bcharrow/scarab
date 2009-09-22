#ifndef _STATE_ESTIMATION_H_
#define _STATE_ESTIMATION_H_

#include <list>
#include "CQuat.h"
#include "ClientCodes.h"

class State 
{
public:
	State();
	State(const State &copy);
	State(const BodyData &data);
	State(const BodyData &data, double rx, double ry, double rz);
	State(double x, double y, double z, double qw, double qx, double qy, double qz);

	double TX;
	double TY;
	double TZ;

	CQuat Q;

	double marker_quality;
};

std::ostream& operator<< (std::ostream& os, const State &foo);

class StateEstimate
{
public:
	StateEstimate();
	StateEstimate(int N, double alpha, double t);
	void SetParam(int N, double alpha, double t);
	void AddStateMeasurement(State &data, double quality = 1.0);
	void UpdateStateEstimate();
	int N;

	double alpha;
	double t;

	double dX, dY, dZ; //! Translation from ViconBody -> RobotBody (in ViconBody frame)
	CQuat dQ; //! Rotation from ViconBody Frame -> RobotBody Frame.
	State robotBodyEstimate;
	void SetOffset(double dX, double dY, double dZ, CQuat dQ);

  bool tracking;
	State estimate;
	std::list<State> history;
};

#endif
