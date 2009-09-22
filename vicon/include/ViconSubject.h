#ifndef _VICON_SUBJECT_
#define _VICON_SUBJECT_

#include <vector>
#include <map>

class Vector3f
{
public:
	Vector3f() { this->x = this->y = this->z = 0.0; }
	Vector3f(double x, double y, double z) { this->x = x; this->y = y; this->z = z; }
	Vector3f(const Vector3f &copy) { this->x = copy.x; this->y = copy.y; this->z = copy.z; }
	
	double x, y, z;
};

class ViconSubject
{
public: 
	ViconSubject();
	ViconSubject(const ViconSubject &copy);
	ViconSubject(const char *filename);
	void LoadVSK(const char *filename);
	int SetCurrentMarkerPoint(std::string &ident, double &x, double &y, double &z);

	void Print();
	void PrintMarkerPointsNow();
	void ClearMarkerPointsNow();

	int ComputeTransformation(double &x, double &y, double &z, double &qw, double &qx, double &qy, double &qz);

	std::string GetName() { return this->name; }
	double GetQuality();
  int GetNumMarkerPointsNow() { return this->markerPointsNow.size(); }

private:
	std::map<std::string, Vector3f> markerPoints;
	std::map<std::string, Vector3f> markerPointsNow;
	std::string name;
};

#endif
