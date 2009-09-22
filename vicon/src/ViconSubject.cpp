#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <math.h>

#if defined( WIN32 ) && defined( TUNE )
	#include <crtdbg.h>
	_CrtMemState startMemState;
	_CrtMemState endMemState;
#endif

#include <tinyxml/tinyxml.h>


#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_matrix.h>

#include "ViconSubject.h"

ViconSubject::ViconSubject()
{ }

ViconSubject::ViconSubject(const ViconSubject &copy) 
  : markerPoints(copy.markerPoints), name(copy.name)
{ }

ViconSubject::ViconSubject(const char *filename)
{
	this->LoadVSK(filename);
}

void ViconSubject::LoadVSK(const char *filename)
{
	TiXmlDocument doc( filename );
	bool loadOkay = doc.LoadFile();

	if ( !loadOkay )
	{
		printf( "Could not load test file '%s'. Error='%s'. Exiting.\n", filename, doc.ErrorDesc() );
		exit( 1 );
	}

	TiXmlElement *root = doc.RootElement();
	TiXmlElement *markerSet = root->FirstChild("MarkerSet")->ToElement();
	TiXmlElement *markers = markerSet->FirstChild("Markers")->ToElement();
	TiXmlElement *tmp = markers->FirstChild("Marker")->ToElement();

	this->name = std::string(root->Attribute("MODEL"));

	while(tmp) {
		//std::cout << tmp->Attribute("POSITION") << std::endl;
		std::stringstream inp(std::string(tmp->Attribute("POSITION")));
		double x,y,z;
		inp >> x >> y >> z;
		this->markerPoints.insert(make_pair(this->name + std::string(":") + std::string(tmp->Attribute("NAME")), Vector3f(x,y,z)));
		tmp = tmp->NextSiblingElement();
	}
}

void ViconSubject::Print()
{
	std::cout << this->name << " MarkerPoints\n";
	std::map<std::string, Vector3f>::iterator i = markerPoints.begin();
	for(;	 
		i != markerPoints.end();
		++i) {
			std::cout << "\t" << i->first << ": " << i->second.x << " " << i->second.y << " " << i->second.z << std::endl;
	}
}

void ViconSubject::PrintMarkerPointsNow()
{
	std::cout << this->name << " MarkerPoints\n";
	std::map<std::string, Vector3f>::iterator i = markerPointsNow.begin();
	for(;	 
		i != markerPointsNow.end();
		++i) {
			std::cout << "\t" << i->first << ": " << i->second.x << " " << i->second.y << " " << i->second.z << std::endl;
	}
}

double ViconSubject::GetQuality()
{
	return (double)this->markerPointsNow.size()/(double)this->markerPoints.size();
}

int ViconSubject::SetCurrentMarkerPoint(std::string &ident, double &x, double &y, double &z)
{
	this->markerPointsNow.insert(make_pair(ident, Vector3f(x, y, z)));

	return 0;
}

void ViconSubject::ClearMarkerPointsNow()
{
	this->markerPointsNow.clear();
}

int ViconSubject::ComputeTransformation(double &x, double &y, double &z, double &qw, double &qx, double &qy, double &qz)
{
	gsl_vector *dbar, *mbar;
	dbar = gsl_vector_alloc(3);
	mbar = gsl_vector_alloc(3);
	double N = 0;
	gsl_matrix *H = gsl_matrix_alloc(3,3);
	gsl_matrix_set_zero(H);
	gsl_vector_set_zero(dbar);
	gsl_vector_set_zero(mbar);

  if(markerPointsNow.size() <= 0) {
    return -1;
  }

	for(std::map<std::string, Vector3f>::iterator i = markerPointsNow.begin();
		i != markerPointsNow.end();
		++i) {
			dbar->data[0] += i->second.x;
			dbar->data[1] += i->second.y;
			dbar->data[2] += i->second.z;


			mbar->data[0] += markerPoints[i->first].x;
			mbar->data[1] += markerPoints[i->first].y;
			mbar->data[2] += markerPoints[i->first].z;

			N += 1.0;
	}

	gsl_vector_scale(dbar, 1/N);
	gsl_vector_scale(mbar, 1/N);

	gsl_matrix *dci, *mci;
	dci = gsl_matrix_alloc(1,3);
	mci = gsl_matrix_alloc(3,1);
	gsl_matrix *tmp = gsl_matrix_alloc(3,3);

	for(std::map<std::string, Vector3f>::iterator i = markerPointsNow.begin();
		i != markerPointsNow.end();
		++i) {

			gsl_matrix_set(dci, 0, 0, i->second.x - dbar->data[0]);
			gsl_matrix_set(dci, 0, 1, i->second.y - dbar->data[1]);
			gsl_matrix_set(dci, 0, 2, i->second.z - dbar->data[2]);

			gsl_matrix_set(mci, 0, 0, markerPoints[i->first].x - mbar->data[0]);
			gsl_matrix_set(mci, 1, 0, markerPoints[i->first].y - mbar->data[1]);
			gsl_matrix_set(mci, 2, 0, markerPoints[i->first].z - mbar->data[2]);

			gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
				1.0, mci, dci,
				0.0, tmp);

			gsl_matrix_add(H, tmp);
	}

	// Do SVD on H
	gsl_matrix *V, *U;
	gsl_vector *S;
	U = H;
	V = gsl_matrix_alloc(3,3);
	S = gsl_vector_alloc(3);
	gsl_vector *work = gsl_vector_alloc(3);
	gsl_linalg_SV_decomp(H, V, S, work);	

	gsl_matrix *R = gsl_matrix_alloc(3,3);

	gsl_blas_dgemm(CblasNoTrans, CblasTrans,
		1.0, V, U, 0.0, R);

	gsl_vector *T = dbar; // result will be put in dbar

	gsl_blas_dgemv(CblasNoTrans, -1.0, R, mbar, 1.0, dbar);

	/*
	std::cout << "R: " << std::endl;
	for(int i=0; i < 3; ++i) {
		for(int j=0; j < 3; ++j) {
			std::cout << gsl_matrix_get(R, i, j) << " ";
		}
		std::cout << "\n";
	}
	*/

	//std::cout << "T: " << T->data[0] << " " << T->data[1] << " " << T->data[2] << std::endl;

	x = T->data[0];
	y = T->data[1];
	z = T->data[2];

	double t = gsl_matrix_get(R, 0, 0) + gsl_matrix_get(R, 1, 1) + gsl_matrix_get(R, 2, 2);
	double r = sqrt(1+t);
	double s = 0.5/r;
	qw = 0.5*r;
	qx = (gsl_matrix_get(R, 2, 1)-gsl_matrix_get(R, 1, 2))*s;
	qy = (gsl_matrix_get(R, 0, 2)-gsl_matrix_get(R, 2, 0))*s;
	qz = (gsl_matrix_get(R, 1, 0)-gsl_matrix_get(R, 0, 1))*s;

	gsl_matrix_free(V);
	gsl_vector_free(S);
	gsl_matrix_free(R);
	gsl_matrix_free(dci);
	gsl_matrix_free(mci);
	gsl_matrix_free(H);
	gsl_vector_free(work);
	gsl_vector_free(dbar);
	gsl_vector_free(mbar);
  gsl_matrix_free(tmp);

  return 0;
}
