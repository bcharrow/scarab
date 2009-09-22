//-----------------------------------------------------------------------------
//
//	Vicon RealTime SDK Sample Code: ClientCodes.cpp
//
//	This sample code is provided as an example of a a client-server 
//	application to obtain raw motion data from the Vicon RealTime Engine
//	for use in third-party data visualization, analysis, or manipulation 
//      software. 
//	This source file initializes some of the structures in
//	sample file ClientCodes.h. Also see ExampleClient.cpp and 
//	ExampleClient.dsp.
//	
//	© 2000-2008 Vicon Motion Systems Limited. All rights reserved.
//-----------------------------------------------------------------------------

#include "ClientCodes.h"

const std::vector< std::string > ClientCodes::MarkerTokens = MakeMarkerTokens();
const std::vector< std::string > ClientCodes::BodyTokens = MakeBodyTokens();

std::ostream& operator<< (std::ostream& os, const BodyData &foo)
{
	os << foo.TX << ", " << foo.TY << ", " << foo.TZ << " :: " << foo.QX << ", " << foo.QY << ", " << foo.QZ << ", " << foo.QW;

	return os;
}