#include <iostream>
#include <cassert>
#include <string>
#include <vector>
#include <algorithm>	
#include <functional>
#include <map>
#include <limits>

#include <math.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <nmw/Config.h>

#include "ViconSubject.h"
#include "StateEstimation.h"
#include "ClientCodes.h"

int GetViconInfo(int sd, std::vector< std::string > &info);
int ParseViconInfo(std::vector< std::string > &info,
                   std::vector< MarkerChannel > &MarkerChannels, 
                   std::vector< BodyChannel > &BodyChannels, 
                   int &FrameChannel,
                   double &fps);

int RequestGetViconData(int sd, 
                        const std::vector< std::string > &info, 
                        std::vector<double> &data);

int RequestViconStream(int sd);
int GetViconData(int sd, 
                 const std::vector< std::string > &info, 
                 std::vector<double> &data);

int ProcessViconData(std::vector<double> &data,
                     std::vector< MarkerChannel > &MarkerChannels, 
                     std::vector< BodyChannel > &BodyChannels, 
                     int &FrameChannel,
                     std::vector< MarkerData > &markerPositions, 
                     std::vector< BodyData > &bodyPositions, 
                     double &timestamp,
                     std::map<std::string, StateEstimate> &state_estimates,
                     std::map<std::string, ViconSubject> &viconSubjects);

int ReadXmlConfig(nmw::Config &c,
                  std::map<std::string, int> &fiducial_mapping,
                  std::map<std::string, StateEstimate> &state_estimates,
                  std::map<std::string, ViconSubject> &viconSubjects,
                  std::string &vskpath);

int ReadXmlConfig(std::string filename, 
                  std::map<std::string, int> &fiducial_mapping,
                  std::map<std::string, StateEstimate> &state_estimates,
                  std::map<std::string, ViconSubject> &viconSubjects);

int ReadConfig(std::string filename, std::map<std::string, int> &fiducial_mapping,
               std::map<std::string, StateEstimate> &state_estimates,
               std::map<std::string, ViconSubject> &viconSubjects);

/*
int PublishData(UDPSocket &sock, std::string group, int port,
				double &timestamp, 
				std::map<std::string, int> &fiducial_mapping,
				std::map<std::string, StateEstimate> &state_estimates,
				double &fps);
*/

int CreateTCPSocket(char *hostname);

