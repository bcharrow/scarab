#include <fstream>
#include <sstream>
#include <cstring>

using namespace std;

#include "CQuat.h"

#include <math.h>

#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif

#include <nmw/Config.h>

#include "ViconSDK.h"

//#define DEBUG


bool recieve(int sd, char * pBuffer, int BufferSize)
{
	char * p = pBuffer;
	char * e = pBuffer + BufferSize;

	int result;

	while(p != e)
	{
		result = recv(sd, p, e - p, 0 );

		if(result < 0)
			return false;
		
		p += result;
	}

	return true;
}

//	There are also some helpers to make the code a little less ugly.

bool recieve(int sd, long int & Val)
{
	return recieve(sd, (char*) & Val, sizeof(Val));
}

bool recieve(int sd, unsigned long int & Val)
{
	return recieve(sd, (char*) & Val, sizeof(Val));
}

bool recieve(int sd, double & Val)
{
	return recieve(sd, (char*) & Val, sizeof(Val));
}

int GetViconInfo(int sd, std::vector< std::string > &info)
{
	try
	{
		const int bufferSize = 2040;
		char buff[bufferSize];
		char * pBuff;

		//- SECTION 2: Request and receive the Info packet- - - - - - - - - - - - -
		//	Request the channel information

		pBuff = buff;

		* ((long int *) pBuff) = ClientCodes::EInfo;
		pBuff += sizeof(long int);
		* ((long int *) pBuff) = ClientCodes::ERequest;
		pBuff += sizeof(long int);

		if(send(sd, buff, pBuff - buff, 0) < 0)
			throw std::string("Error Requesting");

		long int packet;
		long int type;

		if(!recieve(sd, packet))
			throw std::string("Error Recieving");

		if(!recieve(sd, type))
			throw std::string("Error Recieving");

		if(type != ClientCodes::EReply)
			throw std::string("Bad Packet");

		if(packet != ClientCodes::EInfo)
			throw std::string("Bad Reply Type");

		long int size;

		if(!recieve(sd, size))
			throw std::string();

		info.resize(size);

		std::vector< std::string >::iterator iInfo;

		for(iInfo = info.begin(); iInfo != info.end(); iInfo++)
		{
			long int s;
			char c[255];
			char * p = c;

			if(!recieve(sd, s)) 
				throw std::string();

			if(!recieve(sd, c, s)) 
				throw std::string();

			p += s;

			*p = 0;

			*iInfo = std::string(c);
		}
	}
	catch(const std::string & rMsg)
	{
		if(rMsg.empty())
			std::cout << "Error! Error! Error! Error! Error!" << std::endl;
		else
			std::cout << rMsg.c_str() << std::endl;
	}
	return 0;
}

int ParseViconInfo(std::vector< std::string > &info,
				   std::vector< MarkerChannel > &MarkerChannels, std::vector< BodyChannel > &BodyChannels, int &FrameChannel, 
				   double &fps)
{
	//- SECTION 3: Parse the Info packet - - - - - - - - - - - - - - - - - - - 
	//	The info packets now contain the channel names.
	//	Identify the channels with the various Degress of Freedom (DOFs).

	std::vector< std::string >::iterator iInfo;
	for(iInfo = info.begin(); iInfo != info.end(); iInfo++)
	{
		//	Extract the channel type

		int openBrace = iInfo->find('<');

		if(openBrace == (int)iInfo->npos) 
			throw std::string("Bad Channel Id");

		int closeBrace = iInfo->find('>');

		if(closeBrace == (int)iInfo->npos) 
			throw std::string("Bad Channel Id");

		closeBrace++;

		std::string Type = iInfo->substr(openBrace, closeBrace-openBrace);

		//	Extract the channel name

		std::string Name = iInfo->substr(0, openBrace);

		int space = Name.rfind(' ');

		if(space != (int)Name.npos) 
			Name.resize(space);

		std::vector< MarkerChannel >::iterator iMarker;
		std::vector< BodyChannel >::iterator iBody;
		std::vector< std::string >::const_iterator iTypes;

		iMarker = std::find(	MarkerChannels.begin(), 
			MarkerChannels.end(), Name);

		iBody = std::find(BodyChannels.begin(), BodyChannels.end(), Name);

		if(iMarker != MarkerChannels.end())
		{
			//	The channel is for a marker we already have.
			iTypes = std::find(	ClientCodes::MarkerTokens.begin(), ClientCodes::MarkerTokens.end(), Type);
			if(iTypes != ClientCodes::MarkerTokens.end())
				iMarker->operator[](iTypes - ClientCodes::MarkerTokens.begin()) = iInfo - info.begin();
		}
		else
			if(iBody != BodyChannels.end())
			{
				//	The channel is for a body we already have.
				iTypes = std::find(ClientCodes::BodyTokens.begin(), ClientCodes::BodyTokens.end(), Type);
				if(iTypes != ClientCodes::BodyTokens.end())
					iBody->operator[](iTypes - ClientCodes::BodyTokens.begin()) = iInfo - info.begin();
			}
			else
				if((iTypes = std::find(ClientCodes::MarkerTokens.begin(), ClientCodes::MarkerTokens.end(), Type))
					!= ClientCodes::MarkerTokens.end())
				{
					//	The channel is for a new marker.
					MarkerChannels.push_back(MarkerChannel(Name));
					MarkerChannels.back()[iTypes - ClientCodes::MarkerTokens.begin()] = iInfo - info.begin();
				}
				else
					if((iTypes = std::find(ClientCodes::BodyTokens.begin(), ClientCodes::BodyTokens.end(), Type))
						!= ClientCodes::BodyTokens.end())
					{
						//	The channel is for a new body.
						BodyChannels.push_back(BodyChannel(Name));
						BodyChannels.back()[iTypes - ClientCodes::BodyTokens.begin()] = iInfo - info.begin();
						std::cout << "Found bodyChannel " << Name << std::endl;
					}
					else
						if(Type == "<F>")
						{
							FrameChannel = iInfo - info.begin();

							int a, b;

							a = iInfo->find(" ");
							b = iInfo->find(" ", a);

							fps = atof(iInfo->substr(a, b).c_str());
							
							std::cout << fps << " fps" << std::endl;
						}
						else
						{
							//	This could be a new channel type.
						}

	}

	return 0;
}


int RequestViconStream(int sd)
{
	const int bufferSize = 2040;
	char buff[bufferSize];
	char * pBuff;

	try {
		pBuff = buff;

		* ((long int *) pBuff) = ClientCodes::EStreamOn;
		pBuff += sizeof(long int);
		* ((long int *) pBuff) = ClientCodes::ERequest;
		pBuff += sizeof(long int);

		if(send(sd, buff, pBuff - buff, 0) < 0)
			throw std::string("Error Requesting StreamOn");
		return 0;
	}
	catch(const std::string & rMsg)
	{
		if(rMsg.empty())
			std::cout << "Error! Error! Error! Error! Error!" << std::endl;
		else
			std::cout << rMsg.c_str() << std::endl;
	}

	return 0;
}

int GetViconData(int sd, const std::vector< std::string > &info, std::vector<double> &data)
{

	data.resize(info.size());

	try {
		long int packet;
		long int type;

		//	Get and check the packet header.

		if(!recieve(sd, packet))
			throw std::string("Error Recieving");

		if(!recieve(sd, type))
			throw std::string("Error Recieving");

		if(type != ClientCodes::EReply)
			throw std::string("Bad Packet");

		if(packet != ClientCodes::EData)
			throw std::string("Bad Reply Type");

		long int size;
		if(!recieve(sd, size))
			throw std::string();

		if(size != (long int)info.size())
			throw std::string("Bad Data Packet");

		//	Get the data.

		std::vector< double >::iterator iData;

		for(iData = data.begin(); iData != data.end(); iData++)
		{	
			if(!recieve(sd, *iData)) 
				throw std::string();
		}

	}


	catch(const std::string & rMsg)
	{
    puts("!!!!!!\n");
		if(rMsg.empty())
			std::cout << "Error! Error! Error! Error! Error!" << std::endl;
		else
			std::cout << rMsg.c_str() << std::endl;
	}

	return 0;
}

int RequestGetViconData(int sd, const std::vector< std::string > &info, std::vector<double> &data)
{
	//- SECTION 4: Request and receive the Data packet - - - - - - - - - - - - - 	        	
	//  Get the data using the request/reply protocol.

	const int bufferSize = 2040;
	data.resize(info.size());
	char buff[bufferSize];
	char * pBuff;

	try {
		pBuff = buff;

		* ((long int *) pBuff) = ClientCodes::EData;
		pBuff += sizeof(long int);
		* ((long int *) pBuff) = ClientCodes::ERequest;
		pBuff += sizeof(long int);

		if(send(sd, buff, pBuff - buff, 0) < 0)
			throw std::string("Error Requesting");

		long int packet;
		long int type;

		//	Get and check the packet header.

		if(!recieve(sd, packet))
			throw std::string("Error Recieving");

		if(!recieve(sd, type))
			throw std::string("Error Recieving");

		if(type != ClientCodes::EReply)
			throw std::string("Bad Packet");

		if(packet != ClientCodes::EData)
			throw std::string("Bad Reply Type");

		long int size;
		if(!recieve(sd, size))
			throw std::string();

		if(size != (long int)info.size())
			throw std::string("Bad Data Packet");

		//	Get the data.

		std::vector< double >::iterator iData;

		for(iData = data.begin(); iData != data.end(); iData++)
		{	
			if(!recieve(sd, *iData)) 
				throw std::string();
		}
	}

	catch(const std::string & rMsg)
	{
		if(rMsg.empty())
			std::cout << "Error! Error! Error! Error! Error!" << std::endl;
		else
			std::cout << rMsg.c_str() << std::endl;
	}

	return 0;
}


#ifdef DEBUG
double *avg_euler_x;
double *avg_euler_y;
double *avg_euler_z;
int *n;
double *max_deviation;
std::ofstream *fout;
#endif

double tmp_x, tmp_y, tmp_z;

int ProcessViconData(std::vector<double> &data,
					 std::vector< MarkerChannel > &MarkerChannels, std::vector< BodyChannel > &BodyChannels, int &FrameChannel,
					 std::vector< MarkerData > &markerPositions, std::vector< BodyData > &bodyPositions, double &timestamp,
					 std::map<std::string, StateEstimate > &state_estimates,
					 std::map<std::string, ViconSubject> &viconSubjects)
{

	// Clear viconSubject's markerData
	for(std::map<std::string, ViconSubject>::iterator i = viconSubjects.begin();
		i != viconSubjects.end();
		++i) {
			i->second.ClearMarkerPointsNow();
	}

	timestamp = data[FrameChannel];
	markerPositions.resize(MarkerChannels.size());
	bodyPositions.resize(BodyChannels.size());

	//	Get the channels corresponding to the markers.
	//	Y is up
	//	The values are in millimeters

	std::vector< MarkerChannel >::iterator iMarker;
	std::vector< MarkerData >::iterator iMarkerData;

	for(	iMarker = MarkerChannels.begin(), 
		iMarkerData = markerPositions.begin(); 
		iMarker != MarkerChannels.end(); iMarker++, iMarkerData++)
	{

		// Get SubjectName from iMarker->Name
		int name_delim = iMarker->Name.find(":");
		std::string subjectName = iMarker->Name.substr(0, name_delim);

		iMarkerData->X = data[iMarker->X];
		iMarkerData->Y = data[iMarker->Y];
		iMarkerData->Z = data[iMarker->Z];
		if(data[iMarker->O] > 0.5)
			iMarkerData->Visible = false;
		else {
			iMarkerData->Visible = true;
      if(viconSubjects.count(subjectName) > 0) {
        viconSubjects[subjectName].SetCurrentMarkerPoint(iMarker->Name, iMarkerData->X, iMarkerData->Y, iMarkerData->Z);
      }
      else {
        printf("PROBLEM: subject %s does not exist\n", subjectName.c_str());
      }
		}

	}

	for(std::map<std::string, ViconSubject>::iterator i = viconSubjects.begin();
		i != viconSubjects.end();
		++i) {
			//i->second.PrintMarkerPointsNow();
    if(i->second.GetNumMarkerPointsNow() > 0) {
			double x, y, z, qw, qx, qy, qz;
			i->second.ComputeTransformation(x, y, z, qw, qx, qy, qz);
      State tmp_state(x, y, z, qw, qx, qy, qz);

      if(state_estimates.count(i->first)) {
        (state_estimates[i->first]).AddStateMeasurement(tmp_state, 
                                                        i->second.GetQuality());
      }
      else {
        printf("PROBLEM: state_estimate %s does not exist\n", i->first.c_str());
      }
    }
	}

	return 0;
}

int ReadXmlConfig(std::string filename, 
                  std::map<std::string, int> &fiducial_mapping,
                  std::map<std::string, StateEstimate> &state_estimates,
                  std::map<std::string, ViconSubject> &viconSubjects)
{
  nmw::Config c = new nmw::Config(filename.c_str());
  std::string vskpath = filename.substr(0, filename.rfind("/")) + std::string("/vsk/");
  return ReadXmlConfig(c, fiducial_mapping, state_estimates, viconSubjects, vskpath);
}

int ReadXmlConfig(nmw::Config &c,
                  std::map<std::string, int> &fiducial_mapping,
                  std::map<std::string, StateEstimate> &state_estimates,
                  std::map<std::string, ViconSubject> &viconSubjects, std::string &vskpath)
{

  char name_buf[255];
  string name;
  int fid;

  if(c.Load() != 0)
    return -1;

  int i = 0;
  nmw::Config *subject = c.GetChildrenAsRoot("subjects", i++);
  while(subject != NULL) {
    //subject->Print();

    /*
      <subject>
      <id>30</id>
      <vsk>RDK1.vsk</vsk>
      <xyz>0 0 0</xyz>
      <quat>0 0 0 1.0</quat>
      <group>224.0.0.10</group>
      <port>10030</port>
      </subject>
    */

    subject->GetString(name_buf, "vsk");
    fid = subject->GetInt("id", 0);

    std::string path = vskpath + name_buf;
    ViconSubject foo(path.c_str());
    viconSubjects.insert(make_pair(foo.GetName(), foo));
    name = foo.GetName();

    fiducial_mapping[name] = fid;
    std::cout << "Loaded body [" << name << "] :: " << fid << std::endl;
    
    // Setup smoothing parameters
    state_estimates[name].SetParam(1, 0.0, 1);

    // Setup body frame transformation
    double x, y, z, q0, q1, q2, qw;
    x = subject->GetTupleFloat("xyz", 0, 0.0);
    y = subject->GetTupleFloat("xyz", 1, 0.0);
    z = subject->GetTupleFloat("xyz", 2, 0.0);
    
    q0 = subject->GetTupleFloat("quat", 0, 0.0);
    q1 = subject->GetTupleFloat("quat", 1, 0.0);
    q2 = subject->GetTupleFloat("quat", 2, 0.0);
    qw = subject->GetTupleFloat("quat", 3, 1.0);
    
    state_estimates[name].SetOffset(x, y, z, CQuat(q0, q1, q2, qw));

    subject = c.GetChildrenAsRoot("subjects", i++);
  }

  return 0;
}


int ReadConfig(std::string filename, 
               std::map<std::string, int> &fiducial_mapping,
               std::map<std::string, StateEstimate> &state_estimates,
               std::map<std::string, ViconSubject> &viconSubjects)
{
	char line_data_tmp[255];
	std::string line_data;
	std::string name;
	int fid;

	int name_fid_delim;
	int fid_offset_delim;
  std::ifstream fin;
  fin.open(filename.c_str());

  if(!fin) {
    cerr << "Error opening " << filename << "\n";
    return -1;
  }

	while(!fin.eof()) {
		fin.getline(line_data_tmp, 255);
		line_data = std::string(line_data_tmp);

		name_fid_delim = line_data.find("::");
		fid_offset_delim = line_data.find("::", name_fid_delim+1);

		name = line_data.substr(0, name_fid_delim-1);
		fid = atoi(line_data.substr(name_fid_delim+3, fid_offset_delim).c_str());

		ViconSubject foo(name.c_str());
		viconSubjects.insert(make_pair(foo.GetName(), foo));
		name = foo.GetName();

		fiducial_mapping[name] = fid;
		std::cout << "Loaded body [" << name << "] :: " << fid << std::endl;
    
    // Setup smoothing parameters
		state_estimates[name].SetParam(1, 0.0, 1);
		if(fid_offset_delim > 0) {
			fid_offset_delim += 2;
			double x, y, z, q0, q1, q2, qw;
			istringstream offset_stream(line_data.substr(fid_offset_delim));
			std::cout << line_data.substr(fid_offset_delim) << std::endl;
			offset_stream >> x >> y >> z >> q0 >> q1 >> q2 >> qw;

			state_estimates[name].SetOffset(x, y, z, CQuat(q0, q1, q2, qw));
		}
	}	

	fin.close();

	return 0;
}

/*
int PublishData(UDPSocket &sock, std::string group, int port,
				double &timestamp, 
				std::map<std::string, int> &fiducial_mapping,
				std::map<std::string, StateEstimate> &state_estimates,
				double &fps)
{
	char buffer[1024];
	char *ptr = buffer;
	char *count_ptr;
	int buffer_count = 0;

	double time = timestamp / fps;

	std::map<std::string, StateEstimate>::iterator iState;

	// Put current time in outgoing data
	memcpy(ptr, &time, sizeof(time));
	ptr += sizeof(time);

	// Fill in count at end
	count_ptr = ptr;
	ptr += sizeof(uint32_t);
	uint32_t count = 0;

	int32_t id;
	double x, y, z, qw, qx, qy, qz, quality;

	for(iState = state_estimates.begin();
		iState != state_estimates.end();
		++iState) {

    if(fiducial_mapping[iState->first] <= 0)
    continue;
    
    if(!iState->second.tracking)
    continue;
    
    count ++;
		//std::cout.precision(10);
		//std::cout << time << " " << iState->first << " (" << fiducial_mapping[iState->first] << ") " << iState->second.estimate.marker_quality << " :: " << iState->second.robotBodyEstimate << std::endl;

		id = fiducial_mapping[iState->first];
		x = iState->second.robotBodyEstimate.TX/1000.0;
		y = iState->second.robotBodyEstimate.TY/1000.0;
		z = iState->second.robotBodyEstimate.TZ/1000.0;
		qw = iState->second.robotBodyEstimate.Q.w;
		qx = iState->second.robotBodyEstimate.Q.x;
		qy = iState->second.robotBodyEstimate.Q.y;
		qz = iState->second.robotBodyEstimate.Q.z;
		quality = iState->second.estimate.marker_quality;

		memcpy(ptr, &(id), sizeof(id));
		ptr += sizeof(id);

		memcpy(ptr, &x, sizeof(x));
		ptr += sizeof(x);
		memcpy(ptr, &y, sizeof(y));
		ptr += sizeof(y);
		memcpy(ptr, &z, sizeof(z));
		ptr += sizeof(z);
		memcpy(ptr, &qw, sizeof(qw));
		ptr += sizeof(qw);
		memcpy(ptr, &qx, sizeof(qx));
		ptr += sizeof(qx);
		memcpy(ptr, &qy, sizeof(qy));
		ptr += sizeof(qy);
		memcpy(ptr, &qz, sizeof(qz));
		ptr += sizeof(qz);

		memcpy(ptr, &quality, sizeof(quality));
		ptr += sizeof(quality);
	}
	memcpy(count_ptr, &count, sizeof(count));

	buffer_count = ptr - buffer;

	sock.sendTo(buffer, buffer_count, group, port);

	return 0;
}
*/

int CreateTCPSocket(char *hostname)
{
	struct hostent *ptrh;
  struct protoent *ptrp;
	struct sockaddr_in	sad;
	static const int port = 800;

  memset((char*)&sad, 0, sizeof(sad));
  sad.sin_family = AF_INET;

  ptrh = gethostbyname(hostname);
  if(((char*)ptrh) == NULL) {
    printf("Invalid host: %s, using localhost\n", hostname);
    ptrh = gethostbyname("localhost");
  }
  memcpy(&sad.sin_addr, ptrh->h_addr, ptrh->h_length);

  sad.sin_port = htons((u_short)port);
  ptrp = 0;

  printf("Connecting TCP client (%s:%i)\n", hostname, port);

  if ((ptrp = getprotobyname("tcp")) == 0) 
    {
      puts("Cannot map \"tcp\" to protocol number");
      return -1;
    }
   
  int sd = socket(AF_INET, SOCK_STREAM, ptrp->p_proto);

  if (sd < 0) 
    {
      puts("Socket creation failed");
      return -1;
    }
   
  if (connect(sd, (struct sockaddr *)&sad, sizeof(sad)) < 0) 
    {
      puts("Connect failed");
      return -1;
    }

  return sd;
}

