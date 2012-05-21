#ifndef __IPC_BRIDGE_MATLAB_ODOMETRY_AGGREGATOR_ODOMETRY_ARRAY__
#define __IPC_BRIDGE_MATLAB_ODOMETRY_AGGREGATOR_ODOMETRY_ARRAY__
#include <ipc_bridge_matlab/ipc_bridge_matlab.h>
#include <ipc_bridge/msgs/odometry_aggregator_OdometryArray.h>

#include <nav_msgs_Odometry.h>
#include <roslib_Header.h>

namespace ipc_bridge_matlab
{
  namespace odometry_aggregator
  {
    namespace OdometryArray
    {
      static mxArray* ProcessMessage(const ipc_bridge::odometry_aggregator::OdometryArray &msg)
      {
        const char *fields[] = {"header", "array"};
        const int nfields = sizeof(fields)/sizeof(*fields);
        mxArray *out = mxCreateStructMatrix(1, 1, nfields, fields);
        
        mxSetField(out, 0, "header", 
                   ipc_bridge_matlab::Header::ProcessMessage(msg.header));

        const int length = msg.array_length;      
        mxArray *array = mxCreateCellArray(1, &length);
        for (unsigned int i = 0; i < length; i++)
          mxSetCell(array, i, 
                    ipc_bridge_matlab::nav_msgs::Odometry::ProcessMessage(msg.array[i]));
        mxSetField(out, 0, "array", array);
                
        return out;
      }
      
      static int ProcessArray(const mxArray *a, 
                              ipc_bridge::odometry_aggregator::OdometryArray &msg)
      {
        mxArray *field;

        field = mxGetField(a, 0, "header");
        ipc_bridge_matlab::Header::ProcessArray(field, msg.header);

        field = mxGetField(a, 0, "array");
        int nrows = mxGetM(field);
        int ncols = mxGetN(field);

        unsigned int length = nrows;
        if (nrows < ncols)
          length = ncols;
        msg.array_length = length;

        if ((ncols == 0) || (nrows == 0))
          {
            msg.array_length = 0;
            msg.array = 0;
          }

        if (msg.array_length > 0)
          {
            msg.array = new nav_msgs_Odometry[msg.array_length];
            for (unsigned int i = 0; i < msg.array_length; i++)
              {
                mxArray *p = mxGetCell(field, i);
                ipc_bridge_matlab::nav_msgs::Odometry::ProcessArray(p, msg.array[i]);
              }
          }
        
        return SUCCESS;
      }

      static void Cleanup(ipc_bridge::odometry_aggregator::OdometryArray &msg)
      {
        ipc_bridge_matlab::Header::Cleanup(msg.header);
        for (unsigned int i = 0; i < msg.array_length; i++)
          ipc_bridge_matlab::nav_msgs::Odometry::Cleanup(msg.array[i]);
        if (msg.array != 0)
          {
            delete[] msg.array;
            msg.array_length = 0;
            msg.array = 0;
          }
      }
    }
  }
}
#endif
