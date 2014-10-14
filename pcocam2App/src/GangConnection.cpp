/* GangConnection.cpp
 *
 * Revamped PCO area detector driver.
 * The communication connection class for a ganged client
 *
 * Author:  Giles Knap
 *          Jonathan Thompson
 *
 */

#include "GangConnection.h"
#include "GangMemberConfig.h"
#include "GangServer.h"
#include "TraceStream.h"
#include "Pco.h"
#include "epicsExport.h"
#include "iocsh.h"
#include "NDArray.h"

// Area detector parameter names
const char* GangConnection::nameConnected = "PCO_GANGCONN_CONNECTED";
const char* GangConnection::nameServerIp = "PCO_GANGCONN_SERVERIP";
const char* GangConnection::nameServerPort = "PCO_GANGCONN_SERVERPORT";
const char* GangConnection::namePositionX = "PCO_GANGCONN_POSITIONX";
const char* GangConnection::namePositionY = "PCO_GANGCONN_POSITIONY";
const char* GangConnection::nameFunction = "PCO_GANGCONN_FUNCTION";

// Constructor
// When used at the client end, the server is NULL
GangConnection::GangConnection(Pco* pco, TraceStream* trace, const char* serverIp,
		int serverPort)
	: SocketProtocol("GangConnection", "pco_gang")
	, pco(pco)
	, trace(trace)
{
	// Create the parameters
	pco->createParam(nameConnected, asynParamInt32, &handleConnected);
	pco->createParam(nameServerIp, asynParamOctet, &handleServerIp);
	pco->createParam(nameServerPort, asynParamInt32, &handleServerPort);
	pco->createParam(namePositionX, asynParamInt32, &handlePositionX);
	pco->createParam(namePositionY, asynParamInt32, &handlePositionY);
	pco->createParam(nameFunction, asynParamInt32, &handleFunction);
	// Initialise the parameters
	pco->setIntegerParam(handleConnected, 0);
	pco->setStringParam(handleServerIp, serverIp);
	pco->setIntegerParam(handleServerPort, serverPort);
	pco->setIntegerParam(handlePositionX, 0);
	pco->setIntegerParam(handlePositionY, 0);
	pco->setIntegerParam(handleFunction, GangServer::gangFunctionOff);
	// Start the connection
	pco->registerGangConnection(this);
	client(serverIp, serverPort);
	*trace << "Gang client attempting connection" << std::endl;
}

// Destructor
GangConnection::~GangConnection()
{
}

// Parameter write handler
void GangConnection::writeInt32(int parameter, int value)
{
    if(parameter == handlePositionX || parameter == handlePositionY ||
    		parameter == pco->ADSizeX || parameter == pco->ADSizeY)
    {
    	sendMemberConfig();
    }
}

// Send this member's configuration to the server
void GangConnection::sendMemberConfig()
{
	GangMemberConfig config;
	config.fromPco(pco, this);
	transmit('m', 0, config.data(), sizeof(GangMemberConfig));
}

// Send an image to the server
void GangConnection::sendImage(NDArray* image, int sequence)
{
	pco->lock();
	int gangFunction;
	pco->getIntegerParam(handleFunction, &gangFunction);
	pco->unlock();
	if(gangFunction == GangServer::gangFunctionFull)
	{
		NDArrayInfo arrayInfo;
		image->getInfo(&arrayInfo);
		transmit('i', sequence, image->pData, arrayInfo.totalBytes);
	}
}

// A message has been received from the peer.
void GangConnection::receive(char tag, int parameter, void* data, size_t dataSize)
{
	switch(tag)
	{
	case 'a':
		*trace << "Gang client received arm" << std::endl;
		config.toPco(pco);
        pco->post(Pco::requestArm);
        break;
	case 'd':
		*trace << "Gang client received disarm" << std::endl;
        pco->post(Pco::requestDisarm);
        break;
	case 's':
		*trace << "Gang client received start" << std::endl;
		config.toPco(pco);
        pco->post(Pco::requestAcquire);
		break;
	case 'x':
		*trace << "Gang client received stop" << std::endl;
        pco->post(Pco::requestStop);
		break;
	case 'c':
		*trace << "Gang client received server config" << std::endl;
		serverConfig.toPco(pco, this);
		break;
	}
}

// The connection has been made
void GangConnection::connected()
{
	*trace << "Gang client connected" << std::endl;
	pco->lock();
	pco->setIntegerParam(handleConnected, 1);
	pco->callParamCallbacks();
	pco->unlock();
	sendMemberConfig();
}

// This connection has broken
void GangConnection::disconnected()
{
	*trace << "Gang client disconnected" << std::endl;
	pco->lock();
	pco->setIntegerParam(handleConnected, 0);
	pco->callParamCallbacks();
	pco->unlock();
}

// Get a buffer for the reception of a message data buffer
void* GangConnection::getDataBuffer(char tag, int parameter, size_t dataSize)
{
	void* result = NULL;
	switch(tag)
	{
	case 'a':
	case 's':
		if(dataSize == sizeof(GangConfig))
		{
			result = config.data();
		}
		break;
	case 'c':
		if(dataSize == sizeof(GangServerConfig))
		{
			result = serverConfig.data();
		}
		break;
	}
	return result;
}

// C entry point for iocinit
extern "C" int gangConnectionConfig(const char* portName, const char* gangServerIp,
		int gangPortNumber)
{
    Pco* pco = Pco::getPco(portName);
    if(pco != NULL)
    {
        new GangConnection(pco, &pco->gangTrace, gangServerIp, gangPortNumber);
    }
    else
    {
        printf("gangConnectionConfig: Pco \"%s\" not found\n", portName);
    }
    return asynSuccess;
}
static const iocshArg gangConnectionConfigArg0 = {"PCO Port Name", iocshArgString};
static const iocshArg gangConnectionConfigArg1 = {"Gang Server IP", iocshArgString};
static const iocshArg gangConnectionConfigArg2 = {"Gang Port Number", iocshArgInt};
static const iocshArg* const gangConnectionConfigArgs[] =
    {&gangConnectionConfigArg0, &gangConnectionConfigArg1, &gangConnectionConfigArg2};
static const iocshFuncDef configGangConnection =
    {"gangConnectionConfig", 3, gangConnectionConfigArgs};
static void configGangConnectionCallFunc(const iocshArgBuf *args)
{
    gangConnectionConfig(args[0].sval, args[1].sval, args[2].ival);
}

/** Register the functions */
static void gangConnectionRegister(void)
{
    iocshRegister(&configGangConnection, configGangConnectionCallFunc);
}

extern "C" { epicsExportRegistrar(gangConnectionRegister); }
