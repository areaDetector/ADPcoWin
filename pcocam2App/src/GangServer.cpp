/* GangServer.cpp
 *
 * Revamped PCO area detector driver.
 * The communication server class for the master of a ganged pair
 *
 * Author:  Giles Knap
 *          Jonathan Thompson
 *
 */

#include "GangServer.h"
#include "GangClient.h"
#include "TraceStream.h"
#include "Pco.h"
#include "GangConnection.h"
#include "epicsExport.h"
#include "iocsh.h"
#include <cstring>

// Constants
const int GangServer::maxConnections = 3;

// Area detector parameter names
const char* GangServer::nameNumConnections = "PCO_GANGSERV_CONNECTIONS";
const char* GangServer::nameServerPort = "PCO_GANGSERV_PORT";
const char* GangServer::nameFunction = "PCO_GANGSERV_FUNCTION";
const char* GangServer::namePositionX = "PCO_GANGSERV_POSITIONX";
const char* GangServer::namePositionY = "PCO_GANGSERV_POSITIONY";
const char* GangServer::nameFullSizeX = "PCO_GANGSERV_FULLSIZEX";
const char* GangServer::nameFullSizeY = "PCO_GANGSERV_FULLSIZEY";
const char* GangServer::nameQueueSize = "PCO_GANGSERV_QUEUESIZE";
const char* GangServer::nameMissingPieces = "PCO_GANGSERV_MISSINGPIECES";

// Constructor.
GangServer::GangServer(Pco* pco, TraceStream* trace, int gangPortNumber)
	: SocketProtocol("GangServer", "")
	, pco(pco)
	, trace(trace)
{
	// Create the parameters
    pco->createParam(nameNumConnections, asynParamInt32, &handleNumConnections);
    pco->createParam(nameServerPort, asynParamInt32, &handleServerPort);
    pco->createParam(nameFunction, asynParamInt32, &handleFunction);
    pco->createParam(namePositionX, asynParamInt32, &handlePositionX);
    pco->createParam(namePositionY, asynParamInt32, &handlePositionY);
    pco->createParam(nameFullSizeX, asynParamInt32, &handleFullSizeX);
    pco->createParam(nameFullSizeY, asynParamInt32, &handleFullSizeY);
    pco->createParam(nameQueueSize, asynParamInt32, &handleQueueSize);
    pco->createParam(nameMissingPieces, asynParamInt32, &handleMissingPieces);
    // Initialise the parameters
	pco->setIntegerParam(handleNumConnections, 0);
	pco->setIntegerParam(handleServerPort, gangPortNumber);
	pco->setIntegerParam(handleFunction, gangFunctionOff);
	pco->setIntegerParam(handlePositionX, 0);
	pco->setIntegerParam(handlePositionY, 0);
	pco->setIntegerParam(handleFullSizeX, 0);
	pco->setIntegerParam(handleFullSizeY, 0);
	pco->setIntegerParam(handleQueueSize, 0);
	pco->setIntegerParam(handleMissingPieces, 0);
	// Create the client connections
	pco->registerGangServer(this);
	for(int i=0; i<maxConnections; i++)
	{
		clients.push_back(new GangClient(pco, trace, this, i));
	}
	listen(gangPortNumber);
	*trace << "Gang server listening" << std::endl;
}

// Destructor
GangServer::~GangServer()
{
	clearImageQueue();
	std::vector<GangClient*>::iterator pos;
	for(pos=clients.begin(); pos!=clients.end(); ++pos)
	{
		delete *pos;
	}
	clients.clear();
}

// Clear out stashed NDArrays
void GangServer::clearImageQueue()
{
	std::list<std::pair<int, NDArray*> >::iterator pos;
	for(pos=imageQueue.begin(); pos!=imageQueue.end(); ++pos)
	{
		pos->second->release();
		printf("####1 n=%d\n", pco->pNDArrayPool->numBuffers());
	}
	imageQueue.clear();
	pco->lock();
	pco->setIntegerParam(handleMissingPieces, 0);
	pco->unlock();
}

// Parameter write handler
void GangServer::writeInt32(int parameter, int value)
{
	if(parameter == handleFunction)
	{
		configure();
	}
}

// Accept a gang client connection
void GangServer::accepted(int fd)
{
	GangClient* client = getFreeClient();
	if(client)
	{
		*trace << "Gang member connection accepted" << std::endl;
		client->createConnection(fd);
		pco->lock();
		pco->setIntegerParam(handleNumConnections, countConnections());
		pco->callParamCallbacks();
		pco->unlock();
		configure();
	}
	else
	{
		*trace << "Gang member connection rejected" << std::endl;
	}
}

// A client has become disconnected
void GangServer::disconnected(GangClient* client)
{
	*trace << "Gang member disconnected" << std::endl;
	pco->lock();
	pco->setIntegerParam(handleNumConnections, countConnections());
	pco->callParamCallbacks();
	pco->unlock();
}

// Return the first client that is not connected.
GangClient* GangServer::getFreeClient()
{
	GangClient* result = NULL;
	std::vector<GangClient*>::iterator pos;
	for(pos=clients.begin(); pos!=clients.end() && result==NULL; ++pos)
	{
		if(!(*pos)->isConnected())
		{
			result = *pos;
		}
	}
	return result;
}

// Return the number of active connections.
int GangServer::countConnections()
{
	int num=0;
	std::vector<GangClient*>::iterator pos;
	for(pos=clients.begin(); pos!=clients.end(); ++pos)
	{
		if((*pos)->isConnected())
		{
			num++;
		}
	}
	return num;
}

// Send the server configuration to the clients
void GangServer::configure()
{
	GangServerConfig config;
	config.fromPco(pco, this);
	std::vector<GangClient*>::iterator pos;
	for(pos=clients.begin(); pos!=clients.end(); ++pos)
	{
		if((*pos)->isConnected())
		{
			(*pos)->configure(&config);
		}
	}
}

// Return true if the server is in control of the clients
bool GangServer::inControl()
{
	int gangFunction;
	pco->getIntegerParam(handleFunction, &gangFunction);
	return gangFunction != gangFunctionOff;
}

// Arm all the clients
void GangServer::arm()
{
	if(inControl())
	{
		clearImageQueue();
		determineImageSize();
		GangConfig config;
		config.fromPco(pco);
		std::vector<GangClient*>::iterator pos;
		for(pos=clients.begin(); pos!=clients.end(); ++pos)
		{
			if((*pos)->isToBeUsed())
			{
				(*pos)->arm(&config);
			}
		}
	}
}

// Disarm all the clients
void GangServer::disarm()
{
	if(inControl())
	{
		std::vector<GangClient*>::iterator pos;
		for(pos=clients.begin(); pos!=clients.end(); ++pos)
		{
			if((*pos)->isToBeUsed())
			{
				(*pos)->disarm();
			}
		}
	}
}

// Start all the clients acquiring
void GangServer::start()
{
	if(inControl())
	{
		clearImageQueue();
		determineImageSize();
		GangConfig config;
		config.fromPco(pco);
		std::vector<GangClient*>::iterator pos;
		for(pos=clients.begin(); pos!=clients.end(); ++pos)
		{
			if((*pos)->isToBeUsed())
			{
				(*pos)->start(&config);
			}
		}
	}
}

// Stop all the clients acquiring
void GangServer::stop()
{
	if(inControl())
	{
		std::vector<GangClient*>::iterator pos;
		for(pos=clients.begin(); pos!=clients.end(); ++pos)
		{
			if((*pos)->isToBeUsed())
			{
				(*pos)->stop();
			}
		}
	}
}

// Work out the size of the assembled image.
void GangServer::determineImageSize()
{
	int fullSizeX = 0;
	int fullSizeY = 0;
	if(inControl())
	{
		// Where's my bit go?
		int positionX;
		int positionY;
		int sizeX;
		int sizeY;
		pco->lock();
		pco->getIntegerParam(handlePositionX, &positionX);
		pco->getIntegerParam(handlePositionY, &positionY);
		pco->getIntegerParam(pco->ADSizeX, &sizeX);
		pco->getIntegerParam(pco->ADSizeY, &sizeY);
		pco->unlock();
		fullSizeX = positionX + sizeX;
		fullSizeY = positionY + sizeY;
		// Now account for all the client's bits
		std::vector<GangClient*>::iterator pos;
		for(pos=clients.begin(); pos!=clients.end(); ++pos)
		{
			if((*pos)->isToBeUsed())
			{
				(*pos)->determineImageSize(fullSizeX, fullSizeY);
			}
		}
	}
	// Report the results
	pco->lock();
	pco->setIntegerParam(handleFullSizeX, fullSizeX);
	pco->setIntegerParam(handleFullSizeY, fullSizeY);
	pco->callParamCallbacks();
	pco->unlock();
}

// An image has been received on the server.
// Returns true if the image has been consumed.
// This is called in the state machine thread so
// we are allowed to check for completed frames here.
bool GangServer::imageReceived(int sequence, NDArray* image)
{
	bool result = false;
	pco->lock();
	int gangFunction;
	pco->getIntegerParam(handleFunction, &gangFunction);
	pco->unlock();
	if(gangFunction == gangFunctionFull)
	{
		result = true;
		// Place the image in the queue
		imageQueue.push_back(std::pair<int,NDArray*>(sequence, image));
		// Forward any complete images
		makeCompleteImages();
		// Update counters
		pco->lock();
		pco->setIntegerParam(handleQueueSize, (int)imageQueue.size());
		pco->callParamCallbacks();
		pco->unlock();
	}
	return result;
}

// Make any complete images and pass them on
void GangServer::makeCompleteImages()
{
	// Keep processing until we fail to do one
	bool doneOne=true;
	while(!imageQueue.empty() && doneOne)
	{
		doneOne = false;
		int sequence = imageQueue.front().first;
		// Do all the clients have this sequence?
		bool allPresent = true;
		bool missingPiece = false;
		std::vector<GangClient*>::iterator clientPos;
		for(clientPos=clients.begin(); clientPos!=clients.end(); ++clientPos)
		{
			if((*clientPos)->isToBeUsed())
			{
				GangClient::SeqState has = (*clientPos)->hasSequence(sequence);
				allPresent = allPresent && has == GangClient::seqStateYes;
				missingPiece = missingPiece || has == GangClient::seqStateMissing;
			}
		}
		if(missingPiece)
		{
			// Part of this frame has gone missing, count and discard
			imageQueue.front().second->release();
			imageQueue.pop_front();
		}
		else if(allPresent)
		{
			// Alloc an array
			pco->lock();
			int fullSizeX;
			int fullSizeY;
			int dataType;
			int positionX;
			int positionY;
			pco->getIntegerParam(handleFullSizeX, &fullSizeX);
			pco->getIntegerParam(handleFullSizeY, &fullSizeY);
			pco->getIntegerParam(pco->NDDataType, &dataType);
			pco->getIntegerParam(handlePositionX, &positionX);
			pco->getIntegerParam(handlePositionY, &positionY);
			pco->unlock();
			NDArray* outImage = pco->allocArray(fullSizeX, fullSizeY, (NDDataType_t)dataType);
			printf("####8 n=%d\n", pco->pNDArrayPool->numBuffers());
			if(outImage)
			{
				NDArray* inImage = imageQueue.front().second;
				// Copy metadata
				outImage->uniqueId = inImage->uniqueId;
				outImage->timeStamp = inImage->timeStamp;
				outImage->pAttributeList->clear();
				inImage->pAttributeList->copy(outImage->pAttributeList);
				// Copy my piece into it and free
				insertImagePiece(outImage, inImage, positionX, positionY);
				inImage->release();
				printf("####2 n=%d\n", pco->pNDArrayPool->numBuffers());
				imageQueue.pop_front();
				// Copy the client pieces into it
				for(clientPos=clients.begin(); clientPos!=clients.end(); ++clientPos)
				{
					if((*clientPos)->isToBeUsed())
					{
						(*clientPos)->useImage(sequence, outImage);
					}
				}
				// Update counters
				pco->lock();
				pco->setIntegerParam(handleQueueSize, (int)imageQueue.size());
				pco->callParamCallbacks();
				pco->unlock();
				// Pass it on
				pco->imageComplete(outImage);
				// See if there is another one
				doneOne = true;
			}
		}
	}
}

// Copy the in image into the specified position in the out image
void GangServer::insertImagePiece(NDArray* outImage, NDArray* inImage, int xPos, int yPos)
{
	NDArrayInfo inInfo;
	inImage->getInfo(&inInfo);
	NDArrayInfo outInfo;
	outImage->getInfo(&outInfo);
	int inStride = inInfo.bytesPerElement * inInfo.xSize;
	int inLength = std::min(outInfo.xSize-xPos, inInfo.xSize) * inInfo.bytesPerElement;
	if(inLength > 0)
	{
		int outStride = outInfo.bytesPerElement * outInfo.xSize;
		char* out = (char*)outImage->pData + yPos*outStride + xPos*outInfo.bytesPerElement;
		char* in = (char*)inImage->pData;
		printf("#### inStride=%d, inLength=%d, outStride=%d, in=%p, out=%p, ySizeIn=%d, yPos=%d, ySizeOut=%d\n",
				inStride, inLength, outStride, in, out, (int)inInfo.ySize, yPos, (int)outInfo.ySize);
		for(size_t y=0; y<inInfo.ySize && (y+yPos)<outInfo.ySize; ++y)
		{
			if((out+inLength) > ((char*)outImage->pData+outInfo.totalBytes))
			{
				printf("#### Write outside NDArray bounds out=%p, inLength=%d, pData=%p, totBytes=%d, y=%d\n",
						out, inLength, outImage->pData, (int)inInfo.totalBytes, (int)y);
				return;
			}
			else
			{
				memcpy(out, in, inLength);
			}
			out += outStride;
			in += inStride;
		}
	}
}

// C entry point for iocinit
extern "C" int gangServerConfig(const char* portName, int gangPortNumber)
{
    Pco* pco = Pco::getPco(portName);
    if(pco != NULL)
    {
        new GangServer(pco, &pco->gangTrace, gangPortNumber);
    }
    else
    {
        printf("gangServerConfig: Pco \"%s\" not found\n", portName);
    }
    return asynSuccess;
}
static const iocshArg gangServerConfigArg0 = {"PCO Port Name", iocshArgString};
static const iocshArg gangServerConfigArg1 = {"Gang Port Number", iocshArgInt};
static const iocshArg* const gangServerConfigArgs[] =
    {&gangServerConfigArg0, &gangServerConfigArg1};
static const iocshFuncDef configGangServer =
    {"gangServerConfig", 2, gangServerConfigArgs};
static void configGangServerCallFunc(const iocshArgBuf *args)
{
    gangServerConfig(args[0].sval, args[1].ival);
}

/** Register the functions */
static void gangServerRegister(void)
{
    iocshRegister(&configGangServer, configGangServerCallFunc);
}

extern "C" { epicsExportRegistrar(gangServerRegister); }
