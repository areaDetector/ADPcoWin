/* GangServer.h
 *
 * Revamped PCO area detector driver.
 * The communication connection class for a ganged pair
 *
 * Author:  Giles Knap
 *          Jonathan Thompson
 *
 */

#include "GangClient.h"
#include "TraceStream.h"
#include "Pco.h"
#include "GangServer.h"
#include "GangConfig.h"
#include "GangServerConfig.h"
#include <sstream>

// Area detector parameter names
const char* GangClient::nameConnected = "PCO_GANGSERV_CONNECTED";
const char* GangClient::nameUse = "PCO_GANGSERV_USE";
const char* GangClient::namePositionX = "PCO_GANGSERV_POSITIONX";
const char* GangClient::namePositionY = "PCO_GANGSERV_POSITIONY";
const char* GangClient::nameSizeX = "PCO_GANGSERV_SIZEX";
const char* GangClient::nameSizeY = "PCO_GANGSERV_SIZEY";
const char* GangClient::nameQueueSize = "PCO_GANGSERV_QUEUESIZE";

// Connection class constructor
GangClient::Connection::Connection(GangClient* owner)
	: SocketProtocol("GangClient", "pco_gang")
	, owner(owner)
{
}

// Constructor
// When used at the client end, the server is NULL
GangClient::GangClient(Pco* pco, TraceStream* trace, GangServer* gangServer,
		int index)
	: pco(pco)
	, trace(trace)
	, gangServer(gangServer)
	, index(index)
	, connection(NULL)
	, image(NULL)
{
	// Create the parameters
	createIntegerParam(nameConnected, &handleConnected, 0);
	createIntegerParam(nameUse, &handleUse, 0);
	createIntegerParam(namePositionX, &handlePositionX, 0);
	createIntegerParam(namePositionY, &handlePositionY, 0);
	createIntegerParam(nameSizeX, &handleSizeX, 0);
	createIntegerParam(nameSizeY, &handleSizeY, 0);
	createIntegerParam(nameQueueSize, &handleQueueSize, 0);
}

// Destructor
GangClient::~GangClient()
{
	clearImageQueue();
	if(connection)
	{
		delete connection;
	}
}

// Clear out stashed NDArrays
void GangClient::clearImageQueue()
{
	if(image)
	{
		image->release();
		printf("####3 n=%d\n", pco->pNDArrayPool->numBuffers());
	}
	image = NULL;
	std::list<std::pair<int, NDArray*> >::iterator pos;
	for(pos=imageQueue.begin(); pos!=imageQueue.end(); ++pos)
	{
		pos->second->release();
		printf("####3 n=%d\n", pco->pNDArrayPool->numBuffers());
	}
	imageQueue.clear();
}

// Create a parameter
void GangClient::createIntegerParam(const char* name, int* handle, int initialValue)
{
	std::stringstream str;
	str << name << index;
	pco->createParam(str.str().c_str(), asynParamInt32, handle);
	pco->setIntegerParam(*handle, initialValue);
}


// A message has been received from the peer.
void GangClient::receive(char tag, int parameter, void* data, size_t dataSize)
{
	switch(tag)
	{
	case 'm':
		gangMemberConfig.toPco(pco, this);
		break;
	case 'i':
		// Place the image in the queue
		imageQueue.push_back(std::pair<int,NDArray*>(parameter, image));
		image = NULL;
		printf("####9 q=%d\n", (int)imageQueue.size());
		// Get the main thread to forward any complete images
	    pco->post(Pco::requestMakeImages);
		// Update counters
		pco->lock();
		pco->setIntegerParam(handleQueueSize, (int)imageQueue.size());
		pco->callParamCallbacks();
		pco->unlock();
		break;
	}
}

// Return an indicator if the client has an image with the given sequence number.
// Any frames older than the given number are discarded.
// Returns seqStateNo if it does not
//         seqStateYes if it does
//         seqStateMissing if it has a newer frame
GangClient::SeqState GangClient::hasSequence(int s)
{
	GangClient::SeqState result = seqStateNo;
	// Discard older frames
	while(!imageQueue.empty() && (s - imageQueue.front().first) > 0)
	{
		imageQueue.front().second->release();
		imageQueue.pop_front();
	}
	if(imageQueue.empty())
	{
		result = seqStateNo;
	}
	else if(imageQueue.front().first == s)
	{
		result = seqStateYes;
	}
	else
	{
		result = seqStateMissing;
	}
	return result;
}

// This connection has broken
void GangClient::disconnected()
{
	if(connection)
	{
		pco->lock();
		pco->setIntegerParam(handleConnected, 0);
		pco->callParamCallbacks();
		pco->unlock();
		gangServer->disconnected(this);
		delete connection;
		connection = NULL;
	}
}

// Get a buffer for the reception of a message data buffer
void* GangClient::getDataBuffer(char tag, int parameter, size_t dataSize)
{
	void* result = NULL;
	switch(tag)
	{
	case 'm':
		result = gangMemberConfig.data();
		break;
	case 'i':
		pco->lock();
		int sizeX;
		int sizeY;
		int dataType;
		pco->getIntegerParam(handleSizeX, &sizeX);
		pco->getIntegerParam(handleSizeY, &sizeY);
		pco->getIntegerParam(pco->NDDataType, &dataType);
		pco->unlock();
		if(image)
		{
			image->release();
			printf("####5 n=%d\n", pco->pNDArrayPool->numBuffers());
		}
		image = pco->allocArray(sizeX, sizeY, (NDDataType_t)dataType);
		printf("####7 n=%d\n", pco->pNDArrayPool->numBuffers());
		if(image)
		{
			NDArrayInfo arrayInfo;
			image->getInfo(&arrayInfo);
			if(arrayInfo.totalBytes >= dataSize)
			{
				result = image->pData;
			}
		}
		break;
	}
	return result;
}

// Return true if this connection is up
bool GangClient::isConnected()
{
	int connected;
	pco->lock();
	pco->getIntegerParam(handleConnected, &connected);
	pco->unlock();
	return connected != 0;
}

// Return true if this connection is up and the client is to be used
bool GangClient::isToBeUsed()
{
	int connected;
	int use;
	pco->lock();
	pco->getIntegerParam(handleConnected, &connected);
	pco->getIntegerParam(handleUse, &use);
	pco->unlock();
	return connected != 0 && use != 0;
}

// Create the connection object
void GangClient::createConnection(int fd)
{
	if(connection)
	{
		delete connection;
	}
	connection = new Connection(this);
	connection->server(fd);
	pco->lock();
	pco->setIntegerParam(handleConnected, 1);
	pco->callParamCallbacks();
	pco->unlock();
}

// Send the arm message to the client
void GangClient::arm(GangConfig* config)
{
	clearImageQueue();
	connection->transmit('a', 0, config->data(), sizeof(GangConfig));
}

// Send the disarm message to the client
void GangClient::disarm()
{
	connection->transmit('d', 0, NULL, 0);
}

// Send the start message to the client
void GangClient::start(GangConfig* config)
{
	clearImageQueue();
	connection->transmit('s', 0, config->data(), sizeof(GangConfig));
}

// Send the stop message to the client
void GangClient::stop()
{
	connection->transmit('x', 0, NULL, 0);
}

// Send server configuration to the client
void GangClient::configure(GangServerConfig* config)
{
	connection->transmit('c', 0, config->data(), sizeof(GangServerConfig));
}

// Adjust the given full image size so that this
// client bit fits.
void GangClient::determineImageSize(int& fullSizeX, int& fullSizeY)
{
	int positionX;
	int positionY;
	int sizeX;
	int sizeY;
	pco->lock();
	pco->getIntegerParam(handlePositionX, &positionX);
	pco->getIntegerParam(handlePositionY, &positionY);
	pco->getIntegerParam(handleSizeX, &sizeX);
	pco->getIntegerParam(handleSizeY, &sizeY);
	pco->unlock();
	if(positionX + sizeX > fullSizeX)
	{
		fullSizeX = positionX + sizeX;
	}
	if(positionY + sizeY > fullSizeY)
	{
		fullSizeY = positionY + sizeY;
	}
}

// Insert the image with the sequence number into the out image at the
// appropriate place.  Remove the image from the queue.
// TODO: An improvement can be made by not assuming the image is at the head.
void GangClient::useImage(int sequence, NDArray* outImage)
{
	if(!imageQueue.empty())
	{
		// Get information
		int positionX;
		int positionY;
		pco->lock();
		pco->getIntegerParam(handlePositionX, &positionX);
		pco->getIntegerParam(handlePositionY, &positionY);
		pco->unlock();
		// Copy and free the image
		NDArray* inImage = imageQueue.front().second;
		gangServer->insertImagePiece(outImage, inImage, positionX, positionY);
		inImage->release();
		printf("####6 n=%d\n", pco->pNDArrayPool->numBuffers());
		imageQueue.pop_front();
		// Update counters
		pco->lock();
		pco->setIntegerParam(handleQueueSize, (int)imageQueue.size());
		pco->callParamCallbacks();
		pco->unlock();
	}
}
