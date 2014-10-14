/* GangServer.h
 *
 * Revamped PCO area detector driver.
 * The communication server class for the master of a ganged pair
 *
 * Author:  Giles Knap
 *          Jonathan Thompson
 *
 */

#ifndef GANGSERVER_H_
#define GANGSERVER_H_

#include "SocketProtocol.h"
#include <vector>
#include <list>
class Pco;
class GangClient;
class TraceStream;
class GangServerConfig;
class NDArray;

class GangServer : public SocketProtocol
{
friend class GangServerConfig;
public:
	GangServer(Pco* pco, TraceStream* trace, int gangPortNumber);
	virtual ~GangServer();
	virtual void accepted(int fd);
	void disconnected(GangClient* client);
	void arm();
	void disarm();
	void start();
	void stop();
	void configure();
	void writeInt32(int parameter, int value);
	bool imageReceived(int sequence, NDArray* image);
	void makeCompleteImages();
	void insertImagePiece(NDArray* outImage, NDArray* inImage, int xPos, int yPos);
	enum {gangFunctionOff=0, gangFunctionControl=1, gangFunctionFull=2};
	enum {imageTagMask=0x0f, imageTag=0xa0};
private:
	Pco* pco;
	std::vector<GangClient*> clients;
	TraceStream* trace;
	static const int maxConnections;
	static const char* nameNumConnections;
	static const char* nameServerPort;
	static const char* nameFunction;
	static const char* namePositionX;
	static const char* namePositionY;
	static const char* nameFullSizeX;
	static const char* nameFullSizeY;
	static const char* nameQueueSize;
	static const char* nameMissingPieces;
	int handleNumConnections;
	int handleServerPort;
	int handleFunction;
	int handlePositionX;
	int handlePositionY;
	int handleFullSizeX;
	int handleFullSizeY;
	int handleQueueSize;
	int handleMissingPieces;
	std::list<std::pair<int, NDArray*> > imageQueue;
	GangClient* getFreeClient();
	int countConnections();
	bool inControl();
	void determineImageSize();
	void clearImageQueue();
};

#endif /* PCOCAM2APP_SRC_GANGSERVER_H_ */
