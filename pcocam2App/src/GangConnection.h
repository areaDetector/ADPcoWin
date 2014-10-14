/* GangServer.h
 *
 * Revamped PCO area detector driver.
 * The communication connection class for a ganged pair
 *
 * Author:  Giles Knap
 *          Jonathan Thompson
 *
 */
#ifndef GANGCONNECTION_H_
#define GANGCONNECTION_H_

#include "SocketProtocol.h"
#include "GangConfig.h"
#include "GangServerConfig.h"
class GangServer;
class TraceStream;
class Pco;
class GangMembeConfig;
class NDArray;

class GangConnection : public SocketProtocol
{
friend class GangMemberConfig;
friend class GangServerConfig;
public:
	GangConnection(Pco* pco, TraceStream* trace, const char* serverIp, int serverPort);
	virtual ~GangConnection();
	virtual void receive(char tag, int parameter, void* data, size_t dataSize);
	virtual void connected();
	virtual void disconnected();
	virtual void* getDataBuffer(char tag, int parameter, size_t dataSize);
	void writeInt32(int parameter, int value);
	void sendMemberConfig();
	void sendImage(NDArray* image, int sequence);
private:
	Pco* pco;
	TraceStream* trace;
	static const char* nameConnected;
	static const char* nameServerIp;
	static const char* nameServerPort;
	static const char* namePositionX;
	static const char* namePositionY;
	static const char* nameFunction;
	int handleConnected;
	int handleServerIp;
	int handleServerPort;
	int handlePositionX;
	int handlePositionY;
	int handleFunction;
	GangConfig config;
	GangServerConfig serverConfig;
};

#endif /* PCOCAM2APP_SRC_GANGCONNECTION_H_ */
