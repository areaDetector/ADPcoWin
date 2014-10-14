/* GangClient.h
 *
 * Revamped PCO area detector driver.
 * Represents a gang client in the server
 *
 * Author:  Giles Knap
 *          Jonathan Thompson
 *
 */
#ifndef GANGCLIENT_H_
#define GANGCLIENT_H_

#include "SocketProtocol.h"
#include "GangMemberConfig.h"
#include <list>
class GangServer;
class GangConfig;
class GangMemberConfig;
class GangServerConfig;
class TraceStream;
class Pco;
class NDArray;

class GangClient
{
friend class GangMemberConfig;
public:
	class Connection: public SocketProtocol
	{
	private:
		GangClient* owner;
	public:
		Connection(GangClient* owner);
		virtual ~Connection() {}
        virtual void receive(char tag, int parameter, void* data, size_t dataSize)
            {this->owner->receive(tag, parameter, data, dataSize);}
        virtual void disconnected()
            {this->owner->disconnected();}
        virtual void* getDataBuffer(char tag, int parameter, size_t dataSize)
            {return this->owner->getDataBuffer(tag, parameter, dataSize);}
	};
public:
	GangClient(Pco* pco, TraceStream* trace, GangServer* gangServer, int index);
	virtual ~GangClient();
	void receive(char tag, int parameter, void* data, size_t dataSize);
	void disconnected();
	void* getDataBuffer(char tag, int parameter, size_t dataSize);
	bool isConnected();
	bool isToBeUsed();
	void createConnection(int fd);
	void arm(GangConfig* config);
	void disarm();
	void start(GangConfig* config);
	void stop();
	void configure(GangServerConfig* config);
	void determineImageSize(int& fullSizeX, int& fullSizeY);
	enum SeqState {seqStateNo, seqStateYes, seqStateMissing};
	SeqState hasSequence(int s);
	void useImage(int sequence, NDArray* outImage);
private:
	Pco* pco;
	TraceStream* trace;
	GangServer* gangServer;
	int index;
	Connection* connection;
	static const char* nameConnected;
	static const char* nameUse;
	static const char* namePositionX;
	static const char* namePositionY;
	static const char* nameSizeX;
	static const char* nameSizeY;
	static const char* nameQueueSize;
	int handleConnected;
	int handleUse;
	int handlePositionX;
	int handlePositionY;
	int handleSizeX;
	int handleSizeY;
	int handleQueueSize;
	GangMemberConfig gangMemberConfig;
	NDArray* image;
	void createIntegerParam(const char* name, int* handle, int initialValue);
	void clearImageQueue();
	std::list<std::pair<int, NDArray*> > imageQueue;
};

#endif /* PCOCAM2APP_SRC_GANGCONNECTION_H_ */
