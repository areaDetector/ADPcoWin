/* GangServerConfig.h
 *
 * Revamped PCO area detector driver.
 * A class used to transfer of server configuration to the clients
 *
 * Author:  Giles Knap
 *          Jonathan Thompson
 *
 */

#ifndef GANGSERVERCONFIG_H_
#define GANGSERVERCONFIG_H_

class GangServer;
class GangConnection;
class Pco;

class GangServerConfig {
public:
	GangServerConfig();
	~GangServerConfig();
	void toPco(Pco* pco, GangConnection* gangConnection);
	void fromPco(Pco* pco, GangServer* gangServer);
	void* data();
private:
	int gangFunction;
};

#endif /* GANGSERVERCONFIG_H_ */
