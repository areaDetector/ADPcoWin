/* GangMemberConfig.cpp
 *
 * Revamped PCO area detector driver.
 * A class used to transfer configuration from clients
 *
 * Author:  Giles Knap
 *          Jonathan Thompson
 *
 */

#include "GangMemberConfig.h"
#include "Pco.h"
#include "GangConnection.h"
#include "GangClient.h"

// Constructor
GangMemberConfig::GangMemberConfig()
	: positionX(0)
	, positionY(0)
	, sizeX(0)
	, sizeY(0)
{
}

// Destructor
GangMemberConfig::~GangMemberConfig()
{
}

// Fill the config object from the PCO object
void GangMemberConfig::fromPco(Pco* pco, GangConnection* connection)
{
	pco->getIntegerParam(connection->handlePositionX, &positionX);
	pco->getIntegerParam(connection->handlePositionY, &positionY);
	pco->getIntegerParam(pco->ADSizeX, &sizeX);
	pco->getIntegerParam(pco->ADSizeY, &sizeY);
}

// Transfer configuration information to the PCO object
void GangMemberConfig::toPco(Pco* pco, GangClient* client)
{
	pco->setIntegerParam(client->handlePositionX, positionX);
	pco->setIntegerParam(client->handlePositionY, positionY);
	pco->setIntegerParam(client->handleSizeX, sizeX);
	pco->setIntegerParam(client->handleSizeY, sizeY);
}

// Return a pointer to the config data
void* GangMemberConfig::data()
{
	return this;
}
