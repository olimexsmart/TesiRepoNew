/*
 * DTNapp.h
 *
 *  Created on: 04/mag/2016
 *      Author: Fabio Patrone
 *      Contributors: Luca Olivieri
 */

#ifndef DTNAPP_H_
#define DTNAPP_H_

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/random-variable.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/point-to-point-layout-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/bridge-module.h"
#include "ns3/netanim-module.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include "time.h"
#include <string>
#include <fstream>
#include <sstream>
#include "mypacket.h"
#include "DTNNodesMobility.h"

#define DTN_TCP_PORT 4556
#define DTN_UDP_PORT 4556

const int BUNDLEACKSIZE = WIFIHEADERSIZE + LLCHEADERSIZE + NETWORKHEADERSIZE + UDPHEADERSIZE + BUNDLEFRAGMENTHEADERSIZE + BUNDLEHEADERSIZE + PAYLOADACKSIZE;		// size bundle ack
const int BUNDLEDATASIZE = WIFIHEADERSIZE + LLCHEADERSIZE + NETWORKHEADERSIZE + UDPHEADERSIZE + BUNDLEFRAGMENTHEADERSIZE + BUNDLEHEADERSIZE + PAYLOADSIZE;			// maximum bundle size [B] (usata per settare il margine di fine contatto)


struct RoutingEntry {
	RoutingEntry(): destinationEID("0.0.0.0"), destinationEIDMask("0.0.0.0"), sourceIP("0.0.0.0"), nextHopIP("0.0.0.0"), nextHopIPMask("255.255.255.255"), deviceForNextHopIP(NULL), deviceType(0) {}
	Ipv4Address destinationEID;
	Ipv4Address destinationEIDMask;
	Ipv4Address sourceIP;
	Ipv4Address nextHopIP;
	Ipv4Address nextHopIPMask;
	Ptr<NetDevice> deviceForNextHopIP;
	uint32_t deviceType;
};

/*
	This struct is held in the contactTable vector, one entry for each
	node of the DTN network. The vector and his components are filled in
	the main at "//READING CONTACT TABLE" point. Indexing at the same
	poit all the vectors inside the struct will select a contact entry.
 */
struct ContactEntry {
	ContactEntry(): this_node_address("0.0.0.0") {}
	Ipv4Address this_node_address;
	vector<Ipv4Address> node_in_contact_with;
	vector<uint32_t> t_start;
	vector<uint32_t> t_end;
	vector<uint32_t> volumeTraffic;
};

class DtnApp : public Application
{
public:
	DtnApp ();
	virtual ~DtnApp();
	struct SocketInfo {
		SocketInfo(): source_ip_address("0.0.0.0"), source_port(0), sent_data(0), last_fragment_received(0) {}
		Ipv4Address source_ip_address;
		uint32_t source_port;
		uint32_t sent_data;
		uint32_t last_fragment_received;
		Ptr<Packet> bundle;
	};

	//Function prototypes
	void CheckWiredBuffer();
	void CheckWirelessBuffer(Ipv4Address nodeInContactWithWirelessAddress, uint32_t maximumNumberBundlesInCurrentContact);
	//void DeleteAllActiveWirelessSockets();
	void CloseTxSocket(Ptr<Socket> socket, uint32_t packet_size);
	void ConnectionAccept(Ptr<Socket>, const Address& from);
	void CreateBundleAck(Ipv4Address sourceIPAddress, uint32_t bundleSeqNumber);
	void CreateBundleData (Ipv4Address destinationAddress, vector<ContactEntry> contactTable, uint32_t TOV);
	void CreateBundleRequest (Ipv4Address destinationIPAddress);
	void CreateBundleStatusBuffer();
	void DeleteActiveSocketEntry(Ipv4Address sourceIPAddress, uint32_t sourcePort, uint32_t socketType);
	//void EmptyTransmittedWirelessBundles();
	void FindDestination(Ptr<Packet> bundle);
	RoutingEntry GetNextHopAddress(Ipv4Address bundleDestinationEID);
	bool isTransmissionPossibleIntersatellite (Ipv4Address nodeInContactWithWirelessAddress, Ipv4Address destinationAddress);
	void PrintNanosatelliteBufferOccupancy();
	void ProcessBundleFromTCPSocket(Ipv4Address previousHopIPAddress, uint32_t previousHopPort, Ptr<Packet> receivedBundle);
	void ProcessBundleFromUDPSocket(Ipv4Address previousHopIPAddress, uint32_t previousHopPort, Ptr<Packet> receivedBundle);
	void ReceiveBundle(Ptr<Socket> socket);
	void RemoveFromTransmittedBundles(Ipv4Address sourceIPAddress, uint32_t bundleSeqNumber, uint32_t socketType);
	void SendBundle(Ptr<Packet> bundle, RoutingEntry routingEntry);
	//void SetContactEntry(ContactEntry contactEntry);
	void SetRoutingEntry(RoutingEntry routingEntry);
	void Setup(Ptr<Node> node);
	void StopWirelessTransmission(Ipv4Address nodeInContactWithWirelessAddress);
	void UpdateContactInformation(Ptr<Packet> bufferInfo);
	vector<ContactEntry> contactTable;
	vector<uint32_t> bufferOccupancyNodeInContactWith;

	//Prototypes of new functions
	vector<mypacket::BndlPath> FindPath(vector<ContactEntry> contactTable, Ipv4Address destinationAddress, uint32_t TOV, uint32_t SOB);
	void SourceContactGraphRouting(vector< vector<mypacket::BndlPath> > &allPaths, vector<ContactEntry> contactTable, Ipv4Address destAddress, double TOV, uint32_t SOB, vector<mypacket::BndlPath> untilHere);
	vector<mypacket::BndlPath> ChoosePath(vector< vector<mypacket::BndlPath> > allPaths);
	int GetNodeType(Ipv4Address address);
	bool CheckContact(uint32_t volumeRemaining, uint32_t SOB);


private:
	vector<SocketInfo*> active_wired_sockets;
	vector<SocketInfo*> active_wireless_sockets;
	Ptr<Node>         m_node;
	Ptr<Socket>       m_socket;
	vector<Ptr<Packet> > stored_wired_bundles;
	vector<Ptr<Packet> > stored_wireless_bundles;
	vector<Ptr<Packet> > transmitted_wired_bundles;
	vector<Ptr<Packet> > transmitted_wireless_bundles;
	vector<RoutingEntry> routingTable;
	bool contactInProgress;
	bool transmissionInProgress;
	uint32_t dataSentDuringThisContact;
	Ipv4Address nodeInContactWithTxAddress;
	Ipv4Address nodeInContactWithRxAddress;
	uint32_t maximumNumberBundlesInCurrentContact;
};


#endif /* DTNAPP_H_ */
