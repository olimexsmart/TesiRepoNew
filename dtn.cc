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

using namespace ns3;
using namespace std;

DTNNodesMobility* groundStationsNodesMobility;
DTNNodesMobility* nanosatelliteNodesMobility;

int nHotSpots;
int nNanosats;
int nColdSpots;
int nRuralNodesForEachColdSpot;
int nOrbits;
int nBundles;
uint32_t TOV;

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

void PopulateArpCache(NodeContainer allIPNodes);

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
	uint32_t dataSentDuringThisContact;
	bool contactInProgress;
	bool transmissionInProgress;
	Ipv4Address nodeInContactWithTxAddress;
	Ipv4Address nodeInContactWithRxAddress;
	uint32_t maximumNumberBundlesInCurrentContact;
};

DtnApp::DtnApp()
: m_node(NULL),
  m_socket (NULL),
  contactInProgress(false),
  transmissionInProgress(false),
  dataSentDuringThisContact(0),
  nodeInContactWithTxAddress((uint32_t)0),
  nodeInContactWithRxAddress((uint32_t)0),
  maximumNumberBundlesInCurrentContact(0)
{ }


DtnApp::~DtnApp() //Constructor
		{
	m_socket = 0;
	m_node = 0;
		}

void DtnApp::CheckWiredBuffer() {
	if (stored_wired_bundles.size()) {
		vector<Ptr<Packet> >::iterator iter = stored_wired_bundles.begin();
		mypacket::BndlHeader bndlHeader;
		(*iter)->PeekHeader(bndlHeader);
		RoutingEntry routingEntryFound = GetNextHopAddress(bndlHeader.GetDst());
		//Verify it's a bundle from a central node (9.0.0.1)
		//Overwrite nexthop ip, reading it from the path vector
		SendBundle((*iter)->Copy(), routingEntryFound);
		stored_wired_bundles.erase(iter);
	}
	if (stored_wired_bundles.size())
		Simulator::Schedule (Seconds (0.01), &DtnApp::CheckWiredBuffer, this);
	else
		Simulator::Schedule (Seconds (0.1), &DtnApp::CheckWiredBuffer, this);
}

/*
	This function is the hearth of the simulation. 
	Each call can send a single bundle, of any type.
	It's scheduled in the simulator in main() at the
	beginning of the simulation, reading the contact table.
	Is rescheduled by itself depending on the time needed to
	send the current bundle.
	If any bundle has to be sent, is not rescheduled any more, but,
	in case a new bundle gets to a ground station while the 
	contact is still possible, is scheduled againf from FindDestination().
	
	As for now, the new logic is implemented for Data type bundle, the other types are processed
	like before to speed up the work.
	The RoutingEntry construct is not quite useful any more, but is still used to avoid extensive
	modifications to the code.
*/
void DtnApp::CheckWirelessBuffer(Ipv4Address nodeInContactWithWirelessAddress, uint32_t maximumNumberBundlesInCurrentContact) {
	
	if ((stored_wireless_bundles.size() != 0) && !intersatelliteContact) {
		bool bundleNotData = false;
		mypacket::BndlHeader bndlHeader;
		(*stored_wireless_bundles.begin())->PeekHeader(bndlHeader);
		if (bndlHeader.GetBundleType() != 0)
			bundleNotData = true;
		if (contactInProgress || bundleNotData) {
			bool sent = false;
			for (vector<Ptr<Packet> >::iterator iter = stored_wireless_bundles.begin() ; iter != stored_wireless_bundles.end(); ++iter) {
				mypacket::BndlHeader bndlHeader;
				(*iter)->PeekHeader(bndlHeader);
				RoutingEntry routingEntryFound = GetNextHopAddress(bndlHeader.GetDst());
				/*
				 * If - else that does divid the code on the type of bundle processed
				 * Overwrite nexthop reading it from the path vector
				 * the correct vector entry is found firstly finding my ip and then getting the next element
				 * then check if this element is corresponding to nodeInContactWithWirelessAddress && Now() > m_contactTime --- if true, send =  true;
				 *
				 * if not, continue;
				 */
				//begin ack management
				stringstream tmp;
				tmp << "50.0.0." << nHotSpots+nNanosats+nColdSpots+1;
				string addressClass = tmp.str();
				stringstream tmp2;
				tmp2 << "50.0.0." << nHotSpots+1;
				string addressClass2 = tmp2.str();
				uint8_t* nodeInContactAddress = new uint8_t [4];
				uint8_t* thisNodeAddress = new uint8_t [4];
				nodeInContactWithWirelessAddress.Serialize(nodeInContactAddress);
				(m_node->GetObject<Ipv4>()->GetAddress (2, 0)).GetLocal().Serialize(thisNodeAddress);
				if(((routingEntryFound.nextHopIP == Ipv4Address(addressClass.c_str())) && (nodeInContactWithWirelessAddress < Ipv4Address(addressClass2.c_str()))) || (routingEntryFound.nextHopIP == Ipv4Address("50.0.0.0")) || (routingEntryFound.nextHopIP == Ipv4Address("10.0.0.0")) || (((thisNodeAddress[3] > nHotSpots) && (thisNodeAddress[3] <= (nHotSpots+nNanosats))) && ((nodeInContactAddress[3] > nHotSpots) && (nodeInContactAddress[3] <= (nHotSpots+nNanosats))))) {
					routingEntryFound.nextHopIP = nodeInContactWithWirelessAddress;
					routingEntryFound.nextHopIPMask = "255.255.255.255";
				}
				bool send = false;
				if (((thisNodeAddress[3] > nHotSpots) && (thisNodeAddress[3] <= (nHotSpots+nNanosats))) && ((nodeInContactAddress[3] > nHotSpots) && (nodeInContactAddress[3] <= (nHotSpots+nNanosats)))) {
					if (((dataSentDuringThisContact + 1) <= maximumNumberBundlesInCurrentContact) && (bndlHeader.GetBundleType() == 0))
						send = isTransmissionPossibleIntersatellite(nodeInContactWithWirelessAddress, bndlHeader.GetDst());
					else if (bndlHeader.GetBundleType() != 0)
						send = true;
				}
				else if (((routingEntryFound.nextHopIP == nodeInContactWithWirelessAddress) || (routingEntryFound.nextHopIP == bndlHeader.GetDst())) && ((dataSentDuringThisContact + 1) <= maximumNumberBundlesInCurrentContact))
					send = true;
				//End of ack management

				if (send) {
/*					if (bndlHeader.GetBundleType() == 0)
						dataSentDuringThisContact++;*/
					SendBundle((*iter)->Copy(), routingEntryFound);
					uint32_t bundleSize = (*iter)->GetSize();
					sent = true;
					transmissionInProgress = true;
					if (bndlHeader.GetBundleType() == 0)
						Simulator::Schedule (Seconds ((double) bundleSize / TX_RATE_WIRELESS_LINK), &DtnApp::CheckWirelessBuffer, this, nodeInContactWithWirelessAddress, false, maximumNumberBundlesInCurrentContact);
					stored_wireless_bundles.erase(iter);
					break;
				}
				delete [] nodeInContactAddress;
				delete [] thisNodeAddress;
			}
			if (!sent)
				transmissionInProgress = false;
		}
	}
	else
		transmissionInProgress = false;
}

void DtnApp::CloseTxSocket (Ptr<Socket> socket, uint32_t packet_size) {
	Address socketAddress;
	socket->GetSockName(socketAddress);
	InetSocketAddress inetsocketaddress = InetSocketAddress::ConvertFrom(socketAddress);
	vector<SocketInfo*>::iterator iter;
	if (socket->GetSocketType() == Socket::NS3_SOCK_DGRAM){
		for (vector<SocketInfo*>::iterator iter = active_wireless_sockets.begin(); iter != active_wireless_sockets.end(); ++iter) {
			if (((*iter)->source_ip_address == inetsocketaddress.GetIpv4()) && ((*iter)->source_port == inetsocketaddress.GetPort())) {
				(*iter)->sent_data += (packet_size - BUNDLEFRAGMENTHEADERSIZE);
				if ((*iter)->sent_data == (*iter)->bundle->GetSize()) {
					mypacket::BndlHeader bndlHeader;
					(*iter)->bundle->PeekHeader(bndlHeader);
					if (bndlHeader.GetBundleType() != 0)
						RemoveFromTransmittedBundles(bndlHeader.GetOrigin(),bndlHeader.GetOriginSeqno(), 1);
					delete (*iter);
					active_wireless_sockets.erase(iter);
					break;
				}
			}
		}
	}
	else {
		for (vector<SocketInfo*>::iterator iter = active_wired_sockets.begin(); iter != active_wired_sockets.end(); ++iter) {
			if (((*iter)->source_ip_address == inetsocketaddress.GetIpv4()) && ((*iter)->source_port == inetsocketaddress.GetPort())) {
				(*iter)->sent_data += (packet_size - BUNDLEFRAGMENTHEADERSIZE);
				if ((*iter)->sent_data == (*iter)->bundle->GetSize()) {
					socket->Close();
					mypacket::BndlHeader bndlHeader;
					(*iter)->bundle->PeekHeader(bndlHeader);
					RemoveFromTransmittedBundles(bndlHeader.GetOrigin(),bndlHeader.GetOriginSeqno(), 0);
					delete (*iter);
					active_wired_sockets.erase(iter);
					break;

				}
			}
		}
	}
}

void DtnApp::ConnectionAccept (Ptr<Socket> s, const Address & addr) {
	s->SetRecvCallback (MakeCallback (&DtnApp::ReceiveBundle, this));
}

void DtnApp::CreateBundleAck (Ipv4Address sourceAddress, uint32_t bundleSeqNumber) {
	Ptr<Packet> ack = Create<Packet> (PAYLOADACKSIZE);
	mypacket::BndlHeader bndlHeader;
	bndlHeader.SetBundleType(1);
	bndlHeader.SetDst (nodeInContactWithTxAddress);
	bndlHeader.SetOrigin (sourceAddress);
	bndlHeader.SetOriginSeqno (bundleSeqNumber);
	bndlHeader.SetPayloadSize (PAYLOADACKSIZE);
	bndlHeader.SetSrcTimestamp (Simulator::Now ());
	ack->AddHeader (bndlHeader);
	stored_wireless_bundles.insert(stored_wireless_bundles.begin(), ack);
	CheckWirelessBuffer(nodeInContactWithTxAddress, maximumNumberBundlesInCurrentContact);
}

void DtnApp::CreateBundleData (Ipv4Address destinationAddress, vector<ContactEntry> contactTable, uint32_t TOV) {
	Ptr<Packet> bundle = Create<Packet> (PAYLOADSIZE);
	mypacket::BndlHeader bndlHeader;
	bndlHeader.SetBundleType(0);
	bndlHeader.SetOrigin ((m_node->GetObject<Ipv4>()->GetAddress(1, 0)).GetLocal());
	bndlHeader.SetDst (destinationAddress);
	bndlHeader.SetOriginSeqno (bundle->GetUid());
	bndlHeader.SetPayloadSize (PAYLOADSIZE);
	bndlHeader.SetSrcTimestamp (Simulator::Now ());
	//Here set the paht vector through the SCGR algorithm
	bndlHeader.SetPathVector(FindPath(contactTable, destinationAddress, TOV, BUNDLEDATASIZE));
	bundle->AddHeader (bndlHeader);
	stored_wired_bundles.push_back(bundle);
}

void DtnApp::CreateBundleRequest (Ipv4Address destinationAddress) {
	Ptr<Packet> bundle = Create<Packet> (PAYLOADREQUESTSIZE);
	mypacket::BndlHeader bndlHeader;
	bndlHeader.SetBundleType(3);
	bndlHeader.SetOrigin ((m_node->GetObject<Ipv4>()->GetAddress(1, 0)).GetLocal());
	bndlHeader.SetDst (destinationAddress);
	bndlHeader.SetOriginSeqno (bundle->GetUid());
	bndlHeader.SetPayloadSize (PAYLOADREQUESTSIZE);
	bndlHeader.SetSrcTimestamp (Simulator::Now ());
	bundle->AddHeader (bndlHeader);
	stored_wired_bundles.push_back(bundle);
}

//WORKING HERE
/*
		@The logic lays in a recursive function, should be better to 
		 wrap the recursive part in a function that also gets ready
		 all the necessary data structures.
		@First create the body all the functions, including the 
		 parametres. Place them correctly in the code.
		@Implement the logic inside
 */

//Wrapper of the main algorithm, create here the necessary structures
/*
	TOV -> Time Of Validity
	SOB -> Size Of Bundle
 */
//INCONGRUENCE/BUG here: SOB is actually dependent from the result of this operation, let ignore this for now
vector<mypacket::BndlPath> DtnApp :: FindPath(vector<ContactEntry> contactTable, Ipv4Address destinationAddress, uint32_t TOV, uint32_t SOB){
	//Init here all the date structure to start the recursive action
	vector< vector<mypacket::BndlPath> > allPaths;
	vector<mypacket::BndlPath> untilHere;

	allPaths = SourceContactGraphRouting(allPaths, contactTable, destinationAddress, TOV, SOB, untilHere);

	return ChoosePath(allPaths);
}

/*
	The SCGR data structure is a vector of path vectors.
	This function will recursively call itself filling the 
	SCGR with all possible paths. Lastly the final decision on the 
	path will be taken by a separate function.
 */
void DtnApp :: SourceContactGraphRouting(vector< vector<mypacket::BndlPath> > &allPaths, vector<ContactEntry> contactTable, Ipv4Address destAddress, double TOV, uint32_t SOB, vector<mypacket::BndlPath> untilHere){
	//Routine here to find the entry in the contactTable startin from an IP address
	int thisNode; //Holds the index of the contact table correspondig to the node considered in this call
	for(thisNode = 0; thisNode < contactTable.size(); thisNode++)
		if(contactTable[thisNode].this_node_address == destAddress)
			break;


	vector<mypacket::BndlPath> newPath;

	//Now we need to select contacts between now and TOV of the current node
	int tStart;
	int tEnd;
	for(tStart = 0; contactTable[thisNode].t_start[tStart] < Simulator::Now (); tStart ++);
	for(tEnd = 0; contactTable[thisNode].t_end[tEnd] < TOV; tEnd ++);

	for(int k = tStart; k < tEnd; k++)
	{
		newPath = untilHere; //Reset the path at every hop selection
		if(CheckContact(contactTable[thisNode].volumeTraffic[k], SOB)) //Check if this contact is usable
		{	//Setting up the next hop structure to adding it at the end of the newPath
			mypacket::BndlPath pathEntry = new mypacket::BndlPath(contactTable[thisNode].t_start[k], contactTable[thisNode].node_in_contact_with[k]);
			newPath.push_back(pathEntry);

			//This gets to recognize the type of node, if == 1 it's a hotspot
			//If it's a two is a nanosatellite, recall this function
			if(GetNodeType(contactTable[thisNode].node_in_contact_with[k]) == 1)
				allPaths.push_back(newPath);
			else if(GetNodeType(contactTable[thisNode].node_in_contact_with[k]) == 2)
				allPaths = SourceContactGraphRouting(allPaths, contactTable, contactTable[thisNode].node_in_contact_with[k], TOV, SOB, newPath);
		}
	}
}

/*
	In the need of recognizing easily the type of node we are dealing with,
	this function will take an IP address and a flag that indicates if it is
	an RX or TX IP. 
	Keep in mind this needs to know the configuation of the network.
	Returns: 1 -> HotSpot / 2 -> NanoSat / 3 -> ColdSpot
			-1 -> Error
 */
int DtnApp :: GetNodeType(Ipv4Address address){

	uint8_t* addr = new uint8_t[4];
	address.Serialize(addr);

	if(addr[3] <= nHotSpots)
		return 1;
	else if(addr[3] > nHotSpots && addr[3] <= nHotSpots + nNanosats)
		return 2;
	else if(addr[3] > nHotSpots + nNanosats)
		return 3;
	else
		return -1;
}

/*
	Check a series of conditions on which depends if a contact is valid for
	the path of this bundle. 
	As for now, this only check if the traffic volume can handle the size of 
	this bundle.
 */
bool DtnApp :: CheckContact(uint32_t volumeRemaining, uint32_t SOB){
	if(volumeRemaining > SOB) return true;
	else return false;
}

/*
	This function will get the structure that contains all possible paths and chooses the 
	best one according to a decision scheme. Before returning the result, fields that needs to 
	be update are modified.
	This for now simply finds the fastest route, in a future the decision could be made on
	optimization rules like a omogeneous load between nodes.
 */
vector<mypacket::BndlPath> DtnApp :: ChoosePath(vector< vector<mypacket::BndlPath> > allPaths){
	//The second index has to be zero because the delivery time depends on the last contact,
	//the one with the cold spot
	//Yes, the BndlPath vector is in reversed order, keep in mind that and think about to reverse it some time in the future
	int minT = allPaths[0][0].m_contactTime;
	int minI = 0;

	for(int i = 0; i < allPaths.size(); i++)
	{
		if(allPaths[i][0].m_contactTime < minT)
		{
			minT = allPaths[i][0].m_contactTime;
			minI = i;
		}
	}

	return allPaths[minI].m_contactTime;
}


void DtnApp::CreateBundleStatusBuffer() {
	mypacket::BndlHeader bndlHeader;
	uint8_t* coldSpotAddress = new uint8_t[4];
	uint8_t* payload = new uint8_t[8 * ((uint32_t)nColdSpots+1)];
	uint32_t packetStoredSizeForEachColdSpot[(uint32_t)nColdSpots+1];
	for (uint32_t i = 0; i < (nColdSpots+1); i++)
		packetStoredSizeForEachColdSpot[i] = 0;
	for (vector<Ptr<Packet> >::iterator iter = stored_wireless_bundles.begin() ; iter != stored_wireless_bundles.end(); ++iter) {
		(*iter)->PeekHeader(bndlHeader);
		if (bndlHeader.GetDst() == Ipv4Address("9.0.0.1"))
			packetStoredSizeForEachColdSpot[(uint32_t)nColdSpots]++;
		else {
			(bndlHeader.GetDst() & Ipv4Address("255.0.0.0")).Serialize(coldSpotAddress);		// memorizzo la sottorete rappresentante la zona rurale in considerazione in formato uint8_t [4], in modo da poter prelevare il valore intero associato al primo ottetto dell'indirizzo
			packetStoredSizeForEachColdSpot[coldSpotAddress[0]-11]++;
		}
	}
	for (uint32_t i = 0; i < nColdSpots; i++) {
		stringstream coldSpotAddr;
		coldSpotAddr << (11 + i) << ".0.0.1";
		string address = coldSpotAddr.str();
		Ipv4Address coldSpotIPv4Address = Ipv4Address(address.c_str());
		coldSpotIPv4Address.Serialize(coldSpotAddress);
		copy(coldSpotAddress, coldSpotAddress + 4, payload + (i * 8));
		for (uint32_t j = 0; j < 4; j++)
			payload [4 + (i * 8) + j] = packetStoredSizeForEachColdSpot[i]>>(24 - 8 * j);
	}
	stringstream centralNodeAddr;
	centralNodeAddr << "9.0.0.1";
	string address = centralNodeAddr.str();
	Ipv4Address centralNodeIPv4Address = Ipv4Address(address.c_str());
	centralNodeIPv4Address.Serialize(coldSpotAddress);
	copy(coldSpotAddress, coldSpotAddress + 4, payload + ((uint32_t)nColdSpots * 8));
	for (uint32_t j = 0; j < 4; j++)
		payload [4 + ((uint32_t)nColdSpots * 8) + j] = packetStoredSizeForEachColdSpot[(uint32_t)nColdSpots]>>(24 - 8 * j);
	Ptr<Packet> bundleInfo = Create<Packet>(payload, 8 * ((uint32_t)nColdSpots+1));
	bndlHeader.SetBundleType(2);
	Ipv4Address a = (m_node->GetObject<Ipv4>()->GetAddress(1, 0)).GetLocal();
	bndlHeader.SetOrigin ((m_node->GetObject<Ipv4>()->GetAddress(1, 0)).GetLocal());
	bndlHeader.SetDst (nodeInContactWithRxAddress);
	bndlHeader.SetOriginSeqno (bundleInfo->GetUid());
	bndlHeader.SetPayloadSize (8*(nColdSpots+1));
	bndlHeader.SetSrcTimestamp (Simulator::Now ());
	bundleInfo->AddHeader (bndlHeader);
	stored_wireless_bundles.insert(stored_wireless_bundles.begin(), bundleInfo);
	CheckWirelessBuffer(nodeInContactWithRxAddress, maximumNumberBundlesInCurrentContact);
	delete [] coldSpotAddress;
	delete [] payload;
}

/*Not used in this work
void DtnApp::DeleteActiveSocketEntry (Ipv4Address sourceIpAddress, uint32_t sourcePort, uint32_t socketType) {
	if (socketType == 0) {
		for (vector<SocketInfo*>::iterator iter = active_wired_sockets.begin() ; iter != active_wired_sockets.end(); ++iter) {
			if((*iter)->source_ip_address == sourceIpAddress && (*iter)->source_port == sourcePort) {
				delete(*iter);
				active_wired_sockets.erase(iter);
				break;
			}
		}
	}
	else {
		for (vector<SocketInfo*>::iterator iter = active_wireless_sockets.begin() ; iter != active_wireless_sockets.end(); ++iter) {
			if((*iter)->source_ip_address == sourceIpAddress && (*iter)->source_port == sourcePort) {
				delete(*iter);
				active_wireless_sockets.erase(iter);
				break;
			}
		}
	}
}
 */

/*Not used in this work
void DtnApp::DeleteAllActiveWirelessSockets() {
	active_wireless_sockets.clear();
}
 */
/*Not used in this work
void DtnApp::EmptyTransmittedWirelessBundles() {
	for (vector<Ptr<Packet> >::iterator iter = transmitted_wireless_bundles.begin() ; iter != transmitted_wireless_bundles.end();++iter)
		stored_wireless_bundles.insert(stored_wireless_bundles.begin(), (*iter)->Copy());
	transmitted_wireless_bundles.clear();
}*/

/*
	This will get involved but not in this stage of the work.
	Some part of it will be developed to add new functionalities 
	like a two way request-response.
	As for now, his main application is to forward bundles in intermediate
	nodes and provide a report of the simulation. 
*/
void DtnApp::FindDestination(Ptr<Packet> receivedBundle) {
	mypacket::BndlHeader bndlHeader;
	receivedBundle->PeekHeader(bndlHeader);
	if (bndlHeader.GetBundleType() == 0) {
		stringstream fileName;
		fileName <<  "/home/fabio/source/ns-3.21/Temp/Received_by_" << (m_node->GetObject<Ipv4>()->GetAddress (1, 0)).GetLocal() << ".txt";
		string tmp = fileName.str();
		const char* reportName = tmp.c_str();
		ofstream report;
		report.open(reportName, ios::out | ios::app | ios::binary);
		report << "At time " << Simulator::Now().GetSeconds() <<" received bundle data with sequence number " <<  bndlHeader.GetOriginSeqno() <<" from " <<  bndlHeader.GetOrigin() <<	" to " << bndlHeader.GetDst() << "\n";
		report.close();
	}
	//Get your own address
	if ((m_node->GetObject<Ipv4>()->GetAddress (1, 0)).GetLocal() == bndlHeader.GetDst()) {
		/*If the received bundle was a request, create a response. 
		//NOT IMPLEMENTED YET
		if (bndlHeader.GetBundleType() == 3)
			CreateBundleData(bndlHeader.GetOrigin());
			*/
	}
	else {
		receivedBundle->RemoveAllPacketTags();
		RoutingEntry routingEntryFound = GetNextHopAddress(bndlHeader.GetDst());
		if (routingEntryFound.deviceType == 0)
			stored_wired_bundles.push_back(receivedBundle->Copy());
		else {
			stored_wireless_bundles.push_back(receivedBundle->Copy());
			//Schedule a new wireless communication in case there isn't one going on and the contact is still active
			if (contactInProgress && !transmissionInProgress)
				CheckWirelessBuffer(nodeInContactWithRxAddress, maximumNumberBundlesInCurrentContact);
		}
	}
}

//This is still getting involved, but it isn't functional to the new logic
//still used in bundle ack packets
RoutingEntry DtnApp::GetNextHopAddress(Ipv4Address bundleDestinationEID) {
	for (vector<RoutingEntry>::iterator iter = routingTable.begin(); iter != routingTable.end(); ++iter)
		if (((bundleDestinationEID ^ iter->destinationEID) & iter->destinationEIDMask) == Ipv4Address("0.0.0.0"))
			return (*iter);
}

//This probably will be deleted
bool DtnApp::isTransmissionPossibleIntersatellite (Ipv4Address nodeInContactWithWirelessAddress, Ipv4Address destinationAddress) {
	uint8_t* contactAddress = new uint8_t [4];
	uint8_t* destAddr = new uint8_t [4];
	vector<uint32_t> bufferOccupancyNodeInContactWithTemp = bufferOccupancyNodeInContactWith;
	nodeInContactWithWirelessAddress.Serialize(contactAddress);
	contactAddress[0] = 50;
	uint32_t nodeInContactAddress = contactAddress[3] | contactAddress[2]<<8 | contactAddress[1]<<16 | contactAddress[0]<<24;
	nodeInContactWithWirelessAddress = Ipv4Address(nodeInContactAddress);
	if (destinationAddress != Ipv4Address("9.0.0.1")) {
		destinationAddress.Serialize(destAddr);
		destAddr[3] = nHotSpots + nNanosats + destAddr[0] - 10;
		destAddr[0] = 50;
		uint32_t destAddress = destAddr[3] | destAddr[2]<<8 | destAddr[1]<<16 | destAddr[0]<<24;
		destinationAddress = Ipv4Address(destAddress);
	}
	stringstream tmp;
	tmp << "50.0.0." << nHotSpots+1;
	string addressClass = tmp.str();
	Ipv4Address highestHotSpot = Ipv4Address(addressClass.c_str());
	uint32_t timeToDeliveryNodeInContact = 4294967295, timeToDeliveryThisNode = 4294967295, maximumNumberDeliveredBundle = 0, numberPastContacts = 0, numberPastContactsTemp = 0;
	while (true) {
		for (vector<ContactEntry>::iterator iter = contactTable.begin(); iter != contactTable.end(); ++iter) {
			if ((*iter).this_node_address == nodeInContactWithWirelessAddress) {
				uint32_t i = 0;
				numberPastContactsTemp = numberPastContacts;
				for (vector<Ipv4Address>::iterator iter2 = (*iter).node_in_contact_with.begin(); iter2 != (*iter).node_in_contact_with.end(); ++iter2) {
					if ((((destinationAddress == Ipv4Address("9.0.0.1")) && ((*iter2) < highestHotSpot) && ((*iter2) > Ipv4Address("50.0.0.0"))) || (destinationAddress == (*iter2))) && (Simulator::Now().GetMilliSeconds() < (*iter).t_start[i])) {
						if (numberPastContactsTemp == 0) {
							timeToDeliveryNodeInContact = (*iter).t_start[i] - Simulator::Now().GetMilliSeconds();
							maximumNumberDeliveredBundle = floor((double)((*iter).t_end[i] - (*iter).t_start[i]) / 1000 * TX_RATE_WIRELESS_LINK / BUNDLEDATASIZE);
						}
						else
							numberPastContactsTemp--;
						break;
					}
					i++;
				}
			}
			else if ((*iter).this_node_address == (m_node->GetObject<Ipv4>()->GetAddress (2, 0)).GetLocal()) {
				uint32_t i = 0;
				for (vector<Ipv4Address>::iterator iter2 = (*iter).node_in_contact_with.begin(); iter2 != (*iter).node_in_contact_with.end(); ++iter2) {
					if ((((destinationAddress == Ipv4Address("9.0.0.1")) && ((*iter2) < highestHotSpot) && ((*iter2) > Ipv4Address("50.0.0.0"))) || (destinationAddress == (*iter2))) && (Simulator::Now().GetMilliSeconds() < (*iter).t_start[i])) {
						timeToDeliveryThisNode = (*iter).t_start[i] - Simulator::Now().GetMilliSeconds();
						break;
					}
					i++;
				}
			}
		}
		if (timeToDeliveryThisNode <= timeToDeliveryNodeInContact)
			return false;
		else {
			if (destinationAddress == Ipv4Address("9.0.0.1")) {
				if ((bufferOccupancyNodeInContactWithTemp[nColdSpots]) >= maximumNumberDeliveredBundle) {
					numberPastContacts++;
					bufferOccupancyNodeInContactWithTemp[nColdSpots] -= maximumNumberDeliveredBundle;
				}
				else {
					bufferOccupancyNodeInContactWith[nColdSpots]++;
					return true;
				}
			}
			else {
				destinationAddress.Serialize(destAddr);
				if (bufferOccupancyNodeInContactWithTemp[destAddr[3]-nHotSpots-nNanosats-1] >= maximumNumberDeliveredBundle) {
					numberPastContacts++;
					bufferOccupancyNodeInContactWithTemp[destAddr[3]-nHotSpots-nNanosats-1] -= maximumNumberDeliveredBundle;
				}
				else {
					bufferOccupancyNodeInContactWith[destAddr[3]-nHotSpots-nNanosats-1]++;
					return true;
				}
			}
		}
	}
}

void DtnApp::PrintNanosatelliteBufferOccupancy() {
	stringstream fileName;
	fileName << "/home/fabio/source/ns-3.21/Temp/Buffer_Occupancy_" << (m_node->GetObject<Ipv4>()->GetAddress (1, 0)).GetLocal() << ".txt";
	string tmp = fileName.str();
	const char* reportName = tmp.c_str();
	ofstream report;
	report.open(reportName, ios::out | ios::app | ios::binary);
	report << Simulator::Now().GetSeconds() << " " << stored_wireless_bundles.size() << "\n";
	report.close();
	Simulator::Schedule (Seconds (1.0), &DtnApp::PrintNanosatelliteBufferOccupancy, this);
}

/* 
	This  is called by ReceiveBundle when is done receiving all bundle fragments. 
	Since there is the nedd to do different things when receiving from wired or wireless
	interfaces, the code needs to be separated.
*/
void DtnApp::ProcessBundleFromTCPSocket (Ipv4Address previousHopAddress, uint32_t previousHopPort, Ptr<Packet> receivedBundle) {
	FindDestination(receivedBundle);
	mypacket::BndlHeader bndlHeader;
	receivedBundle->PeekHeader(bndlHeader);
	DeleteActiveSocketEntry(previousHopAddress, previousHopPort, 0);
}

//See ProcessBundleFromTCPSocket
void DtnApp::ProcessBundleFromUDPSocket (Ipv4Address previousHopIPAddress, uint32_t previousHopPort, Ptr<Packet> receivedBundle) {
	mypacket::BndlHeader bndlHeader;
	receivedBundle->PeekHeader(bndlHeader);
	if (bndlHeader.GetBundleType() == 0) {			// Data Bundle
		CreateBundleAck(bndlHeader.GetOrigin(), bndlHeader.GetOriginSeqno());
		FindDestination(receivedBundle);
	}
	else if (bndlHeader.GetBundleType() == 1)		// Ack Bundle
		RemoveFromTransmittedBundles(bndlHeader.GetOrigin(), bndlHeader.GetOriginSeqno(), 1);
	else if (bndlHeader.GetBundleType() == 2)		// Status Bundle
		if (((m_node->GetObject<Ipv4>()->GetAddress (1, 0)).GetLocal() > Ipv4Address("10.0.0.0")) && ((m_node->GetObject<Ipv4>()->GetAddress (1, 0)).GetLocal() < Ipv4Address("11.0.0.0")))
			UpdateContactInformation(receivedBundle);
	DeleteActiveSocketEntry(previousHopIPAddress, previousHopPort, 1);
}

/*
	This function puts togheter bundle fragments and it's called on callback by the simulator
	on a receive event. 
	When an entire bundle is put togheter, ProcessBundleFromTCP/UDP is called.
*/
void DtnApp::ReceiveBundle (Ptr<Socket> socket) {
	mypacket::BndlFragmentHeader bndlFragmentHeader;
	Address sender_address;
	Ptr<Packet> receivedFragment = socket->RecvFrom(sender_address);
	InetSocketAddress inetsocketaddress = InetSocketAddress::ConvertFrom(sender_address);
	receivedFragment->RemoveHeader(bndlFragmentHeader);
	bool socketEntryFound = 0;
	if (socket->GetSocketType() == Socket::NS3_SOCK_DGRAM) {
		for (vector<SocketInfo*>::iterator iter = active_wireless_sockets.begin() ; iter != active_wireless_sockets.end(); ++iter) {
			if(((*iter)->source_ip_address == inetsocketaddress.GetIpv4()) && ((*iter)->source_port == inetsocketaddress.GetPort())) {
				socketEntryFound = 1;
				if (bndlFragmentHeader.GetCurrentFragmentNo() == ((*iter)->last_fragment_received + 1)) {
					(*iter)->last_fragment_received++;
					(*iter)->bundle->AddAtEnd(receivedFragment);
					if(bndlFragmentHeader.GetCurrentFragmentNo() == bndlFragmentHeader.GetTotalFragmentsNo())
						ProcessBundleFromUDPSocket(inetsocketaddress.GetIpv4(), inetsocketaddress.GetPort(), (*iter)->bundle->Copy());
				}
				break;
			}
		}
	}
	else {
		for (vector<SocketInfo*>::iterator iter = active_wired_sockets.begin() ; iter != active_wired_sockets.end(); ++iter) {
			if(((*iter)->source_ip_address == inetsocketaddress.GetIpv4()) && ((*iter)->source_port == inetsocketaddress.GetPort())) {
				socketEntryFound = 1;
				if (bndlFragmentHeader.GetCurrentFragmentNo() == ((*iter)->last_fragment_received + 1)) {
					(*iter)->last_fragment_received++;
					(*iter)->bundle->AddAtEnd(receivedFragment);
					if(bndlFragmentHeader.GetCurrentFragmentNo() == bndlFragmentHeader.GetTotalFragmentsNo())
						ProcessBundleFromTCPSocket(inetsocketaddress.GetIpv4(), inetsocketaddress.GetPort(), (*iter)->bundle);
				}
				break;
			}
		}
	}
	if ((socketEntryFound == 0) && (bndlFragmentHeader.GetCurrentFragmentNo() == 1)){ 	//se non ha trovato un cezzo
		if (bndlFragmentHeader.GetCurrentFragmentNo() == bndlFragmentHeader.GetTotalFragmentsNo()) {		//it is a bundle composed by a single fragment
			mypacket::BndlHeader bndlHeader;
			receivedFragment->PeekHeader(bndlHeader);
			SocketInfo* socketInfo = new SocketInfo;
			socketInfo->source_ip_address = inetsocketaddress.GetIpv4();
			socketInfo->source_port = inetsocketaddress.GetPort();
			socketInfo->last_fragment_received = 1;
			socketInfo->bundle = receivedFragment;
			if (socket->GetSocketType() == Socket::NS3_SOCK_DGRAM) {
				active_wireless_sockets.push_back(socketInfo);		// inserisco il socket in trasmissione nella lista di socket wireless attivi
				ProcessBundleFromUDPSocket(inetsocketaddress.GetIpv4(), inetsocketaddress.GetPort(), receivedFragment);
			}
			else {
				active_wired_sockets.push_back(socketInfo);		// inserisco il socket in trasmissione nella lista di socket wired attivi
				ProcessBundleFromTCPSocket(inetsocketaddress.GetIpv4(), inetsocketaddress.GetPort(), receivedFragment);
			}
		}
		else {			//it is a bundle composed by several fragments
			SocketInfo* socketInfo = new SocketInfo;
			socketInfo->source_ip_address = inetsocketaddress.GetIpv4();
			socketInfo->source_port = inetsocketaddress.GetPort();
			socketInfo->last_fragment_received = 1;
			socketInfo->bundle = receivedFragment;
			if (socket->GetSocketType() == Socket::NS3_SOCK_DGRAM)
				active_wireless_sockets.push_back(socketInfo);		// inserisco il socket in trasmissione nella lista di socket wireless attivi
			else
				active_wired_sockets.push_back(socketInfo);		// inserisco il socket in trasmissione nella lista di socket wired attivi
		}
	}
}

void DtnApp::RemoveFromTransmittedBundles (Ipv4Address sourceIPAddress, uint32_t bundleSeqNumber, uint32_t socketType) {
	mypacket::BndlHeader bndlHeader;
	if (socketType == 0) {
		for (vector<Ptr<Packet> >::iterator iter = transmitted_wired_bundles.begin() ; iter != transmitted_wired_bundles.end(); ++iter) {
			(*iter)->PeekHeader(bndlHeader);
			if((bndlHeader.GetOrigin() == sourceIPAddress) && (bndlHeader.GetOriginSeqno() == bundleSeqNumber)) {
				transmitted_wired_bundles.erase(iter);
				break;
			}
		}
	}
	else {
		for (vector<Ptr<Packet> >::iterator iter = transmitted_wireless_bundles.begin() ; iter != transmitted_wireless_bundles.end(); ++iter) {
			(*iter)->PeekHeader(bndlHeader);
			if((bndlHeader.GetOrigin() == sourceIPAddress) && (bndlHeader.GetOriginSeqno() == bundleSeqNumber)) {
				transmitted_wireless_bundles.erase(iter);
				break;
			}
		}
	}
}

//Low level network management: probably doesn't need modifications
void DtnApp::SendBundle (Ptr<Packet> transmittingBundle, RoutingEntry routingEntry) {
	InetSocketAddress dstremoteaddr("0.0.0.0", DTN_TCP_PORT);
	InetSocketAddress srcremoteaddr("0.0.0.0", 0);
	mypacket::BndlHeader bndlHeader;
	mypacket::BndlFragmentHeader bndlFragmentHeader;
	transmittingBundle->PeekHeader(bndlHeader);
	uint32_t transportHeaderSize = TCPHEADERSIZE;
	uint32_t MTUOfConnectedSocket = routingEntry.deviceForNextHopIP->GetMtu();
	srcremoteaddr.SetIpv4(routingEntry.sourceIP);
	dstremoteaddr.SetIpv4(routingEntry.nextHopIP);

	if (!routingEntry.deviceType) {
		m_socket = Socket::CreateSocket (GetNode (), TypeId::LookupByName ("ns3::TcpSocketFactory"));
		dstremoteaddr.SetPort(DTN_TCP_PORT);
		m_socket->SetAttribute ("SegmentSize", UintegerValue (MTUOfConnectedSocket - NETWORKHEADERSIZE - transportHeaderSize));
	}
	else {
		m_socket = Socket::CreateSocket (GetNode (), TypeId::LookupByName ("ns3::UdpSocketFactory"));
		dstremoteaddr.SetPort(DTN_UDP_PORT);
		transportHeaderSize = UDPHEADERSIZE;
	}
	m_socket->Bind(srcremoteaddr);
	m_socket->Connect(dstremoteaddr);
	Address socket_address;
	m_socket->GetSockName(socket_address);
	InetSocketAddress inetsocketaddress = InetSocketAddress::ConvertFrom(socket_address);

	SocketInfo* socketInfo = new SocketInfo;
	socketInfo->source_ip_address = srcremoteaddr.GetIpv4();
	socketInfo->source_port = inetsocketaddress.GetPort();
	socketInfo->sent_data = 0;
	socketInfo->bundle = transmittingBundle->Copy();
	if (!routingEntry.deviceType) {
		active_wired_sockets.push_back(socketInfo);		// inserisco il socket in trasmissione nella lista di socket wired attivi
		transmitted_wired_bundles.push_back(transmittingBundle->Copy());		// inserisco il bundle appena trasmesso nella lista di bundle inviati via link wired
	}
	else {
		active_wireless_sockets.push_back(socketInfo);		// inserisco il socket in trasmissione nella lista di socket wireless attivi
		transmitted_wireless_bundles.push_back(transmittingBundle->Copy());		// inserisco il bundle appena trasmesso nella lista di bundle inviati via link wireless
	}
	if (bndlHeader.GetBundleType() == 0) {
		stringstream fileName;
		fileName << "/home/fabio/source/ns-3.21/Temp/Sent_by_"  << (m_node->GetObject<Ipv4>()->GetAddress (1, 0)).GetLocal() << ".txt";
		string tmp = fileName.str();
		const char* reportName = tmp.c_str();
		ofstream report;
		report.open(reportName, ios::out | ios::app | ios::binary);
		report << "At time " << Simulator::Now().GetSeconds() <<" sent bundle data with sequence number " <<  bndlHeader.GetOriginSeqno() <<" from " << bndlHeader.GetOrigin() << " to " << bndlHeader.GetDst() << " through " << routingEntry.nextHopIP << "\n";
		report.close();
	}
	Ptr<Packet> fragment = 0;
	uint32_t sent_size = 0;
	uint8_t currentFragmentNo = 1;
	m_socket->SetDataSentCallback(MakeCallback(&DtnApp::CloseTxSocket, this));	// callback necessaria per chiudere il socket in trasmissione
	uint32_t fragmentSize = MTUOfConnectedSocket - NETWORKHEADERSIZE - transportHeaderSize - BUNDLEFRAGMENTHEADERSIZE;
	while (sent_size < transmittingBundle->GetSize ()) {
		bndlFragmentHeader.SetCurrentFragmentNo(currentFragmentNo);
		currentFragmentNo++;
		bndlFragmentHeader.SetTotalFragmentsNo(ceil((double)transmittingBundle->GetSize() / fragmentSize));
		fragment = transmittingBundle->CreateFragment(sent_size, min((int)fragmentSize, (int)(transmittingBundle->GetSize() - sent_size)));
		fragment->AddHeader(bndlFragmentHeader);
		m_socket->SendTo (fragment, 0, dstremoteaddr);
		sent_size += fragmentSize;
	}
}

/*
void DtnApp::SetContactEntry(ContactEntry contactEntry) {
	contactTable.push_back(contactEntry);
}
*/

void DtnApp::SetRoutingEntry(RoutingEntry entry) {
	routingTable.push_back(entry);
}

void DtnApp::Setup(Ptr<Node> node) {
	m_node = node;
}


void DtnApp::StopWirelessTransmission (Ipv4Address nodeInContactWithWirelessAddress) {
//  Simulator::Schedule (Seconds (50.0), &DtnApp::EmptyTransmittedWirelessBundles, this);
//  Simulator::Schedule (Seconds (50.0), &DtnApp::DeleteAllActiveWirelessSockets, this);
  contactInProgress = false;
}


void DtnApp::UpdateContactInformation(Ptr<Packet> bufferInfo) {
	Ipv4Address a = (m_node->GetObject<Ipv4>()->GetAddress (1, 0)).GetLocal();
	mypacket::BndlHeader bndlHeader;
	bufferInfo->RemoveHeader(bndlHeader);
	uint8_t* payload = new uint8_t[bufferInfo->GetSize()];
	bufferInfo->CopyData(payload, bufferInfo->GetSize());
	uint8_t* cSAddress = new uint8_t[4];
	uint8_t* storedBundlesSize = new uint8_t[4];
	for (uint32_t i = 0; i < (nColdSpots+1); i++) {		// estraggo le info inserite opportunamente nel payload del bundle
		copy(payload + (i * 8), payload + (i * 8) + 4, cSAddress);
		uint32_t address = cSAddress[3] | cSAddress[2]<<8 | cSAddress[1]<<16 | cSAddress[0]<<24;
		Ipv4Address coldSpotReceivedAddress = Ipv4Address(address);
		copy(payload + (i * 8) + 4, payload + (i * 8) + 8, storedBundlesSize);
		uint32_t BundlesSize = storedBundlesSize[3] | storedBundlesSize[2]<<8 | storedBundlesSize[1]<<16 | storedBundlesSize[0]<<24;
		bufferOccupancyNodeInContactWith[i] = BundlesSize;
	}
	delete [] payload;
	delete [] cSAddress;
	delete [] storedBundlesSize;
	CheckWirelessBuffer(bndlHeader.GetOrigin(), false, maximumNumberBundlesInCurrentContact);
}

int main (int argc, char *argv[])
{
	nHotSpots = 4;
	nNanosats = 16;
	nColdSpots = 8;
	nOrbits = 4;
	nRuralNodesForEachColdSpot = 2;
	nBundles = 1000;
	TOV = 3600000; //One hour in millis

	uint32_t duration = 10*86400;	// [s] 86400 (24h)

	CommandLine cmd;
	cmd.AddValue("nHotSpots", "Number of hot spots", nHotSpots);
	cmd.AddValue("nColdSpots", "Number of cold spots", nColdSpots);
	cmd.AddValue("nNanosats", "Number of nanosatellites", nNanosats);
	cmd.AddValue("nRuralNodesForEachColdSpot", "Number of rural nodes for each cold spot", nRuralNodesForEachColdSpot);
	cmd.AddValue("nBundles", "Number of bundles for each traffic flow", nBundles);
	cmd.AddValue("duration", "Duration of simulation", duration);
	cmd.Parse(argc, argv);

	//Ethernet?
	CsmaHelper csma;
	csma.SetChannelAttribute ("DataRate", StringValue ("800Mbps"));
	csma.SetChannelAttribute ("Delay", TimeValue (NanoSeconds (100)));

	// WIRELESS LINK DEFINITION

	// disable fragmentation for frames below MAX_BUNDLE_SIZE bytes
	stringstream temp;
	temp << BUNDLEDATASIZE;
	string maxBundleSize = temp.str();
	/*Wifi setting setup
		what's going on here I think is a general setup of the wireless data links
		before assigning them to the DTN nodes. Note the Uload and DLoad separation.
	 */
	Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue (maxBundleSize));
	YansWifiPhyHelper wifiPhyUL = YansWifiPhyHelper::Default ();
	wifiPhyUL.Set("TxPowerStart", DoubleValue(100));
	wifiPhyUL.Set("TxPowerEnd", DoubleValue(100));
	wifiPhyUL.Set("TxPowerLevels", UintegerValue(1));
	wifiPhyUL.Set("TxGain", DoubleValue(1.0));
	wifiPhyUL.Set("RxGain", DoubleValue(100));
	wifiPhyUL.Set("ChannelNumber", UintegerValue(1));
	YansWifiPhyHelper wifiPhyDL = YansWifiPhyHelper::Default ();
	wifiPhyDL.Set("TxPowerStart", DoubleValue(100));
	wifiPhyDL.Set("TxPowerEnd", DoubleValue(100));
	wifiPhyDL.Set("TxPowerLevels", UintegerValue(1));
	wifiPhyDL.Set("TxGain", DoubleValue(1.0));
	wifiPhyDL.Set("RxGain", DoubleValue(200));
	wifiPhyDL.Set("ChannelNumber", UintegerValue(10));
	WifiHelper wifiUL;
	WifiHelper wifiDL;
	wifiUL.SetStandard (WIFI_PHY_STANDARD_80211b);
	wifiDL.SetStandard (WIFI_PHY_STANDARD_80211b);
	YansWifiChannelHelper wifiULChannel = YansWifiChannelHelper::Default ();
	YansWifiChannelHelper wifiDLChannel = YansWifiChannelHelper::Default ();
	wifiULChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	wifiDLChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	wifiDLChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(TX_RANGE_WIRELESS_TRANSMISSION_GS_NS));
	wifiULChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(TX_RANGE_WIRELESS_TRANSMISSION_GS_NS));
	wifiPhyUL.SetChannel (wifiULChannel.Create());
	wifiPhyDL.SetChannel (wifiDLChannel.Create());
	NqosWifiMacHelper wifiULMac;
	wifiULMac.SetType ("ns3::AdhocWifiMac");
	NqosWifiMacHelper wifiDLMac;
	wifiDLMac.SetType ("ns3::AdhocWifiMac");
	wifiUL.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",StringValue ("DsssRate11Mbps"), "ControlMode",StringValue ("DsssRate11Mbps"));
	wifiDL.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",StringValue ("DsssRate11Mbps"), "ControlMode",StringValue ("DsssRate11Mbps"));

	// CENTRAL NETWORK DEFINITION
	/*
	Creating and linking together hotspots and the central node with CSMA aka Ethernet.

	From ns3 reference:
	Typically ns-3 NetDevices are installed on nodes using a net device helper. 
	The helper Install method takes a NodeContainer which holds some number of Ptr<Node>. 
	For each of the Nodes in the NodeContainer the helper will instantiate a net device,
	add a MAC address and a queue to the device and install it to the node. 
	For each of the devices, the helper also adds the device into a Container for later use by the caller. 
	This is that container used to hold the Ptr<NetDevice> which are instantiated by the device helper.
	 */
	Ptr<Node> centralNode = CreateObject<Node> ();
	Ptr<Node> centralBackgroundNode = CreateObject<Node> ();
	Ptr<Node> centralBridge = CreateObject<Node> ();
	NodeContainer hotSpotNodesContainer;
	NodeContainer backgroundNodesContainer;
	NodeContainer centralBridgesContainer;
	for (uint32_t i = 0; i < nHotSpots; i++) {
		Ptr<Node> hotSpotNode = CreateObject<Node> ();
		Ptr<Node> perifericBackgroundNode = CreateObject<Node> ();
		Ptr<Node> perifericBridge = CreateObject<Node> ();
		hotSpotNodesContainer.Add(hotSpotNode);
		backgroundNodesContainer.Add(perifericBackgroundNode);
		centralBridgesContainer.Add(perifericBridge);
	}
	NetDeviceContainer centralBridgeDevices;
	NetDeviceContainer centralHotSpotNodesDevices;
	NetDeviceContainer backgroundNodesDevices;
	NetDeviceContainer link = csma.Install(NodeContainer(centralNode, centralBridge));
	centralBridgeDevices.Add(link.Get(1));
	centralHotSpotNodesDevices.Add(link.Get(0));
	link = csma.Install(NodeContainer(centralBackgroundNode, centralBridge));
	centralBridgeDevices.Add(link.Get(1));
	backgroundNodesDevices.Add(link.Get(0));

	//Connecting hotspots through ethernet
	for (uint32_t i = 0; i < nHotSpots; i++) {
		NetDeviceContainer link = csma.Install(NodeContainer(centralBridgesContainer.Get(i), centralBridge));
		centralBridgeDevices.Add(link.Get(1));
		NetDeviceContainer perifericBridgeDevices(link.Get(0));
		link = csma.Install(NodeContainer(hotSpotNodesContainer.Get(i), centralBridgesContainer.Get(i)));
		perifericBridgeDevices.Add(link.Get(1));
		centralHotSpotNodesDevices.Add(link.Get(0));
		link = csma.Install(NodeContainer(backgroundNodesContainer.Get(i), centralBridgesContainer.Get(i)));
		perifericBridgeDevices.Add(link.Get(1));
		backgroundNodesDevices.Add(link.Get(0));
		BridgeHelper bridge;
		bridge.Install (centralBridgesContainer.Get(i), perifericBridgeDevices);
	}
	//Bridge connects multiple LAN together
	BridgeHelper bridge;
	bridge.Install (centralBridge, centralBridgeDevices);
	//This object aggregates IP/TCP/UDP functionality to existing Nodes.
	InternetStackHelper internet;
	internet.Install (centralNode);
	internet.Install (centralBackgroundNode);
	internet.Install (hotSpotNodesContainer);
	internet.Install (backgroundNodesContainer);
	//Probably helps managing ip adresses passing them like strings
	Ipv4AddressHelper ipv4;
	ipv4.SetBase("9.0.0.0", "255.255.0.0");
	ipv4.Assign (centralHotSpotNodesDevices);
	ipv4.SetBase("9.1.0.0", "255.255.0.0");
	ipv4.Assign (backgroundNodesDevices);

	// NANOSATELLITE NETWORK DEFINITION
	NodeContainer nanosatelliteNodesContainer;		//Node container contains nodes, easy
	nanosatelliteNodesContainer.Create(nNanosats);	//Create a node per every nanosatellite
	internet.Install (nanosatelliteNodesContainer);	//Install TCP/UDP + IP on every nanosatellite

	// RURAL NETWORK DEFINITION ######################
	NodeContainer coldSpotNodesContainer;				//Same as before, on cold spots
	vector<NodeContainer> ruralNetworkNodesContainers; //Why this is dynamic
	NodeContainer ruralBridgesContainer;				//Nodes
	NetDeviceContainer coldSpotWiredDevices;
	NetDeviceContainer ruralNodesDevices;
	//These nested for loops creates the numerous rural networks
	for (uint32_t i = 0; i < nColdSpots; i++) {
		Ptr<Node> coldSpotNode = CreateObject<Node> ();
		coldSpotNodesContainer.Add(coldSpotNode);
		NodeContainer ruralNetworkNodesContainer;
		Ptr<Node> ruralBridge = CreateObject<Node> ();
		ruralBridgesContainer.Add(ruralBridge);
		for (uint32_t j = 0; j < nRuralNodesForEachColdSpot; j++) {
			Ptr<Node> ruralNode = CreateObject<Node> ();
			ruralNetworkNodesContainer.Add(ruralNode);
		}
		ruralNetworkNodesContainers.push_back(ruralNetworkNodesContainer);
		NetDeviceContainer ruralNetworkDevicesContainer;
		NetDeviceContainer ruralBridgeDevicesContainer;
		link = csma.Install (NodeContainer(coldSpotNode, ruralBridge));
		ruralNetworkDevicesContainer.Add (link.Get(0));
		coldSpotWiredDevices.Add (link.Get(0));
		ruralBridgeDevicesContainer.Add (link.Get(1));
		for (uint32_t j = 0; j < nRuralNodesForEachColdSpot; j++) {
			link = csma.Install (NodeContainer(ruralNetworkNodesContainer.Get(j), ruralBridge));
			ruralNetworkDevicesContainer.Add (link.Get(0));
			ruralNodesDevices.Add (link.Get(0));
			ruralBridgeDevicesContainer.Add (link.Get(1));
		}
		BridgeHelper bridge;
		bridge.Install (ruralBridge, ruralBridgeDevicesContainer);
		internet.Install (ruralNetworkNodesContainer);
		internet.Install (coldSpotNode);
		stringstream tmp;
		tmp << (11+i) << ".0.0.0";
		string addressClass = tmp.str();
		ipv4.SetBase(Ipv4Address(addressClass.c_str()), "255.255.255.0");
		ipv4.Assign (ruralNetworkDevicesContainer);
	}

	// indirizzamento rete wireless
	//Maybe assigning an IP to every wireless link?
	NetDeviceContainer hotSpotupLinkWirelessDevices = wifiUL.Install(wifiPhyUL, wifiULMac, hotSpotNodesContainer);
	NetDeviceContainer hotSpotdownLinkWirelessDevices = wifiDL.Install(wifiPhyDL, wifiDLMac, hotSpotNodesContainer);
	NetDeviceContainer nanosatelliteupLinkWirelessDevices = wifiUL.Install(wifiPhyUL, wifiULMac, nanosatelliteNodesContainer);
	NetDeviceContainer nanosatellitedownLinkWirelessDevices = wifiDL.Install(wifiPhyDL, wifiDLMac, nanosatelliteNodesContainer);
	NetDeviceContainer coldSpotupLinkWirelessDevices = wifiUL.Install(wifiPhyUL, wifiULMac, coldSpotNodesContainer);
	NetDeviceContainer coldSpotdownLinkWirelessDevices = wifiDL.Install(wifiPhyDL, wifiDLMac, coldSpotNodesContainer);
	NetDeviceContainer hotSpotAndNanosatelliteupLinkWirelessDevices = NetDeviceContainer(hotSpotupLinkWirelessDevices, nanosatelliteupLinkWirelessDevices);
	NetDeviceContainer hotSpotAndNanosatellitedownLinkWirelessDevices = NetDeviceContainer(hotSpotdownLinkWirelessDevices, nanosatellitedownLinkWirelessDevices);
	NetDeviceContainer allUpLinkWirelessDevices = NetDeviceContainer(hotSpotAndNanosatelliteupLinkWirelessDevices, coldSpotupLinkWirelessDevices);
	NetDeviceContainer allDownLinkWirelessDevices = NetDeviceContainer(hotSpotAndNanosatellitedownLinkWirelessDevices, coldSpotdownLinkWirelessDevices);
	ipv4.SetBase("10.0.0.0", "255.0.0.0");
	Ipv4InterfaceContainer upLinkWirelessInterfaces = ipv4.Assign(allUpLinkWirelessDevices);
	ipv4.SetBase("50.0.0.0", "255.0.0.0");
	Ipv4InterfaceContainer downLinkWirelessInterfaces = ipv4.Assign(allDownLinkWirelessDevices);
	Ipv4InterfaceContainer hotSpotupLinkWirelessInterfaces;
	Ipv4InterfaceContainer hotSpotdownLinkWirelessInterfaces;
	for (uint32_t i = 0; i < nHotSpots ; i++ ) {
		hotSpotupLinkWirelessInterfaces.Add((*(upLinkWirelessInterfaces.Begin()+i)));
		hotSpotdownLinkWirelessInterfaces.Add((*(downLinkWirelessInterfaces.Begin()+i)));
	}
	Ipv4InterfaceContainer nanosatelliteupLinkWirelessInterfaces;
	Ipv4InterfaceContainer nanosatellitedownLinkWirelessInterfaces;
	for (uint32_t i = nHotSpots; i < (nHotSpots+nNanosats); i++) {
		nanosatelliteupLinkWirelessInterfaces.Add((*(upLinkWirelessInterfaces.Begin()+i)));
		nanosatellitedownLinkWirelessInterfaces.Add((*(downLinkWirelessInterfaces.Begin()+i)));
	}
	Ipv4InterfaceContainer coldSpotupLinkWirelessInterfaces;
	Ipv4InterfaceContainer coldSpotdownLinkWirelessInterfaces;
	for (uint32_t i = (nHotSpots+nNanosats); i < (nHotSpots+nNanosats+nColdSpots); i++) {
		coldSpotupLinkWirelessInterfaces.Add((*(upLinkWirelessInterfaces.Begin()+i)));
		coldSpotdownLinkWirelessInterfaces.Add((*(downLinkWirelessInterfaces.Begin()+i)));
	}

	// MOBILITY DEFINITION
	MobilityHelper mobility;
	mobility.Install(nanosatelliteNodesContainer);
	mobility.Install(hotSpotNodesContainer);
	mobility.Install(coldSpotNodesContainer);
	nanosatelliteNodesMobility = new DTNNodesMobility;
	groundStationsNodesMobility = new DTNNodesMobility;
	groundStationsNodesMobility->SetInitialPositionGroundStations(hotSpotNodesContainer, coldSpotNodesContainer, nHotSpots, nColdSpots, true);
	nanosatelliteNodesMobility->SetInitialPositionNanosatellites(nanosatelliteNodesContainer, nOrbits, true);
	Simulator::Schedule(Seconds (SAT_POSITION_UPDATE_TIME), &DTNNodesMobility::AdvancePositionNanosatellites, nanosatelliteNodesMobility, 2 * M_PI / nNanosats, nOrbits, (SAT_POSITION_UPDATE_TIME*1000), false);
	Simulator::Schedule(Seconds (SAT_POSITION_UPDATE_TIME), &DTNNodesMobility::AdvancePositionGroundStations, groundStationsNodesMobility, (SAT_POSITION_UPDATE_TIME*1000), false);

	// DTN application definitions
	TypeId udp_tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	TypeId tcp_tid = TypeId::LookupByName ("ns3::TcpSocketFactory");
	Ptr<DtnApp> app[1 + (uint32_t)nHotSpots + (uint32_t)nNanosats + (uint32_t)nColdSpots + ((uint32_t)nColdSpots * (uint32_t)nRuralNodesForEachColdSpot)];

	//CONTACT TABLE
	vector<ContactEntry> contactTable;
	ContactEntry contactEntry;
	NodeContainer allWirelessNodes;
	allWirelessNodes.Add (hotSpotNodesContainer);
	allWirelessNodes.Add (nanosatelliteNodesContainer);
	allWirelessNodes.Add (coldSpotNodesContainer);
	double t_now;
	bool start_contact[(uint32_t)nHotSpots+(uint32_t)nNanosats+(uint32_t)nColdSpots][(uint32_t)nHotSpots+(uint32_t)nNanosats+(uint32_t)nColdSpots];
	for (uint32_t i = 0; i <(nHotSpots+nNanosats+nColdSpots); i++){
		contactEntry.this_node_address = (allWirelessNodes.Get(i)->GetObject<Ipv4>()->GetAddress(2,0)).GetLocal();
		contactTable.push_back(contactEntry);
		for (uint32_t j = 0; j < (nHotSpots+nNanosats+nColdSpots); j++)
			start_contact[i][j] = false;
	}
	for (uint32_t initialcount = 1; initialcount < (duration * 100); initialcount++) { // 8640000 = [(24 x 3600 x 1000)/10ms]
		t_now = initialcount * 10;
		nanosatelliteNodesMobility->AdvancePositionNanosatellites(2 * M_PI / nNanosats, nOrbits, t_now, true);
		groundStationsNodesMobility->AdvancePositionGroundStations(t_now, true);
		for (uint32_t i = 0; i < (nHotSpots+nNanosats+nColdSpots) ; i++) {
			Ptr<MobilityModel> wirelessNode1Mobility =allWirelessNodes.Get(i)->GetObject<MobilityModel> ();
			Vector position1 = wirelessNode1Mobility->GetPosition();
			for(uint32_t j=0; j < (nHotSpots+nNanosats+nColdSpots); j++) {
				if (((i < nHotSpots) && ((j >= nHotSpots) && (j < (nHotSpots+nNanosats)))) || ((i >= (nHotSpots+nNanosats)) && ((j >= nHotSpots) && (j < (nHotSpots+nNanosats)))) || (((i >= nHotSpots) && (i < (nHotSpots+nNanosats))) && (i != j))) {			// HSs or CSs in contact with a SAT or SATs in contact with a GS or a SAT
					Ptr<MobilityModel> wirelessNode2Mobility = allWirelessNodes.Get(j)->GetObject<MobilityModel> ();
					Vector position2 = wirelessNode2Mobility->GetPosition();
					double distance = wirelessNode1Mobility->GetDistanceFrom(wirelessNode2Mobility);
					double threshold = 0;
					if (((i >= nHotSpots) && (i < (nHotSpots+nNanosats))) && ((j >= nHotSpots) && (j < (nHotSpots+nNanosats))))
						threshold = (99*TX_RANGE_WIRELESS_TRANSMISSION_NS_NS/100);
					else
						threshold = (99*TX_RANGE_WIRELESS_TRANSMISSION_GS_NS/100);
					if (distance  <=  threshold) {
						if (start_contact[i][j] == false) {
							start_contact[i][j] = true;
							contactTable[i].t_start.push_back(t_now);
							if ((j < nHotSpots) || (j >= nHotSpots+nNanosats))
								contactTable[i].node_in_contact_with.push_back(allWirelessNodes.Get(j)->GetObject<Ipv4>()->GetAddress(3,0).GetLocal());
							else
								contactTable[i].node_in_contact_with.push_back(allWirelessNodes.Get(j)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal());
						}
					}
					else {
						if (start_contact[i][j] == true) {
							stringstream contactFile;
							contactFile << "/home/fabio/Contact_Tables/Contact_Table_" << nHotSpots << "_HSs_" << nNanosats << "_SATs_" << nColdSpots << "_CSs_" << nOrbits << "_orbits.txt";
							string tmp = contactFile.str();
							const char* reportName = tmp.c_str();
							ofstream report;
							report.open(reportName, ios::out | ios::app | ios::binary);
							report.setf(ios_base::fixed);
							start_contact[i][j] = false;
							contactTable[i].t_end.push_back(t_now);
							report << contactTable[i].this_node_address << " " << *(contactTable[i].node_in_contact_with.end()-1) << " " << *(contactTable[i].t_end.end()-1) << " " << *(contactTable[i].t_start.end()-1) << " " << *((contactTable[i].t_end.end() - contactTable[i].t_start.end()) * TX_RATE_WIRELESS_LINK) << "\n";
							report.close ();
						}
					}
				}
			}
		}
	}
	groundStationsNodesMobility->SetInitialPositionGroundStations(hotSpotNodesContainer, coldSpotNodesContainer, nHotSpots, nColdSpots, false);
	nanosatelliteNodesMobility->SetInitialPositionNanosatellites(nanosatelliteNodesContainer, nOrbits, false);


	// CENTRAL NODE
	Ptr<Socket> receivingTCPSocket;
	InetSocketAddress receivingTCPSocketAddress(Ipv4Address("0.0.0.0"), 1);
	app[0] = CreateObject<DtnApp> ();
	app[0]->Setup(centralNode);
	centralNode->AddApplication(app[0]);
	app[0]->SetStartTime (Seconds (0.5));
	app[0]->SetStopTime (Seconds (duration));
	Simulator::Schedule(Seconds ((rand() % 100 + 1) / 100), &DtnApp::CheckWiredBuffer, app[0]);	// generate secret number between 1 and 10
	receivingTCPSocket = Socket::CreateSocket (centralNode, tcp_tid);
	receivingTCPSocketAddress.SetIpv4((centralNode->GetObject<Ipv4>()->GetAddress(1, 0)).GetLocal());
	receivingTCPSocketAddress.SetPort(DTN_TCP_PORT);
	receivingTCPSocket->Bind(receivingTCPSocketAddress);
	receivingTCPSocket->Listen();
	receivingTCPSocket->SetRecvCallback (MakeCallback (&DtnApp::ReceiveBundle, app[0]));
	receivingTCPSocket->SetAcceptCallback (MakeNullCallback<bool, Ptr<Socket>, const Address &> (), MakeCallback (&DtnApp::ConnectionAccept, app[0]));
	RoutingEntry routingEntry;
	for (uint32_t i = 0; i < nColdSpots; i++) {
		stringstream tmp;
		tmp << (11+i) << ".0.0.0";
		string addressClass = tmp.str();
		routingEntry.destinationEID = Ipv4Address(addressClass.c_str());
		routingEntry.destinationEIDMask = "255.255.255.0";
		routingEntry.sourceIP = "9.0.0.1";
		uint32_t j = floor(nHotSpots / nColdSpots * i + (nHotSpots / (2 * max(nHotSpots, nColdSpots))) - 0.01);
		stringstream tmp2;
		tmp2 << "9.0.0." << (j+2);
		string addressClass2 = tmp2.str();
		routingEntry.nextHopIP = Ipv4Address(addressClass2.c_str());
		routingEntry.nextHopIPMask = "255.255.255.255";
		routingEntry.deviceForNextHopIP = centralNode->GetDevice(0);
		routingEntry.deviceType = 0;
		app[0]->SetRoutingEntry(routingEntry);
	}
	/*routingEntry.destinationEID = "0.0.0.0";
  routingEntry.destinationEIDMask = "0.0.0.0";
  routingEntry.sourceIP = "9.0.0.1";
  routingEntry.nextHopIP = "9.0.0.2";
  routingEntry.nextHopIPMask = "255.255.255.255";
  routingEntry.deviceForNextHopIP = centralNode->GetDevice(0);
  routingEntry.deviceType = 0;
  app[0]->SetRoutingEntry(routingEntry);*/

	// HOT SPOTS
	Ptr<Socket> receivingUDPSocket1;
	InetSocketAddress receivingUDPSocket1Address(Ipv4Address("0.0.0.0"), 1);
	Ptr<Socket> receivingUDPSocket2;
	InetSocketAddress receivingUDPSocket2Address(Ipv4Address("0.0.0.0"), 1);
	for (uint32_t i = 0; i < nHotSpots; i++) {
		app[i + 1] = CreateObject<DtnApp> ();
		app[i + 1]->Setup(hotSpotNodesContainer.Get(i));
		hotSpotNodesContainer.Get(i)->AddApplication(app[i + 1]);
		app[i + 1]->SetStartTime (Seconds (0.5 + 0.00001 * (i + 1)));
		app[i + 1]->SetStopTime (Seconds (duration));
		Simulator::Schedule(Seconds ((rand() % 100 + 1) / 100), &DtnApp::CheckWiredBuffer, app[i + 1]);
		receivingTCPSocket = Socket::CreateSocket(hotSpotNodesContainer.Get(i), tcp_tid);
		receivingTCPSocketAddress.SetIpv4((hotSpotNodesContainer.Get(i)->GetObject<Ipv4>()->GetAddress(1, 0)).GetLocal());
		receivingTCPSocketAddress.SetPort(DTN_TCP_PORT);
		receivingTCPSocket->Bind(receivingTCPSocketAddress);
		receivingTCPSocket->Listen();
		receivingTCPSocket->SetRecvCallback (MakeCallback (&DtnApp::ReceiveBundle, app[i + 1]));
		receivingTCPSocket->SetAcceptCallback (MakeNullCallback<bool, Ptr<Socket>, const Address &> (), MakeCallback (&DtnApp::ConnectionAccept, app[i + 1]));
		receivingUDPSocket1 = Socket::CreateSocket(hotSpotNodesContainer.Get(i), udp_tid);
		receivingUDPSocket1Address.SetIpv4(hotSpotdownLinkWirelessInterfaces.GetAddress(i));
		receivingUDPSocket1Address.SetPort(DTN_UDP_PORT);
		receivingUDPSocket1->Bind(receivingUDPSocket1Address);
		receivingUDPSocket1->SetRecvCallback (MakeCallback (&DtnApp::ReceiveBundle, app[i + 1]));
		receivingUDPSocket2 = Socket::CreateSocket(hotSpotNodesContainer.Get(i), udp_tid);
		receivingUDPSocket2Address.SetIpv4(hotSpotupLinkWirelessInterfaces.GetAddress(i));
		receivingUDPSocket2Address.SetPort(DTN_UDP_PORT);
		receivingUDPSocket2->Bind(receivingUDPSocket2Address);
		routingEntry.destinationEID = "9.0.0.1";							// bundle for the central node
		routingEntry.destinationEIDMask = "255.255.255.255";
		stringstream tmp;
		tmp << "9.0.0." << (i+2);
		string addressClass = tmp.str();
		routingEntry.sourceIP = Ipv4Address(addressClass.c_str());
		routingEntry.nextHopIP = routingEntry.destinationEID;
		routingEntry.nextHopIPMask = "255.255.255.255";
		routingEntry.deviceForNextHopIP = centralHotSpotNodesDevices.Get(i+1);
		routingEntry.deviceType = 0;
		app[i + 1]->SetRoutingEntry(routingEntry);
		routingEntry.destinationEID = "50.0.0.0";							// bundle ack for nanosatellites
		routingEntry.destinationEIDMask = "255.0.0.0";
		stringstream tmp2;
		tmp2 << "50.0.0." << (i+1);
		string addressClass2 = tmp2.str();
		routingEntry.sourceIP = Ipv4Address(addressClass2.c_str());
		routingEntry.nextHopIP = "50.0.0.0";			// upload on nanosats using tx interface
		routingEntry.nextHopIPMask = "255.0.0.0";
		routingEntry.deviceForNextHopIP = hotSpotdownLinkWirelessDevices.Get(i);
		routingEntry.deviceType = 1;
		app[i + 1]->SetRoutingEntry(routingEntry);
		routingEntry.destinationEID = "0.0.0.0";							// bundle data for rural areas
		routingEntry.destinationEIDMask = "0.0.0.0";
		stringstream tmp3;
		tmp3 << "10.0.0." << (i+1);
		string addressClass3 = tmp3.str();
		routingEntry.sourceIP = Ipv4Address(addressClass3.c_str());
		routingEntry.nextHopIP = "10.0.0.0";			// upload on nanosats using tx interface
		routingEntry.nextHopIPMask = "255.0.0.0";
		routingEntry.deviceForNextHopIP = hotSpotupLinkWirelessDevices.Get(i);
		routingEntry.deviceType = 1;
		app[i + 1]->SetRoutingEntry(routingEntry);
	}

	// NANOSATELLITES
	for (uint32_t i = 0; i < nNanosats; i++) {
		app[i + 1 + (uint32_t)nHotSpots] = CreateObject<DtnApp> ();
		app[i + 1 + (uint32_t)nHotSpots]->Setup(nanosatelliteNodesContainer.Get(i));
		nanosatelliteNodesContainer.Get(i)->AddApplication(app[i + 1 + (uint32_t)nHotSpots]);
		app[i + 1 + (uint32_t)nHotSpots]->SetStartTime (Seconds (0.5 + 0.00001 * (i + 1 + (uint32_t)nHotSpots)));
		app[i + 1 + (uint32_t)nHotSpots]->SetStopTime (Seconds (duration));
		receivingUDPSocket1 = Socket::CreateSocket (nanosatelliteNodesContainer.Get(i), udp_tid);
		receivingUDPSocket1Address.SetIpv4(nanosatelliteupLinkWirelessInterfaces.GetAddress(i));
		receivingUDPSocket1Address.SetPort(DTN_UDP_PORT);
		receivingUDPSocket1->Bind(receivingUDPSocket1Address);
		receivingUDPSocket1->SetRecvCallback (MakeCallback (&DtnApp::ReceiveBundle, app[i + 1 + (uint32_t)nHotSpots]));
		receivingUDPSocket2 = Socket::CreateSocket (nanosatelliteNodesContainer.Get(i), udp_tid);
		receivingUDPSocket2Address.SetIpv4(nanosatellitedownLinkWirelessInterfaces.GetAddress(i));
		receivingUDPSocket2Address.SetPort(DTN_UDP_PORT);
		receivingUDPSocket2->Bind(receivingUDPSocket2Address);
		receivingUDPSocket2->SetRecvCallback (MakeCallback (&DtnApp::ReceiveBundle, app[i + 1 + (uint32_t)nHotSpots]));
		Simulator::Schedule(Seconds (1.0), &DtnApp::PrintNanosatelliteBufferOccupancy, app[i + 1 + (uint32_t)nHotSpots]);
		routingEntry.destinationEID = "9.0.0.1";					// bundle for the central node
		routingEntry.destinationEIDMask = "255.255.255.255";
		stringstream tmp;
		tmp << "50.0.0." << nHotSpots+i+1;
		string addressClass = tmp.str();
		routingEntry.sourceIP = Ipv4Address(addressClass.c_str());
		stringstream tmp2;
		tmp2 << "50.0.0." << nHotSpots + nColdSpots+ nNanosats + 1;
		string addressClass2 = tmp2.str();
		routingEntry.nextHopIP = Ipv4Address(addressClass2.c_str());
		routingEntry.nextHopIPMask = "255.255.255.255";
		routingEntry.deviceForNextHopIP = nanosatellitedownLinkWirelessDevices.Get(i);
		routingEntry.deviceType = 1;
		app[i + 1 + (uint32_t)nHotSpots]->SetRoutingEntry(routingEntry);
		for (uint32_t j = 0; j < nColdSpots; j++) {				// bundle for the cold spots
			stringstream tmp3;
			tmp3 << (11+j) << ".0.0.0";
			string addressClass3 = tmp3.str();
			routingEntry.destinationEID = Ipv4Address(addressClass3.c_str());
			routingEntry.destinationEIDMask = "255.255.255.0";
			routingEntry.sourceIP = Ipv4Address(addressClass.c_str());
			stringstream tmp4;
			tmp4 << "50.0.0." << nHotSpots+nNanosats+j+1;
			string addressClass4 = tmp4.str();
			routingEntry.nextHopIP = Ipv4Address(addressClass4.c_str());
			routingEntry.nextHopIPMask = "255.255.255.255";
			routingEntry.deviceForNextHopIP = nanosatellitedownLinkWirelessDevices.Get(i);
			routingEntry.deviceType = 1;
			app[i + 1 + (uint32_t)nHotSpots]->SetRoutingEntry(routingEntry);
			app[i + 1 + (uint32_t)nHotSpots]->bufferOccupancyNodeInContactWith.push_back(0);		// indica la buffer occupancy per bundle destinati ai cold spots
		}
		app[i + 1 + (uint32_t)nHotSpots]->bufferOccupancyNodeInContactWith.push_back(0);			// indica la buffer occupancy per bundle destinati al nodo centrale
		routingEntry.destinationEID = "10.0.0.0";				// bundle ack for ground stations
		routingEntry.destinationEIDMask = "255.0.0.0";
		routingEntry.sourceIP = Ipv4Address(addressClass.c_str());
		routingEntry.nextHopIP = "10.0.0.0";
		routingEntry.nextHopIPMask = "255.0.0.0";
		routingEntry.deviceForNextHopIP = nanosatelliteupLinkWirelessDevices.Get(i);
		routingEntry.deviceType = 1;
		app[i + 1 + (uint32_t)nHotSpots]->SetRoutingEntry(routingEntry);
		routingEntry.destinationEID = "50.0.0.0";				// bundle status for other nanosatellites
		routingEntry.destinationEIDMask = "255.0.0.0";
		routingEntry.sourceIP = Ipv4Address(addressClass.c_str());
		routingEntry.nextHopIP = "50.0.0.0";
		routingEntry.nextHopIPMask = "255.0.0.0";
		routingEntry.deviceForNextHopIP = nanosatelliteupLinkWirelessDevices.Get(i);
		routingEntry.deviceType = 1;
		app[i + 1 + (uint32_t)nHotSpots]->SetRoutingEntry(routingEntry);
	}

	// RURAL NETWORKS
	for (uint32_t i = 0; i < nColdSpots; i++) {
		app[i + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats] = CreateObject<DtnApp> ();
		app[i + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats]->Setup(coldSpotNodesContainer.Get(i));
		coldSpotNodesContainer.Get(i)->AddApplication(app[i + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats]);
		app[i + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats]->SetStartTime (Seconds (0.5 + 0.00001 * (i + 1 + nHotSpots + nNanosats)));
		app[i + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats]->SetStopTime (Seconds (duration));
		Simulator::Schedule(Seconds ((rand() % 100 + 1)/100), &DtnApp::CheckWiredBuffer, app[i + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats]);
		receivingUDPSocket1 = Socket::CreateSocket (coldSpotNodesContainer.Get(i), udp_tid);
		receivingUDPSocket1Address.SetIpv4(coldSpotdownLinkWirelessInterfaces.GetAddress(i));
		receivingUDPSocket1Address.SetPort(DTN_UDP_PORT);
		receivingUDPSocket1->Bind(receivingUDPSocket1Address);
		receivingUDPSocket1->SetRecvCallback (MakeCallback (&DtnApp::ReceiveBundle, app[i + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats]));
		receivingUDPSocket2 = Socket::CreateSocket (coldSpotNodesContainer.Get(i), udp_tid);
		receivingUDPSocket2Address.SetIpv4(coldSpotupLinkWirelessInterfaces.GetAddress(i));
		receivingUDPSocket2Address.SetPort(DTN_UDP_PORT);
		receivingUDPSocket2->Bind(receivingUDPSocket2Address);
		receivingUDPSocket2->SetRecvCallback (MakeCallback (&DtnApp::ReceiveBundle, app[i + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats]));
		routingEntry.destinationEID = "9.0.0.1";							// bundle data for the central node
		routingEntry.destinationEIDMask = "255.255.255.255";
		stringstream tmp;
		tmp << "10.0.0." << nHotSpots+nNanosats+i+1;
		string addressClass = tmp.str();
		routingEntry.sourceIP = Ipv4Address(addressClass.c_str());
		routingEntry.nextHopIP = "10.0.0.0";
		routingEntry.nextHopIPMask = "255.0.0.0";
		routingEntry.deviceForNextHopIP = coldSpotupLinkWirelessDevices.Get(i);
		routingEntry.deviceType = 1;
		app[i + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats]->SetRoutingEntry(routingEntry);
		routingEntry.destinationEID = "50.0.0.0";							// bundle ack for nanosatellites
		routingEntry.destinationEIDMask = "255.0.0.0";
		stringstream tmp2;
		tmp2 << "50.0.0." << nHotSpots+nNanosats+i+1;
		string addressClass2 = tmp2.str();
		routingEntry.sourceIP = Ipv4Address(addressClass2.c_str());
		routingEntry.nextHopIP = "50.0.0.0";
		routingEntry.nextHopIPMask = "255.0.0.0";
		routingEntry.deviceForNextHopIP = coldSpotdownLinkWirelessDevices.Get(i);
		routingEntry.deviceType = 1;
		app[i + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats]->SetRoutingEntry(routingEntry);
		for (uint32_t j = 0; j < nRuralNodesForEachColdSpot; j++) {
			stringstream tmp2;
			tmp2 << (11+i) << ".0.0." << (j+2);
			string addressClass2 = tmp2.str();
			routingEntry.destinationEID = Ipv4Address(addressClass2.c_str());
			routingEntry.destinationEIDMask = "255.255.255.255";
			stringstream tmp3;
			tmp3 << (11+i) << ".0.0.1";
			string addressClass3 = tmp3.str();
			routingEntry.sourceIP = Ipv4Address(addressClass3.c_str());
			routingEntry.nextHopIP = routingEntry.destinationEID;
			routingEntry.nextHopIPMask = routingEntry.destinationEIDMask;
			routingEntry.deviceForNextHopIP = coldSpotWiredDevices.Get(i);
			routingEntry.deviceType = 0;
			app[i + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats]->SetRoutingEntry(routingEntry);
			receivingTCPSocket = Socket::CreateSocket(coldSpotNodesContainer.Get(i), tcp_tid);
			receivingTCPSocketAddress.SetIpv4((coldSpotNodesContainer.Get(i)->GetObject<Ipv4>()->GetAddress(1, 0)).GetLocal());
			receivingTCPSocketAddress.SetPort(DTN_TCP_PORT);
			receivingTCPSocket->Bind(receivingTCPSocketAddress);
			receivingTCPSocket->Listen();
			receivingTCPSocket->SetRecvCallback (MakeCallback (&DtnApp::ReceiveBundle, app[i + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats]));
			receivingTCPSocket->SetAcceptCallback (MakeNullCallback<bool, Ptr<Socket>, const Address &> (), MakeCallback (&DtnApp::ConnectionAccept, app[i + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats]));
			app[j + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats + (uint32_t)nColdSpots + (i * (uint32_t)nRuralNodesForEachColdSpot)] = CreateObject<DtnApp> ();
			app[j + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats + (uint32_t)nColdSpots + (i * (uint32_t)nRuralNodesForEachColdSpot)]->Setup(ruralNetworkNodesContainers[i].Get(j));
			ruralNetworkNodesContainers[i].Get(j)->AddApplication(app[j + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats + (uint32_t)nColdSpots + (i * (uint32_t)nRuralNodesForEachColdSpot)]);
			app[j + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats + (uint32_t)nColdSpots + (i * (uint32_t)nRuralNodesForEachColdSpot)]->SetStartTime (Seconds (0.5 + 0.00001 * (i + 1 + nHotSpots + nNanosats)));
			app[j + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats + (uint32_t)nColdSpots + (i * (uint32_t)nRuralNodesForEachColdSpot)]->SetStopTime (Seconds (duration));
			Simulator::Schedule(Seconds ((rand() % 100 + 1)/100), &DtnApp::CheckWiredBuffer, app[j + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats + (uint32_t)nColdSpots + (i * (uint32_t)nRuralNodesForEachColdSpot)]);
			receivingTCPSocket = Socket::CreateSocket(ruralNetworkNodesContainers[i].Get(j), tcp_tid);
			receivingTCPSocketAddress.SetIpv4((ruralNetworkNodesContainers[i].Get(j)->GetObject<Ipv4>()->GetAddress(1, 0)).GetLocal());
			receivingTCPSocketAddress.SetPort(DTN_TCP_PORT);
			receivingTCPSocket->Bind(receivingTCPSocketAddress);
			receivingTCPSocket->Listen();
			receivingTCPSocket->SetRecvCallback (MakeCallback (&DtnApp::ReceiveBundle, app[j + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats + (uint32_t)nColdSpots + (i * (uint32_t)nRuralNodesForEachColdSpot)]));
			receivingTCPSocket->SetAcceptCallback (MakeNullCallback<bool, Ptr<Socket>, const Address &> (), MakeCallback (&DtnApp::ConnectionAccept, app[j + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats + (uint32_t)nColdSpots + (i * (uint32_t)nRuralNodesForEachColdSpot)]));
			routingEntry.destinationEID = "0.0.0.0";
			routingEntry.destinationEIDMask = "0.0.0.0";
			routingEntry.nextHopIP = Ipv4Address(addressClass3.c_str());
			routingEntry.nextHopIPMask = "255.255.255.255";
			routingEntry.sourceIP = Ipv4Address(addressClass2.c_str());
			routingEntry.deviceForNextHopIP = ruralNodesDevices.Get(i * nRuralNodesForEachColdSpot + j);
			routingEntry.deviceType = 0;
			app[j + 1 + (uint32_t)nHotSpots + (uint32_t)nNanosats + (uint32_t)nColdSpots + (i * (uint32_t)nRuralNodesForEachColdSpot)]->SetRoutingEntry(routingEntry);
		}
	}

	// READING CONTACT TABLE
	//Getting ready to read contact table file
	stringstream contactFile;
	contactFile << "/home/fabio/Contact_Tables/Contact_Table_" << nHotSpots << "_HSs_" << nNanosats << "_SATs_" << nColdSpots << "_CSs_" << nOrbits << "_orbits.txt";
	string contact = contactFile.str();
	const char* contactFileName = contact.c_str();
	ifstream contactReport;
	contactReport.open(contactFileName, ios::out | ios::app | ios::binary);
	contactReport.setf(ios_base::fixed);
	//Working vars
	double endContactTime, startContactTime;
	uint32_t volumeTraffic;
	string sourceAddress, destinationAddress;
	vector<ContactEntry> contactTable;
	ContactEntry contactEntry;
	NodeContainer allWirelessNodes;
	allWirelessNodes.Add (hotSpotNodesContainer);
	allWirelessNodes.Add (nanosatelliteNodesContainer);
	allWirelessNodes.Add (coldSpotNodesContainer);
	/*
		Add an entry to the contactTable structure for each node in the simulation, saving the RX IP.
		This because the routing algorithm begins the computation from the destination address, which
		must be an RX IP. 
	 */
	for (uint32_t i = 0; i <(nHotSpots+nNanosats+nColdSpots); i++){
		if(i < nHotSpots || i >= nHotSpots+nNanosats){
			contactEntry.this_node_address = (allWirelessNodes.Get(i)->GetObject<Ipv4>()->GetAddress(3,0)).GetLocal();
			contactTable.push_back(contactEntry);
		}
		else{
			contactEntry.this_node_address = (allWirelessNodes.Get(i)->GetObject<Ipv4>()->GetAddress(1,0)).GetLocal();
			contactTable.push_back(contactEntry);
		}
	}
	//For every line of the contact table
	while (contactReport >> sourceAddress >> destinationAddress >> endContactTime >> startContactTime >> volumeTraffic) {
		//This for handles the comparison between a string and an Ipv4Address object
		for (uint32_t i = 0; i < (nHotSpots+nNanosats+nColdSpots); i++) {
			stringstream tmp;
			if ((i >= nHotSpots) && (i < (nHotSpots+nNanosats)))
				tmp << "50.0.0." << (i+1);
			else
				tmp << "10.0.0." << (i+1);
			string address = tmp.str();
			//Actually compare the read address and if this is the right entry add the remaing information
			if (destinationAddress == address) {
				Simulator::Schedule(MilliSeconds((uint32_t)startContactTime), &DtnApp::CheckWirelessBuffer, app[i+1], Ipv4Address(destinationAddress.c_str()), true, (floor((double)((uint32_t)endContactTime - (uint32_t)startContactTime) / 1000 * TX_RATE_WIRELESS_LINK / BUNDLEDATASIZE)));
				Simulator::Schedule(MilliSeconds((uint32_t)endContactTime), &DtnApp::StopWirelessTransmission, app[i+1], Ipv4Address(destinationAddress.c_str()));
				contactTable[i].t_start.push_back(startContactTime);
				contactTable[i].t_end.push_back(endContactTime);
				contactTable[i].node_in_contact_with.push_back(Ipv4Address(sourceAddress.c_str()));
				contactTable[i].volumeTraffic.push_back(volumeTraffic);
				//Break from the for loop because we are done here
				break;
			}
		}
	}

	for (uint32_t i = 0; i < (nHotSpots + nNanosats + nColdSpots + 1); i++)
		app[i]->contactTable = contactTable;

	Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/AckTimeout", TimeValue (MilliSeconds (100)));

	/*
	This is almost the end of the main: the remaining thing to do is to Schedule the first 
	occurence of the Simulation in order to start it. Specifically creating Bundles, according to the simlation needs.
	 */
	// Bundle transmission
	for (uint32_t count = 1; count <= nBundles; count++) {
		Simulator::Schedule(Seconds (count), &DtnApp::CreateBundleData, app[0], "11.0.0.2");
		Simulator::Schedule(Seconds (count), &DtnApp::CreateBundleData, app[0], "12.0.0.2");
		Simulator::Schedule(Seconds (count), &DtnApp::CreateBundleData, app[0], "13.0.0.2");
		Simulator::Schedule(Seconds (count), &DtnApp::CreateBundleData, app[0], "14.0.0.2");
		//Simulator::Schedule(Seconds (count), &DtnApp::CreateBundleData, app[(uint32_t)nHotSpots+(uint32_t)nNanosats+(uint32_t)nColdSpots+1], "9.0.0.1");
	}

	NodeContainer allIPNodes = NodeContainer(NodeContainer(centralNode, centralBackgroundNode), hotSpotNodesContainer, backgroundNodesContainer, nanosatelliteNodesContainer, coldSpotNodesContainer);
	for (uint32_t i = 0; i < nColdSpots; i++)
		allIPNodes.Add(ruralNetworkNodesContainers[i]);
	PopulateArpCache(allIPNodes);
	Simulator::Stop (Seconds (duration));
	Simulator::Run ();
	Simulator::Destroy ();
	return 0;
}

//Low level simulation related
void PopulateArpCache (NodeContainer allIPNodes) {
	Ptr<ArpCache> arp = CreateObject<ArpCache> ();
	arp->SetAliveTimeout (Seconds(3600 * 24 * 365));
	for (NodeContainer::Iterator i = allIPNodes.Begin(); i != allIPNodes.End(); ++i) {
		Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
		NS_ASSERT(ip !=0);
		ObjectVectorValue interfaces;
		ip->GetAttribute("InterfaceList", interfaces);
		for(uint32_t j = 0; j != ip->GetNInterfaces (); j ++) {
			Ptr<Ipv4Interface> ipIface = ip->GetInterface (j);
			NS_ASSERT(ipIface != 0);
			Ptr<NetDevice> device = ipIface->GetDevice();
			NS_ASSERT(device != 0);
			Mac48Address addr = Mac48Address::ConvertFrom(device->GetAddress ());
			for(uint32_t k = 0; k < ipIface->GetNAddresses (); k ++) {
				Ipv4Address ipAddr = ipIface->GetAddress (k).GetLocal();
				if(ipAddr == Ipv4Address::GetLoopback())
					continue;
				ArpCache::Entry * entry = arp->Add(ipAddr);
				entry->MarkWaitReply(0);
				entry->MarkAlive(addr);
			}
		}
	}
	for (NodeContainer::Iterator i = allIPNodes.Begin(); i != allIPNodes.End(); ++i) {
		Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
		NS_ASSERT(ip !=0);
		ObjectVectorValue interfaces;
		ip->GetAttribute("InterfaceList", interfaces);
		for(uint32_t j = 0; j != ip->GetNInterfaces (); j++) {
			Ptr<Ipv4Interface> ipIface = ip->GetInterface (j);
			ipIface->SetAttribute("ArpCache", PointerValue(arp));
		}
	}
}
