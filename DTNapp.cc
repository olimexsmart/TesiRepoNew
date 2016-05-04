/*
 * DTNapp.cc
 *
 *  Created on: 04/mag/2016
 *      Author: Fabio Patrone
 *      Contributors: Luca Olivieri
 */

#include "DTNapp.h"

DtnApp::DtnApp() //Constructor
: m_node(NULL),
  m_socket (NULL),
  contactInProgress(false),
  transmissionInProgress(false),
  dataSentDuringThisContact(0),
  nodeInContactWithTxAddress((uint32_t)0),
  nodeInContactWithRxAddress((uint32_t)0),
  maximumNumberBundlesInCurrentContact(0)
{ }


DtnApp::~DtnApp() //Destructor
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
		//If true, overwrite nexthop ip, reading it from the path vector
		if((m_node->GetObject<Ipv4>()->GetAddress (1, 0)).GetLocal() == "9.0.0.1")
			routingEntryFound.nextHopIP = bndlHeader.GetPathVector()[0].GetNodeAddress();


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
	contact is still possible, is scheduled again from FindDestination().

	As for now, the new logic is implemented for Data type bundle, the other types are processed
	like before to speed up the work.
	The RoutingEntry construct is not quite useful any more, but is still used to avoid extensive
	modifications to the code.
 */
void DtnApp::CheckWirelessBuffer(Ipv4Address nodeInContactWithWirelessAddress, uint32_t maximumNumberBundlesInCurrentContact) {

	bool send = false;

	if (stored_wireless_bundles.size() != 0) {
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
				 * If - else that does divide the code on the type of bundle processed
				 * Overwrite nexthop reading it from the path vector
				 * the correct vector entry is found firstly finding my ip and then getting the next element
				 * then check if this element is corresponding to nodeInContactWithWirelessAddress && Now() > m_contactTime --- if true, send =  true;
				 *
				 * if not, continue;
				 */
				if(!bundleNotData){
					//Find the next hop entry in the path vector and save it
					//The address of the node relative of this function call is obtained through this NICE looking expression
					//If decision: 1 if hotspot or nanosat is the caller, 3 if coldspot
					Ipv4Address thisAddress = (m_node->GetObject<Ipv4>()->GetAddress (2, 0)).GetLocal();
					if(GetNodeType(thisAddress) == 1 || GetNodeType(thisAddress) == 2)
						thisAddress = (m_node->GetObject<Ipv4>()->GetAddress (1, 0)).GetLocal();
					else
						thisAddress = (m_node->GetObject<Ipv4>()->GetAddress (3, 0)).GetLocal();

					//Read all the path vector and search for this node address, it must be there
					for (int i = 0; i < bndlHeader.GetPathVector().size(); i++) {
						if(bndlHeader.GetPathVector()[i].GetNodeAddress() == thisAddress)
						{	//When found, verify if the next hop is nodeInContactWithWirelessAddress and the time is coherent with this contact
							if(bndlHeader.GetPathVector()[i + 1].GetNodeAddress() == nodeInContactWithWirelessAddress && bndlHeader.GetPathVector()[i + 1].GetContactTime() <= Simulator::Now())
							{
								routingEntryFound.nextHopIP = bndlHeader.GetPathVector()[i].GetNodeAddress();
								send = true;
								break;
								/*
								 * On this break the bundle is sent to next node.
								 * When this if fails, the send boolean is still on false,
								 * the bundle is not sent and the main for
								 * goes on the next bundle in the wireless buffer
								 */
							}
							else
								break; //On this break nothing is sent
						}
					}
				}
				else
				{
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

					if (((thisNodeAddress[3] > nHotSpots) && (thisNodeAddress[3] <= (nHotSpots+nNanosats))) && ((nodeInContactAddress[3] > nHotSpots) && (nodeInContactAddress[3] <= (nHotSpots+nNanosats)))) {
						if (((dataSentDuringThisContact + 1) <= maximumNumberBundlesInCurrentContact) && (bndlHeader.GetBundleType() == 0))
							send = isTransmissionPossibleIntersatellite(nodeInContactWithWirelessAddress, bndlHeader.GetDst());
						else if (bndlHeader.GetBundleType() != 0)
							send = true;
					}
					else if (((routingEntryFound.nextHopIP == nodeInContactWithWirelessAddress) || (routingEntryFound.nextHopIP == bndlHeader.GetDst())) && ((dataSentDuringThisContact + 1) <= maximumNumberBundlesInCurrentContact))
						send = true;
					//End of ack management
					delete [] nodeInContactAddress;
					delete [] thisNodeAddress;
				}

				if (send) {
					SendBundle((*iter)->Copy(), routingEntryFound);
					uint32_t bundleSize = (*iter)->GetSize();
					sent = true;
					transmissionInProgress = true;
					if (bndlHeader.GetBundleType() == 0)
						Simulator::Schedule (Seconds ((double) bundleSize / TX_RATE_WIRELESS_LINK), &DtnApp::CheckWirelessBuffer, this, nodeInContactWithWirelessAddress, maximumNumberBundlesInCurrentContact);
					stored_wireless_bundles.erase(iter);
					break;
				}

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

	SourceContactGraphRouting(allPaths, contactTable, destinationAddress, TOV, SOB, untilHere);

	return ChoosePath(allPaths);
}

/*
	The SCGR data structure is a vector of path vectors.
	This function will recursively call itself filling the
	SCGR with all possible paths. Lastly the final decision on the
	path will be taken by a separate function.
 */
void DtnApp :: SourceContactGraphRouting(vector< vector<mypacket::BndlPath> > &allPaths, vector<ContactEntry> contactTable, Ipv4Address destAddress, double TOV, uint32_t SOB, vector<mypacket::BndlPath> untilHere){
	//Routine here to find the entry in the contactTable starting from an IP address
	int thisNode; //Holds the index of the contact table corresponding to the node considered in this call
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
		newPath = untilHere; //Reset the path at every hop selection, be sure this simply doesn't create a reference copy
		if(CheckContact(contactTable[thisNode].volumeTraffic[k], SOB)) //Check if this contact is usable
		{	/*
		      There are two IP interfaces, we must refer only to the RX one.
			  Hops between nanosats involve a TX IP (50.0.0.X) and a RX IP (10.0.0.X),
			  We are reading the contact table as an RX IP because we are searching a path
			  starting from the destination node. Because of this, the next hop address is a
			  TX IP. That's not good because for the next call of SCGR or to save it in the
			  path vector we need a RX IP. So here we are ADDRESSING ;) this issue.
			*/
			Ipv4Address nextHop = contactTable[thisNode].node_in_contact_with[k];
			uint8_t* addr = new uint8_t[4];
			nextHop.Serialize(addr);
			if(addr[0] == 50)
				addr[0] = 10;
			else if(addr[0] == 10) //Else if not needed, but I don't care.
			{	//If a HotSpot, we need to write down the wired interface.
				//The time of contact has no meaning in this case and it's not read anywhere.
				addr[0] = 9;
				addr[3] = addr[3] + 1;
			}
			uint32_t destAddress = addr[3] | addr[2]<<8 | addr[1]<<16 | addr[0]<<24;
			nextHop = Ipv4Address(destAddress);

			//Setting up the next hop structure to adding it at the end of the newPath
			mypacket::BndlPath pathEntry = mypacket::BndlPath(contactTable[thisNode].t_start[k], nextHop);
			newPath.insert(newPath.begin(), pathEntry);  //This should produce a vector beginning with an HS and ending with a CS

			//This gets to recognize the type of node, if == 1 it's a hotspot
			//If it's a two is a nanosatellite, recall this function
			if(GetNodeType(contactTable[thisNode].node_in_contact_with[k]) == 1)
				allPaths.push_back(newPath);
			else if(GetNodeType(contactTable[thisNode].node_in_contact_with[k]) == 2)
				SourceContactGraphRouting(allPaths, contactTable, nextHop, contactTable[thisNode].t_start[k], SOB, newPath);
		}
	}
}

/*
	In the need of recognizing easily the type of node we are dealing with,
	this function will take an IP address.
	Keep in mind this needs to know the configuration of the network.
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
	int minT = allPaths[0][0].GetContactTime();
	int minI = 0;

	for(int i = 0; i < allPaths.size(); i++)
	{
		if(allPaths[i][0].GetContactTime() < minT)
		{
			minT = allPaths[i][0].GetContactTime();
			minI = i;
		}
	}

	return allPaths[minI];
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
		fileName <<  "/home/tesista/ns-allinone-3.21/ns-3.21/Temp/Received_by_" << (m_node->GetObject<Ipv4>()->GetAddress (1, 0)).GetLocal() << ".txt";
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
	fileName << "/home/tesista/ns-allinone-3.21/source/ns-3.21/Temp/Buffer_Occupancy_" << (m_node->GetObject<Ipv4>()->GetAddress (1, 0)).GetLocal() << ".txt";
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
		fileName << "/home/tesista/ns-allinone-3.21/ns-3.21/Temp/Sent_by_"  << (m_node->GetObject<Ipv4>()->GetAddress (1, 0)).GetLocal() << ".txt";
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
	CheckWirelessBuffer(bndlHeader.GetOrigin(), maximumNumberBundlesInCurrentContact);
}



