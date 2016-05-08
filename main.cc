/*
 * main.cc
 *
 *  Created on: 04/mag/2016
 *      Author: Fabio Patrone
 *      Contributors: Luca Olivieri
 */

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
#include "DTNapp.h"

using namespace ns3;
using namespace std;


void PopulateArpCache(NodeContainer allIPNodes);


int main (int argc, char *argv[])
{
	//Simulator::EnableParallelSimulation(); just dream about it
	DTNNodesMobility* groundStationsNodesMobility;
	DTNNodesMobility* nanosatelliteNodesMobility;

	nHotSpots = 8;
	nNanosats = 42;
	nColdSpots = 16;
	nOrbits = 4;
	nRuralNodesForEachColdSpot = 2;
	nBundles = 1000;
	TOV = 3600000; //One hour in millis

	//uint32_t duration = 10*86400;	// [s] 86400 (24h)
	uint32_t duration = 86400;	// [s] 86400 (24h)

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
	bool start_contact[nHotSpots + nNanosats + nColdSpots][nHotSpots + nNanosats + nColdSpots];
	for (uint32_t i = 0; i <(nHotSpots+nNanosats+nColdSpots); i++){

		//Modify as in contact table reading, this_node_address must contain the RX IP, which is not the on same interface number across different node topologies
		for (uint32_t i = 0; i <(nHotSpots+nNanosats+nColdSpots); i++){
				if(i < nHotSpots || i >= nHotSpots+nNanosats){ //If hotspot or coldspot
					contactEntry.this_node_address = (allWirelessNodes.Get(i)->GetObject<Ipv4>()->GetAddress(3,0)).GetLocal();
					contactTable.push_back(contactEntry);
				}
				else{ //If nanosat
					contactEntry.this_node_address = (allWirelessNodes.Get(i)->GetObject<Ipv4>()->GetAddress(1,0)).GetLocal();
					contactTable.push_back(contactEntry);
				}
			}

		contactTable.push_back(contactEntry);
		for (uint32_t j = 0; j < (nHotSpots+nNanosats+nColdSpots); j++)
			start_contact[i][j] = false;
	}
	for (uint32_t initialcount = 1; initialcount < (duration * 100); initialcount++) { // 8640000 = [(24 x 3600 x 1000)/10ms] tens of milliseconds
		t_now = initialcount * 10; //milliseconds
		nanosatelliteNodesMobility->AdvancePositionNanosatellites(2 * M_PI / (double)nNanosats, nOrbits, t_now, true);
		groundStationsNodesMobility->AdvancePositionGroundStations(t_now, true);
		for (uint32_t i = 0; i < (nHotSpots+nNanosats+nColdSpots) ; i++) {
			Ptr<MobilityModel> wirelessNode1Mobility = allWirelessNodes.Get(i)->GetObject<MobilityModel> ();
			Vector position1 = wirelessNode1Mobility->GetPosition();
			for(uint32_t j=0; j < (nHotSpots+nNanosats+nColdSpots); j++) {
				if (((i < nHotSpots) && ((j >= nHotSpots) && (j < (nHotSpots+nNanosats)))) || ((i >= (nHotSpots+nNanosats)) && ((j >= nHotSpots) && (j < (nHotSpots+nNanosats)))) || (((i >= nHotSpots) && (i < (nHotSpots+nNanosats))) && (i != j))) {			// HSs or CSs in contact with a SAT or SATs in contact with a GS or a SAT
					Ptr<MobilityModel> wirelessNode2Mobility = allWirelessNodes.Get(j)->GetObject<MobilityModel> ();
					Vector position2 = wirelessNode2Mobility->GetPosition();
					double distance = wirelessNode1Mobility->GetDistanceFrom(wirelessNode2Mobility);
					//A  threshold is defined to ensure a minimum of margin: the contact time is saved as a little bit shorter, this way there is still time for a very narrow ack for example
					double threshold = 0;
					if (((i >= nHotSpots) && (i < (nHotSpots+nNanosats))) && ((j >= nHotSpots) && (j < (nHotSpots+nNanosats))))
						threshold = (99*TX_RANGE_WIRELESS_TRANSMISSION_NS_NS/100);
					else
						threshold = (99*TX_RANGE_WIRELESS_TRANSMISSION_GS_NS/100);
					if (distance  <=  threshold) {
						if (start_contact[i][j] == false) {
							start_contact[i][j] = true;
							contactTable[i].t_start.push_back(t_now);
							//Here remains only a single line with "2", the TX IP

							contactTable[i].node_in_contact_with.push_back(allWirelessNodes.Get(j)->GetObject<Ipv4>()->GetAddress(2,0).GetLocal());

						}
					}
					else {
						if (start_contact[i][j] == true) {
							stringstream contactFile;
							contactFile << "/home/tesista/Contact_Tables/Contact_Table_" << nHotSpots << "_HSs_" << nNanosats << "_SATs_" << nColdSpots << "_CSs_" << nOrbits << "_orbits.txt";
							string tmp = contactFile.str();
							const char* reportName = tmp.c_str();
							ofstream report;
							report.open(reportName, ios::out | ios::app | ios::binary);
							report.setf(ios_base::fixed);
							start_contact[i][j] = false;
							contactTable[i].t_end.push_back(t_now);
							report << contactTable[i].this_node_address << " " << *(contactTable[i].node_in_contact_with.end()-1) << " " << *(contactTable[i].t_end.end()-1) << " " << *(contactTable[i].t_start.end()-1) << " " << (long)((*(contactTable[i].t_end.end() - 1) / 1000.0 - *(contactTable[i].t_start.end() - 1) / 1000.0) * TX_RATE_WIRELESS_LINK) << " " << initialcount / duration << "%" <<"\n";
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
  routingEntry.sourceIP = "9.0.0.1";~
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
/*	stringstream contactFile;
	contactFile << "/home/tesista/Contact_Tables/Contact_Table_" << nHotSpots << "_HSs_" << nNanosats << "_SATs_" << nColdSpots << "_CSs_" << nOrbits << "_orbits.txt";
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
	//	Add an entry to the contactTable structure for each node in the simulation, saving the RX IP.
	//	This because the routing algorithm begins the computation from the destination address, which
	//	must be an RX IP.

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
	while (contactReport >> destinationAddress >> sourceAddress >> endContactTime >> startContactTime >> volumeTraffic) {
		//This for handles the comparison between a string and an Ipv4Address object
		for (uint32_t i = 0; i < (nHotSpots+nNanosats+nColdSpots); i++) {
			stringstream tmp;
			if ((i >= nHotSpots) && (i < (nHotSpots+nNanosats)))
				tmp << "10.0.0." << (i+1);
			else
				tmp << "50.0.0." << (i+1);
			string address = tmp.str();
			//Actually compare the read address and if this is the right entry add the remaining information
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
*/
	Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/AckTimeout", TimeValue (MilliSeconds (100)));

	/*
	This is almost the end of the main: the remaining thing to do is to Schedule the first 
	occurence of the Simulation in order to start it. Specifically creating Bundles, according to the simlation needs.
	 */
	// Bundle transmission
	for (uint32_t count = 1; count <= nBundles; count++) {
//		Simulator::Schedule(Seconds (count), &DtnApp::CreateBundleData, app[0], "11.0.0.2");
//		Simulator::Schedule(Seconds (count), &DtnApp::CreateBundleData, app[0], "12.0.0.2");
//		Simulator::Schedule(Seconds (count), &DtnApp::CreateBundleData, app[0], "13.0.0.2");
//		Simulator::Schedule(Seconds (count), &DtnApp::CreateBundleData, app[0], "14.0.0.2");
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
