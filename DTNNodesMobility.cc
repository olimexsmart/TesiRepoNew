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
#include <math.h>
#include <cmath>
#include "DTNNodesMobility.h"
#include <string>
#include <fstream>
#include <vector>

using namespace std;
using namespace ns3;

void DTNNodesMobility::SetInitialPositionNanosatellites(NodeContainer nanosatellitesContainer, uint32_t nOrbits, bool firstTime) {
	stringstream fileName;
	fileName << tempPath <<"COORDINATE NANOSATELLITES.txt";
	string tmp = fileName.str();
	const char* reportName = tmp.c_str();
	ofstream report;
	report.open(reportName, ios::out | ios::app | ios::binary);
	for (uint32_t orbit = 0; orbit < nOrbits; orbit++) {
		vector <DTNNodePosition> vector;
		for (uint32_t i = 0; i < (nanosatellitesContainer.GetN()/nOrbits); i++) {
			Ptr<MobilityModel> mobility = nanosatellitesContainer.Get(i + (orbit * nanosatellitesContainer.GetN() / nOrbits))->GetObject<MobilityModel> ();
			double phi =  (2 * M_PI * (i * nOrbits + orbit) / nanosatellitesContainer.GetN());
			Vector initialPosition = Vector ((RADIUS_ORBIT) * cos (phi) * cos ((M_PI * orbit / nOrbits)) - (RADIUS_ORBIT) * sin (phi) * cos(ORBIT_INCLINATION_ANGLE)* sin ((M_PI * orbit / nOrbits)), RADIUS_ORBIT * cos (phi) * sin ((M_PI * orbit / nOrbits)) + RADIUS_ORBIT * cos ((M_PI * orbit / nOrbits))* sin (phi)* cos(ORBIT_INCLINATION_ANGLE), RADIUS_ORBIT * sin (phi) * sin(ORBIT_INCLINATION_ANGLE)); //conversione coordinate
			mobility->SetPosition (initialPosition);
			DTNNodePosition dtnNodePosition;
			dtnNodePosition.node = nanosatellitesContainer.Get(i + (orbit * nanosatellitesContainer.GetN() / nOrbits));
			vector.push_back(dtnNodePosition);
			report<< " x " << floor(initialPosition.x) << "         y " <<  floor(initialPosition.y)  << "            z " << floor(initialPosition.z) << "\n";
		}
		if (firstTime)
			nodeList.push_back(vector);
	}
	report.close();
}

void DTNNodesMobility::SetInitialPositionGroundStations(NodeContainer hotSpotNodesContainer, NodeContainer coldSpotNodesContainer, double nHotSpots, double nColdSpots, bool firstTime) {
	stringstream contactFile;
	contactFile << contactTablePath << "Coordinate_" << nHotSpots << "_HSs_" << nColdSpots << "_CSs.txt";
	string contact = contactFile.str();
	const char* contactFileName = contact.c_str();
	ifstream contactReport;
	contactReport.open(contactFileName, ios::out | ios::app | ios::binary);
	contactReport.setf(ios_base::fixed);
	double latitude, longitude;
	vector <DTNNodePosition> vector;
	for (uint32_t count = 0; count < hotSpotNodesContainer.GetN(); count++) {
		contactReport >> latitude >> longitude;
		Ptr<MobilityModel> mobility = hotSpotNodesContainer.Get(count)->GetObject<MobilityModel> ();
		Vector initialPosition = Vector (RADIUS_EARTH * sin(M_PI/2 - latitude) * cos(longitude), RADIUS_EARTH * sin(M_PI/2 - latitude) * sin(longitude), RADIUS_EARTH * cos(M_PI/2 - latitude)); //conversione coordinate
		mobility->SetPosition (initialPosition);
		DTNNodePosition dtnNodePosition;
		dtnNodePosition.node = hotSpotNodesContainer.Get(count);
		dtnNodePosition.latitude = M_PI/2 - latitude;
		dtnNodePosition.longitude = longitude;
		vector.push_back(dtnNodePosition);
	}
	for (uint32_t count = 0; count < coldSpotNodesContainer.GetN(); count++) {
		contactReport >> latitude >> longitude;
		Ptr<MobilityModel> mobility = coldSpotNodesContainer.Get(count)->GetObject<MobilityModel> ();
		Vector initialPosition = Vector (RADIUS_EARTH * sin(M_PI/2 - latitude) * cos(longitude), RADIUS_EARTH * sin(M_PI/2 - latitude) * sin(longitude), RADIUS_EARTH * cos(M_PI/2 - latitude)); //conversione coordinate
		mobility->SetPosition (initialPosition);
		DTNNodePosition dtnNodePosition;
		dtnNodePosition.node = coldSpotNodesContainer.Get(count);
		dtnNodePosition.latitude = M_PI/2 - latitude;
		dtnNodePosition.longitude = longitude;
		vector.push_back(dtnNodePosition);
	}
	contactReport.close();
	if (firstTime)
		nodeList.push_back(vector);
}

void DTNNodesMobility::AdvancePositionNanosatellites(double initialphi, uint32_t nOrbits, uint32_t t_now, bool preSimulation) {
    uint32_t i = 0;
	uint32_t updateCont = t_now / (SAT_POSITION_UPDATE_TIME * 1000);
	double beta, phi;
/*	stringstream fileName;
	fileName << tempPath << "Coordinate SAT1.txt";
	string tmp = fileName.str();
	const char* reportName = tmp.c_str();
	ofstream report;
//	report.open(reportName, ios::out | ios::app | ios::binary); */
	for (uint32_t orbit = 0; orbit < nOrbits; orbit++){
		beta = (M_PI * orbit / nOrbits);
		for (vector<DTNNodePosition>::iterator iter = nodeList[orbit].begin(); iter != nodeList[orbit].end(); ++iter) {
			phi = (2 * M_PI * i / nodeList[orbit].size()) + (updateCont * SAT_POSITION_UPDATE_ANGLE) + (orbit * initialphi);
			i++;
			Ptr<MobilityModel> mobility = (*iter).node->GetObject<MobilityModel>();
			Vector updatedPosition = Vector (RADIUS_ORBIT * cos (phi) * cos (beta) - RADIUS_ORBIT  * sin (phi) * cos(ORBIT_INCLINATION_ANGLE) * sin (beta), RADIUS_ORBIT * cos (phi) * sin(beta) + RADIUS_ORBIT * cos (beta) * cos(ORBIT_INCLINATION_ANGLE)* sin(phi), RADIUS_ORBIT * sin (phi) * sin(ORBIT_INCLINATION_ANGLE)); //conversione coordinate
			mobility->SetPosition (updatedPosition);
//			if ((iter == nodeList[orbit].begin()) && (orbit == 0))
//				report<< " x " << floor(updatedPosition.x) << "         y " <<  floor(updatedPosition.y)  << "            z " << floor(updatedPosition.z) << "\n";
	   }
	   i = 0;
	}
//	report.close();
	if (!preSimulation)
		Simulator::Schedule (MilliSeconds (SAT_POSITION_UPDATE_TIME * 1000), &DTNNodesMobility::AdvancePositionNanosatellites, this, initialphi, nOrbits, (t_now + (SAT_POSITION_UPDATE_TIME*1000)), preSimulation); //10 ms
}

void DTNNodesMobility::AdvancePositionGroundStations(uint32_t t_now, bool preSimulation) {
/*	stringstream fileName;
	fileName << tempPath << "Coordinate HS1.txt";
	string tmp = fileName.str();
	const char* reportName = tmp.c_str();
	ofstream report;
//	report.open(reportName, ios::out | ios::app | ios::binary);*/
	uint32_t updateCont = t_now / (SAT_POSITION_UPDATE_TIME * 1000);
	for (vector<DTNNodePosition>::iterator iter = nodeList.begin()->begin() ; iter != nodeList.begin()->end(); ++iter) {
		Ptr<MobilityModel> mobility = (*iter).node->GetObject<MobilityModel>();
		double updatedLongitude = (updateCont * SAT_POSITION_UPDATE_BETA) + (*iter).longitude;
		Vector updatedPosition = Vector (RADIUS_EARTH * sin((*iter).latitude) * cos(updatedLongitude), RADIUS_EARTH * sin((*iter).latitude) * sin(updatedLongitude), RADIUS_EARTH * cos((*iter).latitude)); //conversione coordinate
		mobility->SetPosition (updatedPosition);
//		if (iter == nodeList.begin()->begin())
//			report<< " x " << floor(updatedPosition.x) << "         y " <<  floor(updatedPosition.y)  << "            z " << floor(updatedPosition.z) << "\n";
	}
//	report.close();
	if (!preSimulation)
		Simulator::Schedule (MilliSeconds (SAT_POSITION_UPDATE_TIME * 1000), &DTNNodesMobility::AdvancePositionGroundStations, this, (t_now + (SAT_POSITION_UPDATE_TIME*1000)), preSimulation); //10 ms
}
