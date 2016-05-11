#ifndef SATELLITEMOBILITY_H
#define SATELLITEMOBILITY_H

#include "ns3/mobility-module.h"
#include <iostream>
#include <cmath>
#include "mypacket.h"

#include "ns3/mobility-module.h"
#include <iostream>
#include <cmath>
#include "mypacket.h"

const double RADIUS_EARTH = 6371000;			// radius of the Earth [m]	6371000
const double HEIGHT_SAT = 281655;				// height of nanosats [m]	200000
const double RADIUS_ORBIT = RADIUS_EARTH + HEIGHT_SAT;				// radius of nanosats orbit from the centre of the Earth [m]

const double MASS_EARTH = 5.9736e24;			// mass of the Earth [kg]
const double GRAVITATIONAL_CONST = 6.673e-11;			// gravity constant [N*m^2/kg^2]
const double LIGHT_SPEED = 3e8;					// light speed [m/s]

const double TX_RANGE_WIRELESS_TRANSMISSION_GS_NS = 600000;		// tx range of wireless transmission ground stations - nanosatellites [m]	600000
const double TX_RANGE_WIRELESS_TRANSMISSION_NS_NS = 500000;		// tx range of wireless transmission nanosatellites - nanosatellites [m]	500000
const double TX_RATE_WIRELESS_LINK = 63052;		// transmission rate of wireless links [Bps]
const double DELAY_WIRELESS_LINK = HEIGHT_SAT / LIGHT_SPEED;	// delay of wireless links [s]
const double SAT_AVERAGE_VEL = sqrt((GRAVITATIONAL_CONST * MASS_EARTH) / RADIUS_ORBIT);	// average velocity of nanosats [m/s]
const double ORBIT_LENGTH = 2 * M_PI * RADIUS_ORBIT;			// orbit length [m]
const double ORBIT_TIME = ORBIT_LENGTH / SAT_AVERAGE_VEL;		// orbit time [s]

const double ORBIT_INCLINATION_ANGLE = 1.51; // [rad] (86.4)
const double CONTACT_LENGTH = 2 * RADIUS_ORBIT * acos((pow(RADIUS_ORBIT, 2) + pow(RADIUS_EARTH, 2) - pow(TX_RANGE_WIRELESS_TRANSMISSION_GS_NS, 2)) / (2 * RADIUS_ORBIT * RADIUS_EARTH));		// contact length [m]
const double CONTACT_ANGLE_MAX =  CONTACT_LENGTH / RADIUS_ORBIT;

const double SAT_POSITION_UPDATE_TIME = 0.01;	// time interval to update the nanosats position [s]
const double SAT_POSITION_UPDATE_SPACE = SAT_POSITION_UPDATE_TIME * SAT_AVERAGE_VEL;	// space covered to each nanosat during the update interval [m]
const double SAT_POSITION_UPDATE_ANGLE = SAT_POSITION_UPDATE_SPACE / RADIUS_ORBIT;// angle covered to each nanosat during the update interval [rad]
const double SAT_POSITION_UPDATE_BETA = M_PI / 12 * SAT_POSITION_UPDATE_TIME / 3600; // RAA updating [PI_/12 : 3600 = x : 0,01]  nanosat's position updating [m] during the update interval because of the eart's rotation

const string tempPath = "/home/olli/ns-allinone-3.21/ns-3.21/Temp/";
const string contactTablePath = "/home/olli/ns-allinone-3.21/ns-3.21/Contact_Tables/";

using namespace std;
using namespace ns3;

class DTNNodesMobility
{
  private:
	struct DTNNodePosition {
		DTNNodePosition(): latitude(0), longitude(0) {}
		Ptr<Node> node;
	    double latitude;
	    double longitude;
	};

  public:
	void SetInitialPositionNanosatellites(NodeContainer nanosatellitesContainer, uint32_t nOrbits, bool firstTime);
	void SetInitialPositionGroundStations(NodeContainer hotSpotNodesContainer, NodeContainer coldSpotNodesContainer, double nHotSpots, double nColdSpots, bool firstTime);
	void AdvancePositionNanosatellites(double initialphi, uint32_t nOrbits, uint32_t t_now, bool preSimulation);
	void AdvancePositionGroundStations(uint32_t t_now, bool preSimulation);
	vector <vector <DTNNodePosition> > nodeList;
};

#endif /* SATELLITEMOBILITY_H */
