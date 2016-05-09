/*
 * SimulationConfigurator.h
 *
 *  Created on: 04/mag/2016
 *  Author: Luca Olivieri
 *
 *      This code handles all input and outputs of the simulation.
 *      This also handles all parameters
 */

#ifndef SIMULATIONCONFIGURATOR_H_
#define SIMULATIONCONFIGURATOR_H_
/*
#include <string>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include "time.h"
#include <string>
#include <fstream>
#include <sstream>
 */

#include <stdlib.h>
#include <stdio.h>

class SimulationConfigurator
{
public:
	//All get of fields below
	bool LoadConfiguration(const char * path);

private:
	uint32_t nHotSpots;
	uint32_t nNanosats;
	uint32_t nColdSpots;
	uint32_t nRuralNodesForEachColdSpot;
	uint32_t nOrbits;
	uint32_t duration;

	char * inputFiles;
	char * tempFiles;
	char * outputFiles;
	char * simulationName;

	char * createSimulationName();
};




#endif /* SIMULATIONCONFIGURATOR_H_ */
