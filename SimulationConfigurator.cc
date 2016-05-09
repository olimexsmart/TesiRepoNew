/*
 * SimulationConfigurator.cc
 *
 *  Created on: 04/mag/2016
 *  Author: Luca Olivieri
 */


#include "SimulationConfigurator.h"


SimulationConfigurator :: SimulationConfigurator(){
	nHotSpots = 1;
	nNanosats = 1;
	nColdSpots = 1;
	nRuralNodesForEachColdSpot = 1;
	nOrbits = 1;
	duration = 1;

	inputFiles = NULL;
	tempFiles = NULL;
	outputFiles = NULL;

	simulationName = NULL;
}


bool SimulationConfigurator :: LoadConfiguration(const char * path){
	FILE *f = fopen(path, "r");

	if(f == NULL) return false;

	char buffer[250];
	char c;

	while(c != EOF){
		c = fgetc(f);

	}
}

