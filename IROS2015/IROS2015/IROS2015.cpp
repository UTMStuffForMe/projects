// IROS2015.cpp : Defines the entry point for the console application.
//

/******************************************************************************
| This contains the code used for the submission to IROS 2015 entitled		  |
| "Learning to Trick Cost-Based Planners into Cooperative Behavior"		      |
|																			  |
| Redistributables are also included, as well as past experimental data.	  |
|																			  |
| Create a new stat_results file for output if desired.						  |
|																			  |
******************************************************************************/

// for memory leak detection



#ifdef _WIN32
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#include <tchar.h>
#include <direct.h>// if windows use direct.h if linux use unistd.h
#else
#include <sys/stat.h>
#endif
// warning disabling
//#pragma warning(push) 
//#pragma warning(disable:4996) 

// Parallelization
//#include <omp.h>

// Standard includes
#include <stdio.h>

// Project-specific includes
#include "../../../libraries/Simulation/SimTypeNE.h"
#include "../../../libraries/Domains/UTM/UTMDomainDetail.h"
#include "../../../libraries/Domains/UTM/UTMDomainAbstract.h"

/*void loopOverTypeHandling(){
	// TODO: DIRECTORY NAMING CONVENTION
	//std::string dirname = "stat_results/";
	//_mkdir(dirname.c_str());

	//string dir = EXPERIMENT_FOLDER+dirname;

	std::string rwd_names[MultiagentTypeNE::TypeHandling::NMODES] = {
		"blind_reward-",
		"weighted_reward-",
		"crossweighted_reward-",
		"multimind_reward-",
	};

	std::string conflict_names[MultiagentTypeNE::TypeHandling::NMODES] = {
		"blind_conflict-",
		"weighted_conflict-",
		"crossweighted_conflict-",
		"multimind_conflict-",
	};

	for (int i=0; i<MultiagentTypeNE::NMODES; i++){
		for (int r=0; r<5; r++){
			printf("************* RUN %i STARTING ***********\n",r);
			printf("mode type %i started. ", i);
			UTMDomainAbstract* domain = new UTMDomainAbstract();

			NeuroEvoParameters* NE_params = new NeuroEvoParameters(domain->n_state_elements,domain->n_control_elements);
			MultiagentTypeNE* MAS = new MultiagentTypeNE(domain->n_agents, NE_params, MultiagentTypeNE::TypeHandling(i),domain->n_types);

			SimTypeNE sim(domain, MAS, MultiagentTypeNE::TypeHandling(i));
			sim.runExperiment();

			sim.outputRewardLog(rwd_names[i]+to_string(r)+".csv");
			sim.outputMetricLog(conflict_names[i]+to_string(r)+".csv");
			delete ((UTMDomainAbstract*)domain);
		}
	}
}*/


vector<int> consecutive(int a, int b){
	// inclusive
	vector<int> v;
	for (; a<=b; a++) v.push_back(a);
	return v;
}

void loopOverDomainParameters(void modeChanger(UTMModes*, int val), int nparams, UTMModes* modes){
	vector<int> vals = consecutive(0,nparams-1); // meant for use with enums
	for (int val : vals){
		for (int r=0; r<10; r++){
			printf("RUN %i STARTING \n",r);
			//srand(uint(time(NULL)));
			//srand(1);

			modeChanger(modes, val);
			UTMDomainAbstract* domain = new UTMDomainAbstract(modes);
			NeuroEvoParameters* NE_params = new NeuroEvoParameters(domain->n_state_elements,domain->n_control_elements);
			MultiagentTypeNE* MAS = new MultiagentTypeNE(domain->n_agents, NE_params, MultiagentTypeNE::BLIND,domain->n_types);

			SimTypeNE sim(domain, MAS, MultiagentTypeNE::BLIND);
			sim.runExperiment();

			sim.outputMetricLog(MAS->type_file_name(), r);
			
			delete (UTMDomainAbstract*)domain;
			delete NE_params;
			delete MAS;
		}
	}
}

void rwdChanger(UTMModes* modes, int rwd){
	modes->_reward_mode = UTMModes::RewardMode(rwd);
}


void loopOverRewardTypes(){
	UTMModes* params = new UTMModes();
	loopOverDomainParameters(rwdChanger, 1,params);
	delete params;
}

void detailedSim(){
	srand(uint(time(NULL)));
	UTMDomainDetail* domain = new UTMDomainDetail();

	NeuroEvoParameters* NE_params = new NeuroEvoParameters(domain->n_state_elements,domain->n_control_elements);
	MultiagentTypeNE* MAS = new MultiagentTypeNE(domain->n_agents, NE_params, MultiagentTypeNE::BLIND,domain->n_types);

	SimTypeNE sim(domain, MAS, MultiagentTypeNE::BLIND);
	sim.runExperiment();

	sim.outputMetricLog("detailsim.csv");
	delete ((UTMDomainAbstract*)domain);
}

#ifdef _WIN32
int _tmain(int argc, _TCHAR* argv[]){
#else
int main(){
#endif
	loopOverRewardTypes();
	_CrtDumpMemoryLeaks(); // memory leak checking
	std::system("pause");
	return 0;
}
