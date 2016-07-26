// IROS2015.cpp : Defines the entry point for the console application.
// Copyright 2016 Carrie Rebhuhn

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
#include <direct.h>  // if windows use direct.h if linux use unistd.h
#else
#include <sys/stat.h>
#endif
// warning disabling
// #pragma warning(push)
// #pragma warning(disable:4996)

// Parallelization
// #include <omp.h>

// Standard includes
#include <stdio.h>
#include <vector>

// Project-specific includes
//#include "../../../libraries/Simulation/SimTypeNE.h"
#include "Simulation/SimNE.h"
#include "Domains/UTM/Detail/UTMDomainDetail.h"
#include "Domains/UTM/UTMDomainAbstract.h"

using std::vector;


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


vector<int> consecutive(int a, int b) {
    // inclusive
    vector<int> v;
    for (; a <= b; a++) v.push_back(a);
    return v;
}

void abstract_UTM_simulation(UTMModes* modes, int r) {
    UTMDomainAbstract* domain = new UTMDomainAbstract(modes);

    size_t n_inputs = domain->n_state_elements;        // # nn inputs
    size_t n_outputs = domain->n_control_elements;     // # nn outputs
    NeuroEvoParameters* NE_params;
    NE_params = new NeuroEvoParameters(n_inputs, n_outputs);

    size_t n_agents = domain->n_agents;
    MultiagentNE* MAS = new MultiagentNE(n_agents, NE_params);

    SimNE sim(domain, MAS);
    sim.runExperiment();

    sim.outputMetricLog("metrics.csv", r);

    delete domain;
    delete NE_params;
    delete MAS;
}

void abstract_UTM_simulation_difference(UTMModes* modes, int r) {
    UTMDomainAbstract* domain = new UTMDomainAbstract(modes);

    size_t n_inputs = domain->n_state_elements;        // # nn inputs
    size_t n_outputs = domain->n_control_elements;     // # nn outputs
    NeuroEvoParameters* NE_params;
    NE_params = new NeuroEvoParameters(n_inputs, n_outputs);

    size_t n_agents = domain->n_agents;
    MultiagentNE* MAS = new MultiagentNE(n_agents, NE_params);

    SimNE sim(domain, MAS);
   // sim.runExperimentDifference();
    sim.runExperiment();
    sim.outputMetricLog("metrics.csv", r);

    delete domain;
    delete NE_params;
    delete MAS;
}

void loopOverDomainParameters(void modeChanger(UTMModes*, int val),
    int nparams, UTMModes* modes) {
    vector<int> vals = consecutive(0, nparams - 1);  // meant for use with enums
    for (int val : vals) {
        for (int r = 0; r < 10; r++) {
            printf("RUN %i STARTING \n", r);
            modeChanger(modes, val);
            abstract_UTM_simulation(modes,r);
        }
    }
}

void rwdChanger(UTMModes* modes, int rwd) {
    modes->_reward_mode = UTMModes::RewardMode(rwd);
}

void domainChanger(UTMModes* modes, int domain_num) {
    modes->domain_num = domain_num;
}

void loopOverRewardTypes() {
    UTMModes* params = new UTMModes();
    loopOverDomainParameters(rwdChanger, 1, params);
    delete params;
}

void loopOverDomains(int n_domains) {
    UTMModes* params = new UTMModes();
    loopOverDomainParameters(domainChanger, n_domains, params);
    delete params;
}

void detailedSim(UTMModes* modes) {
    srand(size_t(time(NULL)));
    UTMDomainDetail* domain = new UTMDomainDetail(modes);

    int n_inputs = domain->n_state_elements;
    int n_outputs = domain->n_control_elements;
    NeuroEvoParameters* NE_params;
    NE_params = new NeuroEvoParameters(n_inputs, n_outputs);

    int n_agents = domain->n_agents;
    int n_types = domain->n_types;
    MultiagentNE* MAS = new MultiagentNE(n_agents, NE_params);

    SimNE sim(domain, MAS);
    sim.runExperiment();

    sim.outputMetricLog("detailsim.csv");
    delete domain;
    delete modes;
}

/*
void generateNewDomains(int n_domains) {
    UTMModes* params = new UTMModes();
    UTMFileNames* filehandler = new UTMFileNames(params);
    int n_sectors = params->get_n_sectors();
    int n_types = params->get_n_types();

    for (int i = 0; i < n_domains; i++) {
        // Generate a new airspace
        params->domain_num = i;
        std::string domain_dir = filehandler->createDomainDirectory();
        TypeGraphManager* highGraph = new TypeGraphManager(n_sectors, n_types, 200.0, 200.0);
        highGraph->print_graph(domain_dir);  // saves the graph
        delete highGraph;
    }
    delete filehandler;
    delete params;
}*/


#ifdef _WIN32
int _tmain(int, _TCHAR*) {
#else
int main() {
#endif
    UTMModes* params = new UTMModes();
    abstract_UTM_simulation_difference(params,0);
   //int n_domains = 1;
    //int n_domains = 100;
    //generateNewDomains(n_domains);
    //loopOverDomains(n_domains);
    //loopOverRewardTypes();
    //detailedSim(params);
    _CrtDumpMemoryLeaks();  // memory leak checking
    std::system("pause");
    return 0;
}
