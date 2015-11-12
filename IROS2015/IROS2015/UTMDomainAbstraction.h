#pragma once
#include "c:\users\carrie\documents\visual studio 2012\projects\libraries\domains\atfmsectordomain\atfmsectordomain.h"
class UTMDomainAbstraction :
	public ATFMSectorDomain
{
public:
	UTMDomainAbstraction(bool deterministic);
	~UTMDomainAbstraction(void);
	enum CongestionMode{SECTOR, LINK}; // setting
	int cmode; // the mode for congestion accounting

	std::vector<std::vector<int> > edge_time; // time that you need to be on an edge
	
	matrix2d connection_time; // time of UAV for each connection (regardless of type)
	matrix3d connection_capacity; // capacity for each type of UAV [connection][connection][type]
	
	matrix1d getDifferenceReward();
	double getDifferenceReward(int i); // get a specific difference reward for an agent
	double getGlobalReward();
		
	enum {GLOBAL, DIFFERENCE};
	int rewardType;
	matrix2d sectorTraffic; //[sector][epoch]

	virtual matrix1d getPerformance();
	virtual matrix1d getRewards();
	virtual void incrementUAVPath();
	virtual void detectConflicts();
	virtual void getPathPlans();
	virtual void getPathPlans(std::list<std::shared_ptr<UAV> > &new_UAVs);
	virtual void exportLog(std::string fid, double G);
	virtual void reset();
};

