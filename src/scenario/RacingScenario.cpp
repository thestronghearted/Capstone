/*
 * @(#) RacingScenario.cpp   1.0   Mar 13, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani, Joshua Auerbach
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#include "config/RobogenConfig.h"
#include "config/StartPositionConfig.h"
#include "scenario/Environment.h"
#include "scenario/RacingScenario.h"
#include "Robot.h"
#include "Models.h"

namespace robogen {

RacingScenario::RacingScenario(boost::shared_ptr<RobogenConfig> robogenConfig) :
		Scenario(robogenConfig), curTrial_(0) {
}

RacingScenario::~RacingScenario() {

}

bool RacingScenario::setupSimulation() {

	// Compute robot start position,
	startPosition_.push_back(this->getCurrentStartPosition()->getPosition());

	return true;

}

bool RacingScenario::afterSimulationStep() {

	return true;
}

bool RacingScenario::endSimulation() {

	// Compute robot ending position from its closest part to the origin
	double minDistance = std::numeric_limits<double>::max();
	const std::vector<boost::shared_ptr<Model> >& bodyParts = this->getRobots()[0]->getBodyParts();
	for (unsigned int i = 0; i < bodyParts.size(); ++i) {
		osg::Vec2 curBodyPos = osg::Vec2(bodyParts[i]->getRootPosition().x(), bodyParts[i]->getRootPosition().y());
		osg::Vec2 curDistance = startPosition_[startPosition_.size()-1] - curBodyPos;
		if (curDistance.length() < minDistance) {
			minDistance = curDistance.length();
		}
	}

	distances_.push_back(minDistance);
	curTrial_++;
	// Set next starting position
	this->setStartingPosition(curTrial_);
	return true;

}

double RacingScenario::getFitness() {
	double fitness = 1000000;
	for (unsigned int i = 0; i < distances_.size(); ++i) {
		if (distances_[i] < fitness)
			fitness = distances_[i];
	}

	return fitness;
}

bool RacingScenario::remainingTrials() {
	boost::shared_ptr<StartPositionConfig> startPos = this->getRobogenConfig()->getStartingPos();
	return curTrial_ < startPos->getStartPosition().size();
}

int RacingScenario::getCurTrial() const {
	return curTrial_;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * Called when determining the fitness scores of each robot in a swarm.
 * Calculates the fitness score of each robot in a swarm for the racing scenario.
 * The score is calculated by comparing the distance moved by a robot during the amount of time 
 * the simulation was active
 */
std::vector<double> RacingScenario::getFitnessLevel()
{
	std::vector<double> fitnesses;
	for (unsigned int x = 0;x<numberOfRobots;x++)
	{
		double fitness = 1000000;
		for (unsigned int i = 0; i < 1; ++i) {
			if (distances_[x] < fitness)
				fitness = distances_[x];
		}
		fitnesses.push_back(fitness);
	}
	return fitnesses;
}

/**
 * Called when the simulation is about to start.
 * Creates the setup for the simulation of each robot in a swarm, which is stored in the robots robotMessage.
 */
bool RacingScenario::setupSimulations(int numberofRobots) {
	numberOfRobots = numberofRobots;
	// Compute robot start position,
	for (unsigned int i = 0;i<numberOfRobots;i++)
	{
		std::vector<osg::Vec2> startPositiontemp;
		startPositiontemp.push_back(this->getRobogenConfig()->getStartingPos()->getStartPosition(i)->getPosition());	
		startPositions_.push_back(startPositiontemp);
	}
	return true;

}

/**
 * Called when the simulation is completed or cancelled.
 * Ends the simulation of the swarm.
 */
bool RacingScenario::endSimulations() {
    
	for (unsigned int x = 0; x<numberOfRobots; x++)
	{
		// Compute robot ending position from its closest part to the origin
		double minDistance = std::numeric_limits<double>::max();
		const std::vector<boost::shared_ptr<Model> >& bodyParts = this->getRobots()[x]->getBodyParts();
		for (unsigned int i = 0; i < bodyParts.size(); ++i) {
			osg::Vec2 curBodyPos = osg::Vec2(bodyParts[i]->getRootPosition().x(), bodyParts[i]->getRootPosition().y());
			osg::Vec2 curDistance = startPositions_[x][startPositions_[x].size()-1] - curBodyPos;
			if (curDistance.length() < minDistance) {
				minDistance = curDistance.length();
			}
		}
		distances_.push_back(minDistance);
		curTrial_++;
		// Set next starting position
		this->setStartingPosition(curTrial_);
	}	
	return true;

}





}
