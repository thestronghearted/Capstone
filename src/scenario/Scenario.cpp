/*
 * @(#) Scenario.cpp   1.0   Mar 13, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Joshua Auerbach
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
#include <iostream>
#include "config/RobogenConfig.h"
#include "config/TerrainConfig.h"
#include "model/objects/BoxObstacle.h"
#include "scenario/Scenario.h"
#include "scenario/Terrain.h"
#include "Robot.h"
#include "Environment.h"

namespace robogen {

Scenario::Scenario(boost::shared_ptr<RobogenConfig> robogenConfig) :
		robogenConfig_(robogenConfig), startPositionId_(0) {

}

Scenario::~Scenario() {

}

/**
 * @author: Kevaalin
 * @return a robot's current position in the terrain
 * From here is the code to retrieve the starting positions
 */
boost::shared_ptr<StartPosition> Scenario::getCurrentStartPosition() {
	return robogenConfig_->getStartingPos()->getStartPosition(
			startPositionId_);
}


/**
 * @return the configuration file
 */
boost::shared_ptr<RobogenConfig> Scenario::getRobogenConfig() {
	return robogenConfig_;
}

void Scenario::setStartingPosition(int id) {
	startPositionId_ = id;
}

boost::shared_ptr<Environment> Scenario::getEnvironment() {
	return environment_;
}

bool Scenario::init(dWorldID odeWorld, dSpaceID odeSpace,
		std::vector<boost::shared_ptr<Robot>> robot) { //initialise for multiple robots

	environment_ = boost::shared_ptr<Environment>(new
			Environment(odeWorld, odeSpace, robogenConfig_));

	if(!environment_->init()) {
		return false;
	}


	robots = robot;
	//assign multiple double locations
	std::vector<double> minX;
	std::vector<double> maxX;
	std::vector<double> minY;
	std::vector<double> maxY;
	std::vector<double> minZ;
	std::vector<double> maxZ;

	for (int i = 0; i< robots.size();i++)
	{
	// Setup robot position
		minX.push_back(0);
		maxX.push_back(0);
		minY.push_back(0);
		maxY.push_back(0);
		minZ.push_back(0);
		maxZ.push_back(0);
	}
	
	/**
	 * Creates two new vectors in the Scenario class, one to store the starting positions of each robot
	 * and one to store each robots azimuth value.
	 */
	std::vector<osg::Vec2> arrStartingPosition;
	std::vector<float> arrAzimth;


	/**
	 * Iterates through the robots and takes the starting positions and azimuths of the robots 
	 * and stores it in the relevant vectors.
	 */
	for (int i = 0; i < robots.size(); i++)
	{
		osg::Vec2 inputPos = robogenConfig_->getStartingPos()->getStartPosition(i)->getPosition();
		arrStartingPosition.push_back(inputPos);
		float inputAzi = robogenConfig_->getStartingPos()->getStartPosition(i)->getAzimuth();
		arrAzimth.push_back(inputAzi);
	}

	/**
	 * Iterates through the robots and rotates them according to the azimuth value for the simulator.
	 */
	for(int i = 0; i < robots.size(); i++){
	osg::Quat roboRot;
	roboRot.makeRotate(osg::inDegrees(arrAzimth[i]), osg::Vec3(0,0,1));
	
		robots[i]->rotateRobot(roboRot);
		robots[i]->getBB(minX[i], maxX[i], minY[i], maxY[i], minZ[i], maxZ[i]);
		robots[i]->translateRobot(
			osg::Vec3(arrStartingPosition[i].x(),
					arrStartingPosition[i].y(),
					robogenConfig_->getTerrainConfig()->getHeight()
						+ inMm(2) - minZ[i]));
		robots[i]->getBB(minX[i], maxX[i], minY[i], maxY[i], minZ[i], maxZ[i]);
	}
	
	/**
	 * Produces an output statement, declaring the area in the terrain that each robot will occupy.
	 * Note: The robots actual output can be any shape, depending on the robot file, however, the
	 *       output will always depict the space taken as a rectangular area.
	 */
	for (int i = 0; i < robots.size();i++)
	{
	std::cout
			<< "The " << i <<" robot is enclosed in the AABB(minX, maxX, minY, maxY, minZ, maxZ) ("
			<< minX[i] << ", " << maxX[i] << ", " << minY[i] << ", " << maxY[i] << ", "
			<< minZ[i] << ", " << maxZ[i] << ")" << std::endl;
	std::cout << "Obstacles in this range will not be generated" << std::endl << std::endl;
	}
	
	
	// Setup obstacles
	boost::shared_ptr<ObstaclesConfig> obstacles =
			robogenConfig_->getObstaclesConfig();
	
	// Instance the boxes above the maximum terrain height
	const std::vector<osg::Vec3>& c = obstacles->getCoordinates();
	const std::vector<osg::Vec3>& s = obstacles->getSizes();
	const std::vector<float>& d = obstacles->getDensities();
	const std::vector<osg::Vec3>& rotationAxis = obstacles->getRotationAxes();
	const std::vector<float>& rotationAngles = obstacles->getRotationAngles();

	obstaclesRemoved_ = false;

	double overlapMaxZ=minZ[0];
	/**
	 * Iterates through the obstacle textfile, if one is provided, and adds it to the scenario.
	 * Note: In cases where an obstacle may overlap with a robot, the simulator will change the
	 *       scenario file, based on the Obstacle Overlap Policy. By default it will remove the
	 *		 obstacle from the simulation.
	 */
	for (unsigned int i = 0; i < c.size(); ++i) {
		boost::shared_ptr<BoxObstacle> obstacle(
									new BoxObstacle(odeWorld, odeSpace, c[i],
											s[i], d[i], rotationAxis[i],
											rotationAngles[i]));
		double oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ;
		obstacle->getAABB(oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ);

		// Do not insert the obstacle if it is in the robot range
		bool inRangeX = false;
		bool inRangeY = false;
		bool inRangeZ = false;
		for (int i = 0; i < robogenConfig_->getNumberOfRobots(); i++){
			if ((oMinX <= minX[i] && oMaxX >= maxX[i]) || (oMinX >= minX[i] && oMinX <= maxX[i])
					|| (oMaxX >= minX[i] && oMaxX <= maxX[i])) {
				inRangeX = true;
			}
	
			if ((oMinY <= minY[i] && oMaxY >= maxY[i]) || (oMinY >= minY[i] && oMinY <= maxY[i])
					|| (oMaxY >= minY[i] && oMaxY <= maxY[i])) {
				inRangeY = true;
			}
	
			if ((oMinZ <= minZ[i] && oMaxZ >= maxZ[i]) || (oMinZ >= minZ[i] && oMinZ <= maxZ[i])
					|| (oMaxZ >= minZ[i] && oMaxZ <= maxZ[i])) {
				inRangeZ = true;
			}
		}
		

		// Do not insert obstacles in the robot range
		// If statements to check that no obstacles overlapping with robots
		if (!(inRangeX && inRangeY && inRangeZ)) {
			environment_->addObstacle(obstacle);
		} else {
			if (robogenConfig_->getObstacleOverlapPolicy() ==
					RobogenConfig::ELEVATE_ROBOT) {

				if (oMaxZ > overlapMaxZ)
					overlapMaxZ = oMaxZ;
				environment_->addObstacle(obstacle);

			} else {
				obstacle->remove();
				obstaclesRemoved_ = true;
			}
		}

	}
	// Print statement for all obstacles being generated
	std::cout << "All obstacles have been generated..." << std::endl;
	std::cout << "Any overlapping obstacles have been removed from the simulation..." << std::endl;

	if (robogenConfig_->getObstacleOverlapPolicy() ==
			RobogenConfig::ELEVATE_ROBOT) {
	for (int i = 0; i < robots.size();i++)
	{
		robots[i]->translateRobot(
				osg::Vec3(robogenConfig_->getStartingPos()->getStartPosition(i)->getPosition().x(), robogenConfig_->getStartingPos()->getStartPosition(i)->getPosition().y(),
						overlapMaxZ + inMm(2) - minZ[i]));
	}
	}

	
	// Setup light sources
	boost::shared_ptr<LightSourcesConfig> lightSourcesConfig =
			robogenConfig_->getLightSourcesConfig();

	std::vector<boost::shared_ptr<LightSource> > lightSources;
	this->getEnvironment()->setLightSources(lightSources);

	std::vector<osg::Vec3> lightSourcesCoordinates =
			lightSourcesConfig->getCoordinates();
	std::vector<float> lightSourcesIntensities =
				lightSourcesConfig->getIntensities();
	for (unsigned int i = 0; i < lightSourcesCoordinates.size(); ++i) {
		lightSources.push_back(boost::shared_ptr<LightSource>(
					new LightSource(odeSpace, lightSourcesCoordinates[i],
							lightSourcesIntensities[i])));

	}
	environment_->setLightSources(lightSources);


	// optimize the physics!  replace all fixed joints with composite bodies
	for (int i = 0;i<robots.size();i++)
	{
	robots[i]->optimizePhysics();
	}
	return true;
}


void Scenario::prunemul(){
	environment_.reset();
	for (int i = robots.size()-1;i > -1;i--)
	{
		robots[i].reset();
		robots.pop_back();
	}
}

std::vector<boost::shared_ptr<Robot>> Scenario::getRobots() {
	return robots;
}


}
