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

bool Scenario::init(dWorldID odeWorld, dSpaceID odeSpace,
		boost::shared_ptr<Robot> robot) {

	environment_ = boost::shared_ptr<Environment>(new
			Environment(odeWorld, odeSpace, robogenConfig_));

	if(!environment_->init()) {
		return false;
	}


	robot_ = robot;

	// Setup robot position
	double minX = 0;
	double maxX = 0;
	double minY = 0;
	double maxY = 0;
	double minZ = 0;
	double maxZ = 0;

	// Starting position and orientation
	osg::Vec2 startingPosition =
			robogenConfig_->getStartingPos()->getStartPosition(
					startPositionId_)->getPosition();
	float startingAzimuth = 0.1;// robogenConfig_->getStartingPos()->getStartPosition(startPositionId_)->getAzimuth();
	osg::Quat roboRot;
	roboRot.makeRotate(osg::inDegrees(startingAzimuth), osg::Vec3(0,0,1));

	robot->rotateRobot(roboRot);
	robot->getBB(minX, maxX, minY, maxY, minZ, maxZ);
	robot->translateRobot(
			osg::Vec3(startingPosition.x(),
					startingPosition.y(),
					robogenConfig_->getTerrainConfig()->getHeight()
						+ inMm(2) - minZ));
	robot->getBB(minX, maxX, minY, maxY, minZ, maxZ);

	std::cout
			<< "The robot is enclosed in the AABB(minX, maxX, minY, maxY, minZ, maxZ) ("
			<< minX << ", " << maxX << ", " << minY << ", " << maxY << ", "
			<< minZ << ", " << maxZ << ")" << std::endl;
	std::cout << "Obstacles in this range will not be generated" << std::endl << std::endl;

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

	double overlapMaxZ=minZ;

	for (unsigned int i = 0; i < c.size(); ++i) {
		boost::shared_ptr<BoxObstacle> obstacle(
									new BoxObstacle(odeWorld, odeSpace, c[i],
											s[i], d[i], rotationAxis[i],
											rotationAngles[i]));
		double oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ;
		obstacle->getAABB(oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ);

		/*
		float oMinX = c[i].x() - s[i].x() / 2;
		float oMaxX = c[i].x() + s[i].x() / 2;
		float oMinY = c[i].y() - s[i].y() / 2;
		float oMaxY = c[i].y() + s[i].y() / 2;
		float oMinZ = c[i].z() - s[i].z() / 2;
		float oMaxZ = c[i].z() + s[i].z() / 2;
		 */

		// Do not insert the obstacle if it is in the robot range
		bool inRangeX = false;
		if ((oMinX <= minX && oMaxX >= maxX) || (oMinX >= minX && oMinX <= maxX)
				|| (oMaxX >= minX && oMaxX <= maxX)) {
			inRangeX = true;
		}

		bool inRangeY = false;
		if ((oMinY <= minY && oMaxY >= maxY) || (oMinY >= minY && oMinY <= maxY)
				|| (oMaxY >= minY && oMaxY <= maxY)) {
			inRangeY = true;
		}

		bool inRangeZ = false;
		if ((oMinZ <= minZ && oMaxZ >= maxZ) || (oMinZ >= minZ && oMinZ <= maxZ)
				|| (oMaxZ >= minZ && oMaxZ <= maxZ)) {
			inRangeZ = true;
		}

		// Do not insert obstacles in the robot range
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

	if (robogenConfig_->getObstacleOverlapPolicy() ==
			RobogenConfig::ELEVATE_ROBOT) {

		robot->translateRobot(
				osg::Vec3(startingPosition.x(), startingPosition.y(),
						overlapMaxZ + inMm(2) - minZ));
	}

	// Setup light sources
	boost::shared_ptr<LightSourcesConfig> lightSourcesConfig =
			robogenConfig_->getLightSourcesConfig();

	// todo do we need to do overlap check with light sources??

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
	robot->optimizePhysics();

	return true;
}

/**
 * @author: Kevaalin
 * From here is the code to retrieve the starting positions
 */
boost::shared_ptr<StartPosition> Scenario::getCurrentStartPosition() {
	return robogenConfig_->getStartingPos()->getStartPosition(
			startPositionId_);
}

void Scenario::prune(){
	environment_.reset();
	robot_.reset();
}

//boost::shared_ptr<Robot> Scenario::getRobot() {
//	return robot_;
//}

boost::shared_ptr<RobogenConfig> Scenario::getRobogenConfig() {
	return robogenConfig_;
}

void Scenario::setStartingPosition(int id) {
	startPositionId_ = id;
}

boost::shared_ptr<Environment> Scenario::getEnvironment() {
	return environment_;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Scenario::inita(dWorldID odeWorld, dSpaceID odeSpace,
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
	// Starting position and orientation
	/**
	 * Creates new vector of type osg::Vec2, which is a vector of the starting position 
	 * for each Robot (Formatted as: x-pos, y-pos, azimuth)
	 */
	std::vector<osg::Vec2> arrStartingPosition;
	std::vector<float> arrAzimth;
	//std::vector<boost::shared_ptr<StartPosition> > insertStartPositions = robogenConfig_->getStartingPos()->getStartPosition(
	//				startPositionId_)->getPosition();
	
	/**
	 * Stores the azimuths into a vector
	 */

/*	for (int i = 0; i < robots.size();i++)
	{
		osg::Vec2 input = osg::Vec2f(i-0.5*i, 0);
		arrStartingPosition.push_back(input);
		if (i != 0)
		{
			arrAzimth.push_back(0);
		}
		else
		{
			arrAzimth.push_back(90); ////////still need to remove hard code
		}
		
	}
*/
	/**
	 * Reading Starting Positions from the textfiles and not hardcoded as done above
	 */
	for (int i = 0; i < robogenConfig_->getNumberOfRobots(); i++)
	{
		osg::Vec2 inputPos = robogenConfig_->getStartingPos()->getStartPosition(i)->getPosition();
		arrStartingPosition.push_back(inputPos);
		float inputAzi = robogenConfig_->getStartingPos()->getStartPosition(i)->getAzimuth();
		arrAzimth.push_back(inputAzi);
	}
	/**
	 * Reading in the S.Pos from the config, storing it as a vec. pushing it to vec of vectors
	 */

	
/*	for (int i = 0; i < robots.size(); i++)
	{
		osg::Vec2 startingPosConfig = robogenConfig_->getStartingPos()->getStartPosition(startPositionId_)->getPosition();
		float floatingAzimuth = robogenConfig_->getStartingPos()->getStartPosition(startPositionId_)->getAzimuth();
		arrStartingPosition.push_back(startingPosConfig);
		arrAzimth.push_back(floatingAzimuth);
	}
*/

	//for efficeint memory management do below ------ i am not sure
	//delete input;
	//delete insertStartPositions;
    //  											      |
	//---> still need to sort out Azimuth below ----------v	
	//float startingAzimuth = robogenConfig_->getStartingPos()->getStartPosition(
	//		startPositionId_)->getAzimuth();


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
	
	for (int i = 0; i < robots.size();i++)
	{
	std::cout
			<< "The " << i <<" robot is enclosed in the AABB(minX, maxX, minY, maxY, minZ, maxZ) ("
			<< minX[i] << ", " << maxX[i] << ", " << minY[i] << ", " << maxY[i] << ", "
			<< minZ[i] << ", " << maxZ[i] << ")" << std::endl;
	std::cout << "Obstacles in this range will not be generated" << std::endl << std::endl;
	}
	 /////////////////////////need to edit code later
	
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

	double overlapMaxZ=minZ[0]; ////////////////////////////////////////////////////////////////////////////////////

	for (unsigned int i = 0; i < c.size(); ++i) {
		boost::shared_ptr<BoxObstacle> obstacle(
									new BoxObstacle(odeWorld, odeSpace, c[i],
											s[i], d[i], rotationAxis[i],
											rotationAngles[i]));
		double oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ;
		obstacle->getAABB(oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ);

		/*
		float oMinX = c[i].x() - s[i].x() / 2;
		float oMaxX = c[i].x() + s[i].x() / 2;
		float oMinY = c[i].y() - s[i].y() / 2;
		float oMaxY = c[i].y() + s[i].y() / 2;
		float oMinZ = c[i].z() - s[i].z() / 2;
		float oMaxZ = c[i].z() + s[i].z() / 2;
		 */

		// Do not insert the obstacle if it is in the robot range
		bool inRangeX = false;
		bool inRangeY = false;
		bool inRangeZ = false;
		for (int i = 0; i < robogenConfig_->getNumberOfRobots(); i++){
			//bool inRangeX = false;
			if ((oMinX <= minX[i] && oMaxX >= maxX[i]) || (oMinX >= minX[i] && oMinX <= maxX[i])
					|| (oMaxX >= minX[i] && oMaxX <= maxX[i])) {
				inRangeX = true;
			}

			//bool inRangeY = false;
			if ((oMinY <= minY[i] && oMaxY >= maxY[i]) || (oMinY >= minY[i] && oMinY <= maxY[i])
					|| (oMaxY >= minY[i] && oMaxY <= maxY[i])) {
				inRangeY = true;
			}

			//bool inRangeZ = false;
			if ((oMinZ <= minZ[i] && oMaxZ >= maxZ[i]) || (oMinZ >= minZ[i] && oMinZ <= maxZ[i])
					|| (oMaxZ >= minZ[i] && oMaxZ <= maxZ[i])) {
				inRangeZ = true;
			}
		}
		// Do not insert obstacles in the robot range
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
/*
	if (robogenConfig_->getObstacleOverlapPolicy() ==
			RobogenConfig::ELEVATE_ROBOT) {

		robot->translateRobot(
				osg::Vec3(startingPosition.x(), startingPosition.y(),
						overlapMaxZ + inMm(2) - minZ));
	}
*/
	
	// Setup light sources
	boost::shared_ptr<LightSourcesConfig> lightSourcesConfig =
			robogenConfig_->getLightSourcesConfig();

	// todo do we need to do overlap check with light sources??

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
