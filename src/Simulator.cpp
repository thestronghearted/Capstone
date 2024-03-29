/*
 * @(#) Simulator.cpp   1.0   Nov 27, 2014
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Joshua Auerbach
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

#include "Simulator.h"
#include "utils/RobogenCollision.h"
#include "Models.h"
#include "Robot.h"
#include "viewer/WebGLLogger.h"

//#define DEBUG_MASSES

// ODE World
extern dWorldID odeWorld;

// Container for collisions
extern dJointGroupID odeContactGroup;

namespace robogen{

unsigned int runSimulations(boost::shared_ptr<Scenario> scenario,  //runs the simulation
		boost::shared_ptr<RobogenConfig> configuration,
		const std::vector<std::reference_wrapper<robogenMessage::Robot> > robotMessage,
		IViewer *viewer, boost::random::mt19937 &rng) {
	boost::shared_ptr<FileViewerLog> log;
	return runSimulations(scenario, configuration,
			robotMessage, viewer, rng, false, log);
}

unsigned int runSimulations(boost::shared_ptr<Scenario> scenario,
		boost::shared_ptr<RobogenConfig> configuration,
		const std::vector<std::reference_wrapper<robogenMessage::Robot>> robotMessage, IViewer *viewer,
		boost::random::mt19937 &rng,
		bool onlyOnce, boost::shared_ptr<FileViewerLog> log) {
			
	bool constraintViolated = false;
	boost::random::normal_distribution<float> normalDistribution;
	boost::random::uniform_01<float> uniformDistribution;

	while (scenario->remainingTrials() && (!constraintViolated)) {

		// ---------------------------------------
		// Simulator initialization
		// ---------------------------------------

		dInitODE();

		// Create ODE world
		odeWorld = dWorldCreate();

		// Set gravity from config
		osg::Vec3 gravity = configuration->getGravity();
		dWorldSetGravity(odeWorld, gravity.x(), gravity.y(), gravity.z());

		dWorldSetERP(odeWorld, 0.1);
		dWorldSetCFM(odeWorld, 10e-6);
		dWorldSetAutoDisableFlag(odeWorld, 1);

		// Create collision world
		dSpaceID odeSpace = dSimpleSpaceCreate(0);

		// Create contact group
		odeContactGroup = dJointGroupCreate(0);

		// wrap all this in block so things get cleaned up before shutting down
		// ode
		{

		// ---------------------------------------
		// Generate Robot(s)
		// ---------------------------------------

		std::vector<boost::shared_ptr<Robot>> robots;
		for (int i = 0; i < robotMessage.size();i++)
		{
			boost::shared_ptr<Robot> robot(new Robot);
			if (!robot->init(odeWorld, odeSpace, robotMessage[i])) {
				std::cout << "Problems decoding the robot. Quit."
						<< std::endl;
				return SIMULATION_FAILURE;
			}
		robots.push_back(robot);
		}	

		// used for debugging --disabled		
#ifdef DEBUG_MASSES 
		float totalMass = 0;
		for (unsigned int i = 0; i < robot->getBodyParts().size(); ++i) {
			float partMass = 0;
			for (unsigned int j = 0;
					j < robot->getBodyParts()[i]->getBodies().size(); ++j) {

				dMass mass;
				dBodyGetMass(robot->getBodyParts()[i]->getBodies()[j], &mass);
				partMass += mass.mass;

			}
			std::cout << robot->getBodyParts()[i]->getId() <<  " has mass: "
					<< partMass * 1000. << "g" << std::endl;
			totalMass += partMass;
		}

		std::cout << "total mass is " << totalMass * 1000. << "g" << std::endl;
#endif
	
		
		// display trial number
		if (robots.size() > 1)
		{
			 std::cout << std::endl << "Evaluating swarm "
				<< ", trial: " << scenario->getCurTrial()
				<< std::endl;
		}else
		{
			std::cout << std::endl << "Evaluating individual " << robots[0]->getId()
				<< ", trial: " << scenario->getCurTrial()
				<< std::endl;
		}

		// Register sensors
		std::vector<std::vector<boost::shared_ptr<Sensor> >> sensors;
		for (int i = 0; i < robots.size();i++)
		{
			std::vector<boost::shared_ptr<Sensor> > sensor =
				robots[i]->getSensors();
			sensors.push_back(sensor);
		}
		std::vector<std::vector<boost::shared_ptr<TouchSensor>>> touchSensors;
		for (int j = 0;j < robots.size();j++)
		{
			std::vector<boost::shared_ptr<TouchSensor> > touchSensor;
			for (unsigned int i = 0; i < sensors[j].size(); ++i) {
				if (boost::dynamic_pointer_cast<TouchSensor>(
						sensors[j][i])) {
					touchSensor.push_back(
							boost::dynamic_pointer_cast<TouchSensor>(
									sensors[j][i]));
				}
			}
			touchSensors.push_back(touchSensor);
		}

		// Register robot motors
		std::vector<std::vector<boost::shared_ptr<Motor> >> motors;
		for(int i = 0; i < robots.size(); i++)
		{
			motors.push_back(robots[i]->getMotors());
		}

		// configure robot motors
		for(int j = 0; j < robots.size(); j++)
		{
			for(unsigned int i=0; i< motors[j].size(); i++) {
				motors[j][i]->setMaxDirectionShiftsPerSecond(
							configuration->getMaxDirectionShiftsPerSecond());

			}
		}

		// Register brain and body parts 
		std::vector<boost::shared_ptr<NeuralNetwork> > neuralNetworks;
		std::vector<std::vector<boost::shared_ptr<Model> >> bodyParts;
		for(int i =0; i < robots.size(); i++)
		{
			boost::shared_ptr<NeuralNetwork> neuralNetwork =
					robots[i]->getBrain();
			std::vector<boost::shared_ptr<Model> > bodyPart =
					robots[i]->getBodyParts();
			neuralNetworks.push_back(neuralNetwork);
			bodyParts.push_back(bodyPart);
		}
		
		
		// Initialize scenario
		if (!scenario->init(odeWorld, odeSpace, robots)) {
			std::cout << "Cannot initialize scenario. Quit."
					<< std::endl;
			return SIMULATION_FAILURE;
		}

		// Configure obstacles
		if((configuration->getObstacleOverlapPolicy() ==
				RobogenConfig::CONSTRAINT_VIOLATION) &&
				scenario->wereObstaclesRemoved()) {
			std::cout << "Using 'contraintViolation' obstacle overlap policy,"
					<< " and ostacles were removed, so will return min fitness."
					<< std::endl;
			constraintViolated = true;
			break;
		}

		
		// Setup environment
		boost::shared_ptr<Environment> env =
				scenario->getEnvironment();

		if (!scenario->setupSimulations(robots.size())) {
			std::cout << "Cannot setup scenario. Quit."
					<< std::endl;
			return SIMULATION_FAILURE;
		}

		bool visualize = (viewer != NULL);
		if(visualize && !viewer->configureScenes(bodyParts, scenario)) {
			std::cout << "Cannot configure scene. Quit."
					<< std::endl;
			return SIMULATION_FAILURE;
		}
	
		/***
         * Init the webGLLogger
         */
        boost::shared_ptr<WebGLLogger> webGLlogger;
        if (log && log->isWriteWebGL()) {
        	webGLlogger.reset(new WebGLLogger(log->getWebGLFileName(),
        										scenario));
        }


		//setup vectors for keeping velocities
		dReal previousLinVel[3];
		dReal previousAngVel[3];

		
		
		// ---------------------------------------
		// Main Loop
		// ---------------------------------------
		int count = 0;
		double t = 0;

		boost::shared_ptr<CollisionData> collisionData(
				new CollisionData(scenario) );
		
		double step = configuration->getTimeStepLength();

		// Threading method BEGIN ------------------------------------------------------

		/*
		 lambda(updateRobot(...)) used by threading within the upcoming while loop.
		 UpdateRobot(...) is  Responsible for updating the state of every robot after every frame.
		*/
		auto updateRobot = [&](boost::shared_ptr<robogen::Robot> robot, // robots[k]
                 boost::shared_ptr<robogen::RobogenConfig> configurations, // configuration
                std::vector<boost::shared_ptr<robogen::Model>> bodyParts,// bodyparts[k]]
                std::vector<boost::shared_ptr<robogen::Sensor>> sensors, // sensors[k]
                boost::shared_ptr<NeuralNetwork> neuralNetwork, // neuralnetwork[k]
                std::vector<boost::shared_ptr<robogen::Motor>> motors) // motors[k]
		{
			if (configuration->isCapAlleration()) 
			{
				dBodyID rootBody =
						robot->getCoreComponent()->getRoot()->getBody();
				const dReal *angVel, *linVel;

				angVel = dBodyGetAngularVel(rootBody);
				linVel = dBodyGetLinearVel(rootBody);

				if(t > 0) {
					double angAccel = dCalcPointsDistance3(
							angVel, previousAngVel);
					double linAccel = dCalcPointsDistance3(
							linVel, previousLinVel);

					if(angAccel > configuration->getMaxAngularAcceleration() ||
					linAccel > configuration->getMaxLinearAcceleration()) {

						printf("EVALUATION CANCELED: max accel");
						printf(" exceeded at time %f.", t);
						printf(" Angular accel: %f, Linear accel: %f.\n",
								angAccel, linAccel);
						printf("Will give %f fitness.\n", MIN_FITNESS);
						constraintViolated = true;
					}

				}

				// save current velocities as previous
				for(unsigned int j=0; j<3; j++) {
					previousAngVel[j] = angVel[j];
					previousLinVel[j] = linVel[j];
				}
			}

			float networkInput[MAX_INPUT_NEURONS];
			float networkOutputs[MAX_OUTPUT_NEURONS];
			
			// Update Sensors for each body part on the robot
			for (unsigned int i = 0; i < bodyParts.size();++i) 
			{
				if (boost::dynamic_pointer_cast<
						PerceptiveComponent>(bodyParts[i])) {
					boost::dynamic_pointer_cast<
							PerceptiveComponent>(bodyParts[i])->updateSensors(
							env);
				}
			}

			if(((count - 1) % configuration->getActuationPeriod()) == 0) 
			{
				// Feed neural network
				for (unsigned int i = 0; i < sensors.size(); ++i) {
					networkInput[i] = sensors[i]->read();

					// Add sensor noise: Gaussian with std dev of
					// sensorNoiseLevel * actualValue
					if (configuration->getSensorNoiseLevel() > 0.0) {
						networkInput[i] += (normalDistribution(rng) *
								configuration->getSensorNoiseLevel() *
								networkInput[i]);
					}
				}
				if (log) {
					log->logSensors(networkInput, sensors.size());
				}


				::feed(neuralNetwork.get(), &networkInput[0]);

				// Step the neural network
				::step(neuralNetwork.get(), t);

				// Fetch the neural network ouputs
				::fetch(neuralNetwork.get(), &networkOutputs[0]);

				// Send control to motors
				for (unsigned int i = 0; i < motors.size(); ++i) {

					// Add motor noise:
					// uniform in range +/- motorNoiseLevel * actualValue
					if(configuration->getMotorNoiseLevel() > 0.0) {
						networkOutputs[i] += (
									((uniformDistribution(rng) *
									2.0 *
									configuration->getMotorNoiseLevel())
									- configuration->getMotorNoiseLevel())
									* networkOutputs[i]);
					}


					if (boost::dynamic_pointer_cast<
							RotationMotor>(motors[i])) {
						boost::dynamic_pointer_cast<RotationMotor>(motors[i]
						)->setDesiredVelocity(networkOutputs[i], step *
										configuration->getActuationPeriod());
					} else if (boost::dynamic_pointer_cast<
							ServoMotor>(motors[i])) {
						boost::dynamic_pointer_cast<ServoMotor>(motors[i]
						)->setDesiredPosition(networkOutputs[i], step *
								configuration->getActuationPeriod());
					}

				}

				if(log) {
					log->logMotors(networkOutputs, motors.size());
				}
			}

			bool motorBurntOut = false;
			for (unsigned int i = 0; i < motors.size(); ++i) 
			{
				motors[i]->step( step ) ;

				if (motors[i]->isBurntOut()) {
					std::cout << "Motor burnt out, will terminate now "
							<< std::endl;
					motorBurntOut = true;
					constraintViolated = true;
				}

			}

			if(constraintViolated || motorBurntOut) 
			{
				//::break;
			}
				
		};
		
		// THREADING METHOD END ---------------------------------------------------------
		
		// run simulation while certain conditions are met
		while ((t < configuration->getSimulationTime())
			   && (!(visualize && viewer->done()))) {
				   
			// update the visualizer
			if(visualize) {
				if(!viewer->frame(t, count)) {
					continue;
				}
			}
			// print '.' after every 500th frame
			if ((count++) % 500 == 0) {
			 	std::cout << "." << std::flush;
			}
			// Collision detection
			dSpaceCollide(odeSpace, collisionData.get(), odeCollisionCallback);

			// Step the world by one timestep
			dWorldStep(odeWorld, step);

			// Empty contact groups used for collisions handling
			dJointGroupEmpty(odeContactGroup);
			if (configuration->isDisallowObstacleCollisions() &&
					collisionData->hasObstacleCollisions()) {
				constraintViolated = true;
				break;
			}

			/**
			 * loop through every robot, and update it, using threads
			 */
			std::vector<std::thread> threads;
			for(int k = 0 ; k < robots.size();k++)
			{	
				threads.push_back(std::thread(updateRobot, robots[k], configuration, bodyParts[k], sensors[k], neuralNetworks[k], motors[k]));
			}
			//join threads
			for(int k = 0 ; k < robots.size();k++)
			{	
				threads[k].join();
			}

			// Elapsed time since last call
			env->setTimeElapsed(step);

			t += step;

		}

		// ++++++++++++++++++++++++++++++++++++++
		// End of Main Loop
		// ++++++++++++++++++++++++++++++++++++++

		// check whether scenario was succesfully completed
		if (!scenario->endSimulations()) {
			std::cout << "Cannot complete scenario. Quit."
					<< std::endl;
			return SIMULATION_FAILURE;
		}

		// ---------------------------------------
		// Simulator finalization
		// ---------------------------------------

		// Destroy the WebGlLogger, since contains pointer to scenario
		if(webGLlogger) {
			webGLlogger.reset();
		}
		} // end code block protecting objects for ode code clean up


		// scenario has a shared ptr to the robot, so need to prune it
		scenario->prunemul();

		// Destroy the joint group
		dJointGroupDestroy(odeContactGroup);

		// Destroy ODE space
		dSpaceDestroy(odeSpace);

		// Destroy ODE world
		dWorldDestroy(odeWorld);

		// Destroy the ODE engine
		dCloseODE();

		if(constraintViolated || onlyOnce) {
			break;
		}
	}
	if(constraintViolated)
		return CONSTRAINT_VIOLATED;
	return SIMULATION_SUCCESS;
}
}
