/*
 * @(#) RacingScenario.cpp   1.0   Mar 13, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
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
#include "scenario/BalancingScenario.h"
#include "Robot.h"
#include "Models.h"

#include "evolution/engine/EvolverLog.h"

namespace robogen {

BalancingScenario::BalancingScenario(boost::shared_ptr<RobogenConfig> robogenConfig) :
		Scenario(robogenConfig), curTrial_(0) {

	this->setEnvironment(boost::shared_ptr<Environment>(new Environment()));

}

BalancingScenario::~BalancingScenario() {

}

bool BalancingScenario::setupSimulation() {

	// Compute robot start position,
	startPosition_.push_back(this->getCurrentStartPosition()->getPosition());
	accelsX_.push_back(0);
	accelsY_.push_back(0);
	anglesX_.push_back(0);
	anglesY_.push_back(0);

	return true;

}

bool BalancingScenario::afterSimulationStep() {

  std::vector<boost::shared_ptr<Sensor> > sensors;
  boost::dynamic_pointer_cast<CoreComponentModel, Model>(getRobot()->getCoreComponent())->getSensors(sensors);
  accelsX_[curTrial_] = std::max(accelsX_[curTrial_], boost::dynamic_pointer_cast<SimpleSensor, Sensor>(sensors[0])->read());
  accelsY_[curTrial_] = std::max(accelsY_[curTrial_], boost::dynamic_pointer_cast<SimpleSensor, Sensor>(sensors[1])->read());
  double angle;
  osg::Vec3 rotaxis;
  getRobot()->getCoreComponent()->getRootAttitude().getRotate(angle,rotaxis);
  osg::Vec3 rotation = rotaxis*angle;
  anglesX_[curTrial_] += abs(rotation.x());
  anglesY_[curTrial_] += abs(rotation.y());
  
  return true;
}

bool BalancingScenario::endSimulation() {

	// Compute robot ending position from its closest part to the origin
	double minDistance = std::numeric_limits<double>::max();
	const std::vector<boost::shared_ptr<Model> >& bodyParts = this->getRobot()->getBodyParts();
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

double BalancingScenario::getFitness() {

	double fitness = -1000000;
	for (unsigned int i = 0; i < distances_.size(); ++i) {
	  double trialFit = distances_[i]*500-log(anglesX_[i]+anglesY_[i]+accelsX_[i]+accelsY_[i]);
	  if (trialFit > fitness) {
			fitness = trialFit;
			distance = distances_[i];
			angle = (anglesX_[i]+anglesY_[i]+accelsX_[i]+accelsY_[i]);
	  }
	}
	
	return fitness;
}

  std::pair<double,double> BalancingScenario::getIntermediateValues() {
    return std::make_pair(distance, angle);
  }

bool BalancingScenario::remainingTrials() {
	boost::shared_ptr<StartPositionConfig> startPos = this->getRobogenConfig()->getStartingPos();
	return curTrial_ < startPos->getStartPosition().size();
}

int BalancingScenario::getCurTrial() const {
	return curTrial_;
}

}
