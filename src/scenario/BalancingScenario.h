/*
 * @(#) RacingScenario.h   1.0   Mar 13, 2013
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
#ifndef ROBOGEN_BALANCING_SCENARIO_H_
#define ROBOGEN_BALANCING_SCENARIO_H_

#include <osg/Vec3>
#include "scenario/Scenario.h"
#include <vector>

namespace robogen {

/**
 * Racing Scenario.
 * The robot that can cover the longer distance in the given simulation time wins.
 * The distance is computed as the euclidean distance from the starting to the ending position computed using as
 * reference the core component.
 */
class BalancingScenario: public Scenario {

public:

	/**
	 * Initializes a BalancingScenario
	 */
	BalancingScenario(boost::shared_ptr<RobogenConfig> robogenConfig);

	/**
	 * Destructor
	 */
	virtual ~BalancingScenario();

	/**
	 * Methods inherited from {@link #Scenario}
	 */
	virtual bool setupSimulation();
	virtual bool afterSimulationStep();
	virtual bool endSimulation();
	virtual double getFitness();
	virtual std::pair<double, double> getIntermediateValues();
	virtual bool remainingTrials();
	virtual int getCurTrial() const;

private:

	std::vector<osg::Vec2> startPosition_;
	std::vector<double> distances_;
	std::vector<float> anglesX_, anglesY_, accelsX_, accelsY_;
	double distance,angle;
	unsigned int curTrial_;

};

}

#endif /* ROBOGEN_RACING_SCENARIO_H_ */
