/*
 * @(#) CoreComponentRenderModel.cpp   1.0   Feb 5, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Andrea Maesani, Joshua Auerbach
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
#include <osgDB/ReadFile>

#include "render/callback/BodyCallback.h"

#include "render/components/CoreComponentRenderModel.h"
#include "render/Mesh.h"

namespace robogen {

CoreComponentRenderModel::CoreComponentRenderModel(
		boost::shared_ptr<CoreComponentModel> model) :
		RenderModel(model) {
	this->mesh_.reset(new Mesh());
}

CoreComponentRenderModel::~CoreComponentRenderModel() {

}

bool CoreComponentRenderModel::initRenderModel() {

	bool meshLoading = this->mesh_->loadMesh("../models/CoreComponent.stl");

	if (!meshLoading) {
		std::cerr << "[CoreComponentRenderModel] Error loading model"
				<< std::endl;
		return false;
	}

	if (isDebugActive()) {
		this->showDebugView();
		return true;
	}

	// display with plate down, as this is how will be in reality
	// (we want the arduino to be on top so wires can come out)
	this->mesh_->getMesh()->setAttitude(
			osg::Quat(osg::inDegrees(180.0), osg::Vec3(1, 0, 0)));


	this->getRootNode()->addChild(this->mesh_->getMesh());
	this->getRootNode()->setUpdateCallback(
			new BodyCallback(this->getModel(),
					CoreComponentModel::B_CORE_COMPONENT_ID));

	return true;

}

void CoreComponentRenderModel::showDebugView() {

	osg::ref_ptr<osg::PositionAttitudeTransform> pat = this->attachBox(
			CoreComponentModel::B_CORE_COMPONENT_ID,
			CoreComponentModel::WIDTH, CoreComponentModel::WIDTH,
			CoreComponentModel::WIDTH);

	// show the axis for the root node
	if (boost::dynamic_pointer_cast<CoreComponentModel>(this->getModel())->
			hasSensors() )
		attachAxis(pat);

}

void CoreComponentRenderModel::setColor(osg::Vec4 color) {
	this->mesh_->setColor(color);
}

}
