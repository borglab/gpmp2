/**
 *  @file   GaussianPriorWorkspacePoseArm.h
 *  @brief  Gaussian prior defined on the workspace pose of any link of an arm
 *          given its state in configuration space
 *  @author Mustafa Mukadam
 *  @date   Jan 8, 2018
 **/

#pragma once

#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/kinematics/GaussianPriorWorkspacePose.h>

namespace gpmp2 {

// template use ArmModel as robot type
typedef GaussianPriorWorkspacePose<ArmModel> GaussianPriorWorkspacePoseArm;

}  // namespace gpmp2
