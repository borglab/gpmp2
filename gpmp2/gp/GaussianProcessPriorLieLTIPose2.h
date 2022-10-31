
/**
 *  @file  GaussianProcessPriorLieLTIPose3.h
 *  @brief GP prior works on any Lie group
 *  @author Jing Dong
 *  @date Oct 3, 2016
 **/

#pragma once

#include <gpmp2/gp/GaussianProcessPriorLieLTI.h>
#include <gtsam/geometry/Pose2.h>

namespace gpmp2 {

typedef GaussianProcessPriorLieLTI<gtsam::Pose2> GaussianProcessPriorLieLTIPose2;

}  // namespace gpmp2
