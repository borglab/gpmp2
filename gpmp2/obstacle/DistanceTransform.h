/**
 *  @file   DistanceTransform.h
 *  @brief  Converts distance transform binary 2D and 3D volumes
 *  @author Matthew King-Smith
 *  @date   Nov 16, 2025
 **/

#pragma once

#include <gpmp2/obstacle/PlanarSDF.h>
#include <gpmp2/obstacle/SignedDistanceField.h>

#include <vector>

/// @note The namespace dt (distance transform) computes signed distance fields according to
///        Felzenszwalb & Huttenlocher Algorithm: https://cs.brown.edu/people/pfelzens/papers/dt-final.pdf
namespace gpmp2::dt
{
    /// @brief Rounds matrix to specified decimation
    /// @param[in] mat_
    /// @param[in] decimals_
    /// @return Rounded to decimation matrix
    gtsam::Matrix roundMatrix(gtsam::Matrix const & mat_, int const decimals_);

    std::vector<gtsam::Matrix> computeSignedDistanceField(std::vector<gtsam::Matrix> const & binaryMap_, double const cellSize_);

    /// @brief Compute the signed distance field of an obstacle-map
    /// @param binaryMap_ (Obstacle map where 1 = obstacle and 0 = free space)
    /// @param cellSize_ resolution of map
    /// @return signed distance field of binary map
    gtsam::Matrix computeSignedDistanceField(gtsam::Matrix const & binaryMap_, double const cellSize_);
} // namespace gpmp2::dt
