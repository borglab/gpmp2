/**
 *  @file   SignedDistanceFieldConverter.cpp
 *  @brief  Converts binary 2D and 3D volumes to signed distance fields
 *  @author Matthew King-Smith
 *  @date   Nov 16, 2025
 **/

#include <gpmp2/obstacle/DistanceTransform.h>

#include <cmath>
#include <limits>

namespace
{

Eigen::VectorXd computeEuclideanDistanceTransform(Eigen::VectorXd const & func_)
{
    size_t const dim = func_.size();
    size_t k = 0; ///< Index of right most parabola in lower envelope
    Eigen::VectorXi v(dim); ///< Locations of parabolas in lower envelope
    Eigen::VectorXd z(dim + 1); ///< Locations of boundaries between parabolas
    v[0] = 0;
    z[0] = -std::numeric_limits<double>::infinity();
    z[1] =  std::numeric_limits<double>::infinity();

    // Compute lower envelope
    double s; /// < Parabola intersection point
    for(size_t q = 1; q < dim; ++q)
    {

        while(true)
        {
            s = ((func_[q] + static_cast<double>(q*q)) - (func_[v[k]] + static_cast<double>(v[k]*v[k]))) / (2.0 * static_cast<double>(q - v[k]));

            if (s > z[k]) break;

            if (k == 0) break;
            k--;
        }
        ++k;
        v[k]   = q;
        z[k]   = s;
        z[k+1] = std::numeric_limits<double>::infinity();
    }

    k = 0;
    Eigen::VectorXd distanceTransform = Eigen::VectorXd::Zero(dim);
    for(size_t q = 0; q < dim; ++q)
    {
        while (z[k+1] < static_cast<double>(q)) ++k;
        distanceTransform[q] = static_cast<double>((q - v[k]) * (q - v[k])) + func_[v[k]];
    }
    return distanceTransform;
}

std::vector<gtsam::Matrix> computeEDTFromBinary(std::vector<gtsam::Matrix> const & binaryVolume)
{
    if(binaryVolume.empty())
    {
        std::cerr << "Cannot compute edt of binaryVolume which is empty!\n";
        return {};
    }

    size_t const Z = binaryVolume.size();
    size_t const Y = binaryVolume[0].rows();
    size_t const X = binaryVolume[0].cols();

    // --- Step 1: build function map ---
    std::vector<gtsam::Matrix> edt(Z, gtsam::Matrix(Y, X));

    for (size_t z = 0; z < Z; ++z)
    {
        edt[z] = binaryVolume[z].unaryExpr([](double const x_)
        {
            return x_ == 1.0 ? 0.0 : std::numeric_limits<double>::max();
        });
    }

    // --- Step 2: X pass (rows) ---
    for (size_t z = 0; z < Z; ++z)
    {
        for (size_t y = 0; y < Y; ++y)
        {
            Eigen::VectorXd f = edt[z].row(y).transpose();
            Eigen::VectorXd d = computeEuclideanDistanceTransform(f);
            edt[z].row(y) = d.transpose();
        }
    }

    // --- Step 3: Y pass (cols) ---
    for (size_t z = 0; z < Z; ++z)
    {
        for (size_t x = 0; x < X; ++x)
        {
            Eigen::VectorXd f = edt[z].col(x);
            Eigen::VectorXd d = computeEuclideanDistanceTransform(f);
            edt[z].col(x) = d;
        }
    }

    if(Z < 2)
        // Only a single matrix nothing left to compute
        return edt;

    // --- Step 4: Z pass ---
    for (size_t y = 0; y < Y; ++y)
    {
        for (size_t x = 0; x < X; ++x)
        {
            Eigen::VectorXd f(Z);

            for (size_t z = 0; z < Z; ++z)
                f[z] = edt[z](y, x);

            Eigen::VectorXd d = computeEuclideanDistanceTransform(f);

            for (size_t z = 0; z < Z; ++z)
                edt[z](y, x) = d[z];
        }
    }

    return edt;
}

} // namespace anonymous

gtsam::Matrix gpmp2::dt::roundMatrix(gtsam::Matrix const & mat_, int const decimals_)
{
    double scale = std::pow(10.0, decimals_);
    return (mat_ * scale).array().round() / scale;
}

gtsam::Matrix gpmp2::dt::computeSignedDistanceField(gtsam::Matrix const & binaryMap_, double const cellSize_)
{
    return computeSignedDistanceField(std::vector<gtsam::Matrix>{binaryMap_}, cellSize_).at(0);
}

std::vector<gtsam::Matrix> gpmp2::dt::computeSignedDistanceField(
    std::vector<gtsam::Matrix> const & binaryVolume_,
    double cellSize_)
{
    // distance to object (inside obstacles)
    auto distObj = computeEDTFromBinary(binaryVolume_);

    size_t const volSize = binaryVolume_.size();

    // build inverse
    std::vector<gtsam::Matrix> inv(volSize);
    for (size_t z = 0; z < volSize; ++z)
    {
        inv[z] = gtsam::Matrix::Ones(binaryVolume_[z].rows(),
                                    binaryVolume_[z].cols()) - binaryVolume_[z];
    }

    // distance to background
    auto distBg = computeEDTFromBinary(inv);

    // combine → signed distance
    std::vector<gtsam::Matrix> sdf(volSize);

    for (size_t z = 0; z < volSize; ++z)
    {
        sdf[z] =
            (distObj[z].array().sqrt() -
             distBg[z].array().sqrt()) * cellSize_;
    }

    return sdf;
}