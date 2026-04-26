/**
 *  @file   testDistanceTransform.cpp
 *  @brief  Test cases when converting obstacle map to signed distance field
 *  @author Matthew King-Smith
 *  @date   Apr 25, 2026
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Matrix.h>
#include <gpmp2/obstacle/DistanceTransform.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;

TEST(DistanceTransform, single_pixel) {
    Matrix singleObstacleMap {Matrix::Zero(3,3)};
    singleObstacleMap << 0, 0, 0,
                         0, 1, 0,
                         0, 0, 0;

    Matrix expectedSDF(3,3);
    expectedSDF << sqrt(2), 1, sqrt(2),
                   1,       -1, 1,
                   sqrt(2), 1, sqrt(2);

    const auto computedSDF = gpmp2::dt::computeSignedDistanceField(singleObstacleMap, 1.0);

    EXPECT(assert_equal(expectedSDF, computedSDF, 1e-6));
}

TEST(DistanceTransform, empty)
{
    Matrix const m = Matrix::Zero(3,3);

    auto const sdf = gpmp2::dt::computeSignedDistanceField(m, 1.0);

    // convention: large positive everywhere
    EXPECT((sdf.array() > 0).all());
}

TEST(DistanceTransform, full)
{
    Matrix const m = Matrix::Ones(3,3);

    auto const sdf = gpmp2::dt::computeSignedDistanceField(m, 1.0);

    // everything inside obstacle → negative
    EXPECT((sdf.array() <= 0).all());
}

TEST(DistanceTransform, horizontal_line)
{
    Matrix m = Matrix::Zero(5,5);
    m.row(2).setOnes();

    Matrix expected(5,5);
    expected <<
        2, 2, 2, 2, 2,
        1, 1, 1, 1, 1,
       -1,-1,-1,-1,-1,
        1, 1, 1, 1, 1,
        2, 2, 2, 2, 2;

    auto const sdf = gpmp2::dt::computeSignedDistanceField(m, 1.0);
    EXPECT(assert_equal(expected, sdf, 1e-6));
}

TEST(DistanceTransform, vertical_line)
{
    Matrix m = Matrix::Zero(5,5);
    m.col(2).setOnes();

    Matrix expected(5,5);
    expected <<
        2, 1,-1, 1, 2,
        2, 1,-1, 1, 2,
        2, 1,-1, 1, 2,
        2, 1,-1, 1, 2,
        2, 1,-1, 1, 2;

    auto const sdf = gpmp2::dt::computeSignedDistanceField(m, 1.0);
    EXPECT(assert_equal(expected, sdf, 1e-6));
}

TEST(DistanceTransform, square_block)
{
    Matrix m = Matrix::Zero(5,5);
    m.block(1,1,3,3).setOnes();

    Matrix expected(5,5);
    expected <<
        sqrt(2), 1, 1, 1, sqrt(2),
        1,      -1,-1,-1, 1,
        1,      -1,-2,-1, 1,
        1,      -1,-1,-1, 1,
        sqrt(2), 1, 1, 1, sqrt(2);

    auto const sdf = gpmp2::dt::computeSignedDistanceField(m, 1.0);
    EXPECT(assert_equal(expected, sdf, 1e-6));
}

TEST(DistanceTransform, diagonal)
{
    Matrix m = Matrix::Zero(3,3);
    m(0,0) = 1;
    m(1,1) = 1;
    m(2,2) = 1;

    auto const sdf = gpmp2::dt::computeSignedDistanceField(m, 1.0);

    // just check invariants (exact values messy)
    EXPECT((sdf.diagonal().array() < 0).all());
}

TEST(DistanceTransform3D, single_voxel_center)
{
    std::vector<Matrix> vol(5, Matrix::Zero(5,5));
    vol[2](2,2) = 1;  // center voxel

    auto const sdf = gpmp2::dt::computeSignedDistanceField(vol, 1.0);

    // center slice check
    Matrix expected(5,5);
    expected <<
         std::sqrt(8), std::sqrt(5), 2, std::sqrt(5), std::sqrt(8),
         std::sqrt(5), std::sqrt(2), 1, std::sqrt(2), std::sqrt(5),
         2,             1,          -1, 1,          2,
         std::sqrt(5), std::sqrt(2), 1, std::sqrt(2), std::sqrt(5),
         std::sqrt(8), std::sqrt(5), 2, std::sqrt(5), std::sqrt(8);

    EXPECT(assert_equal(expected, sdf[2], 1e-6));
}

TEST(DistanceTransform3D, z_plane)
{
    std::vector<Matrix> vol(5, Matrix::Zero(5,5));
    vol[2].setOnes();  // entire middle slice

    auto const sdf = gpmp2::dt::computeSignedDistanceField(vol, 1.0);

    EXPECT(assert_equal(Matrix::Constant(5,5,2),  sdf[0], 1e-6));
    EXPECT(assert_equal(Matrix::Constant(5,5,1),  sdf[1], 1e-6));
    EXPECT(assert_equal(Matrix::Constant(5,5,-1), sdf[2], 1e-6));
    EXPECT(assert_equal(Matrix::Constant(5,5,1),  sdf[3], 1e-6));
    EXPECT(assert_equal(Matrix::Constant(5,5,2),  sdf[4], 1e-6));
}

TEST(DistanceTransform3D, vertical_line)
{
    std::vector<Matrix> vol(5, Matrix::Zero(5,5));
    for (int z = 0; z < 5; z++)
        vol[z](2,2) = 1;

    auto const sdf = gpmp2::dt::computeSignedDistanceField(vol, 1.0);

    Matrix expected(5,5);
    expected <<
        std::sqrt(8), std::sqrt(5), 2, std::sqrt(5), std::sqrt(8),
        std::sqrt(5), std::sqrt(2), 1, std::sqrt(2), std::sqrt(5),
        2,             1,          -1, 1,          2,
        std::sqrt(5), std::sqrt(2), 1, std::sqrt(2), std::sqrt(5),
        std::sqrt(8), std::sqrt(5), 2, std::sqrt(5), std::sqrt(8);

    // all slices identical
    for (int z = 0; z < 5; z++)
        EXPECT(assert_equal(expected, sdf[z], 1e-6));
}

TEST(DistanceTransform3D, corner_voxel)
{
    std::vector<Matrix> vol(5, Matrix::Zero(5,5));
    vol[0](0,0) = 1;

    auto sdf = gpmp2::dt::computeSignedDistanceField(vol, 1.0);

    EXPECT(assert_equal(sdf[0](0,0), -1.0, 1e-6));
    EXPECT(assert_equal(sdf[0](1,0),  1.0, 1e-6));
    EXPECT(assert_equal(sdf[1](0,0),  1.0, 1e-6));
    EXPECT(assert_equal(sdf[1](1,1),  std::sqrt(3), 1e-6));
}

TEST(DistanceTransform3D, solid_cube)
{
    std::vector<Matrix> vol(5, Matrix::Zero(5,5));

    for (int z = 1; z <= 3; z++)
        vol[z].block(1,1,3,3).setOnes();

    auto sdf = gpmp2::dt::computeSignedDistanceField(vol, 1.0);

    EXPECT(assert_equal(sdf[2](2,2), -2.0, 1e-6)); // center deepest
    EXPECT(assert_equal(sdf[2](1,1), -1.0, 1e-6)); // surface
    EXPECT(assert_equal(sdf[0](2,2), 1.0, 1e-6)); // outside above
}

TEST(DistanceTransform3D, empty_map)
{
    std::vector<Matrix> vol(5, Matrix::Zero(5,5));

    auto const sdf = gpmp2::dt::computeSignedDistanceField(vol, 1.0);

    for (int z = 0; z < 5; z++)
        EXPECT(sdf[z].array().isFinite().all());
}

TEST(DistanceTransform3D, full_map)
{
    std::vector<Matrix> vol(5, Matrix::Ones(5,5));

    auto sdf = gpmp2::dt::computeSignedDistanceField(vol, 1.0);

    for (int z = 0; z < 5; z++)
        EXPECT((sdf[z].array() <= 0).all());
}

/* ************************************************************************** */
/* main function */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
