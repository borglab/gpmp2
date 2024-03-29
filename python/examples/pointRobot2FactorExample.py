"""Point Robot Factor Graph Example"""

import matplotlib.pyplot as plt
import numpy as np
from gpmp2 import (BodySphere, BodySphereVector, GaussianProcessPriorLinear,
                   ObstaclePlanarSDFFactorGPPointRobot,
                   ObstaclePlanarSDFFactorPointRobot, PlanarSDF, PointRobot,
                   PointRobotModel)
from gpmp2.datasets.generate2Ddataset import generate2Ddataset
from gpmp2.utils.plot_utils import (plotEvidenceMap2D, plotPointRobot2D,
                                    plotSignedDistanceField2D)
from gpmp2.utils.signedDistanceField2D import signedDistanceField2D
from gtsam import (DoglegOptimizer, DoglegParams, GaussNewtonOptimizer,
                   GaussNewtonParams, NonlinearFactorGraph, Point2, Point3,
                   PriorFactorVector, Values, noiseModel)
from gtsam.symbol_shorthand import V, X

dataset = generate2Ddataset("MultiObstacleDataset")
rows = dataset.rows
cols = dataset.cols
cell_size = dataset.cell_size
origin_point2 = Point2(dataset.origin_x, dataset.origin_y)

# Signed Distance field
field = signedDistanceField2D(dataset.map, cell_size)
sdf = PlanarSDF(origin_point2, cell_size, field)

figure1 = plt.figure(0)
axis1 = figure1.gca()  # for 3-d, set gca(projection='3d')
plotSignedDistanceField2D(figure1, axis1, field, dataset.origin_x,
                          dataset.origin_y, dataset.cell_size)

# settings
total_time_sec = 10.0
total_time_step = 20
total_check_step = 50.0
delta_t = total_time_sec / total_time_step
check_inter = int(total_check_step / total_time_step - 1)

use_GP_inter = True

# point robot model
pR = PointRobot(2, 1)
spheres_data = np.asarray([0.0, 0.0, 0.0, 0.0, 1.5])
nr_body = spheres_data.shape[0]
sphere_vec = BodySphereVector()
sphere_vec.push_back(
    BodySphere(int(spheres_data[0]), spheres_data[4],
               Point3(spheres_data[1:4])))
pR_model = PointRobotModel(pR, sphere_vec)

# GP
Qc = np.identity(2)
Qc_model = noiseModel.Gaussian.Covariance(Qc)

# Obstacle avoid settings
cost_sigma = 0.5
epsilon_dist = 4.0

# prior to start/goal
pose_fix = noiseModel.Isotropic.Sigma(2, 0.0001)
vel_fix = noiseModel.Isotropic.Sigma(2, 0.0001)

# start and end conf
start_conf = np.asarray([0, 0])
start_vel = np.asarray([0, 0])
end_conf = np.asarray([17, 14])
end_vel = np.asarray([0, 0])
avg_vel = (end_conf / total_time_step) / delta_t

# plot param
pause_time = total_time_sec / total_time_step

# init optimization
graph = NonlinearFactorGraph()
init_values = Values()

for i in range(0, total_time_step + 1):
    key_pos = X(i)
    key_vel = V(i)

    #% initialize as straight line in conf space
    pose = start_conf * float(total_time_step - i) / float(
        total_time_step) + end_conf * i / float(total_time_step)
    vel = avg_vel
    print(pose)
    init_values.insert(key_pos, pose)
    init_values.insert(key_vel, vel)

    #% start/end priors
    if i == 0:
        graph.push_back(PriorFactorVector(key_pos, start_conf, pose_fix))
        graph.push_back(PriorFactorVector(key_vel, start_vel, vel_fix))
    elif i == total_time_step:
        graph.push_back(PriorFactorVector(key_pos, end_conf, pose_fix))
        graph.push_back(PriorFactorVector(key_vel, end_vel, vel_fix))

    # GP priors and cost factor
    if i > 0:
        key_pos1 = X(i - 1)
        key_pos2 = X(i)
        key_vel1 = V(i - 1)
        key_vel2 = V(i)

        temp = GaussianProcessPriorLinear(key_pos1, key_vel1, key_pos2,
                                          key_vel2, delta_t, Qc_model)
        graph.push_back(temp)

        #% cost factor
        graph.push_back(
            ObstaclePlanarSDFFactorPointRobot(key_pos, pR_model, sdf,
                                              cost_sigma, epsilon_dist))

        #% GP cost factor
        if use_GP_inter and check_inter > 0:
            for j in range(1, check_inter + 1):
                tau = j * (total_time_sec / total_check_step)
                graph.add(
                    ObstaclePlanarSDFFactorGPPointRobot(
                        key_pos1,
                        key_vel1,
                        key_pos2,
                        key_vel2,
                        pR_model,
                        sdf,
                        cost_sigma,
                        epsilon_dist,
                        Qc_model,
                        delta_t,
                        tau,
                    ))

use_trustregion_opt = True

if use_trustregion_opt:
    parameters = DoglegParams()
    parameters.setVerbosity("ERROR")
    optimizer = DoglegOptimizer(graph, init_values, parameters)
else:
    parameters = GaussNewtonParams()
    # parameters.setRelativeErrorTol(1e-5)
    # parameters.setMaxIterations(100)
    parameters.setVerbosity("ERROR")
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters)

print(f"Initial Error = {graph.error(init_values)}\n")

optimizer.optimizeSafely()
result = optimizer.values()

print(f"Final Error = {graph.error(result)}\n")

#%% plot final values
figure = plt.figure(1)
axis = figure.gca()
# plot world
plotEvidenceMap2D(figure, axis, dataset.map, dataset.origin_x,
                  dataset.origin_y, cell_size)
for i in range(total_time_step + 1):
    axis.set_title("Optimized Values")
    # plot arm
    conf = result.atVector(X(i))
    plotPointRobot2D(figure, axis, pR_model, conf)
    plt.pause(pause_time)

plt.show()
