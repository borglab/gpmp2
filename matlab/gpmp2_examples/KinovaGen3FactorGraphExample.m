% 7DOF Kinova Gen3 manipulator example with factor graph in MATLAB
% @author Matthew King-Smith
% @date 05-12-2023

close all;
clear;

% import dynamic libraries
import gtsam.*
import gpmp2.*

%% Generate dataset
dataset = generate3Ddataset('KinovaBoxDataset');
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
origin_point3 = Point3(origin');
cell_size = dataset.cell_size;

% sdf
disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);
disp('calculating signed distance field done');

% arm: KinovaGen3
arm = generateArm('KinovaGen3');

% start and goal joint position and velocity configurations
start_conf = zeros(7,1);
end_conf = [-pi/4, pi/3, 0, -pi/5, 0, 0, 0]';
start_vel = zeros(7,1);
end_vel = zeros(7,1);

% plotting colors
green = [0.4667 0.6745 0.1882];
cyan = [0, 1, 1];

% plot problem setting
fig1 = figure(1);
clf;
hold on;
set(fig1,'Name', 'Problem Settings');
title('Problem Setup')
plotRobotModel(arm, start_conf);
plotRobotModel(arm, end_conf, green);
plotMap3D(dataset.corner_idx, origin, cell_size);
xlabel('x');
ylabel('y');
zlabel('z');
grid on, view(3)
hold off;

%% settings
total_time_sec = 10;
total_time_step = 50;
total_check_step = 100;
delta_t = total_time_sec / total_time_step;
check_inter = total_check_step / total_time_step - 1;

% GP
Qc = 1 * eye(7);
Qc_model = noiseModel.Gaussian.Covariance(Qc); 

% obstacle cost and hinge loss distance settings
cost_sigma = 0.02;
epsilon_dist = 0.1;

% noise model
fix_sigma = 0.0001;
pose_fix_model = noiseModel.Isotropic.Sigma(7, fix_sigma);
vel_fix_model = noiseModel.Isotropic.Sigma(7, fix_sigma);

% init sdf
sdf = SignedDistanceField(origin_point3, cell_size, size(field, 1), ...
    size(field, 2), size(field, 3));
for z = 1:size(field, 3)
    sdf.initFieldData(z-1, field(:,:,z)');
end

%% initial traj
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);

% plot initial trajectory
for i=0:total_time_step
    hold on;
    % plot arm
    conf = init_values.atVector(symbol('x', i));
    plotRobotModel(arm, conf, cyan);
    pause(0.1)
end
hold off;

%% init optimization
graph = NonlinearFactorGraph;

for i = 0 : total_time_step
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);
   
    % priors
    if i==0
        graph.add(PriorFactorVector(key_pos, start_conf, pose_fix_model));
        graph.add(PriorFactorVector(key_vel, start_vel, vel_fix_model));
    elseif i==total_time_step
        graph.add(PriorFactorVector(key_pos, end_conf, pose_fix_model));
        graph.add(PriorFactorVector(key_vel, end_vel, vel_fix_model));
    end
    
    % GP priors and cost factor
    if i > 0
        key_pos1 = symbol('x', i-1);
        key_pos2 = symbol('x', i);
        key_vel1 = symbol('v', i-1);
        key_vel2 = symbol('v', i);
        graph.add(GaussianProcessPriorLinear(key_pos1, key_vel1, ...
            key_pos2, key_vel2, delta_t, Qc_model));
        
        % cost factor
        graph.add(ObstacleSDFFactorArm(...
            key_pos, arm, sdf, cost_sigma, epsilon_dist));
               
        % GP cost factor
        if check_inter > 0
            for j = 1:check_inter
                tau = j * (total_time_sec / total_check_step);
                graph.add(ObstacleSDFFactorGPArm( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    arm, sdf, cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
            end
        end
    end
end

%% Factor graph optimization
solver = 'LM'; %'DL','GN'

if strcmp(solver,'LM')
    parameters = LevenbergMarquardtParams;
    parameters.setVerbosity('ERROR');
    parameters.setlambdaInitial(1000.0);
    optimizer = LevenbergMarquardtOptimizer(graph, init_values, parameters);
elseif strcmp(solver,'DL')
    parameters = DoglegParams;
    parameters.setVerbosity('ERROR');
    optimizer = DoglegOptimizer(graph, init_values, parameters);
elseif strcmp(solver,'GN')
    parameters = GaussNewtonParams;
    parameters.setVerbosity('ERROR');
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
end


fprintf('Initial Error = %d\n', graph.error(init_values));

optimizer.optimize();

result = optimizer.values();
fprintf(' Final Error = %d\n', graph.error(result));

%% Plot results

% plot final values
fig2 = figure(2);
set(fig2, 'Name', 'Optimization Results');
clf;
title('Result Values');
% plot world, initial and terminal configurations
hold on;
plotMap3D(dataset.corner_idx, origin, cell_size);
plotRobotModel(arm, start_conf);
plotRobotModel(arm, end_conf, green);
xlabel('x');
ylabel('y');
zlabel('z');
grid on, view(3);
hold off;
for i=0:total_time_step
    hold on;
    % plot arm
    conf = result.atVector(symbol('x', i));
    plotRobotModel(arm, conf, cyan);
    pause(0.1);
end
hold off
