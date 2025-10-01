% 7DOF Kinova Gen3 manipulator example with factor graph in MATLAB
% @author Matthew King-Smith
% @date 05-17-2023

close all;
clear;

% import dynamic libraries
import gtsam.*
import gpmp2.*

% flag for recording a video of animation
rcdVid = false;

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
start_vel = zeros(7,1);

% convert feasible terminal configuration to an end-effector SE(3) pose 
end_conf = [-pi/4, pi/3, 0, -pi/5, 0, 0, 0]';
jposes = arm.fk_model().forwardKinematicsPose(end_conf);
ee_des_pose = Pose3(Rot3.Ypr(jposes(1,end),jposes(2,end),jposes(3,end)), ...
    Point3(jposes(4,end),jposes(5,end),jposes(6,end)));

% ee_des_vel = [linear_rate; angular_rate]^T
ee_des_vel = zeros(6,1);

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

% noise model for end-effector pose and rates
ee_fix_model = noiseModel.Isotropic.Sigma(12, 1e-5);

% init sdf
sdf = SignedDistanceField(origin_point3, cell_size, size(field, 1), ...
    size(field, 2), size(field, 3));
for z = 1:size(field, 3)
    sdf.initFieldData(z-1, field(:,:,z)');
end

%% initial traj and optimization 
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);

graph = NonlinearFactorGraph;

for i = 0 : total_time_step
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);
   
    % priors
    if i==0
        graph.add(PriorFactorVector(key_pos, start_conf, pose_fix_model));
        graph.add(PriorFactorVector(key_vel, start_vel, vel_fix_model));
    elseif i==total_time_step
        graph.add(WorkspacePoseVelocityPrior(key_pos, key_vel,...
                                             ee_fix_model, arm.fk_model(),...
                                             ee_des_pose, ee_des_vel));
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
    parameters.setRelativeErrorTol(1e-9);
    parameters.setMaxIterations(100000);
    parameters.setAbsoluteErrorTol(1e-9);
    optimizer = LevenbergMarquardtOptimizer(graph, init_values, parameters);
elseif strcmp(solver,'DL')
    parameters = DoglegParams;
    parameters.setVerbosity('ERROR');
    parameters.setMaxIterations(1000.0);
    optimizer = DoglegOptimizer(graph, init_values, parameters);
elseif strcmp(solver,'GN')
    parameters = GaussNewtonParams;
    parameters.setVerbosity('ERROR');
    parameters.setMaxIterations(1000.0);
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
end


fprintf('Initial Error = %d\n', graph.error(init_values));

optimizer.optimize();

result = optimizer.values();

fprintf(' Final Error = %d\n', graph.error(result));

%% Save data

% joint-space positions and rates
init_joint_pos =  utilities.extractVectors(init_values,'x')';
result_joint_pos = utilities.extractVectors(result,'x')';
result_joint_rates = utilities.extractVectors(result,'v')';

numSteps = total_time_step + 1;
% workspace-space joint poses and velocities
joints_poses = nan(6,7,numSteps);
joints_vels = nan(6,7,numSteps);
for i=1:total_time_step+1
    joints_pose(:,:,i) = arm.fk_model().forwardKinematicsPose(result_joint_pos(:,i));
    joints_vel(:,:,i) = arm.fk_model().forwardKinematicsVel(result_joint_pos(:,i),result_joint_rates(:,i));
end

% correct for discontinuities when using euler angle representation
joints_pose(1:3,end,:) = unwrap(joints_pose(1:3,end,:));

% separate out translation and rotation
ee_ypr_deg = rad2deg(ee_des_pose.rotation.ypr);
ee_pos_m = ee_des_pose.translation;

%% Plot results

% plotting colors
green = [0.4667 0.6745 0.1882];
cyan = [0, 1, 1];

% Plotting sizes and settings
if ~strcmp(get(0,'defaultTextInterpreter'),'latex')
    set(0,'defaulttextinterpreter','latex');
    set(0,'DefaultLegendInterpreter','latex');
    set(0, 'defaultAxesTickLabelInterpreter','latex');
    set(0,'defaultfigurecolor',[1 1 1]);
    set(0,'defaultAxesFontName','Times New Roman');
    set(0,'DefaultAxesXGrid','on');
    set(0,'DefaultAxesYGrid','on');
    set(0,'DefaultAxesZGrid','on');
    set(0,'DefaultAxesBox','on');
    set(0,'defaultAxesFontSize',16);
    set(0,'DefaultLegendFontSize', 16);
end

% plot final values
fig1 = figure(1);
set(fig1, 'Name', 'Factore Graph Initialization and Optimization Results');
clf;
for kk = 1:1:6 
    ax(kk) = subplot(1,6,kk);
end

subplot(ax(1));
title('\textbf{Initial Guess Trajectory}');

% plot world, initial and terminal configurations
hold on;
plotMap3D(dataset.corner_idx, origin, cell_size);
plotRobotModel(arm, start_conf);
plotRobotModel(arm, end_conf, green);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
axis([-0.2 0.6 -0.3, 0.5, 0 1.4])
set(ax(1),'Position',[0.0422 -0.8174 0.2912 2.6390]);
view(140,30);

subplot(ax(2));
title('\textbf{Factor Graph Optimization Result}');
hold on;
plotMap3D(dataset.corner_idx, origin, cell_size);
plotRobotModel(arm, start_conf);
plotRobotModel(arm, end_conf, green);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
axis([-0.2 0.6 -0.3, 0.5, 0 1.4])
set(ax(2),'Position',[0.3866 -0.8236 0.2897 2.6390]);
view(140,30);

title(ax(3),'\textbf{End-Effector Plots}');
xlabel(ax(3),'Time [s]');
ylabel(ax(3),'Euler Angles [deg]')
set(ax(3),'Position',[0.7443 0.79  0.2505 0.18]);

xlabel(ax(4),'Time [s]');
ylabel(ax(4),'Postion [m]')
set(ax(4),'Position',[0.7443 0.5427 0.2505 0.18]);

xlabel(ax(5),'Time [s]');
ylabel(ax(5),'Angular Velocity [deg/s]')
set(ax(5),'Position',[0.7443 0.3 0.2505 0.18]);

xlabel(ax(6),'Time [s]');
ylabel(ax(6),'Linear Velocity [m/s]')
set(ax(6),'Position',[0.7443 0.0562 0.2505 0.18]);

axis(ax(3), [0, total_time_sec, -10, 185]);
axis(ax(4), [0, total_time_sec, min(min(joints_pose(4:6,end,:))), max(max(joints_pose(4:6,end,:)))]);
axis(ax(5), [0, total_time_sec, min(min(rad2deg(joints_vel(4:6,end,:)))), max(max(rad2deg(joints_vel(4:6,end,:))))]);
axis(ax(6), [0, total_time_sec, min(min(joints_vel(1:3,end,:))), max(max(joints_vel(1:3,end,:)))]);

clrs = {'r',green,'b'};
for kk = 1:3
    % terminal waypoint
    line(ax(3),total_time_sec,ee_ypr_deg(kk),'marker','o','markerfacecolor',clrs{kk},'markeredgecolor',clrs{kk},'linestyle','none');
    line(ax(4),total_time_sec,ee_pos_m(kk),'marker','o','markerfacecolor',clrs{kk},'markeredgecolor',clrs{kk},'linestyle','none');
    line(ax(5),total_time_sec,ee_des_vel(kk+3),'marker','o','markerfacecolor',clrs{kk},'markeredgecolor',clrs{kk},'linestyle','none');
    line(ax(6),total_time_sec,ee_des_vel(kk),'marker','o','markerfacecolor',clrs{kk},'markeredgecolor',clrs{kk},'linestyle','none');
    
    % actual poses and velocites
    ee_ypr_h(kk) = line(ax(3),nan(1,1),nan(1,1),'color',clrs{kk});
    ee_pos_h(kk) = line(ax(4),nan(1,1),nan(1,1),'color',clrs{kk});
    ee_angVel_h(kk) = line(ax(5),nan(1,1),nan(1,1),'color',clrs{kk});
    ee_linVel_h(kk) = line(ax(6),nan(1,1),nan(1,1),'color',clrs{kk});
end

legend(ax(3),ee_ypr_h, {'$\psi$','$\theta$','$\phi$'}, 'Position', [0.7445,0.875,0.0328,0.0704]);
legend(ax(4),ee_pos_h, {'$x$','$y$','$z$'}, 'Position', [0.7445,0.631,0.0317,0.0704]);
legend(ax(5),ee_angVel_h, {'$\omega_x$','$\omega_y$','$\omega_z$'}, 'Position', [0.7445,0.3,0.0363,0.0704]);
legend(ax(6),ee_linVel_h, {'$\dot{x}$','$\dot{y}$','$\dot{z}$'}, 'Position', [0.7445,0.0562,0.0317,0.0704]);

t = 0:delta_t:total_time_sec;

if rcdVid
    v = VideoWriter('KinovaGen3_PoseStabilization','Uncompressed AVI');
    open(v);
end

for i=1:total_time_step+1
    hold on;
    subplot(ax(1));
    plotRobotModel(arm, init_joint_pos(:,i), cyan);
    subplot(ax(2));
    plotRobotModel(arm, result_joint_pos(:,i), cyan);
    for kk = 1:3
        set(ee_ypr_h(kk),'Xdata',t(1:i),'Ydata',rad2deg(joints_pose(kk,end,1:i)));
        set(ee_pos_h(kk),'Xdata',t(1:i),'Ydata',joints_pose(kk+3,end,1:i));
        set(ee_angVel_h(kk),'Xdata',t(1:i),'Ydata',rad2deg(joints_vel(kk+3,end,1:i)));
        set(ee_linVel_h(kk),'Xdata',t(1:i),'Ydata',joints_vel(kk,end,1:i));
    end
    if rcdVid
        frame = getframe(gcf);
        writeVideo(v,frame);
    else
        pause(0.1);
    end
end
hold off;
if rcdVid
    close(v);
end
