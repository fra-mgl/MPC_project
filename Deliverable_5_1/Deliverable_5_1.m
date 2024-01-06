addpath(fullfile('..', 'src'));

close all
clear all
clc

% %% TODO: This files produces plots for deliverable 5.1

Ts = 1/20; % Sample time
Tf = 8; % simulation end time

rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
H = 7; % Horizon length in seconds


%%

%Data
x0 = [zeros(1, 9), 1 0 3]';
ref = [1.2, 0, 3, 0]';

%Disturbance
rocket.mass = 2.13;

%SIMULATE WITH CONTROLLER FROM DELIVERABLE 4
cd(fullfile('..', 'Deliverable_4_1'));
% change current directory to access controllers in Deliverable 4.1
disp("Simulating using controllers from Deliverable 4.1");
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
cd(fullfile('..', 'Deliverable_5_1'));
% restore current directory

% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

%Simulation without estimator, since it is not implemented in the z
%controller of deliverable 4.1
[T4, X4, U4, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);

%Visualize
rocket.anim_rate = 0.7; %
ph4 = rocket.plotvis(T4, X4, U4, Ref);
ph4.fig.Name = 'Merged lin. MPC disturbance without estimator'; 

%SIMULATE WITH UPDATED CONTROLLER FROM DELIVERABLE 5.1
disp("Simulating using controllers from Deliverable 5.1");
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

%Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

%Now simulate with estimator
[T, X, U, Ref, Z_hat] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);

%Visualize
rocket.anim_rate = 0.7; 
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC offset free tracking'; 


