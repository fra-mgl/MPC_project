addpath(fullfile('..', 'src'));

close all
clear all
clc

% %% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time
Tf = 8; % simulation end time

rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
H = 7; % Horizon length in seconds


%%

% simulate with disturbance
x0 = [zeros(1, 9), 1 0 3]';
ref = [1.2, 0, 3, 0]';

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

% [T4, X4, U4, Ref, Z_hat4] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);
[T4, X4, U4, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);

% Visualize
rocket.anim_rate = 0.7; % Increase this to make the animation faster
ph4 = rocket.plotvis(T4, X4, U4, Ref);
ph4.fig.Name = 'Merged lin. MPC  disturbance without estimator'; % Set a figure title

% rmpath(fullfile('..', 'Deliverable_4_1'));


%SIMULATE WITH UPDATED FROM DELIVERABLE 4
disp("Simulating using controllers from Deliverable 5.1");
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

[T, X, U, Ref, Z_hat] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);

% Visualize
rocket.anim_rate = 0.7; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC offset free tracking'; % Set a figure title

%[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);
