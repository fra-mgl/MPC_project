addpath(fullfile('..', 'src'));

close all
clear all
clc

% %% TODO: This file produces the plot required for deliverable 5.2

Ts = 1/20; % Sample time
Tf = 20; % simulation end time

rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
H = 3; % Horizon length in seconds


%%
%Define the controllers
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);


%Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

%Data
x0 = [zeros(1, 9), 1 0 3]';
ref = [1.2, 0, 3, 0]';

%Disturbance
rocket.mass = 2.13;
rocket.mass_rate = -0.27;

%Simulate
[T, X, U, Ref, Z_hat] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);

%Visualize
rocket.anim_rate = 1; 
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC varying disturbance'; 

