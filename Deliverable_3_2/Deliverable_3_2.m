addpath(fullfile('..', 'src'));

close all
clear all
clc

% %% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time
Tf = 14; % simulation end time

rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
H = 7; % Horizon length in seconds
pos_ref = -4;
gamma_ref = 0.61;

%% sys_x

% Design MPC controller
mpc_x = MpcControl_x(sys_x, Ts, H);

x0 = [0 0 0 0]'; % w_y, beta, v_x, x

% openloop
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x0, pos_ref);
U_opt(:, end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us);

% closed loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, 10, @mpc_x.get_u, pos_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);

%% sys_y

% Design MPC controller
mpc_y = MpcControl_y(sys_y, Ts, H);

x0 = [0 0 0 0]'; % w_x, alpha, v_y, y

% open loop
[u, T_opt, X_opt, U_opt] = mpc_y.get_u(x0, pos_ref);
U_opt(:, end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us);

% closed loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, x0, Tf, @mpc_y.get_u, pos_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);

%% sys_z

% Design MPC controller
mpc_z = MpcControl_z(sys_z, Ts, H);

x0 = [0 0]'; % vz, z

% openloop
[u, T_opt, X_opt, U_opt] = mpc_z.get_u(x0, pos_ref);
U_opt(:, end+1) = NaN;
U_opt = U_opt + us(3);
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us);

% closed loop
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, x0, Tf, @mpc_z.get_u, pos_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us);

%% sys_roll

% Design MPC controller
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

x0 = [0 0]'; % wz, gamma

% openloop
[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(x0, gamma_ref);
U_opt(:, end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us);

% simulation
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, x0, Tf, @mpc_roll.get_u, gamma_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);