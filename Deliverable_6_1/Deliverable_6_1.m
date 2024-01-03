addpath(fullfile('..', 'src'));

close all
clear all
clc

% rocket initialization
Ts = 1/20;
H = 4; % Horizon length in seconds

% NOTE: rocket and controlelr objects are re-initialized each time to
% ensure no influence among different simulations


%% NONLINEAR controller

% rocket initialization
rocket = Rocket(Ts);
nmpc = NmpcControl(rocket, H);

% MPC reference with default maximum roll = 15 deg
ref = @(t_, x_) ref_TVC(t_);

x0 = zeros(12,1);

%Closed loop simulation;
Tf = 30;
rocket.anim_rate = 2;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
ph = rocket.plotvis(T, X , U , Ref);
ph.fig.Name = 'Nonlinear MPC'; % Set a figure title

%% NONLINEAR controller w/ roll_max = 50°

% rocket initialization
rocket = Rocket(Ts);
nmpc = NmpcControl(rocket, H);

% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref = @(t_, x_) ref_TVC(t_, roll_max);

x0 = zeros(12,1);

%Closed loop simulation;
Tf = 30;
rocket.anim_rate = 2;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
ph = rocket.plotvis(T, X , U , Ref);
ph.fig.Name = 'Nonlinear MPC - roll_max = 50 deg'; % Set a figure title

%% LINEAR controller w/ roll_max = 50°
% Linear controllers are from Deliverable 4.1

% rocket initialization 
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref = @(t_, x_) ref_TVC(t_, roll_max);

x0 = zeros(12,1);

%Closed loop simulation;
Tf = 30;
rocket.anim_rate = 2;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);
ph = rocket.plotvis(T, X , U , Ref);
ph.fig.Name = 'Linear MPC - roll_max = 50 deg'; % Set a figure title
