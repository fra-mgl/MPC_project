addpath(fullfile('..', 'src'));

close all
clear all
clc

% rocket initialization
Ts = 1/20;
H = 4; % Horizon length in seconds

% NOTE: rocket and controlelr objects are re-initialized each time to
% ensure no influence among different simulations

%% OPEN-LOOP

% x0 = zeros(12,1);
% ref = -ones(4,1);
% % Evaluate once and plot optimal open−loop trajectory,
% % pad last input to get consistent size with time and state
% [u, T_opt, X_opt, U_opt] = nmpc.get_u(x0, ref);
% U_opt(:,end+1) = nan;
% ph = rocket.plotvis(T_opt, X_opt, U_opt, ref);
% ph.fig.Name = 'Open-loop nonlinear MPC.';


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

%% LINEAR controller w/ roll_max = 50°
% addpath(fullfile('..', 'Deliverable_4_1')); % to get controllers from this directory

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

% safely remove path of the other deliverable
% rmpath(fullfile('..', 'Deliverable_4_1'));

% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref = @(t_, x_) ref_TVC(t_, roll_max);

x0 = zeros(12,1);

%Closed loop simulation;
Tf = 30;
rocket.anim_rate = 2;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);
ph = rocket.plotvis(T, X , U , Ref);

