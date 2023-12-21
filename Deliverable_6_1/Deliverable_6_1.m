addpath(fullfile('..', 'src'));

close all
clear all
clc

Ts = 1/20;
rocket = Rocket(Ts);
H = 1; % Horizon length in seconds
nmpc = NmpcControl(rocket, H);

%% OPEN-LOOP

x0 = zeros(12,1);
ref = -ones(4,1);
% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = nmpc.get_u(x0, ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref);
ph.fig.Name = 'Open-loop nonlinear MPC.';


%% CLOSED-LOOP
% MPC reference with default maximum roll = 15 deg
ref = @(t_, x_) ref_TVC(t_);
% % MPC reference with specified maximum roll = 50 deg
% roll_max = deg2rad(50);
% ref = @(t_, x_) ref_TVC(t_, rollmax);
% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = nmpc.get_u(x, ref);
U_opt(:,end+1) = nan;
% ph = rocket.plotvis(T opt, X opt, U opt, ref);
% Tf = 30;
% [T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get u, ref);
