addpath(fullfile('..', 'src'));

close all
clear all
clc

%% TODO: This file should produce all the plots for the deliverable


rocket = Rocket(Ts);
Tf = 2.0;                                          % Simulation end time
x0=[deg2rad([0 0 0, 0 0 0]), 0 0 0, 0 0 0]';%(w,phi,v,p)Initialstate
u  = [deg2rad([0 0]),  62.443,    0  ]';

[T, X, U] = rocket.simulate(x0, Tf, u);
rocket.anim_rate = 1;
rocket.vis(T, X, U);
% (d1 d2 Pavg Pdiff) Constant input
% Simulate unknown, nonlinear model
% Visualize at 1.0x real−time

%%
Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim()       % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us) % Linearize the nonlinear model about trim point
