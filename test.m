clc; clf; clear;
addpath("./src")
addpath("./utils")
Ts = 1/20;
Tf = 2.0;

%% 1-step simulation
clc;
rocket = Rocket(Ts);

% state
w = [0 0 0];
phi = [0 0 0]; % alpha, beta ± 0.17
v = [0 0 0];
p = [0 0 0];

% input
d1 = 0.26; % ± 0.26
d2 = 0;
Pavg = 56.67; % 50 - 80
Pdiff = 0; % 20 - 80

u = [d1, d2, Pavg, Pdiff]';
x = [w, phi, v, p]';
x_dot = rocket.f(x, u);
display_x_dot(x_dot);

[xs, us] = rocket.trim();
sys = rocket.linearize(xs,us);

%% complete simulation
rocket = Rocket(Ts);
x0=[deg2rad([2 -2 0, -2 2 0]), 0 0 0, 0 0 0]'; %(w,phi,v,p) Initial state
u = [deg2rad([2 0]), 60, 0]';

[T, X, U] = rocket.simulate(x0, Tf, u);
rocket.anim_rate = 1.0; rocket.vis(T, X, U);