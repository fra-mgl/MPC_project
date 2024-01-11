addpath(fullfile('..', 'src'));

close all
clear all
clc

Ts = 1/40; % Higher sampling rate for this part!

%% TEST 1
Tf = 5; % simulation time
rocket = Rocket(Ts);
H = 3; % Horizon length in seconds
rocket.mass = 1.75;

expected_delay = 3; % number of steps
rocket.delay = 3; % 0 if not specified, ACTUAL DELAY

nmpc = NmpcControl(rocket, H, expected_delay);
x0 = zeros(12, 1);
ref = [0.5, 0, 1, deg2rad(65)]';

% SIMULATION
rocket.anim_rate = 1;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
ph = rocket.plotvis(T, X , U , Ref);
ph.fig.Name = 'Fully delay-compensating controller, delay=0.075s, horizon H=3s'; % Set a figure title

%% TEST 2
Tf = 2; % simulation time
rocket = Rocket(Ts);
H = 4; % Horizon length in seconds
rocket.mass = 1.75;

expected_delay = 5; % number of steps
rocket.delay = 5; % 0 if not specified, ACTUAL DELAY

nmpc = NmpcControl(rocket, H, expected_delay);
x0 = zeros(12, 1);
ref = [0.5, 0, 1, deg2rad(65)]';

% SIMULATION
rocket.anim_rate = 1;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
ph = rocket.plotvis(T, X , U , Ref);
ph.fig.Name = 'Fully delay-compensating controller, delay=0.125s, horizon H=4s'; % Set a figure title

%% TEST 3
Tf = 5; % simulation time
rocket = Rocket(Ts);
H = 2.5; % Horizon length in seconds
rocket.mass = 1.75;

expected_delay = 1; % number of steps
rocket.delay = 3; % 0 if not specified, ACTUAL DELAY

nmpc = NmpcControl(rocket, H, expected_delay);
x0 = zeros(12, 1);
ref = [0.5, 0, 1, deg2rad(65)]';

% SIMULATION
rocket.anim_rate = 1;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
ph = rocket.plotvis(T, X , U , Ref);
ph.fig.Name = 'Partially delay-compensating controller, expected delay=0.025s, actual delay = 0.075s, horizon H=2.5s'; % Set a figure title

%% TEST 4
Tf = 5; % simulation time
rocket = Rocket(Ts);
H = 2.5; % Horizon length in seconds
rocket.mass = 1.75;

expected_delay = 1; % number of steps
rocket.delay = 4; % 0 if not specified, ACTUAL DELAY

nmpc = NmpcControl(rocket, H, expected_delay);
x0 = zeros(12, 1);
ref = [0.5, 0, 1, deg2rad(65)]';

% SIMULATION
rocket.anim_rate = 1;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
ph = rocket.plotvis(T, X , U , Ref);
ph.fig.Name = 'Partially delay-compensating controller, expected delay=0.025s, actual delay = 0.1s, horizon H=2.5s'; % Set a figure title

