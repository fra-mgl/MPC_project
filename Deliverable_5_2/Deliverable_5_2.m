addpath(fullfile('..', 'src'));

close all
clear all
clc

% %% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sample time
Tf = 20; % simulation end time

rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
H = 7; % Horizon length in seconds


%%
clc;
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);


% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

% simulate with disturbance
x0 = [zeros(1, 9), 1 0 3]';
ref = [1.2, 0, 3, 0]';

rocket.mass = 2.13;
rocket.mass_rate = -0.27;

%[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);

[T, X, U, Ref, Z_hat] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);

% Visualize
rocket.anim_rate = 1; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title


% [T, X, U, Ref, Z_hat] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);
% Z_hat(end,:)

%%

% TODO: simulazione con constraints sull'input come in precedenza (56-80)
% si vede che il razzo comincia a salire perchè la massa è diminuita

% abbassiamo constraint sull'input perchè perdendo massa, hovering non è
% più vincolato a 56% ma a un valore minore di Pavg
% facendo così la reference viene traccata bene finchè c'è carburante nel
% razzo
% quando questo finisce, il razzo inizia la caduta libera e per compensare
% il controllore satura l'input a 80%
% dopo alcuni secondi in questa situazione, il problema MPC diventa
% infeasible ma a questo punto il contesto fisico in cui ci troviamo perde
% di significato (ie. razzo in caduta libera e senza possibilità di
% controllo perchè il carburante è finito