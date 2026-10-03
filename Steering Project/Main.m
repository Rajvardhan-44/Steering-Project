%% Steering Geometry Solver
clc;
clear;
close all;

%% Make all project folders visible
addpath(genpath(pwd));
fprintf('=========================================\n');
fprintf('   Steering Geometry Performance Solver\n');
fprintf('=========================================\n\n');

%% Load Vehicle & Tyre Data
Front = FrontTyre();
Rear  = RearTyre();
car   = VehicleData();

%% Tyre Curve Check (Lateral Force vs Slip Angle at different loads)
% Edit these arrays freely to check the tyre curve at whatever
% vertical loads you want (e.g. static load, static +/- load transfer).
Fz_front = [1500 2444 3200 4000];   % N
Fz_rear  = [2500 3850 4800 6000];   % N
gamma_check = 0;                     % camber angle (rad), for this check
PlotTyreCurve(Front, Fz_front, gamma_check, 'Front Tyre');
PlotTyreCurve(Rear,  Fz_rear,  gamma_check, 'Rear Tyre');

%% Run Solver
fprintf('Running Solver...\n\n');
Result = Solver(car, Front, Rear);
% Synthesize Steering Mechanism
[MechParams, MechFit] = SteeringSynthesis(car, Result);
fprintf('\nSolver Finished Successfully.\n');

%% Display Results
Info(car, Result);