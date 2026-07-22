%% Steering Geometry Solver
clc;
clear;
close all;

%% Make all project folders visible
addpath(genpath(pwd));

fprintf('=========================================\n\n');
fprintf('   Steering Geometry Performance Solver\n\n');
fprintf('=========================================\n\n');

%% Load Vehicle & Tyre Data
Front = FrontTyre();
Rear  = RearTyre();
car   = VehicleData();

%% Run Solver
fprintf('Running Solver...\n\n');

Result = Solver(car, Front, Rear);

fprintf('\nSolver Finished Successfully.\n');

%% Display Results
Info(car, Result);
