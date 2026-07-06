clc;
clear;
close all;

%% Make all project folders visible

addpath(genpath(pwd));

%% Tyre Data

Front = FrontTyre();
Rear  = RearTyre();

%% Vehicle Data

car = VehicleData();

%% Maximum Speed + Slip Angles

Result = FindSpeed(car,Front,Rear);

%% Verify Radius using Rear Slip Angles

Angle = FindAngle(Result,car);