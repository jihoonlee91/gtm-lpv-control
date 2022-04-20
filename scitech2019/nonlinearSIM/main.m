%% initialize
close all
clear
clc

load('matlab.mat')

%% simulation setup
tf  = 5; % simulation time [s]
dt  = 0.01; % integration time step during simulation [s]

tic
simOut = sim('model','SimulationMode','normal', ...
            'StopTime', num2str(tf), ...
            'FixedStep', num2str(dt), ...
            'SaveOutput','on','OutputSaveName','youtNew',...
            'SignalLogging','on','SignalLoggingName','logsout');
toc