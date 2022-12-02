%% Setup 
close all; clear all; clc % initialization
setup % setup simulink model

%% trimming
trim_gtm

%% linearization
linearize_gtm

%% longitudinal CAS
design_lon_cas

%% lateral-directional CAS
design_lat_cas

%% longitudinal autopilot
design_lon_autopilot

%% lateral-directional autopilot
design_lat_autopilot

%% save result
save('data/lpv_design_result.mat')

%% plot result
plot_bode_all
plot_cas_lon_step
plot_cas_lat_step
plot_auto_lon_step
plot_auto_lat_step
