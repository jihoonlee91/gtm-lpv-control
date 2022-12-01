%% Setup 
close all; clear all; clc % initialization
setup % setup simulink model

%% trimming
trim_gtm

%% linearization
linearize_gtm

%% longitudinal autopilot
design_lon_autopilot

%% lateral-directional autopilot
design_lat_autopilot

%% longitudinal CAS
design_lon_cas

%% lateral-directional CAS
design_lat_cas

%%
save('data/lpv_design_result.mat')