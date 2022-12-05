%% setup
close all; clear all; clc % initialization
setup % setup simulink model

%% trim and linearize
trim_gtm
linearize_gtm
save('data/lpv_design_prep.mat')

%% control design
design_lon_cas % longitudinal CAS
design_lat_cas % lateral-directional CAS
design_lon_autopilot % longitudinal autopilot
design_lat_autopilot % lateral-directional autopilot
save('data/lpv_design_result.mat')
