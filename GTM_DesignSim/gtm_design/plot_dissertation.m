% figures for jihoon lee's phd dissertation
setup
load lpv_design_result.mat
set(groot,'DefaultLineLineWidth',1)

%% model investigation
plot_eta % aerodynamic coefficient
plot_trim_map % flight envelope

%% modeling
plot_parameter_grid % scheduling parameter grid
plot_trim_condition % trim alpha and throttle
plot_pzmap % open-loop pole

%% control design
plot_bode_all % closed-loop bode plot
plot_cas_lon_step % longitudinal cas step response
plot_cas_lat_step % lateral-directional cas step response
plot_auto_lon_step % longitudinal autopilot step response
plot_auto_lat_step % lateral-directional autopilot step response

%% simulation
plot_pushover_and_pullup_lpvu % push-over and pull-up maneuver with lpv controller
plot_pushover_and_pullup_hinf % push-over and pull-up maneuver with h-infinity controller
plot_highg_turn_lpvu % high-g turn with lpv controller
plot_highg_turn_hinf % high-g turn with h-infinity controller
plot_waypoint_following % waypoint following with lpv and h-infinity controllers
plot_orbit_following % circular trajectory tracking with lpv and h-infinity controllers
plot_helical_climb % helical climbing with lpv and h-infinity controllers under fast morphing