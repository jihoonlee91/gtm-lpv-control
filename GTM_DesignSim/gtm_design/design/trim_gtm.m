% scheduling parameter gridding
sas_grid  = 60:10:120; % synthetic airspeed grid points [kt]
alt_grid  = 0:5000:10000; % altitude grid points [ft]
mor_grid  = -25:12.5:0; % symmetric morphing parameter grid points [%]
num_sas   = numel(sas_grid); % number of sas grid points
num_alt   = numel(alt_grid); % number of alt grid points
num_mor   = numel(mor_grid); % number of mor grid points
domain = rgrid({'rho1_sas_param','rho2_alt_param','rho3_mor_param'},{sas_grid,alt_grid,mor_grid}); % parameter grid

clearvars MWS_grid sys_sas_trim sys_tas_trim sys_alt_trim sys_mor_trim sys_alp_trim sys_pitch_trim sys_throttle_trim sys_aileron_trim sys_elevator_trim sys_rudder_trim % clear old variables

% trim point by point
for k = 1:num_mor
for j = 1:num_alt
for i = 1:num_sas
    sas0 = sas_grid(i); % target sas [kt]
    alt0 = alt_grid(j); % target alt [ft]
    mor0 = mor_grid(k); % target mor [%]
    tas0 = sas2tas(sas0,alt0); % target tas [kts]

    fprintf('trimming... #%d @ TAS: %d kt (SAS: %d kt), Alt: %d ft, Mor: %.3g %%\n',sub2ind([num_sas num_alt num_mor],i,j,k),tas0,sas0,alt0,mor0) % display progress
    [MWS,Xtrim,Trimcond,Err] = trimgtm_morph_try(tas0,alt0,mor0); % trim calculation
    MWS_grid(i,j,k) = MWS; % keep MWS for linearization
    
    % convert trim conditions to 'y=x' systems
    sys_sas_trim(:,:,i,j,k)      = ss(0,0,0,Trimcond.sas); % [kts]
    sys_tas_trim(:,:,i,j,k)      = ss(0,0,0,Trimcond.tas); % [kts]
    sys_alt_trim(:,:,i,j,k)      = ss(0,0,0,Trimcond.altitude); % [ft]
    sys_mor_trim(:,:,i,j,k)      = ss(0,0,0,Trimcond.morphing); % [%]
    sys_alp_trim(:,:,i,j,k)      = ss(0,0,0,Trimcond.alpha); % [deg]
    sys_pitch_trim(:,:,i,j,k)    = ss(0,0,0,Trimcond.pitch); % [deg]
    sys_throttle_trim(:,:,i,j,k) = ss(0,0,0,Trimcond.throttle); % [%]
    sys_aileron_trim(:,:,i,j,k)  = ss(0,0,0,Trimcond.aileron); % [deg]
    sys_elevator_trim(:,:,i,j,k) = ss(0,0,0,Trimcond.elevator); % [deg]
    sys_rudder_trim(:,:,i,j,k)   = ss(0,0,0,Trimcond.rudder); % [deg]
end
end
end

% scheduled trim parameter
sch_sas_trim      = pss(sys_sas_trim,domain); % [deg]
sch_tas_trim      = pss(sys_tas_trim,domain); % [deg]
sch_alt_trim      = pss(sys_alt_trim,domain); % [deg]
sch_mor_trim      = pss(sys_mor_trim,domain); % [deg]
sch_alp_trim      = pss(sys_alp_trim,domain); % [deg]
sch_pitch_trim    = pss(sys_pitch_trim,domain); % [deg]
sch_throttle_trim = pss(sys_throttle_trim,domain); % [%]
sch_aileron_trim  = pss(sys_aileron_trim,domain); % [deg]
sch_elevator_trim = pss(sys_elevator_trim,domain); % [deg]
sch_rudder_trim   = pss(sys_rudder_trim,domain); % [deg]

% parameter packaging
sch_param.sch_sas_trim      = sch_sas_trim;
sch_param.sch_tas_trim      = sch_tas_trim;
sch_param.sch_alt_trim      = sch_alt_trim;
sch_param.sch_mor_trim      = sch_mor_trim;
sch_param.sch_alp_trim      = sch_alp_trim;
sch_param.sch_pitch_trim    = sch_pitch_trim;
sch_param.sch_throttle_trim = sch_throttle_trim;
sch_param.sch_aileron_trim  = sch_aileron_trim;
sch_param.sch_elevator_trim = sch_elevator_trim;
sch_param.sch_rudder_trim   = sch_rudder_trim;
