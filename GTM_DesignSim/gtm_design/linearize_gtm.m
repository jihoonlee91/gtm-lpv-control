% setting for simulation
set_param('gtm_design','MinStep','auto')
set_param('gtm_design/Control/Controller','commented','on') % switch off controller
set_param('gtm_design/Guidance','commented','on') % switch off controller
set_param('gtm_design/Monitor','commented','on') % switch off controller

clearvars londyn2_grid londyn5_grid latdyn3_grid latdyn4_grid % clear old variables
for k = 1:num_mor 
for j = 1:num_alt
for i = 1:num_sas
    sas0 = sas_grid(i); % target sas [kt]
    alt0 = alt_grid(j); % target alt [ft]
    mor0 = mor_grid(k); % target mor [%]
    tas0 = sas2tas(sas0,alt0); % target tas [kts]

    % linearization
    fprintf('Linearizing... #%d @ TAS: %d kt (SAS: %d kt), Alt: %d ft, Mor: %.3g %%\n',sub2ind([num_sas num_alt num_mor],i,j,k),tas0,sas0,alt0,mor0)
    [sys,londyn,latdyn]=linmodel(MWS_grid(i,j,k),1,0,0);

    % extract short-period dynamics & modify input
    Alon2 = londyn.A(2:3,2:3); % x = alp [rad], q [rad/s]
    Blon2 = londyn.B(2:3,1); % u = dele [deg]
    Clon2 = [eye(2); londyn.A(2,2)*tas0*knots2fps 0]; % y = alp [rad], q [rad/s], az [ft/s^2]
    Dlon2 = [0; 0; londyn.B(2,1)*tas0*knots2fps];

    % eliminate roll angle dynamics
    Alat3 = latdyn.A(1:3,1:3); % x = bet [rad], p [rad/s], r [rad/s] 
    Blat3 = [latdyn.B(1:3,1:2) (latdyn.B(1:3,4)-latdyn.B(1:3,3))/2]; % u = dela [deg], delr [deg], etaa [%]
    Clat3 = eye(3); % y = x;
    Dlat3 = 0;
    
    % collect LTI models
    londyn2_grid(:,:,i,j,k) = ss(Alon2,Blon2,Clon2,Dlon2);
    latdyn3_grid(:,:,i,j,k) = ss(Alat3,Blat3,Clat3,Dlat3);

    % augment altitude dynamics & modify input
    Alon5 = [londyn.A zeros(4,1); 0 -tas0*knots2fps 0 tas0*knots2fps 0]; % x = tas [ft/s], alp [rad], q [rad/s], theta [rad], h [ft]
    Blon5 = [londyn.B(:,4)+londyn.B(:,5) londyn.B(:,1); 0 0]; % u = delt [%], dele [deg]
    Clon5 = eye(5); % y = x
    Dlon5 = 0;

    % modify input
    Alat4 = latdyn.A; % x = bet [rad], p [rad/s], r [rad/s], phi [deg]
    Blat4 = [latdyn.B(:,1:2) (latdyn.B(:,4)-latdyn.B(:,3))/2]; % u = dela [deg], delr [deg], etaa [%]
    Clat4 = eye(4); % y = x
    Dlat4 = 0;
    
    % collect LTI models
    londyn5_grid(:,:,i,j,k) = ss(Alon5,Blon5,Clon5,Dlon5);
    latdyn4_grid(:,:,i,j,k) = ss(Alat4,Blat4,Clat4,Dlat4);
end
end
end

% construct LPV model
Plon5 = pss(londyn5_grid,domain);
Plat4 = pss(latdyn4_grid,domain);
Plon2 = pss(londyn2_grid,domain);
Plat3 = pss(latdyn3_grid,domain);

save lpv_design_data.mat