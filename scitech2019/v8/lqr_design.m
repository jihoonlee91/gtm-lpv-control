function param = lqr_design(h0,VT0,eta0,alp_trim,delt_trim,dele_trim)

%% linearization
bet_trim = 0;
omega_trim = zeros(3,1);
th_trim = alp_trim;
angle_trim = [0;th_trim;0];
wind_trim = [VT0;alp_trim;bet_trim];
p0 = [0;0;-h0];

[Alon,Blon,~,~] = lin_mod([wind_trim;omega_trim;angle_trim;p0],[delt_trim;dele_trim;0;0],eta0); % linearize
Alon(4,:) = Alon(4,:)-Alon(2,:); Blon(4,:) = Blon(4,:)-Blon(2,:); % state transformation from theta to gamma
x_trim_lon = [VT0;alp_trim;omega_trim(2);th_trim-alp_trim]; % longitudinal state trim value
x_trim_lat = zeros(4,1); % lateral state trim value

%% control design
% longitudinal state = [VT alp Q gamma]
% performance output = [VT gamma]
% state augmentation
Elon   = [1 0 0 0
          0 0 0 1];
AlonAug = [Alon, zeros(4,2);
           Elon, zeros(2,2)];
BlonAug = [Blon; zeros(2,2)];
Qlon = diag([1 100 10 100 1 100]);
Rlon = diag([1 100]);
Klon = lqr(AlonAug,BlonAug,Qlon,Rlon);
% eig(AlonAug-BlonAug*Klon)

% lateral state = [bet P R ph]
% performance output = [bet ph]
% state augmentation
% Elat   = [1 0 0 0;
%           0 0 0 1];
% AlatAug = [Alat, zeros(4,2); 
%            Elat, zeros(2,2)];
% BlatAug = [Blat; zeros(2,2)];
% Qlat = eye(6);
% Rlat = eye(2)*100;
% Klat = lqr(AlatAug,BlatAug,Qlat,Rlat);
Klat = zeros(size(Klon));

% packaging
param.Klon = Klon;
param.Klat = Klat;
param.u_trim = [delt_trim;0;dele_trim;0];
param.x_trim_lon = x_trim_lon;
param.x_trim_lat = x_trim_lat;