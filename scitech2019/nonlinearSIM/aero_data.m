global alp_grd eta1_grd eta2_grd dele_grd

alp_grd  = (-10:0.5:20)*pi/180;
dele_grd = [SPEC.dele_ll 0 SPEC.dele_ul];
eta1_grd = 0:0.5:1;
eta2_grd = 0:0.5:1;

global CD_grd CL_grd Cm_grd

load('aero_coeff.mat') % raw data

CF_S = [0.840 0.820 0.810;
         0.984 0.962 0.949;
         1.129 1.104 1.087]; % reference planform area used in XFLR5
CF_cbar = [0.288 0.299 0.351;
            0.275 0.286 0.336;
            0.265 0.276 0.325]; % reference mac used in XFLR5
CF_b = [3.000 2.810 2.354;
         3.722 3.490 2.908;
         4.446 4.170 3.462]; % reference span used in XFLR5

CD_f = 0.4; % fuselage drag coefficient
S_f = 0.084; % fuselage section area

% [alp dele eta1 eta2]
CL_grd = permute(CL,[4 3 1 2]);
CD_grd = permute(CD,[4 3 1 2])+CD_f*S_f/CF_S(1,1);
Cm_grd = permute(CM,[4 3 1 2]);

for ii = 1:3
    for jj = 1:3
        % compensation for different normalization parameter
        CL_grd(:,:,ii,jj) = CL_grd(:,:,ii,jj)./CF_S(1,1).*CF_S(ii,jj);
        CD_grd(:,:,ii,jj) = CD_grd(:,:,ii,jj)./CF_S(1,1).*CF_S(ii,jj);
        Cm_grd(:,:,ii,jj) = Cm_grd(:,:,ii,jj)./CF_S(1,1).*CF_S(ii,jj)./CF_cbar(1,1).*CF_cbar(ii,jj);
    end
end