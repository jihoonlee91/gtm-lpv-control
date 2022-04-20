function [CX,CY,CZ,Cl,Cm,Cn,CD,CL] = aero_model(alp,bet,VT,h,eta,P,Q,R,dela,dele,delr)

global alp_grd eta1_grd eta2_grd dele_grd CD_grd CL_grd Cm_grd

eta1 = eta(1);
eta2 = eta(2);

% trimming

alp_trm  = min(max(min( alp_grd), alp),max( alp_grd));
dele_trm = min(max(min(dele_grd),dele),max(dele_grd));
eta1_trm = min(max(min(eta1_grd),eta1),max(eta1_grd));
eta2_trm = min(max(min(eta2_grd),eta2),max(eta2_grd));

% longitudinal coefficient interpolation

CD = interpn(alp_grd,dele_grd,eta1_grd,eta2_grd,CD_grd,...
             alp_trm,dele_trm,eta1_trm,eta2_trm);
CL = interpn(alp_grd,dele_grd,eta1_grd,eta2_grd,CL_grd,...
             alp_trm,dele_trm,eta1_trm,eta2_trm);
Cm = interpn(alp_grd,dele_grd,eta1_grd,eta2_grd,Cm_grd,...
             alp_trm,dele_trm,eta1_trm,eta2_trm);
         
% lateral coefficient

CC = 0;
Cl = 0;
Cn = 0;

% coordinate change from wind to body

CX = cos(alp)*cos(bet)*(-CD) -cos(alp)*sin(bet)*(-CC) -sin(alp)*(-CL);
CY =          sin(bet)*(-CD)          +cos(bet)*(-CC)        +0*(-CL);
CZ = cos(bet)*sin(alp)*(-CD) -sin(alp)*sin(bet)*(-CC) +cos(alp)*(-CL);

end