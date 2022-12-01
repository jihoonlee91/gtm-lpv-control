function [MWS,Xtrim,Trimcond,Err] = trimgtm_morph(tas,alt,mor)

loadmws(init_design(),'gtm_design'); % initialize model

MWS = grabmws('gtm_design'); % grab model

% set target
target = struct('tas',tas,'gamma',0);
ini.Altitude  =  alt; % ft
MWS.bias.morphing = mor; % set trim symmetric morphing parameter [%]

% set initial conditions
ini.tas       =  tas;      % knots
ini.alpha     =  3+5*rand;        % deg
ini.gamma     =  rand;        % deg
ini.stab      =  0;
ini.lat       =  37.827926944;
ini.lon       = -75.494061666;
MWS.StatesInp = SetICs(ini);

loadmws(MWS,'gtm_design');

set_param('gtm_design/Control/Controller','commented','on')
set_param('gtm_design/Guidance','commented','on')
set_param('gtm_design/Monitor','commented','on')

[MWS,Xtrim,Trimcond,Err] = trimgtm(target,'elev',0);