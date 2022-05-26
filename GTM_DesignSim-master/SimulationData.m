%%
setup
MWS               = init_design('GTM');
target            = struct('gamma',0,'tas',85); % target trim condition [knot]
MWS.StatesInp(9)  = 3000; % set altitude [ft]
MWS.bias.morphing = -12.5; % set morphing parameter
loadmws(MWS,'gtm_design'); % load MWS
[MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose); % trim calculation
loadmws(MWS,'gtm_design'); % load MWS

%%
[tsim,xsim,ysim]=sim('gtm_design',[0 10]);
