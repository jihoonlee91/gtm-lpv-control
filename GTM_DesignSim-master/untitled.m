MWS = init_design('GTM_T2');

MWS.StatesInp(9) = 1000;
loadmws(MWS,'gtm_design');

target = struct('gamma',0,'tas',80,'alpha',4.5);
PitchSurf = 'elev';
verbose = 1;

[MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose);

loadmws(SimVars);

[sys,londyn,latdyn]=linmodel();
