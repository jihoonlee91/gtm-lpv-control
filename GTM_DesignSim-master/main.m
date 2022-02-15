% Trim to set of true air speeds
% speeds = [46:2:58,60:10:130]; % knots
% altitudes = 0:1000:10000; % feet
% alphas = [-5,-2:2:22,26,30,35,40]; % deg
% etas = [-25,0]; % percent

speeds = 100; % knots
altitudes = 1000; % feet
alphas = 3; % deg

% Compute trim points
MWS=init_design();
for i = 1:length(speeds)
for j = 1:length(altitudes)
for k = 1:length(alphas)
    disp(['TAS: ',num2str(speeds(i)),' knots, H: ',num2str(altitudes(j)),' ft, AoA: ',num2str(alphas(k)),' deg'])
    MWS.StatesInp(9) = altitudes(j);
    loadmws(MWS,'gtm_design');
    [MWS,Xt,Tc,Err] = trimgtm(struct('alpha',alphas(k),'gamma',0,'tas',speeds(i)));
%     if (Err>1e-3) % skip
%         Tc=struct('alpha',NaN,'elevator',NaN,'throttle',NaN);
%         disp('fail')
%     else
        loadmws(MWS);
        [sys,londyn,latdyn]=linmodel();
%         disp('done')
%     end
end
end
end
