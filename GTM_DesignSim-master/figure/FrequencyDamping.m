%
%----------- Frequency/Damping for level-flight trim conditions ------------
%
% Script calculates frequency and damping of approximate 4th order models over 
% a set of level flight trim condtions.

% $Id: example3.m 4852 2013-08-06 22:12:54Z cox $
% d.e.cox@larc.nasa.gov

close all

%% Trim to set of angles of attack
AOA=[2,4,6,8,  10,11,12,13,14,  16,18,20];
%AOA=[2:3:20];

% Allocate memory
shortperiod = zeros(length(AOA),2);
phugiod     = zeros(length(AOA),2);

% Comute Trim points
loadmws(init_design(),'gtm_design');
MWS = grabmws('gtm_design');

MWS.StatesInp(9)  = 800; % set altitude
%%
MWS.bias.morphing = -25; % set symmetric morphing parameter

loadmws(MWS,'gtm_design'); % load MWS

fprintf('  Trimming/Linearizing at alpha:\n');

for trimpt=[1:length(AOA)],
  fprintf(1,'      %3.2f,',AOA(trimpt));
  [MWS,Xt,Tc,Err]=trimgtm( struct('alpha',AOA(trimpt),'gamma',0),[],0);
  fprintf(1,' Residual=%3.2e\n',Err);
  loadmws(MWS,'gtm_design'); 
  [sys,lon,lat]=linmodel(MWS);
  [wn,z]=damp(lon);
  % Just assuming frequency seperation here...
  phugoid1(trimpt,:)     = [wn(1)/2/pi,z(1)];
  shortperiod1(trimpt,:) = [wn(3)/2/pi,z(3)];
end

%%
MWS.bias.morphing = -12.5; % set symmetric morphing parameter

loadmws(MWS,'gtm_design'); % load MWS

fprintf('  Trimming/Linearizing at alpha:\n');

for trimpt=[1:length(AOA)],
  fprintf(1,'      %3.2f,',AOA(trimpt));
  [MWS,Xt,Tc,Err]=trimgtm( struct('alpha',AOA(trimpt),'gamma',0),[],0);
  fprintf(1,' Residual=%3.2e\n',Err);
  loadmws(MWS,'gtm_design'); 
  [sys,lon,lat]=linmodel(MWS);
  [wn,z]=damp(lon);
  % Just assuming frequency seperation here...
  phugoid2(trimpt,:)     = [wn(1)/2/pi,z(1)];
  shortperiod2(trimpt,:) = [wn(3)/2/pi,z(3)];
end

%%
MWS.bias.morphing = 0; % set symmetric morphing parameter

loadmws(MWS,'gtm_design'); % load MWS

fprintf('  Trimming/Linearizing at alpha:\n');

for trimpt=[1:length(AOA)],
  fprintf(1,'      %3.2f,',AOA(trimpt));
  [MWS,Xt,Tc,Err]=trimgtm( struct('alpha',AOA(trimpt),'gamma',0),[],0);
  fprintf(1,' Residual=%3.2e\n',Err);
  loadmws(MWS,'gtm_design'); 
  [sys,lon,lat]=linmodel(MWS);
  [wn,z]=damp(lon);
  % Just assuming frequency seperation here...
  phugoid3(trimpt,:)     = [wn(1)/2/pi,z(1)];
  shortperiod3(trimpt,:) = [wn(3)/2/pi,z(3)];
end

save FrequencyDamping.mat AOA phugoid1 phugoid2 phugoid3 shortperiod1 shortperiod2 shortperiod3

%% Make alpha trim point plot
figure(1), clf
subplot(221),plot(AOA,phugoid1(:,1),'-s'),grid on, hold on
title('Phugoid Mode');ylabel('Frequency [Hz]');xlabel('Angle of Attack [deg]');
subplot(223),plot(AOA,phugoid1(:,2),'-s'),grid on, hold on
title('Phugoid Mode');ylabel('Damping Ratio');xlabel('Angle of Attack [deg]');
subplot(222),plot(AOA,shortperiod1(:,1),'-s'),grid on, hold on
title('Short-Period Mode');ylabel('Frequency [Hz]');xlabel('Angle of Attack [deg]');
subplot(224),plot(AOA,shortperiod1(:,2),'-s'),grid on, hold on
title('Short-Period Mode');ylabel('Damping Ratio');xlabel('Angle of Attack [deg]');

% Make alpha trim point plot
figure(1)
subplot(221),plot(AOA,phugoid2(:,1),'--o'),grid on, grid minor
subplot(223),plot(AOA,phugoid2(:,2),'--o'),grid on, grid minor
subplot(222),plot(AOA,shortperiod2(:,1),'--o'),grid on, grid minor
subplot(224),plot(AOA,shortperiod2(:,2),'--o'),grid on, grid minor

% Make alpha trim point plot
figure(1)
subplot(221),plot(AOA,phugoid3(:,1),':^'),grid on
legend('\eta_s = -25%','\eta_s = -12.5%','\eta_s = 0%','location','best')
subplot(223),plot(AOA,phugoid3(:,2),':^'),grid on
legend('\eta_s = -25%','\eta_s = -12.5%','\eta_s = 0%','location','best')
subplot(222),plot(AOA,shortperiod3(:,1),':^'),grid on
legend('\eta_s = -25%','\eta_s = -12.5%','\eta_s = 0%','location','best')
subplot(224),plot(AOA,shortperiod3(:,2),':^'),grid on
legend('\eta_s = -25%','\eta_s = -12.5%','\eta_s = 0%','location','best')

exportgraphics(gcf,'FrequencyDamping.pdf')