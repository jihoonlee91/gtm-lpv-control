close all





%
%--------------- Calculate set of level flight trim conditions ----------------
%
% Script calculates a set of level flight trim condtions for different
% calibrated airspeeds and angles-of-attack.  Plots trim conditions as
% a function of specified target condition

% $Id: example2.m 4852 2013-08-06 22:12:54Z cox $
% d.e.cox@larc.nasa.gov

% -------------eas plot----------------
% Trim to set of equivalent air speeds
% speeds=[[46:2:58],[60:10:130]];
speeds=[[40:55],[60:5:105],[110:125]];

%% 44~118

% Allocate variables to plot
alpha11=zeros(length(speeds),1);
elevator11=zeros(length(speeds),1);
throttle11=zeros(length(speeds),1);

% Compute trim points
MWS=init_design();
MWS.StatesInp(9)  = 0; % set altitude
MWS.bias.morphing = 0; % set symmetric morphing parameter
loadmws(MWS,'gtm_design'); % load MWS

fprintf('\nLevel Flight Trim\n  Trimming at tas:\n');
for trimpt=[1:length(speeds)],
  fprintf(1,'     %3.2f,',speeds(trimpt));
  loadmws(MWS,'gtm_design'); 
  [MWS,Xt,Tc,Err]=trimgtm(struct('tas',speeds(trimpt),'gamma',0), 'elev', 0);
  if (Err>1e-3),  % try again, different starting poing
      loadmws(init_design(),'gtm_design');
MWS = grabmws('gtm_design');
MWS.StatesInp(9)  = 0; % set altitude
MWS.bias.morphing = 0; % set symmetric morphing parameter
loadmws(MWS,'gtm_design'); % load MWS
      fprintf('   Poor convergence, trying again. ');
      [MWS,Xt,Tc,Err]=trimgtm(struct('eas',speeds(trimpt),'gamma',0), 'elev', 0);
      if (Err>1e-3)  % No joy, skip point.
          Tc=struct('alpha',NaN,'elevator',NaN,'throttle',NaN);
          fprintf(1,'Failed to Trim  ');
          else fprintf(1,'Done  ');
      end
  end
  fprintf(1,' Residual=%3.2e\n',Err);
  alpha11(trimpt)   =Tc.alpha;   
  elevator11(trimpt)=Tc.elevator;
  throttle11(trimpt)=Tc.throttle;
end
fprintf(' Done\n');

% Make tas trim point plot
figure(1),clf
subplot(121),
[ax,h1,h2]=plotyy(speeds,[elevator11,alpha11],speeds,throttle11); grid on
set(ax,{'YColor'},{'blue';'red'});
set([h1;h2],{'Color'},{'blue';'blue';'red'});
set([h1;h2],{'Marker'},{'o';'s';'>'})
set([h1;h2],{'MarkerFaceColor'},{'blue';'none';'red'});
set([h1;h2],{'LineStyle'},{'--';'-';':'});
title('\eta_s = 0%')

% title('Trimmed level flight by eas');
legend([h1;h2], { 'Elevator','Angle of Attack','Throttle'} );  
set([h1;h2],{'Marker'},{'o';'s';'>'});
ylabel(ax(1),'Elevator [deg],  Angle of Attack [deg]');
ylabel(ax(2),'Throttle [%]');
xlabel('True Airspeed [knot]')
drawnow
%% 48~

% Allocate variables to plot
alpha12=zeros(length(speeds),1);
elevator12=zeros(length(speeds),1);
throttle12=zeros(length(speeds),1);

% Compute trim points
MWS=init_design();
MWS.StatesInp(9)  = 0; % set altitude
MWS.bias.morphing = -25; % set symmetric morphing parameter
loadmws(MWS,'gtm_design'); % load MWS

fprintf('\nLevel Flight Trim\n  Trimming at tas:\n');
for trimpt=[1:length(speeds)],
  fprintf(1,'     %3.2f,',speeds(trimpt));
  loadmws(MWS,'gtm_design'); 
  [MWS,Xt,Tc,Err]=trimgtm(struct('tas',speeds(trimpt),'gamma',0), 'elev', 0);
  if (Err>1e-3),  % try again, different starting poing
      loadmws(init_design(),'gtm_design');
MWS = grabmws('gtm_design');
MWS.StatesInp(9)  = 0; % set altitude
MWS.bias.morphing = -25; % set symmetric morphing parameter
loadmws(MWS,'gtm_design'); % load MWS
      fprintf('   Poor convergence, trying again. ');
      [MWS,Xt,Tc,Err]=trimgtm(struct('eas',speeds(trimpt),'gamma',0), 'elev', 0);
      if (Err>1e-3)  % No joy, skip point.
          Tc=struct('alpha',NaN,'elevator',NaN,'throttle',NaN);
          fprintf(1,'Failed to Trim  ');
          else fprintf(1,'Done  ');
      end
  end
  fprintf(1,' Residual=%3.2e\n',Err);
  alpha12(trimpt)   =Tc.alpha;   
  elevator12(trimpt)=Tc.elevator;
  throttle12(trimpt)=Tc.throttle;
end
fprintf(' Done\n');

% Make tas trim point plot
figure(1)
subplot(122),
[ax,h1,h2]=plotyy(speeds,[elevator12,alpha12],speeds,throttle12); grid on
set(ax,{'YColor'},{'blue';'red'});
set([h1;h2],{'Color'},{'blue';'blue';'red'});
set([h1;h2],{'Marker'},{'o';'s';'>'})
set([h1;h2],{'MarkerFaceColor'},{'blue';'none';'red'});
set([h1;h2],{'LineStyle'},{'--';'-';':'});
title('\eta_s = -25%')

% title('Trimmed level flight by eas');
legend([h1;h2], { 'Elevator','Angle of Attack','Throttle'} );  
set([h1;h2],{'Marker'},{'o';'s';'>'});
ylabel(ax(1),'Elevator [deg],  Angle of Attack [deg]');
ylabel(ax(2),'Throttle [%]');
xlabel('True Airspeed [knot]')
drawnow
exportgraphics(gcf,'MorphingTrim0ft.pdf')

%%
alpha21=zeros(length(speeds),1);
elevator21=zeros(length(speeds),1);
throttle21=zeros(length(speeds),1);

% Compute trim points
MWS=init_design();
MWS.StatesInp(9)  = 5000; % set altitude
MWS.bias.morphing = 0; % set symmetric morphing parameter
loadmws(MWS,'gtm_design'); % load MWS

fprintf('\nLevel Flight Trim\n  Trimming at tas:\n');
for trimpt=[1:length(speeds)],
  fprintf(1,'     %3.2f,',speeds(trimpt));
  loadmws(MWS,'gtm_design'); 
  [MWS,Xt,Tc,Err]=trimgtm(struct('tas',speeds(trimpt),'gamma',0), 'elev', 0);
  if (Err>1e-3),  % try again, different starting poing
      loadmws(init_design(),'gtm_design');
MWS = grabmws('gtm_design');
MWS.StatesInp(9)  = 5000; % set altitude
MWS.bias.morphing = 0; % set symmetric morphing parameter
loadmws(MWS,'gtm_design'); % load MWS
      fprintf('   Poor convergence, trying again. ');
      [MWS,Xt,Tc,Err]=trimgtm(struct('eas',speeds(trimpt),'gamma',0), 'elev', 0);
      if (Err>1e-3)  % No joy, skip point.
          Tc=struct('alpha',NaN,'elevator',NaN,'throttle',NaN);
          fprintf(1,'Failed to Trim  ');
          else fprintf(1,'Done  ');
      end
  end
  fprintf(1,' Residual=%3.2e\n',Err);
  alpha21(trimpt)   =Tc.alpha;   
  elevator21(trimpt)=Tc.elevator;
  throttle21(trimpt)=Tc.throttle;
end
fprintf(' Done\n');

% Make tas trim point plot
figure(2),clf
subplot(121),
[ax,h1,h2]=plotyy(speeds,[elevator21,alpha21],speeds,throttle21); grid on
set(ax,{'YColor'},{'blue';'red'});
set([h1;h2],{'Color'},{'blue';'blue';'red'});
set([h1;h2],{'Marker'},{'o';'s';'>'})
set([h1;h2],{'MarkerFaceColor'},{'blue';'none';'red'});
set([h1;h2],{'LineStyle'},{'--';'-';':'});
title('\eta_s = 0%')

% title('Trimmed level flight by eas');
legend([h1;h2], { 'Elevator','Angle of Attack','Throttle'} );  
set([h1;h2],{'Marker'},{'o';'s';'>'});
ylabel(ax(1),'Elevator [deg],  Angle of Attack [deg]');
ylabel(ax(2),'Throttle [%]');
xlabel('True Airspeed [knot]')
drawnow
%%

% Allocate variables to plot
alpha22=zeros(length(speeds),1);
elevator22=zeros(length(speeds),1);
throttle22=zeros(length(speeds),1);

% Compute trim points
MWS=init_design();
MWS.StatesInp(9)  = 5000; % set altitude
MWS.bias.morphing = -25; % set symmetric morphing parameter
loadmws(MWS,'gtm_design'); % load MWS

fprintf('\nLevel Flight Trim\n  Trimming at tas:\n');
for trimpt=[1:length(speeds)],
  fprintf(1,'     %3.2f,',speeds(trimpt));
  loadmws(MWS,'gtm_design'); 
  [MWS,Xt,Tc,Err]=trimgtm(struct('tas',speeds(trimpt),'gamma',0), 'elev', 0);
  if (Err>1e-3),  % try again, different starting poing
      loadmws(init_design(),'gtm_design');
MWS = grabmws('gtm_design');
MWS.StatesInp(9)  = 5000; % set altitude
MWS.bias.morphing = -25; % set symmetric morphing parameter
loadmws(MWS,'gtm_design'); % load MWS
      fprintf('   Poor convergence, trying again. ');
      [MWS,Xt,Tc,Err]=trimgtm(struct('eas',speeds(trimpt),'gamma',0), 'elev', 0);
      if (Err>1e-3)  % No joy, skip point.
          Tc=struct('alpha',NaN,'elevator',NaN,'throttle',NaN);
          fprintf(1,'Failed to Trim  ');
          else fprintf(1,'Done  ');
      end
  end
  fprintf(1,' Residual=%3.2e\n',Err);
  alpha22(trimpt)   =Tc.alpha;   
  elevator22(trimpt)=Tc.elevator;
  throttle22(trimpt)=Tc.throttle;
end
fprintf(' Done\n');

% Make tas trim point plot
figure(2)
subplot(122),
[ax,h1,h2]=plotyy(speeds,[elevator22,alpha22],speeds,throttle22); grid on
set(ax,{'YColor'},{'blue';'red'});
set([h1;h2],{'Color'},{'blue';'blue';'red'});
set([h1;h2],{'Marker'},{'o';'s';'>'})
set([h1;h2],{'MarkerFaceColor'},{'blue';'none';'red'});
set([h1;h2],{'LineStyle'},{'--';'-';':'});
title('\eta_s = -25%')

% title('Trimmed level flight by eas');
legend([h1;h2], { 'Elevator','Angle of Attack','Throttle'} );  
set([h1;h2],{'Marker'},{'o';'s';'>'});
ylabel(ax(1),'Elevator [deg],  Angle of Attack [deg]');
ylabel(ax(2),'Throttle [%]');
xlabel('True Airspeed [knot]')
drawnow
exportgraphics(gcf,'MorphingTrim5000ft.pdf')

%% 52~

alpha31=zeros(length(speeds),1);
elevator31=zeros(length(speeds),1);
throttle31=zeros(length(speeds),1);

% Compute trim points
MWS=init_design();
MWS.StatesInp(9)  = 10000; % set altitude
MWS.bias.morphing = 0; % set symmetric morphing parameter
loadmws(MWS,'gtm_design'); % load MWS

fprintf('\nLevel Flight Trim\n  Trimming at tas:\n');
for trimpt=[1:length(speeds)],
  fprintf(1,'     %3.2f,',speeds(trimpt));
  loadmws(MWS,'gtm_design'); 
  [MWS,Xt,Tc,Err]=trimgtm(struct('tas',speeds(trimpt),'gamma',0), 'elev', 0);
  if (Err>1e-3),  % try again, different starting poing
      loadmws(init_design(),'gtm_design');
MWS = grabmws('gtm_design');
MWS.StatesInp(9)  = 10000; % set altitude
MWS.bias.morphing = 0; % set symmetric morphing parameter
loadmws(MWS,'gtm_design'); % load MWS
      fprintf('   Poor convergence, trying again. ');
      [MWS,Xt,Tc,Err]=trimgtm(struct('eas',speeds(trimpt),'gamma',0), 'elev', 0);
      if (Err>1e-3)  % No joy, skip point.
          Tc=struct('alpha',NaN,'elevator',NaN,'throttle',NaN);
          fprintf(1,'Failed to Trim  ');
          else fprintf(1,'Done  ');
      end
  end
  fprintf(1,' Residual=%3.2e\n',Err);
  alpha31(trimpt)   =Tc.alpha;   
  elevator31(trimpt)=Tc.elevator;
  throttle31(trimpt)=Tc.throttle;
end
fprintf(' Done\n');

% Make tas trim point plot
figure(3),clf
subplot(121),
[ax,h1,h2]=plotyy(speeds,[elevator31,alpha31],speeds,throttle31); grid on
set(ax,{'YColor'},{'blue';'red'});
set([h1;h2],{'Color'},{'blue';'blue';'red'});
set([h1;h2],{'Marker'},{'o';'s';'>'})
set([h1;h2],{'MarkerFaceColor'},{'blue';'none';'red'});
set([h1;h2],{'LineStyle'},{'--';'-';':'});
title('\eta_s = 0%')

% title('Trimmed level flight by eas');
legend([h1;h2], { 'Elevator','Angle of Attack','Throttle'} );  
set([h1;h2],{'Marker'},{'o';'s';'>'});
ylabel(ax(1),'Elevator [deg],  Angle of Attack [deg]');
ylabel(ax(2),'Throttle [%]');
xlabel('True Airspeed [knot]')
drawnow
%% 55~

% Allocate variables to plot
alpha32=zeros(length(speeds),1);
elevator32=zeros(length(speeds),1);
throttle32=zeros(length(speeds),1);

% Compute trim points
MWS=init_design();
MWS.StatesInp(9)  = 10000; % set altitude
MWS.bias.morphing = -25; % set symmetric morphing parameter
loadmws(MWS,'gtm_design'); % load MWS

fprintf('\nLevel Flight Trim\n  Trimming at tas:\n');
for trimpt=[1:length(speeds)],
  fprintf(1,'     %3.2f,',speeds(trimpt));
  loadmws(MWS,'gtm_design'); 
  [MWS,Xt,Tc,Err]=trimgtm(struct('tas',speeds(trimpt),'gamma',0), 'elev', 0);
  if (Err>1e-3),  % try again, different starting poing
      loadmws(init_design(),'gtm_design');
MWS = grabmws('gtm_design');
MWS.StatesInp(9)  = 10000; % set altitude
MWS.bias.morphing = -25; % set symmetric morphing parameter
loadmws(MWS,'gtm_design'); % load MWS
      fprintf('   Poor convergence, trying again. ');
      [MWS,Xt,Tc,Err]=trimgtm(struct('eas',speeds(trimpt),'gamma',0), 'elev', 0);
      if (Err>1e-3)  % No joy, skip point.
          Tc=struct('alpha',NaN,'elevator',NaN,'throttle',NaN);
          fprintf(1,'Failed to Trim  ');
          else fprintf(1,'Done  ');
      end
  end
  fprintf(1,' Residual=%3.2e\n',Err);
  alpha32(trimpt)   =Tc.alpha;   
  elevator32(trimpt)=Tc.elevator;
  throttle32(trimpt)=Tc.throttle;
end
fprintf(' Done\n');

% Make tas trim point plot
figure(3)
subplot(122),
[ax,h1,h2]=plotyy(speeds,[elevator32,alpha32],speeds,throttle32); grid on
set(ax,{'YColor'},{'blue';'red'});
set([h1;h2],{'Color'},{'blue';'blue';'red'});
set([h1;h2],{'Marker'},{'o';'s';'>'})
set([h1;h2],{'MarkerFaceColor'},{'blue';'none';'red'});
set([h1;h2],{'LineStyle'},{'--';'-';':'});
title('\eta_s = -25%')

% title('Trimmed level flight by eas');
legend([h1;h2], { 'Elevator','Angle of Attack','Throttle'} );  
set([h1;h2],{'Marker'},{'o';'s';'>'});
ylabel(ax(1),'Elevator [deg],  Angle of Attack [deg]');
ylabel(ax(2),'Throttle [%]');
xlabel('True Airspeed [knot]')
drawnow
exportgraphics(gcf,'MorphingTrim10000ft.pdf')

save MorphingTrim.mat speeds elevator11 elevator12 elevator21 elevator22 elevator31 elevator32 alpha11 alpha12 alpha21 alpha22 alpha31 alpha32 throttle11 throttle12 throttle21 throttle22 throttle31 throttle32