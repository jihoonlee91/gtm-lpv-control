%% Basic setup
close all
clear all
clc
setup

% Coeff_throttle = [0.000033035538475  -0.008354385314499 1.191126728328792 31.518959847101669];
% Coeff_thrust   = [0.000014847593013,  0, 0, 0.412662356277121];
% T_max = 2*polyval(Coeff_thrust,polyval(Coeff_throttle,100)); % Maximum thrust at sea level without ram drag at 123000 RPM = 30.6 [lbf]
% altitude up -> pressure down -> thrust down (multiplication)
% true airspeed up -> ram drag up -> thrust down (subtraction)
% 0.01 * 0.5 * air density * true airspeed * true airspeed * 5.9018 [lbf]

%% Define parameter grid
% eas_grid  = 30:5:150; % [knot]
% alt_grid  = 0:5000:60000; % 700~1100 [ft]
% mor_grid  = -25:25:0; % between -25 and 0 [%]
tas_grid  = 40:1:130; % [knot]
alt_grid  = 0:5000:35000; % 700~1100 [ft]
mor_grid  = -25:25:0; % between -25 and 0 [%]

configs = combvec(tas_grid,alt_grid,mor_grid);

%% Trim calculation
clearvars MWS_configs Xtrim_configs Trimcond_configs Err_configs
min = 0;
max = 0;
figure
tic
for i = 1:length(configs)
    loadmws(init_design('GTM'),'gtm_design');
    MWS = grabmws('gtm_design');
    MWS.StatesInp(9)  = configs(2,i); % set altitude [ft]
    MWS.bias.morphing = configs(3,i); % set symmetric morphing parameter [%]
    appendmws(MWS,'gtm_design'); % append MWS    
    
    [MWS,Xtrim,Trimcond,Err] = trimgtm(struct('gamma',0,'tas',configs(1,i)),'elev',0); % trim calculation
    if Err > 1e-3 % Fail
        [MWS,Xtrim,Trimcond,Err] = trimgtm(struct('gamma',0,'tas',configs(1,i)),'elev',0); % trim re-calculation
    end
    Trimcond.morphing = configs(3,i);

    % save trim results
    MWS_configs(i) = MWS;
    Xtrim_configs(:,i) = Xtrim;
    Trimcond_configs(i) = Trimcond;
    Err_configs(i) = Err;

    if (Err > 1e-3) % trim fail
        Trimcond.eas = NaN;
        Trimcond.tas = NaN;
        Trimcond.alpha = NaN;
        Trimcond.altitude = NaN;
        Trimcond.throttle = NaN;
        Trimcond.elevator = NaN;
        disp(['FAIL : TAS ',num2str(configs(1,i)),' knot / Altitude ',num2str(configs(2,i)/1000),' kft / Morphing ',num2str(configs(3,i)),' %'])
    else
        eas = Trimcond.eas;
        tas = Trimcond.tas;
        alp = Trimcond.alpha;
        alt = Trimcond.altitude;
        thr = Trimcond.throttle;
        ele = Trimcond.elevator;
        mor = Trimcond.morphing;
        disp(['KEEP : EAS ',num2str(round(eas,1)),' knot / TAS ',num2str(round(configs(1,i),1)),' knot / AoA ',num2str(round(alp,2)),' deg / Altitude ',num2str(alt/1000),' kft / Throttle ',num2str(round(thr,1)),' % / Elevator ',num2str(round(ele,2)),' deg / Morphing ',num2str(mor),' %'])
    end

    if (Err < 1e-3)
        if mor == mor_grid(1)
            subplot(121)
            scatter(tas,alt,[],thr,'filled','SizeData',200)
            hold on
            textscatter(tas,alt,string(round(thr,1)),'TextDensityPercentage',100)
            if min == 0
                title('Minimum Span')
                xlabel('TAS [knot]'), ylabel('Altitude [kft]')
                grid
                c = colorbar;
                c.Ticks = 0:10:100;
                c.Label.String = 'Throttle [%]';
                caxis([0 100])
                min = min+1;
            end
        elseif mor == mor_grid(end)
            subplot(122)
            scatter(tas,alt,[],thr,'filled','SizeData',200)
            hold on
            textscatter(tas,alt,string(round(thr,1)))
            if max == 0
                title('Maximum Span')
                xlabel('TAS [knot]'), ylabel('Altitude [kft]')
                grid
                c = colorbar;
                c.Ticks = 0:10:100;
                c.Label.String = 'Throttle [%]';
                caxis([0 100])
                max = max+1;
            end
        end
        drawnow
    end
end
TrimCalcTime = toc;

%%
save FlightEnvelope.mat MWS_configs Xtrim_configs Trimcond_configs Err_configs tas_grid alt_grid mor_grid TrimCalcTime

%%
eas = [Trimcond_configs.eas].';
tas = [Trimcond_configs.tas].';
alp = [Trimcond_configs.alpha].';
alt = [Trimcond_configs.altitude].';
thr = [Trimcond_configs.throttle].';
ele = [Trimcond_configs.elevator].';
mor = [Trimcond_configs.morphing].';

Trimdata = [eas,tas,alp,alt/1000,thr,ele,mor];
Trimdata_valid = Trimdata(:,~isnan(Trimdata(1,:)));

%%
% figure
% subplot(121)
% scatter(eas(mor==mor_grid(1)),alt(mor==mor_grid(1)),[],thr(mor==mor_grid(1)),'filled','SizeData',200)
% hold on
% textscatter(eas(mor==mor_grid(1)),alt(mor==mor_grid(1)),string(round(thr(mor==mor_grid(1)),1)),'TextDensityPercentage',100)
% xlabel('EAS [knot]'), ylabel('Altitude [kft]'), title('Minimum Span')
% grid
% c = colorbar;
% c.Ticks = 0:10:100;
% c.Label.String = 'Throttle [%]';
% caxis([0 100])
% 
% subplot(122)
% scatter(eas(mor==mor_grid(end)),alt(mor==mor_grid(end)),[],thr(mor==mor_grid(end)),'filled','SizeData',200)
% hold on
% textscatter(eas(mor==mor_grid(end)),alt(mor==mor_grid(end)),string(round(thr(mor==mor_grid(end)),1)),'TextDensityPercentage',100)
% xlabel('EAS [knot]'), ylabel('Altitude [kft]'), title('Maximum Span')
% grid
% c = colorbar;
% c.Ticks = 0:10:100;
% c.Label.String = 'Throttle [%]';
% caxis([0 100])

%%
figure
subplot(121)
scatter(tas(mor==mor_grid(1)),alt(mor==mor_grid(1))/1000,[],thr(mor==mor_grid(1)),'filled','SizeData',200)
% hold on
% textscatter(tas(mor==mor_grid(1)),alt(mor==mor_grid(1)),string(round(thr(mor==mor_grid(1)),1)),'TextDensityPercentage',100)
xlabel('TAS [knot]'), ylabel('Altitude [kft]'), title('Minimum Span')
grid
c = colorbar;
c.Ticks = 0:10:100;
c.Label.String = 'Throttle [%]';
caxis([0 100])

subplot(122)
scatter(tas(mor==mor_grid(end)),alt(mor==mor_grid(end))/1000,[],thr(mor==mor_grid(end)),'filled','SizeData',200)
% hold on
% textscatter(tas(mor==mor_grid(end)),alt(mor==mor_grid(end)),string(round(thr(mor==mor_grid(end)),1)),'TextDensityPercentage',100)
xlabel('TAS [knot]'), ylabel('Altitude [kft]'), title('Maximum Span')
grid
c = colorbar;
c.Ticks = 0:10:100;
c.Label.String = 'Throttle [%]';
caxis([0 100])

%%
% figure
% scatter(tas(mor==mor_grid(1)),alt(mor==mor_grid(1)),[],throttle(mor==mor_grid(end))-throttle(mor==mor_grid(1)),'filled','SizeData',200)
% hold on
% textscatter(tas(mor==mor_grid(1)),alt(mor==mor_grid(1)),string(round(throttle(mor==mor_grid(end))-throttle(mor==mor_grid(1)),1)))
% xlabel('TAS [knot]'), ylabel('Altitude [kft]'), title('Minimum Span')
% grid
% c = colorbar;
% c.Ticks = -100:2:100;
% c.Label.String = 'Throttle Difference (Maxspan-Minspan) [%p]';
% caxis([-10 10])
