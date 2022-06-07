%% Basic setup
% Define parameter grid
tas_grid  = 60:10:110; % [knot]
alt_grid  = [0 3000 6000]; % 700~1100 [ft]
mor_grid  = [-25 0]; % between -25 and 0 [%]

configs = combvec(tas_grid,alt_grid,mor_grid);

%% Trim calculation
clearvars MWS_configs Xtrim_configs Trimcond_configs Err_configs
i = 1;
while 1
    tas = configs(1,i); % target tas [knot]
    alt = configs(2,i); % target alt [ft]
    mor = configs(3,i); % target mor [%]
    
    loadmws(init_design('GTM'),'gtm_design');
    MWS.StatesInp(9)  = alt; % set altitude [ft]
    MWS.bias.morphing = mor; % set symmetric morphing parameter [%]
    appendmws(MWS,'gtm_design'); % append MWS    
%     [MWS,Xtrim,Trimcond,Err] = trimgtm(struct('gamma',0,'eas',eas),'elev',1); % trim calculation
    [MWS,Xtrim,Trimcond,Err] = trimgtm(struct('gamma',0,'tas',tas),'elev',1); % trim calculation

    if (Err > 1e-3) % trim fail
        configs(:,i) = [];
        disp(['Remove : ',num2str(tas),' knot / ',num2str(alt),' ft / ',num2str(mor),' %'])
    else
        disp(['Keep : ',num2str(tas),' knot / ',num2str(alt),' ft / ',num2str(mor),' %'])
        % save trim results
        MWS_configs(i) = MWS;
        Xtrim_configs(:,i) = Xtrim;
        Trimcond_configs(i) = Trimcond;
        Err_configs(i) = Err;
        i = i+1;
    end

    if i > length(configs)
        break
    end
end

configs1 = configs(:,configs(3,:)==-25);
configs2 = configs(:,configs(3,:)==0);
figure
scatter(configs1(1,:),configs1(2,:)), hold on
figure
scatter(configs2(1,:),configs2(2,:))

FlightEnvelope = [Trimcond_configs.tas;Trimcond_configs.eas;[Trimcond_configs.altitude]/1000;Trimcond_configs.throttle;Trimcond_configs.alpha;Trimcond_configs.elevator;configs(3,:)];

%%
figure
subplot(121)
scatter(FlightEnvelope(1,configs(3,:)==mor_grid(1)),FlightEnvelope(3,configs(3,:)==mor_grid(1)),[],FlightEnvelope(4,configs(3,:)==mor_grid(1)),'filled','SizeData',100)
hold on
textscatter(FlightEnvelope(1,configs(3,:)==mor_grid(1)),FlightEnvelope(3,configs(3,:)==mor_grid(1)),string(FlightEnvelope(4,configs(3,:)==mor_grid(1))))
xlabel('TAS [knot]'), ylabel('Altitude [kft]'), title('Minimum Span')
grid
colorbar('Ticks',0:10:100)
caxis([0 100])

subplot(122)
scatter(FlightEnvelope(1,configs(3,:)==mor_grid(end)),FlightEnvelope(3,configs(3,:)==mor_grid(end)),[],FlightEnvelope(4,configs(3,:)==mor_grid(end)),'filled','SizeData',100)
hold on
textscatter(FlightEnvelope(1,configs(3,:)==mor_grid(end)),FlightEnvelope(3,configs(3,:)==mor_grid(end)),string(FlightEnvelope(4,configs(3,:)==mor_grid(end))))
xlabel('TAS [knot]'), ylabel('Altitude [kft]'), title('Maximum Span')
grid
colorbar('Ticks',0:10:100)
caxis([0 100])

figure
subplot(121)
scatter(FlightEnvelope(2,configs(3,:)==mor_grid(1)),FlightEnvelope(3,configs(3,:)==mor_grid(1)),[],FlightEnvelope(4,configs(3,:)==mor_grid(1)),'filled','SizeData',100)
hold on
textscatter(FlightEnvelope(2,configs(3,:)==mor_grid(1)),FlightEnvelope(3,configs(3,:)==mor_grid(1)),string(FlightEnvelope(4,configs(3,:)==mor_grid(1))))
xlabel('EAS [knot]'), ylabel('Altitude [kft]'), title('Minimum Span')
grid
colorbar('Ticks',0:10:100)
caxis([0 100])

subplot(122)
scatter(FlightEnvelope(2,configs(3,:)==mor_grid(end)),FlightEnvelope(3,configs(3,:)==mor_grid(end)),[],FlightEnvelope(4,configs(3,:)==mor_grid(end)),'filled','SizeData',100)
hold on
textscatter(FlightEnvelope(2,configs(3,:)==mor_grid(end)),FlightEnvelope(3,configs(3,:)==mor_grid(end)),string(FlightEnvelope(4,configs(3,:)==mor_grid(end))))
xlabel('EAS [knot]'), ylabel('Altitude [kft]'), title('Maximum Span')
grid
colorbar('Ticks',0:10:100)
caxis([0 100])

