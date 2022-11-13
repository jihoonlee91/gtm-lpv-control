%% Basic setup
% Define parameter grid
tas_grid  = 60:10:110; % [knot]
alt_grid  = [0 3000 6000]; % 700~1100 [ft]
mor_grid  = [-25 -12.5 0]; % between -25 and 0 [%]

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

configs1 = configs(:,configs(3,:) == -25);
configs2 = configs(:,configs(3,:) == 0);
figure
scatter(configs1(1,:),configs1(2,:)), hold on
figure
scatter(configs2(1,:),configs2(2,:))