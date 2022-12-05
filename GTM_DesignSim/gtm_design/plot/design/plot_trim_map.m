%% initialization
close all; clear all; clc

% simulink model setup
setup

set_param('gtm_design','MinStep','auto')
set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','on') % switch off controller
set_param('gtm_design/Guidance','commented','on') % switch off controller
set_param('gtm_design/Monitor','commented','on') % switch off controller

%% test region
% scheduling parameter gridding
tas_min = 40;
tas_int = 2.5;
tas_max = 130;
tas_grid  = tas_min:tas_int:tas_max; % [kt]
alt_min = 0;
alt_int = 5000;
alt_max = 40000;
alt_grid  = alt_min:alt_int:alt_max; % 700~1100 [ft]
mor       = 0; % between -25 and 0 [%]

%% trimming
num_tas  = numel(tas_grid); % number tas grid
num_alt  = numel(alt_grid); % number alt grid
[tas_mesh,alt_mesh] = meshgrid(tas_grid,alt_grid);
config = zeros(num_alt,num_tas);
figure('units','centimeters','position',[20 2 20 20]);
for j = 1:num_alt
for i = 21:num_tas
    tas = tas_grid(i); % trim tas [kt]
    alt = alt_grid(j); % trim alt [ft]
    [MWS,Xtrim,Trimcond,Err,trial] = trimgtm_morph_try(tas,alt,mor,3);

    if (Err < 1e-3)
        fprintf('trimming #%d @ tas: %d kt, alt: %d ft, mor: %.3g %% : SUCCESS \n',sub2ind([num_tas num_alt],i,j),tas,alt,mor)
        X = [tas-tas_int/2;tas-tas_int/2;tas+tas_int/2;tas+tas_int/2];
        Y = [alt-alt_int/2;alt+alt_int/2;alt+alt_int/2;alt-alt_int/2];
        config(j,i) = Trimcond.throttle; % trim success
        patch(X,Y,config(j,i))
    else
        fprintf('trimming #%d @ tas: %d kt, alt: %d ft, mor: %.3g %% : FAIL \n',sub2ind([num_tas num_alt],i,j),tas,alt,mor)
        config(j,i) = NaN; % trim fail
    end
    
    xlim([tas_min-tas_int tas_max+tas_int])
    ylim([0 alt_max+alt_int])
    xlabel('True Airspeed [kt]')
    ylabel('Altitude [ft]')
    drawnow
    shading flat
    Ax = gca;
    Ax.Layer = 'top';
    h = colorbar;
    h.Limits = [0 100];
    h.Label.String = 'Throttle [%]';
    grid on
    grid minor
end
end
xlim([42.5 122.5])
ylim([0 35000])
exportgraphics(gcf,'figure/trim_map.png','resolution',300)

% figure('units','centimeters','position',[20 2 20 20]);
% for j = 1:num_alt
% for i = 1:num_tas
%     patch(X,Y,config(j,i))
% end
% end

%%
open trim_map.fig
hold on
grid minor
set(gcf,'units','centimeters','position',[20 2 20 12]);
for k = 50:10:120
    plot([k 100],[0 30000],'--k')
end
exportgraphics(gcf,'figure/trim_map.png','resolution',300)
