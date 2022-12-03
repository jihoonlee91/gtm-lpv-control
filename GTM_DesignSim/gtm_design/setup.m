% d.e.cox@larc.nasa.gov
% $Id: setup.m 4852 2013-08-06 22:12:54Z cox $
% modified by jihoon lee (leejihoon@snu.ac.kr)

% Add mat file directory
addpath(genpath('./config/'));
addpath(genpath('./data/'));
addpath(genpath('./sfunction/'));
addpath(genpath('./trajectory3/'));
addpath(genpath('./mfiles'));
addpath(genpath('./design'));
addpath(genpath('./obj'));
addpath(genpath('./libs'));
addpath(genpath('./figure/'));
addpath(genpath('./plot/'));
addpath(genpath('./LPVToolsV1.0/'));
run('./LPVToolsV1.0/addlpv');
rehash path

% Close model if open, prompt if changed.
if  not(isempty(find_system('SearchDepth',0,'Name','gtm_design')))
    if strcmp(get_param('gtm_design','Dirty'),'on')
	  fprintf(1,'Saving System...');
	  clearmws % Suppreses warning mesage about model workspace
	  clear savechanges % clear temporary response variable
	  save_system('gtm_design');
	  fprintf(1,' Done.\n');
    end
    % terminate if left in paused state. matlab will not close paused model.
    if strcmp(get_param('gtm_design','SimulationStatus'),'paused')
        gtm_design([],[],[],'term');
    end
    if strcmp(get_param('gtm_design','SimulationStatus'),'paused')
        gtm_design([],[],[],'term');
    end
    if strcmp(get_param('gtm_design','SimulationStatus'),'paused')
        gtm_design([],[],[],'term');
    end
    bdclose('gtm_design');
end

% Clear simout* variables to ensure clean initialization
clear simout simout_names SimWSout

% Clear precompiled functions, required to reflect changes in mfiles that exist outside working directory.
clear functions

% Open block diagram
warning('off','Simulink:SL_MdlFileNotOnPath'); % Suppress spurious warnings
open_system('gtm_design');  

% load nominal starting point
loadmws(init_design('GTM'),'gtm_design');
warning('on','Simulink:SL_MdlFileNotOnPath');

% set configuration parameter
set_param('gtm_design','MinStep','auto')
set_param('gtm_design','ReturnWorkspaceOutputs','off')

% basic parameters
knots2fps = 1.68781; % kt to ft/s
m2ft      = 3.28084; % m to ft
d2r       = pi/180; % deg to rad
r2d       = 1/d2r; % rad to deg
g         = 32.174; % gravitational acceleration [ft/s^2]
s = tf('s'); % Laplace variable

% load data
load FixedWingBus.mat % for path-following guidance
load T2_polynomial_aerodatabase.mat % for aerodynamic analysis
load lpv_design_result.mat % controller data