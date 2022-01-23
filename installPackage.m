close all;
clear all;
clc;

% currentDirectory = pwd;
% currentDirectoryIndex = strfind(currentDirectory,'\');
% aboveDirectory = currentDirectory(1:currentDirectoryIndex(end)-1);
% addpath(genpath([aboveDirectory,'\package']));

masterDirectory = pwd;

%% install LPVToolsV1.0
oldFolder = cd('package\LPVToolsV1.0');
addlpv
cd(oldFolder)

%% install ROLMIP
addpath(genpath([masterDirectory,'\package','\robust_lmi_parser']));

%% install YALMIP
addpath(genpath([masterDirectory,'\package','\YALMIP-master']));

%% install SDPT3-4.0
oldFolder = cd('package\SDPT3-4.0');
Installmex(1)
cd(oldFolder)

%% install LPVcore-v0.2
oldFolder = cd('package\LPVcore-v0.2');
install_lpvcore
cd(oldFolder)
