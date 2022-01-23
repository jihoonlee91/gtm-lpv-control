%Clear workspace
clear;

%Get root project directory
projectRootDir = fileparts(mfilename('fullpath'));

%Set location where derived files are placed
Simulink.fileGenControl('set', 'CacheFolder', fullfile(projectRootDir,'work'), ...
    'CodeGenFolder', fullfile(projectRootDir,'work'));