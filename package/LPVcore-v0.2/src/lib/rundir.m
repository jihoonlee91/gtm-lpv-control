function rundir(dirName, base)
%RUNDIR Recursively runs scripts in a directory.
%
%   Syntax:
%       r = rundir(dirName, base)
%
%   Inputs:
%       dirName: name of the directory with scripts (full or relative path).
%       base: base path to the directory.
%

    if nargin <= 1
        base = dirName;
    end

    cd(dirName);
    
    l = dir;
    for i=1:numel(l)
        if l(i).isdir && ~strcmp(l(i).name(1), '.')
            % recurse
            rundir(l(i).name, [base, '/', l(i).name])
        elseif ~l(i).isdir && endsWith(l(i).name, '.m')
            save('rundir_tmp.mat');
            run(l(i).name);
            clearvars; close all; clc;
            load('rundir_tmp.mat'); %#ok<LOAD>
        end
    end
    delete('rundir_tmp.mat');
    
    cd ..
end
