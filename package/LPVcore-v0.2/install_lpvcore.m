%% Installation script for LPVcore
other_paths = which('core/@pmatrix/pmatrix.m', '-all');

addpath(genpath('src'))
if ~isempty(other_paths)
    fprintf('LPVcore was already installed at:\n\n');
    disp(cellfun(@(x) x(1:end-28), other_paths, 'UniformOutput', false))
else
    disp('''src'' has been added to your MATLAB path');
end

%% Check dependencies
v = ver;
% Show a warning on R2019b and older
if verLessThan('matlab', '9.8')
    warning('Some features of LPVcore may not work for MATLAB R2019b or older');
end

% Standard toolboxes
requiredToolboxes = ["Control System Toolbox"; ...
    "Optimization Toolbox"; ...
    "Robust Control Toolbox"; ...
    "Statistics and Machine Learning Toolbox"; ...
    "System Identification Toolbox"; ...
    ];
tf = ismember(requiredToolboxes,{v.Name});
missingToolboxes = requiredToolboxes((tf~=1));
if ~all(tf)
    warning('LPVcore requires additional toolboxes listed here:')
    fprintf('\n');
    disp(missingToolboxes)
end

% Synthesis toolboxes
TF2(1:3) = false;
if isempty(which('rolmipvar/evalpar')); TF2(1) = true; end
if isempty(which('sdpt3.m')); TF2(2) = true; end
if isempty(which('operators/eig_yalmip_internal.m')); TF2(3) = true; end

if any(TF2)
    synthesisTB = ["ROLMIP";"SDPT3";"YALMIP"];
    missingSynthesisTB = synthesisTB((TF2 == 1));
    fprintf('For synthesis, install the following toolboxes:\n\n');
    disp(missingSynthesisTB);
end

%% Next steps
% Success
fprintf(' ---- Installation successful! ---- \n\n');

% Test
disp(' --> To perform an extensive test of LPVcore, type ''test''. Note: this test takes about 5 minutes to complete.');
    
% Permanent save to path
disp(' --> To permanently add LPVcore to your MATLAB path, type ''savepath''.');

% Refer to tutorial
fprintf(' --> If you''re new, open <a href="matlab:opentoline(''examples/tutorial.m'', 1)">examples/tutorial.m</a> for a tutorial.\n\n');
