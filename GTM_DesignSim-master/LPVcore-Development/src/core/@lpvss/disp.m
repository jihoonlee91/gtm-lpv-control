function disp(obj)
% DISP Provide info on the lpvss object
%
%   Syntax:
%       disp(sys)
%
%   Inputs:
%       sys: lpvss object
%
narginchk(1, 1);

if isempty(obj)
    fprintf('Empty LPV-SS model');
    return;
end

if isct(obj)
    Qstr  = '(d/dt)';
    emptyQstr = '      ';
else
    Qstr  = '(q^-1)';
    emptyQstr = '      ';
end

% Model type
if isct(obj)
    timeStr = 'Continuous-time';
else
    timeStr = 'Discrete-time';
end
% Gridded Y/N
if isa(obj, 'lpvgridss')
    gridStr = 'gridded';
else
    gridStr = '';
end
fprintf('%s %s LPV-SS model\n\n', timeStr, gridStr);

% Determine whether to display B and D
if obj.Nu > 0
    inputStrB = ['+ ', paramString(obj.B, 'B'), ' u'];
    if obj.D ~= 0
        inputStrD = ['+ ', paramString(obj.D, 'D'), ' u'];
    else
        inputStrD = '';
    end
else
    inputStrB = '';
    inputStrD = '';
end
% State equation
fprintf('    %sx = %s x %s\n', ...
    Qstr, paramString(obj.A, 'A'), inputStrB);
% Output equation
fprintf('    %sy = %s x %s\n\n', ...
    emptyQstr, paramString(obj.C, 'C'), inputStrD);

% Description of input-scheduling-output
fprintf('with\n\n');
fprintf('    Nr. of inputs:             %d\n', obj.Nu);
fprintf('    Nr. of scheduling signals: %d (extended: %d)\n', obj.Np, obj.Nrho);
fprintf('    Nr. of outputs:            %d\n', obj.Ny);
fprintf('    Nr. of states:             %d\n', obj.Nx);
if isa(obj, 'lpvgridss')
    fprintf('    Interpolation method:      %s\n', obj.InterpolationMethod);
end
fprintf('\n');

% Description of sampling grid (for lpvgridss)
if isa(obj, 'lpvgridss')
    fprintf('%s\n', gridString(obj));
end

% Sample time
if ~isct(obj)
    if obj.Ts == -1; TsStr = 'unspecified';
    else;            TsStr = sprintf('%g %s',obj.Ts,obj.TimeUnit);                      end
    
    fprintf('%s: %s.\n', 'Sample time', TsStr);
end

end

%% Local functions
function s = paramString(K, name)
    % String representation of parameter (A, B, C, D)
    if isconst(K)
        s = ['   ', name];
    else
        s = [name, '(p)'];
    end
end

function s = gridString(obj)
    % String representation of SamplingGrid
    grid = obj.SamplingGrid;
    f = fieldnames(grid);
    N = 1;
    for i=1:numel(f)
        N = N * numel(grid.(f{i}));
    end
    s = sprintf(['Sampling grid:\n\n', ...
        '\tNr. of samples:\t\t\t   %i', ...
        '\n'], N);
    for i=1:numel(f)
        s = [s, sprintf('\t\t''%s'': %i grid points\n', f{i}, numel(grid.(f{i})))]; %#ok<AGROW> 
    end
end
