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
fprintf('%s LPV-SS model\n\n', timeStr);

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
fprintf('    Nr. of outputs:            %d\n\n', obj.Ny);

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
