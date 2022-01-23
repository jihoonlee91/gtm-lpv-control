function disp(obj)
% DISP Provide info on the lpvio object
%
%   Syntax:
%       disp(sys)
%
%   Inputs:
%       sys: lpvio object
%
narginchk(1, 1);

if isempty(obj)
    fprintf('Empty LPV-IO model');
    return;
end

if isct(obj)
    Qstr  = '(d/dt)';
    Qistr = 'd^i/dt^i';
    emptyQstr = '      ';
else
    Qstr  = '(q^-1)';
    Qistr = 'q^-i';
    emptyQstr = '      ';
end

% Model type
if isct(obj)
    timeStr = 'Continuous-time';
else
    timeStr = 'Discrete-time';
end
fprintf('%s LPV-IO model\n\n', timeStr);

if obj.Na > 0
    Astr = ['(I + A(p,', Qstr, '))'];
else
    Astr = ['         ', emptyQstr, '  '];
end

if obj.Nu > 0
    Bstr = ['B(p,', Qstr, ')'];
    inputStr = [Bstr, ' u'];
else
    inputStr = '0';
end

% IO equation
fprintf('    %s y = %s\n\n', ...
    Astr, inputStr);

% Descprition of polynomials A, B
fprintf('with\n\n');
if obj.Na > 0
    fprintf('\tA(p,%s) = sum( A(p){i} %s, i = 1,...,Na)\n', ...
        Qstr, Qistr);
end
if obj.Nu > 0
    fprintf('\tB(p,%s) = sum( B(p){i} %s, i = 0,...,Nb)\n', ...
        Qstr, Qistr);
end
fprintf('\n');

% Description of input-scheduling-output
fprintf('and\n\n');
if obj.Na > 0
    fprintf('    Na: %d\n', obj.Na);
end
if obj.Nu > 0
    fprintf('    Nb: %d\n', obj.Nb);
end
fprintf('\n');
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
