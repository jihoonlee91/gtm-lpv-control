function disp(obj, verbose)
% DISP Provide info on the lpvidpoly object
%
%   Syntax:
%       disp(sys)
%       disp(sys, verbose)
%
%   Inputs:
%       sys: LPVIDPOLY object
%       verbose: integer denoting the amount of information displayed. 0:
%           low, 1: high. Default: 0.
%
narginchk(1, 2);

if nargin < 2
    verbose = 0;
end

assert(isint(verbose) && verbose >= 0 && verbose <= 1, ...
    'verbose must be 0 or 1');

if isempty(obj)
    fprintf('Empty %s %s\n\n',lower(LPVcore.messageStr('LPVcore:timemap:DT')),lower(LPVcore.messageStr('LPVcore:lpvio:lpvio')));
    return;
end

if ~isempty(obj.SchedulingTimeMap) && strcmp(obj.SchedulingTimeMap.Domain, 'ct')
    Qstr  = 'd/dt';
    Qistr = 'd^i/dt^i';
else
    Qstr  = 'q^-1';
    Qistr = 'q^-i';
end

isA = ~iseye(obj.A);

if isA
    Astr  = sprintf('(I + A(p,%s)) y(t)',Qstr);
    Apstr = sprintf('\n\tA(p,%s) = sum( A(p){i} %s, i = 1,...,Na)',Qstr,Qistr);
else
    Astr  = '  y(t)';
    Apstr = '';
end


isB = obj.Nu > 0;

if isB
    Bstr  = sprintf('B(p,%s)',Qstr);
    Bpstr = sprintf('\n\tB(p,%s) = sum( B(p){i+1} %s, i = 0,...,Nb)',Qstr,Qistr);
else
    Bstr = '';
    Bpstr = '';
end


isC = ~iseye(obj.C);

if isC
    Cstr  = sprintf('(I + C(p,%s))',Qstr);
    Cpstr = sprintf('\n\tC(p,%s) = sum( C(p){i} %s, i = 1,...,Nc)',Qstr,Qistr);
else
    Cstr = '';
    Cpstr = '';
end

isD = ~iseye(obj.D);

if isD
    Dstr  = sprintf('(I + D(p,%s))^-1 ',Qstr);
    Dpstr = sprintf('\n\tD(p,%s) = sum( D(p){i} %s, i = 1,...,Nd)',Qstr,Qistr);
else
    Dstr = '';
    Dpstr = '';
end

isF = ~iseye(obj.F);

if isF
    Fstr  = sprintf('(I + F(p,%s))^-1 ',Qstr);
    Fpstr = sprintf('\n\tF(p,%s) = sum( F(p){i} %s, i = 1,...,Nf)',Qstr,Qistr);
else
    Fstr = '';
    Fpstr = '';
end

if isF || isB
    Ustr = ' u(t)';
else
    Ustr = '';
end

if isD || isC
    Estr = ' e(t)';
else
    Estr = '';
end

if (isF || isB) && (isD || isC)
    Pstr = ' + ';
else
    Pstr = '';
end

isCt = ~isempty(obj.SchedulingTimeMap) && strcmp('ct', obj.SchedulingTimeMap.Domain);
if  isCt
    timeStr = LPVcore.messageStr('LPVcore:timemap:CT');
    timeStr = [upper(timeStr(1)),timeStr(2:end)];
else    
    timeStr = LPVcore.messageStr('LPVcore:timemap:DT');
    timeStr = [upper(timeStr(1)),timeStr(2:end)];
end

fprintf('%s LPV-%s model\n\n   %s = %s%s%s%s%s%s%s\n\n',timeStr, ...
    modelStructureName(obj), Astr, Fstr, Bstr, Ustr, Pstr, Dstr, Cstr, Estr)
fprintf( 'with%s%s%s%s%s\n\n', Apstr, Bpstr, Cpstr, Dpstr, Fpstr);

% Input delay
if obj.Nu > 0 && obj.InputDelay(1) ~= 0
    if obj.Nu == 1
        fprintf('Input delay: %g\n', obj.InputDelay(1));
    else
        fprintf('Input delays (listed by channel): [%s]\n', ...
            num2str(obj.InputDelay'));
    end
end

% Noise variance
if verbose >= 1
    fprintf('Noise variance:\n');
    disp(obj.NoiseVariance);
end

% Sample time
if ~isCt
    if obj.Ts == -1; TsStr = 'unspecified';
    else;            TsStr = sprintf('%g %s',obj.Ts,obj.TimeUnit);                      end
    
    fprintf('%s: %s.\n', 'Sample time', TsStr);
end

if obj.Na > 0; naStr = sprintf('  Na=%d',obj.Na); else; naStr = ''; end
if obj.Nu > 0; nbStr = sprintf('  Nb=%d',obj.Nb); else; nbStr = ''; end
if obj.Nc > 0; ncStr = sprintf('  Nc=%d',obj.Nc); else; ncStr = ''; end
if obj.Nd > 0; ndStr = sprintf('  Nd=%d',obj.Nd); else; ndStr = ''; end
if obj.Nf > 0; nfStr = sprintf('  Nf=%d',obj.Nf); else; nfStr = ''; end
           
fprintf('\nParameterization:\n');
fprintf('   Polynomial orders:%s%s%s%s%s\n',naStr,nbStr,ncStr,ndStr,nfStr);
fprintf('   Number of coefficients: %d (free: %d)\n', ...
    nparams(obj), nparams(obj, 'free'));
fprintf('\nStatus:\n%s\n',obj.Report.Status);

end

