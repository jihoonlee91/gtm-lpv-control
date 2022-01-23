function disp(obj)
% DISP Provide info on the lpvidss object
%

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

if isempty(obj)
    fprintf('Empty %s %s\n\n',lower(LPVcore.messageStr('LPVcore:timemap:DT')),lower(LPVcore.messageStr('LPVcore:lpvss:lpvss')));
    return;
end


A = obj.A;  isA = ~isempty(A) + ~isconst(A);
if isA ==0;      Astr = '         ';
elseif isA == 1; Astr = '  A  x(t)';
else;            Astr = 'A(p) x(t)';
end

B = obj.B;  isB = ~isempty(B) + ~isconst(B);

if isB ==0;      Bstr = '         ';
elseif isB == 1; Bstr = '  B  u(t)';
else;            Bstr = 'B(p) u(t)';
end

C = obj.C;  isC = ~isempty(C) + ~isconst(C);

if isC ==0;      Cstr = '         ';
elseif isC == 1; Cstr = '  C  x(t)';
else;            Cstr = 'C(p) x(t)';
end

D = obj.D;  isD = ~isempty(D) + ~isconst(D);

if isD ==0;      Dstr = '         ';
elseif isD == 1; Dstr = '  D  u(t)';
else;            Dstr = 'D(p) u(t)';
end

isInnov = obj.IsInnovationForm_;

if isInnov
    K = obj.K;  isK = ~isempty(K) + ~isconst(K);

    if isK ==0;      Kstr = '';
    elseif isK == 1; Kstr = '  K  e(t)'; 
    else;            Kstr = 'K(p) e(t)';
    end

else

    G = obj.G;  isG = ~isempty(G) + ~isconst(G);

    if isG ==0;     Gstr = '';
    elseif isG == 1
        Gstr = '  G  v(t)';
    else;           Gstr = 'G(p) v(t)';
    end
    
    H = obj.H;  isH = ~isempty(H) + ~isconst(H);

    if isH ==0;     Hstr = '';
    elseif isH == 1
        Hstr = '  H  e(t)';
    else;           Hstr = 'H(p) e(t)';
    end

end

Xstr = ' x(t+1) = ';
Ystr = '   y(t) = ';


if isA == 0 && isC == 0; Astr = ''; Cstr = ''; end
if isB == 0 && isD == 0; Bstr = ''; Dstr = ''; end
    


if isA ~=0 && isB ~= 0; Xplus = ' + ';
else;                   Xplus = '   ';  end

if isC ~=0 && isD ~= 0; Yplus = ' + ';
else;                   Yplus = '   ';  end


timeStr = LPVcore.messageStr('LPVcore:timemap:DT');
if isInnov
    fprintf('%s %s\n\n',[upper(timeStr(1)),timeStr(2:end)],lower(LPVcore.messageStr('LPVcore:lpvidss:lpvidssinnov')));
else
    fprintf('%s %s\n\n',[upper(timeStr(1)),timeStr(2:end)],lower(LPVcore.messageStr('LPVcore:lpvidss:lpvidssgen')));
end


if isInnov
    
    if (isA ~=0 || isB ~=0) && isK ~= 0; Xkplus = ' + ';
    else;                                Xkplus = '   ';  end
    
    
    if isA ~=0 || isB ~=0 || isK ~=0 
        fprintf( '\t%s%s%s%s%s%s\n\t%s%s%s%s + e(t)\n\n', Xstr, Astr, Xplus, Bstr, Xkplus, Kstr, Ystr, Cstr, Yplus, Dstr   );
    else
        fprintf( '\t%s%s%s%s + e(t)\n\n', Ystr, Cstr, Yplus, Dstr   );
    end
    
    fprintf('with\n\te(t) ~ N(0,NoiseVariance)\n\n');
    
else
    
	if (isA ~=0 || isB ~=0) && isG ~= 0; Xkplus = ' + ';
    else;                                Xkplus = '   ';  end
    
    if (isC ~=0 || isD ~=0) && isH ~= 0; Ykplus = ' + ';
    else;                                Ykplus = '   ';  end
    
    if (isA ~=0 || isB ~=0 || isG ~=0) && (isC ~=0 || isD ~=0 || isH ~=0)
        fprintf( '\t%s%s%s%s%s%s\n\t%s%s%s%s%s%s\n\n', Xstr, Astr, Xplus, Bstr, Xkplus, Gstr, Ystr, Cstr, Yplus, Dstr, Ykplus, Hstr );
    elseif isA ==0 && isB ==0 && isG ==0
        fprintf( '\nans=\n\t%s%s%s%s%s%s\n\n', Ystr, Cstr, Yplus, Dstr, Ykplus, Hstr );
    else
        fprintf( '\t%s%s%s%s%s%s\n\n', Xstr, Astr, Xplus, Bstr, Xkplus, Gstr );
    end
    
    fprintf('with\n\t[v(t);e(t)] ~ N(0,NoiseVariance)\n\n');
    
end


if obj.Ts == -1; TsStr = 'unspecified';
else;            TsStr = sprintf('%g %s',obj.Ts,obj.TimeUnit);                      end

fprintf('Sampling time: %s\n', TsStr);

fprintf('\nParameterization:\n');
if isD ~=0; fprintf('   Feedthrough: yes\n'); else; fprintf('   Feedthrough: no\n'); end
fprintf('   Number of free coefficients: %d\n',nparams(obj));
fprintf('\nStatus:\n%s\n',obj.Report.Status);

end

