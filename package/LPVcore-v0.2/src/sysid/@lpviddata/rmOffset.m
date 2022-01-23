function data = rmOffset(data,OutputOff,SchedulingOff,InputOff)
%RMOFFSET Removes the offset from the given data.
%
%   data = rmOffset(data,OutputOffset,SchedulingOffset,InputOffset)
%       removes OutputOffset from the output signal, removes
%       SchedulingOffset from the scheduling signal, and removes
%       InputOffset from the input signal in DATA.
%
%   DATA is the time-domain estimation data captured in an LPVIDDATA
%      object.
%   OUTPUTOFFSET is a scalar or Ny-by-1 dimensional real vector. If the
%      output dimension (Ny) is bigger than 1 and OUTPUTOFFSET is scalar
%      then OUTPUTOFFSET is applied to all output signals.
%   SCHEDULINGOFFSET is a scalar or Np-by-1 dimensional real vector. If the
%      scheduling dimension (Np) is bigger than 1 and SCHEDULINGOFFSET is
%      scalar then SCHEDULINGOFFSET is applied to all output signals.
%   INPUTOFFSET is a scalar or Nu-by-1 dimensional real vector. If the
%      input dimension (Nu) is bigger than 1 and INPUTOFFSET is scalar
%      then INPUTOFFSET is applied to all output signals.

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    [N,ny,np,nu] = size(data);
    
    if nargin < 4; InputOff = []; end
    if nargin < 3; SchedulingOff = []; end
    if nargin < 2; OutputOff = []; end
    
    
    if ~validateOff(OutputOff, ny)
        LPVcore.error('LPVcore:general:InvalidInputArgument','second',sprintf('scalar or %d-by-1 dimensional real vector',ny),'lpviddata/rmOffset'); 
    end
    
    if ~validateOff(SchedulingOff, np)
        LPVcore.error('LPVcore:general:InvalidInputArgument','third',sprintf('scalar or %d-by-1 dimensional real vector',np),'lpviddata/rmOffset');
    end

    if ~validateOff(InputOff, nu)
        LPVcore.error('LPVcore:general:InvalidInputArgument','fourth',sprintf('scalar or %d-by-1 dimensional real vector',nu),'lpviddata/rmOffset');
    end

    
    if isscalar(OutputOff)
        data.y = data.y - ones(size(data.y))*OutputOff;
    elseif ~isempty(OutputOff)
        if iscolumn(OutputOff); OutputOff = OutputOff'; end
        data.y = data.y - repmat(OutputOff,[N 1]);
    end
    
    if isscalar(SchedulingOff)
        data.p = data.p - ones(size(data.p))*SchedulingOff;
    elseif ~isempty(SchedulingOff)
        if iscolumn(SchedulingOff); SchedulingOff = SchedulingOff'; end
        data.p = data.p - repmat(SchedulingOff,[N 1]);
    end
    
    if isscalar(InputOff)
        data.u = data.u - ones(size(data.u))*InputOff;
    elseif ~isempty(InputOff)
        if iscolumn(InputOff); InputOff = InputOff'; end
        data.u = data.u - repmat(InputOff,[N 1]);
    end
    
end  
    
    
function bool = validateOff(num, n)

    bool = false;

    if isempty(num) || ...
            ( isscalar(num) && isnumeric(num) && ~isinf(num) && ~isnan(num) ) || ...
            ( isvector(num) && numel(num) == n && isnumeric(num) && ~any(isinf(num)) && ~any(isnan(num)) )
        bool = true;
    end

end