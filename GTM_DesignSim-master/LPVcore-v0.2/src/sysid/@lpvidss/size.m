function varargout = size(obj,varargin)
% SIZE The size of the linear parameter-varying state-space representation
% with noise model.
%
%   D = size(A), returns the two-element row vector D = [ny,nu] containing
%   the number of outputs and inputs of the input-output representation/
%
%
%   [m,n] = SIZE(A), returns the number of outputs and inputs in A as
%   separate output variables.
%
%   D = SIZE(A,dim) returns the length of the dimension specified by the
%   scalar dim. If DIM > 3, D will be 1.
%
% See also: SIZE, LPVIDSS/NDIMS.
%

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    no = nargout;

    if isempty(varargin)
        if no == 0
            if isa(obj.Structure_.SchedulingTimeMap,'timemap')
                np = obj.Structure_.SchedulingTimeMap.SchedulingDimension;
            else
                np =0;
            end
            if obj.IOSize(1) == 1; adjO  = ''; else; adjO  = 's'; end
            if np == 1;            adjS  = ''; else; adjS  = 's'; end
            if obj.IOSize(2) == 1; adjI  = ''; else; adjI  = 's'; end
            if obj.StateSize == 1; adjSt = ''; else; adjSt = 's'; end
            
            if isempty(obj.G) && isempty(obj.H)
                fprintf('%s\n', LPVcore.messageStr('LPVcore:lpvidss:sizeInnovation',obj.IOSize(1),adjO,np,adjS,obj.IOSize(2),adjI,obj.StateSize,adjSt));
            else
                fprintf('%s\n', LPVcore.messageStr('LPVcore:lpvidss:sizeGeneral',   obj.IOSize(1),adjO,np,adjS,obj.IOSize(2),adjI,obj.StateSize,adjSt));
            end
            
            varargout = {};
        elseif no == 1
            varargout{1} = [obj.Ny, obj.Nu];
        elseif no >= 2
            varargout = cell(1,no);
            varargout(:) = {1};
            varargout{1} = obj.Ny;
            varargout{2} = obj.Nu;
        end
    else
        if no <= 1
            tmp_ = varargin{1};

            if isscalar(tmp_) && isnumeric(tmp_) && tmp_ > 0 && uint16(tmp_) == tmp_

                if tmp_ >= 3; varargout{1} = 1;
                else; varargout{1} = size(obj, tmp_);
                end
            else
                LPVcore.error('LPVcore:general:InvalidInputArgument', 'second','positive scalar integer','lpvidss/size');
            end

        else
            ctrlMsgUtils.error('MATLAB:TooManyOutputs');
        end
    end
end