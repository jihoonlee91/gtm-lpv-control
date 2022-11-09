function varargout = size(obj,varargin)
%SIZE The dimension of the input, scheduling, output data set.
%
%   m = size(A), returns a scalar number containing the
%   number of data points in the data set.
%
%   [m,n] = size(A), returns the scalar numbers containing the
%   number of data points and number of output chanels.
%
%   [m,n,o] = size(A), returns the scalar numbers containing the
%   number of data points, number of output chanels, number of scheduling
%   chanels.
%
%   [m,n,o,q] = size(A), returns the scalar numbers containing the
%   number of data points, number of output chanels, number of scheduling
%   chanels, number of input chanels.
%
%   m = size(A,dim), returns a scalar number containing the dim dimension.
%   In case dim = 2, then m is the number of data points in the data set.
%   In case dim = 2, then m is the number of output chanels. In case dim =
%   3, then m is the number of scheduling chanels. In case dim = 4, then m
%   is the number of input chanels. Else m = 1
%
%   See also LPVIDDATA/LENGTH, LPVIDDATA

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    narginchk(1,2);     % Validate correct number of inputs
    no = nargout;

    if isempty(varargin)
        
        
        switch no
            case 0 
            
                if obj.OSISize(1) ~= 1; adjO = 's'; else; adjO = ''; end
                if obj.OSISize(2) ~= 1; adjS = 's'; else; adjS = ''; end
                if obj.OSISize(3) ~= 1; adjI = 's'; else; adjI = ''; end

                fprintf(['Data set with %d output',adjO,', %d scheduling signal',adjS,', and %d input',adjI,'.\n'], obj.OSISize);
                fprintf('The number of data points is %d.\n',obj.lengthDataSet);

            case no == 1
                [ST,~] = dbstack;   % Small workaround to get the variable editor to show the object properties
                if isstruct(ST) && size(ST,1)>1 && strcmpi(ST(2).file, 'arrayviewfunc.m')
                    varargout{1} = [obj.OSISize(1),obj.OSISize(2)];%obj.lengthDataSet;
                else
                    varargout{1} = [obj.lengthDataSet, obj.OSISize];
                end
            otherwise
                varargout = cell(1,no);
                varargout(:) = {1};
                tmp_ = {obj.lengthDataSet, obj.OSISize(1), obj.OSISize(2), obj.OSISize(3)};
                n = min(no,4);
                varargout(1:n) = tmp_(1:n);
        end
    else
        
        if no > 1; ctrlMsgUtils.error('MATLAB:TooManyOutputs'); end
        
        if isscalar(varargin{1}) && isnumeric(varargin{1}) && ~any(isnan(varargin{1})) && ~any(isinf(varargin{1})) && uint16(varargin{1}) == varargin{1} && varargin{1} > 0
            
            switch varargin{1}
                case 1; varargout{1} = obj.lengthDataSet;
                case 2; varargout{1} = obj.OSISize(1);
                case 3; varargout{1} = obj.OSISize(2);
                case 4; varargout{1} = obj.OSISize(3);
                otherwise; varargout{1} = 1;
            end
        else
            LPVcore.error('LPVcore:general:InvalidInputArgument','second','positive integer','lpviddata/size');
        end
        
    end
end