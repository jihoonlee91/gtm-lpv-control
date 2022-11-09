classdef lpvrivOptions < lpvsysidOptions
%LPVRIVOPTIONS  Creates option set for the LPVRIV command.
%
%   OPT = lpvrivOptions returns the default options for LPVRIV.
%
%   OPT = lpvrivOptions('Option1',Value1,'Option2',Value2,...) uses name/value
%   pairs to override the default values for 'Option1','Option2',...
%
%   The supported options are:
%
%   Initialization:  Choose between initialization methods. Specify as one
%   of:
%           'template': initialization using the coefficients of the
%               provided template system.
%           'polypre': pre-estimation using an LPV-ARX model structure.
%       Default: 'template'
%
%   MaxIterations: maximum number of iterations. Default: 10.
%
%   Display: whether to display the progress window ('off' or 'on').
%
    
    properties
        % 'on' to display progress window, 'off' to disable
        Display
        % Max. number of iterations
        MaxIterations
        % Initialization method
        Initialization
    end
    
    methods
        function obj = lpvrivOptions(varargin)
            %LPVRIVOPTIONS Construct an instance of this class
            %   Detailed explanation goes here
            obj.Display = 'off';
            obj.MaxIterations = 10;
            obj.Initialization = 'template';
            obj.CommandName = 'lpvriv';
            obj = initOptions(obj, varargin{:});
        end

        function obj = set.Initialization(obj, val)
            assert(any(strcmp(val, {'template', 'polypre'})), ...
                'Initialization method must be one of: ''template'', ''polypre''');
            obj.Initialization = val;
        end

        function obj = set.MaxIterations(obj, val)
            assert(isint(val) && val >= 0, 'MaxIterations must be a non-negative integer');
            obj.MaxIterations = val;
        end
    end
end

