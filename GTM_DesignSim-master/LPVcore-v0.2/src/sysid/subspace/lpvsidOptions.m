classdef (CaseInsensitiveProperties) lpvsidOptions < lpvsysidOptions
    %LPVSIDOPTIONS Option set for lpvsid command
    %
    %   Syntax:
    %       opts = lpvsidOptions
    %
    %       opts = lpvsidOptions('Name', 'Value', ...)
    %
    %   Name-Value pairs:
    %
    %       Display: whether to display the estimation progress. Specify as
    %           one of:
    %               'on': information is displayed
    %               'off': information is not displayed
    %           Default: 'off'.
    %
    %       PastWindowSize: size of past window to use for estimating the
    %           state sequence. Specified as a positive integer.
    %           Default: 6.
    %
    
    properties
        % Past window size used to estimate the state sequence. Specified
        % as a positive integer. Default: 6
        PastWindowSize
        
        % Whether to display estimation progress. Specified as 'on' or
        % 'off'. Default: 'off'.
        Display
    end
    
    methods
        function obj = lpvsidOptions(varargin)
            %LPVSIDOPTIONS Construct an instance of this class
            obj.PastWindowSize = 6;
            obj.Display = 'off';
            obj = initOptions(obj, varargin{:});
        end
        
        function this = set.PastWindowSize(this, value)
            assert(isint(value) && value > 0, ...
                '''PastWindowSize'' must be a positive integer');
            this.PastWindowSize = value;
        end
        
        function this = set.Display(this,Display)
           this.Display = idoptions.utValidateEstimationOptions('Display', Display);
        end
    end
end

