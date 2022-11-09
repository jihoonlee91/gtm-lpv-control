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
    %       Kernelization: whether to use kernelization to decrease computational
    %           complexity, at the cost of increased bias. Default: true.
    %
    %       KernelizationRegularizationConstant: Tikhonov regularization
    %           constant used for solving the dual problem (19). Default:
    %           1E-5.
    %
    
    properties
        % Past window size used to estimate the state sequence. Specified
        % as a positive integer. Default: 6
        PastWindowSize
        
        % Whether to display estimation progress. Specified as 'on' or
        % 'off'. Default: 'off'.
        Display
        
        % Whether to use kernelization to decrease computational
        % complexity, the the cost of increased bias. Default: true
        Kernelization
        
        KernelizationRegularizationConstant
    end
    
    methods
        function obj = lpvsidOptions(varargin)
            %LPVSIDOPTIONS Construct an instance of this class
            obj.PastWindowSize = 6;
            obj.Display = 'off';
            obj.Kernelization = true;
            obj.KernelizationRegularizationConstant = 1E-5;
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
        
        function this = set.Kernelization(this, value)
            assert(islogical(value) && isscalar(value), ...
                '''Kernelization'' must be a logical scalar');
            this.Kernelization = value;
        end
    end
end

