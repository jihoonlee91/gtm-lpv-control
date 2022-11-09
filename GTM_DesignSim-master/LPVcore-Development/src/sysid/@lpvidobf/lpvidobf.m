classdef lpvidobf < lpvidss
    %LPVIDOBF Orthonormal basis functions LPV model with identifiable parameters
    %
    % Represents the following model structure:
    %
    %       +--------+   +-------+   +--------+   e     
    %       |        |   |       |   |        |   |     
    %   u-->|   Hf   +-->|  OBF  +-->|   Wf   +-->+-->y 
    %       |        |   |       |   |        |         
    %       +--------+   +-------+   +--------+         
    %
    % with:
    %   u: input signal
    %   y: output signal
    %   e: white noise signal
    %   Wf / Hf: Wiener and Hammerstein filters, respectively. These contain
    %       all identifiable coefficients of the model structure and may be
    %       scheduling dependent by specifying them as PMATRIX objects.
    %   OBF: an LTI model containing a bank of orthonormal basis functions.
    %       All coefficients of this model are pre-determined based on the
    %       parameterization of the OBFs (in terms of pole locations and
    %       number of extensions.
    %
    % The LPVIDOBF model structure is based on Chapter 9 of "Modeling and
    % identification of linear parameter-varying systems" by R. Toth
    % (2010). See that reference for more information.
    %
    % Syntax:
    %   sys = lpvidobf(v, ne, Wf, Hf, NoiseVariance, Ts)
    %   sys = lpvidobf(v, ne, Wf, Hf, NoiseVariance, Ts, Name, Value)
    %
    % Inputs:
    %   v: vector of pole locations of the basis-generating inner
    %       functions.
    %   ne: number of extensions of the inner functions.
    %   Wf: Wiener filter, specified as a numeric vector or
    %       scheduling-dependent PMATRIX object. The size of W must be
    %       compatible with the OBF bank: it must be nb-by-1, with nb the
    %       number of extensions (ne) times the number of poles (v) plus 1
    %       for the direct feedthrough term.
    %   Hf: Hammerstein filter, specified as a numeric vector or
    %       scheduling-dependent PMATRIX object. The size of H must be
    %       compatible with the OBF bank: it must be nb-by-1, with nb the
    %       number of extensions (ne) times the number of poles (v) plus 1
    %       for the direct feedthrough term.
    %   NoiseVariance: covariance matrix of model output noise.
    %   Ts: Sampling time. Currently, only discrete-time (DT) LPVIDOBF
    %       objects are supported. Hence, Ts must be -1 or positive.
    %
    
    properties (SetAccess=immutable)
        % Wiener filter, specified as an idpmatrix object.
        % Set to empty if unused.
        Wf
        % Hammerstein filter, specified as an idpmatrix object.
        % Set to empty if unused.
        Hf
        % Pole locations of the inner function
        v
        % Number of extensions of the inner function
        ne
    end

    properties (Hidden, SetAccess=immutable)
        % Balanced tate-space realization of Hambo basis functions
        M
    end

    methods
        function obj = lpvidobf(v, ne, Wf, Hf, NoiseVariance, Ts, varargin)
            %LPVIDOBF Construct an instance of this class
            %   Detailed explanation goes here
            
            %% Input validation
            narginchk(2, Inf);
            nargoutchk(0, 1);
            if nargin <= 2
                Wf = [];
            end
            % Number of coefficiens of W and H
            nb = ne * numel(v) + 1;  % "+1" because of feedthrough term
            % Check type of W
            if ~(isempty(Wf) || isa(Wf, 'pmatrix') || (isnumeric(Wf) && isvector(Wf)))
                error('''W'' must be a vector-valued pmatrix');
            end
            % Expand W if passed as scalar
            if ~isempty(Wf) && isscalar(Wf)
                Wf = Wf * ones(1, nb);
            end
            % Check size of W
            if ~(isempty(Wf) || (size(Wf, 2) == nb && size(Wf, 1) == 1))
                error('''Wf'' must have size 1-by-%d', nb);
            end
            if nargin <= 3
                Hf = [];
            end
            % Check type of H
            if ~(isempty(Hf) || isa(Hf, 'pmatrix') || (isnumeric(Hf)) && isvector(Hf))
                error('''H'' must be a vector-valued pmatrix');
            end
            % Expand H if passed as scalar
            if ~isempty(Hf) && isscalar(Hf)
                Hf = Hf * ones(nb, 1);
            end
            if ~(isempty(Hf) || (size(Hf, 2) == 1 && size(Hf, 1) == nb))
                error('''Hf'' must have size %d-by-1', nb);
            end
            if nargin <= 4
                NoiseVariance = 1;
            end
            if nargin <= 5
                Ts = -1;
            end
            if Ts == 0 || (Ts < 0 && Ts ~= -1)
                error('Sampling time %d is not supported for LPVIDOBF', Ts);
            end

            % Construct state-space coefficients from specified OBF
            % parametrization and Wiener-Hammerstein filters
            M = hambo(v, ne, Ts);
            [A, B, C, D] = ssdata(M);
            % Overwrite B matrix with Hf for Hammerstein model structure
            if ~isempty(Hf)
                B = Hf(2:end, :);
            end
            % Overwrite C matrix with Wf for Wiener model structure
            if ~isempty(Wf)
                C = Wf(:, 2:end);
            end
            % Overwrite D matrix
            if ~isempty(Hf)
                D = D * Hf(1, :);
            end
            if ~isempty(Wf)
                D = Wf(:, 1) * D;
            end
            nx = size(A, 1);
            K = zeros(nx, 1);
            obj = obj@lpvidss(A, B, C, D, 'innovation', K, [], ...
                NoiseVariance, Ts, varargin{:});
            % Store basis function state space realization for later usage in, e.g., LPVOBFEST
            obj.M = M;
            % A matrix is non-free
            obj.A.Free = false;
            % B matrix is non-free if Hf is empty
            if isempty(Hf)
                obj.B.Free = false;
            end
            % C matrix is non-free if Wf is empty
            if isempty(Wf)
                obj.C.Free = false;
            end
            % D matrix is non-free is Wf and Hf are empty
            if isempty(Hf) && isempty(Wf)
                obj.D.Free = false;
            end
            % K matrix is non-free
            obj.K.Free = false;
            % Set properties
            obj.v = v;
            obj.ne = ne;
            obj.Wf = Wf;
            obj.Hf = Hf;
        end
    end
end

