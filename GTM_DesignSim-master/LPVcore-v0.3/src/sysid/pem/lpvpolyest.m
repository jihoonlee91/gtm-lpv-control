function sys = lpvpolyest(data, template_sys, options)
%LPVPOLYEST Estimation of LPVIDPOLY system
%
%   Syntax:
%       sys = lpvpolyest(data, template_sys)
%       sys = lpvpolyest(data, template_sys, options)
%
%   Inputs:
%       data: lpviddata object
%       template_sys: lpvidpoly object that indicates the structure of the
%           model to be identified.
%       options: lpvpolyestOptions object
%
%   Outputs:
%       sys: estimated system.
%

%% Input validation
assert(isa(data, 'lpviddata'), '''data'' must be an lpviddata object');
assert(isa(template_sys, 'lpvidpoly'), '''template_sys'' must be an lpvidpoly object');
if nargin <= 2
    options = lpvpolyestOptions;
end
assert(isa(options, 'lpvpolyestOptions'), ...
    '''options'' must be an lpvpolyestOptions object');

%% Check if model structure is ARX
if isarx(template_sys)
    sys = lpvarx(data, template_sys, options);
    return;
end

%% Create GUI if Display is on
gui = progressgui(template_sys, data, options);

%% Initialization
if strcmpi(options.Initialization, 'template')
    progressmsg(gui, 'Initialization using the provided template system');
    progressmsg(gui, '');
    init_sys = template_sys;
elseif strcmpi(options.Initialization, 'riv')
    assert(issiso(template_sys), ['Initialization method ''riv''', ...
        ' is only available for SISO models']);
    assert(isbj(template_sys), ['Initialization method ''riv''', ...
        ' is only available for LPV-OE and LPV-BJ model structures']);
    % Check if C and D are LTI
    assert(isconst(template_sys.C) && isconst(template_sys.D), ...
        ['Currently, ''riv'' only supports LPV-BJ models with non-parameter-varying ', ...
        '''C'' and ''D''']);
    progressmsg(gui, 'Initialization using refined IV');
    init_sys = riv(data, template_sys);
    % Since rivLPV calls pem internally, the progress GUI is closed for BJ
    % models. Thus, we need to re-open it after calling riv.
    if ~isoe(template_sys)
        gui = progressgui(disp, template_sys, data, method);
        progressmsg(gui, 'Initialization using refined IV');
    end
    progressmsg(gui, 'Finished initialization');
    progressmsg(gui, '');
elseif strcmpi(options.Initialization, 'polypre')
    % Pre-estimation should not display anything
    optionsNoDisplay = options;
    optionsNoDisplay.Display = 'off';
    optionsNoDisplay.Verbose = false;
    if isarmax(template_sys)
        % Pre-estimation using ARX
        progressmsg(gui, 'Initialization using lpvarx pre-estimation');
        arx_sys = template_sys;
        arx_sys.C = {eye(arx_sys.Ny)};
        init_sys = lpvarx(data, arx_sys, optionsNoDisplay);
        % Randomize coefficients of C
        init_sys.C = rand(template_sys.C, [-0.25, 0.25]);
    elseif isoe(template_sys)
        % Pre-estimation using ARX
        progressmsg(gui, 'Initialization using lpvarx pre-estimation');
        arx_sys = template_sys;
        % Convert OE to ARX
        arx_sys.A = arx_sys.F;
        arx_sys.F = {eye(arx_sys.Ny)};
        init_sys = lpvarx(data, arx_sys, optionsNoDisplay);
        % Convert ARX to OE
        init_sys.F = init_sys.A;
        init_sys.A = {eye(init_sys.Ny)};
    elseif isbj(template_sys)
        % Two pre-estimation steps: ARX --> OE
        progressmsg(gui, 'Initialization using lpvarx pre-estimation (step 1 / 2)');
        arx_sys = template_sys;
        % Convert BJ to ARX
        arx_sys.A = arx_sys.F;
        arx_sys.F = {eye(arx_sys.Ny)};
        arx_sys.C = {eye(arx_sys.Ny)};
        arx_sys.D = {eye(arx_sys.Ny)};
        arx_sys = lpvarx(data, arx_sys, optionsNoDisplay);
        progressmsg(gui, 'Initialization using lpvoe pre-estimation (step 2 / 2)');
        % Convert ARX to OE
        oe_sys = arx_sys;
        oe_sys.F = oe_sys.A;
        oe_sys.A = {eye(oe_sys.Ny)};
        init_sys = lpvoe(data, oe_sys, optionsNoDisplay);
        % Randomize free coefficients of C and D
        init_sys.C = rand(template_sys.C, [-0.25, 0.25]);
        init_sys.D = rand(template_sys.D, [-0.25, 0.25]);
    end
    progressmsg(gui, 'Finished initialization');
    progressmsg(gui, '');
end

%% Select algorithm
if strcmp(options.SearchMethod, 'gradient')
    % Gradient-based
    sys = lpvpolygrad_(data, init_sys, options, gui);
else
    % Pseudo-linear least squares
    sys = lpvpolypseudols_(data, init_sys, options, gui);
end

%% Update report
sys.Report = gui.Report;

