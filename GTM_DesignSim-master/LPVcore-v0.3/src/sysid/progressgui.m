classdef progressgui < handle
    %PROGRESSGUI Display progress window during system identification
    %   Syntax:
    %       h = progressgui(sys, data, options)
    %
    %   Inputs:
    %       visible: logical scalar denoting whether the GUI should be
    %           visible.
    %       sys: lpvidpoly or lpvidss object used in estimation. Information such as
    %           the IO size, delays and number of parameters are extracted.
    %       data: lpviddata object with IO data.
    %       options: option set used with command
    %
    %   Example usage:
    %       h = progressgui(sys, data, options);
    %       for iter=1:N
    %           cost = randn;
    %           h.update(iter, cost);
    %       end
    %
    
    properties (SetAccess = private)
        Sys;        % Template LPVIDSYS model structure used in identification
        Data;       % Dataset used in identification
        Options;    % Option set used in identification
        Visible;    % Whether GUI is visible
        Report;     % Report generated
    end
    
    properties (Access = private, Hidden)
        terminated_ = false;
        command_;    % Character array describing the method
        name_;
        p_;     % Reference to plot
        iter_; % Vector of iteration numbers
        cost_; % Vector of costs associated with iterations in iter_
        view_; % Estimation progress GUI
        report_;  % Estimation report
    end
    
    methods
        function obj = progressgui(sys, data, options)
            %PROGRESSGUI Construct an instance of this class
            
            % Argument validation
            narginchk(3, 3);
            assert(isa(sys, 'lpvidsys'), ...
                'System must be an lpvidsys object.');
            assert(isa(data, 'lpviddata'), ...
                'Data must be an lpviddata object.');
            assert(isa(options, 'lpvsysidOptions'), ...
                'Options must be an lpvsysidOptions object');
            
            obj.Visible = strcmpi(options.Display, 'on');
            obj.Sys = sys;
            obj.report_ = sys.Report;
            obj.Data = data;
            obj.Options = options;
            obj.command_ = options.CommandName;
            % Update report with method, data, and options used
            obj.report_.Method = obj.command_;
            obj.report_.Status = ['Estimated using ', upper(obj.command_), ...
                ' with prediction focus'];
            obj.report_.DataUsed = data;
            obj.report_.OptionsUsed = options;
            
            % Create GUI
            obj.name_ = ['Identification of LPV-', sys.Type, ...
                ' model ', sys.Name];
            if numel(obj.command_) > 0
                obj.name_ = [obj.name_, ' using ', obj.command_];
            end
            
            if obj.Visible
                if verLessThan('matlab', '9.10')
                    % Before R2021a
                    obj.view_ = idpack.EstimProgress.getEstimProgressView(true, true);
                else
                    % R2021a and later
                    obj.view_ = idguis.internal.identapp.dialogs.EstimationProgressDialog.getEstimProgressView(true, true);
                end
                obj.view_.clearAllInfo();
                obj.intro();
            end
        end
        
        function obj = progressmsg(obj, msg)
            % PROGRESSMSG Add progress message to GUI
            %
            %   Syntax:
            %       obj = progressmsg(obj, message)
            %       
            %   Inputs:
            %       msg: (str) message to add
            %
            if obj.Visible
                idDisplayEstimationInfo('Progress', msg, obj.view_);
                % Pause is needed to refresh GUI on MATLAB > R2021a
                pause(0.001);
            end
        end
        
        function obj = update(obj, iter, cost, step)
            % UPDATE Add iteration with cost to GUI
            %
            %   Syntax:
            %       obj = update(obj, iter, cost, step)
            %
            %   Inputs:
            %       iter: (int) interation number
            %       cost: (scalar) cost function associated with this
            %       	iteration
            %       step: (scalar) step size associated with this iteration
            %
            narginchk(4, 4);
            assert(isscalar(iter) && iter == floor(iter),...
                'Iteration must be an integer.');
            assert(isscalar(cost), 'Cost must be a scalar.');
            assert(isscalar(step), 'Step must be a scalar.');
            
            obj.progress(iter, cost, step);
        end
        
        function obj = terminate(obj, est_sys, conditionId)
            % TERMINATE Annotate title with termination condition
            %
            %   Syntax:
            %       obj = terminate(obj, est_sys, conditionId)
            %
            %   Inputs:
            %       est_sys: estimated system
            %       conditionId: ID of termination condition, specified as
            %       a SysIdTermConditions.
            %
            assert(isa(est_sys, 'lpvrep'), 'Estimated system must be of type lpvrep');
            if ~isa(conditionId, 'SysIdTermConditions')
                error('Condition ID must be of type SysIdTermConditions');
            end
            obj.Sys = est_sys;
            [~, sim_fit_] = compare(obj.Data, obj.Sys, Inf);
            [~, pred_fit_] = compare(obj.Data, obj.Sys, 1);
            sim_fit = NaN(1, est_sys.Ny);
            pred_fit = NaN(1, est_sys.Ny);
            for i=1:est_sys.Ny
                sim_fit(i) = sim_fit_(i);
                pred_fit(i) = pred_fit_(i);
            end
            % Termination condition
            obj.terminated_ = true;
            switch conditionId
                case SysIdTermConditions.FunctionTolerance
                    reason = 'tolerance on cost function improvement';
                case SysIdTermConditions.StepTolerance
                    reason = 'tolerance on step size';
                case SysIdTermConditions.MaxIterationsReached
                    reason = 'max. nr. of iterations reached';
                case SysIdTermConditions.NonIterative
                    reason = 'non-iterative identification procedure';
                otherwise
                    error('Unknown termination condition');
            end
            resultsText = ['Identification terminated due to ', reason, '.'];
            % Update report
            obj.report_.Fit.PredictionFit = pred_fit;
            obj.report_.Fit.SimulationFit = sim_fit;
            % Update GUI
            if obj.Visible
                idDisplayEstimationInfo('Results', ...
                    resultsText, ...
                    obj.view_);
                idDisplayEstimationInfo('Results', ...
                    sprintf('Simulation fit to estimation data:           [%s]%%', num2str(sim_fit)), ...
                    obj.view_);
                idDisplayEstimationInfo('Results', ...
                    sprintf('Prediction fit to estimation data (1 step): [%s]%%', num2str(pred_fit)), ...
                    obj.view_);
                % Pause is needed to refresh GUI on MATLAB > R2021a
                pause(0.001);
            end
        end
        
        function v = get.Report(obj)
            if ~obj.terminated_
                warning('Report is retrieved before termination');
            end
            v = obj.report_;
        end
    end
    
    methods (Access = private)
        function intro(obj)
            % Display algorithm name
            if ~obj.Visible; return; end
            idDisplayEstimationInfo('Intro', ...
                obj.name_, ...
                obj.view_);
            % Display information on data
            ny = size(obj.Data.y, 2);
            nu = size(obj.Data.u, 2);
            np = size(obj.Data.p, 2);
            N = obj.Data.lengthDataSet;
            datastr = ['Data has ', num2str(ny), ' outputs, ', num2str(np), ...
                ' scheduling signals, ', num2str(nu), ' inputs and ', ...
                num2str(N), ' samples.'];
            idDisplayEstimationInfo('Intro', ...
                'Estimation data: Time domain data', obj.view_);
            idDisplayEstimationInfo('Intro', datastr, obj.view_);
            % Pause is needed to refresh GUI on MATLAB > R2021a
            pause(0.001);
        end
        
        function progress(obj, iter, cost, step)
            if ~obj.Visible; return; end
            if mod(iter, 10) == 1
                progressmsg(obj, sprintf('Iteration    \t    Cost    \t    Step size'));
            end
            if ~isinf(step)
                progressmsg(obj, sprintf('%d    \t    %d    \t    %d', iter, cost, step));
            else
                progressmsg(obj, sprintf('%d    \t    %d    \t    N/A', iter, cost));
            end
        end
    end
end

