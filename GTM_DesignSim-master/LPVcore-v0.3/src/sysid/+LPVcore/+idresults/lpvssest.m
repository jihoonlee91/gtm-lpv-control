classdef (CaseInsensitiveProperties, TruncatedProperties) lpvssest < ...
      idresults.arx & idresults.SearchInfo
%LPVPOLYEST Search results for LPV polynomial estimation using LPVSSEST.
   
% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).
%
% Note: Template: idresults.ssest 
   
   methods
      function obj = lpvssest(Options, Command)
         % lpvpolyest(Options, CommandName)
         if nargin<2
            Command = '';
         end
         
         SearchMethod = {};
         if nargin<1
            Options = [];
         elseif ~isempty(Options)
            SearchMethod = {Options.SearchMethod};
         end
         
         obj = obj@idresults.arx(Options,Command);
         obj = obj@idresults.SearchInfo(SearchMethod{:});
      end
      
      
      function obj = createReport(obj, yh, npar, data, options, method, status)
        
       	eh = (data.OutputData-yh);
        N = size(data,1);

        obj.OptionsUsed = options;
        obj.Method = method;
        obj.RandState = rng;

        obj.Fit.LossFcn = eh(:)'*eh(:)/N;

        obj.DataUsed = LPVcore.estimDataReport(struct(), data, options.OutputOffset, options.SchedulingOffset, options.InputOffset);

        [FitNRMSE, MSE] = idpack.utFitPercent({yh}, {data.OutputData}, true);
        [~, FPE, AIC, AICc, nAIC, BIC] = idpack.utComputeMetrics(eh, N, npar, size(data,2), true);

        obj.Fit.FitPercent = FitNRMSE;
        obj.Fit.MSE = MSE;
        obj.Fit.FPE = FPE;
        obj.Fit.AIC = AIC;
        obj.Fit.AICc = AICc;
        obj.Fit.nAIC = nAIC;
        obj.Fit.BIC = BIC;

        obj.Status = status;

      end
      
   end
end
