classdef (CaseInsensitiveProperties, TruncatedProperties) lpvpolyest < ...
      idresults.arx & idresults.SearchInfo
%LPVPOLYEST Search results for LPV polynomial estimation using LPVPOLYEST.
   
% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).
%
% Note: Template: idresults.polyest

    
   
   methods
      function obj = lpvpolyest(Options, Command)
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
   end
end
