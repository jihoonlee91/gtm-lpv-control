function ed = estimDataReport(ed, data, yOff, pOff, uOff)
% Populate DataUsed structure (ed) based on estimation data.

% Based on idresults.estimDataReport. However, the LPV extention need the
% schedlung signal to be included.

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

ed.Name = data.Name;
ed.OutputOffset = yOff;
ed.SchedulingOffset = pOff;
ed.InputOffset =  uOff;
ed.Type = idresults.getDataTypeStr(true, true, []);
ed.Ts = data.Ts;
ed.Length = size(data,1);