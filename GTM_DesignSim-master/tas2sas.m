function sas = tas2sas(tas,h)
% tas [knot]
% sas [knot]
% h [ft]

sas = 100-30000/(30000-h)*(100-tas);