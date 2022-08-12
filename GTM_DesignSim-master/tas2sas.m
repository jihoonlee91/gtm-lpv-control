function sas = tas2sas(tas,h)

sas = 100-30000/(30000-h)*(100-tas);