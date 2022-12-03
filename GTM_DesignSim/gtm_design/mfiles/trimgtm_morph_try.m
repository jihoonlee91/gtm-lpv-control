function [MWS,Xtrim,Trimcond,Err,trial] = trimgtm_morph_try(tas,alt,mor)

trial = 1;
while 1
    [MWS,Xtrim,Trimcond,Err] = trimgtm_morph(tas,alt,mor);
    if (Err < 1e-3)
        break
    elseif trial >= 4
        warning('TRIM FAIL')
        break
    else
        warning('Poor convergence, trying again.\n');
        trial = trial+1;
    end
end
