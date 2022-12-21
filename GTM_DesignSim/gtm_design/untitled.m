%%
min(gamlon2hinf_grid,[],'all')
max(gamlon2hinf_grid,[],'all')
min(gamlon2h2op_grid,[],'all')
max(gamlon2h2op_grid,[],'all')
fprintf('lon2 LPVu Gam = %.5g (%.3g%% back-off from %.5g)\n',gamlon2lpvu,(gamlon2lpvu/infolon2lpvu.MinGamma-1)*100,infolon2lpvu.MinGamma)
fprintf('lon2 Hinf Syn Time: %.1f sec \n',timelon2hinf)
fprintf('lon2 H2op Syn Time: %.1f sec \n',timelon2h2op)
fprintf('lon2 LPVu Syn Time: %.1f sec \n',timelon2lpvu)

%%
min(gamlat3hinf_grid,[],'all')
max(gamlat3hinf_grid,[],'all')
min(gamlat3h2op_grid,[],'all')
max(gamlat3h2op_grid,[],'all')
fprintf('lat3 LPVu Gam = %.5g (%.3g%% back-off from %.5g)\n',gamlat3lpvu,(gamlat3lpvu/infolat3lpvu.MinGamma-1)*100,infolat3lpvu.MinGamma)
fprintf('lat3 Hinf Syn Time: %.1f sec \n',timelat3hinf)
fprintf('lat3 H2op Syn Time: %.1f sec \n',timelat3h2op)
fprintf('lat3 LPVu Syn Time: %.1f sec \n',timelat3lpvu)

%%
min(gamlon5hinf_grid,[],'all')
max(gamlon5hinf_grid,[],'all')
min(gamlon5h2op_grid,[],'all')
max(gamlon5h2op_grid,[],'all')
fprintf('lon5 LPVu Gam = %.5g (%.3g%% back-off from %.5g)\n',gamlon5lpvu,(gamlon5lpvu/infolon5lpvu.MinGamma-1)*100,infolon5lpvu.MinGamma)
fprintf('lon5 Hinf Syn Time: %.1f sec \n',timelon5hinf)
fprintf('lon5 H2op Syn Time: %.1f sec \n',timelon5h2op)
fprintf('lon5 LPVu Syn Time: %.1f sec \n',timelon5lpvu)

%%
min(gamlat4hinf_grid,[],'all')
max(gamlat4hinf_grid,[],'all')
min(gamlat4h2op_grid,[],'all')
max(gamlat4h2op_grid,[],'all')
fprintf('lat4 LPVu Gam = %.5g (%.3g%% back-off from %.5g)\n',gamlat4lpvu,(gamlat4lpvu/infolat4lpvu.MinGamma-1)*100,infolat4lpvu.MinGamma)
fprintf('lat4 Hinf Syn Time: %.1f sec \n',timelat4hinf)
fprintf('lat4 H2op Syn Time: %.1f sec \n',timelat4h2op)
fprintf('lat4 LPVu Syn Time: %.1f sec \n',timelat4lpvu)

