load Linearization.mat

figure
subplot(121), bode(P_lon), grid on
subplot(122), bode(P_lat), grid on

exportgraphics(gcf,'BodeLonOL.pdf')
