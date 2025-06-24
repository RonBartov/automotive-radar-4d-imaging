function plotDDMAOffsets(dopOffsets,Ntx)
%% Description:
% This function plots the Doppler Division Multiple Access (DDMA) offsets 
% for each Tx antenna (Ntx offsets) 

fig = findfig('DDMA Tx Doppler offsets'); ax = polaraxes(fig);
h = PolarQuiver(ax,dopOffsets(1:Ntx)*2*pi,1);
hold(ax,'on');
hvx = PolarQuiver(ax,dopOffsets(Ntx+1:end)*2*pi,1,'k','Color',0.7*[1 1 1]);
title(ax,fig.Name);
str = strsplit(strtrim(sprintf('Tx_{%i} ',1:Ntx)));
legend([h;hvx(1)],str{:},'Tx_{virtual}');
ax.ThetaLim = [-180 180];
end