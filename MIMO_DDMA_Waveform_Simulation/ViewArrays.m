function ViewArrays(txPos,rxPos,vxPos)
%% Description: 
% This function plots a configuration of rectangular 2D MIMO transceiver
% together with the virtual array 


fig = figure('Tag',mfilename);
t = tiledlayout(fig,2,1,'TileSpacing','loose');

% Plot Tx an Rx arrays 
ax = nexttile(t);
clrs = colororder(ax);

txClrs = clrs;
hRx = scatter(ax,rxPos(1,:),rxPos(2,:),[], ...
    'filled','MarkerEdgeColor','k');

hold(ax,'on');
hTx = plot(ax,txPos(1,:),txPos(2,:),'^','MarkerFaceColor',txClrs(3,:));
hold(ax,'off');

xlabel(ax,'Y position (\lambda/2)'); ylabel(ax,'Z position (\lambda/2)');
title(ax,sprintf('%i element transmit array\n%i element receive array',size(txPos,2),size(rxPos,2)));
grid(ax,'on'); grid(ax,'minor'); set(ax,'YDir','reverse');
legend([hRx hTx],'Rx element','Tx element','Location','SouthEast');
xlim(ax,'padded'); ylim(ax,'padded')

% Plot virtual array 
axVx = nexttile(t);

hVx = scatter(axVx,vxPos(1,:),vxPos(2,:),[], ...
    'filled','Marker','s','MarkerEdgeColor','k'); 

xlabel(axVx,'Y position (\lambda/2)'); ylabel(axVx,'Z position (\lambda/2)');
title(axVx,sprintf('%i element virtual array',size(vxPos,2)));
grid(axVx,'on'); grid(axVx,'minor'); set(axVx,'YDir','reverse');
legend(hVx,'Virtual element','Location','SouthEast');
xlim(axVx,'padded'); ylim(axVx,'padded');
axis(ax,axis(axVx));

end
