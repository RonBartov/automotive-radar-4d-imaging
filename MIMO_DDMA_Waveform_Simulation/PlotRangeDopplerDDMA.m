function PlotRangeDopplerDDMA(snr,rgBins,dpBins,prf,Ntx,Nv)
%% Description:
% This function plots the Range-DDMA Offset graph that obtained from a 2D
% MIMO radar transceiver

%% Code:
fig = findfig('Range-Doppler Map'); ax2 = axes(fig);
ax = axes(fig);
ax.Position = ax2.Position;
ax.Position(4) = 0.85*ax2.Position(4);
ax.Position(2) = ax2.Position(2)+ax2.Position(4)-ax.Position(4);

snrdB = pow2db(snr);
[~,iMax] = max(snrdB(:));
[~,iRx,~] = ind2sub(size(snrdB),iMax);
imagesc(ax,dpBins/prf,rgBins,squeeze(snrdB(:,iRx,:)));

dpVrt = (1-(Nv-Ntx)/Nv)/2;
verts = [ ...
    dpVrt rgBins(1);dpVrt rgBins(end);0.5 rgBins(1);0.5 rgBins(end); ...
    -dpVrt rgBins(1);-dpVrt rgBins(end);-0.5 rgBins(1);-0.5 rgBins(end); ...
    ];
faces = [ ...
    [1 2 4 3 1]; ...
    [1 2 4 3 1]+4; ...
    ];
patch(ax,'Vertices',verts,'Faces',faces,'FaceColor','k','FaceAlpha',0.3,'EdgeColor','none');
text(ax,mean([dpVrt 0.5]),mean(rgBins([1 end])),'Unassigned Doppler','FontWeight','bold','Color','w','HorizontalAlignment','center','VerticalAlignment','middle','Rotation',90);
text(ax,-mean([dpVrt 0.5]),mean(rgBins([1 end])),'Unassigned Doppler','FontWeight','bold','Color','w','HorizontalAlignment','center','VerticalAlignment','middle','Rotation',90);

xlabel(ax,'Doppler (normalized)'); ylabel(ax,'Range (m)');
grid(ax,'on'); grid(ax,'minor'); ax.Layer = 'top'; ax.GridColor = 'w'; ax.MinorGridColor = 'w';
ax.YDir = 'normal';
ylabel(colorbar(ax),'SNR (dB)');
clim(ax,[0 30]);

ax2.Position([1 3]) = ax.Position([1 3]);
set(ax2,'XAxisLocation','bottom','XLim',[-180 180],'XDir','reverse','Color','none','YAxisLocation','left','YTick',[],'YColor','none');
xlabel(ax2,'DDMA Doppler Offset (deg)');
title(ax,sprintf('Range-Doppler Map for Rx: %i',iRx));

zm = zoom(fig);
zm.ActionPostCallback = @(fig,axSt)xlim(ax2,fliplr(-xlim(ax)*360));
ax2.UserData = zm;
end

function fig = findfig(name)
fig = findobj(groot,'Type','figure','Name',name);
if isempty(fig) || ~any(ishghandle(fig))
    fig = figure('Name',name);
    fig.Visible = 'on';
    fig.MenuBar = 'figure';
elseif any(ishghandle(fig))
    fig = fig(ishghandle(fig));
end
fig = fig(1); clf(fig); figure(fig);
end
