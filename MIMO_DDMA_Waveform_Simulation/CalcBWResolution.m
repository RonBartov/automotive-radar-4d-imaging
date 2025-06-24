function bw = CalcBWResolution(txArray,rxArray,fc,type)
%% Description:
% This function calculates the azimuth and elevation resolution of the 2D
% MIMO transceiver accoeding to txArray and rxArray properties.
% An additional plots of azimuth and elevation directivities will be
% created

%% Code:
angs = -90:0.1:90;
if strcmpi(type,'Elevation')
    patRx = patternElevation(rxArray,fc,'Elevation',angs);
    patTx = patternElevation(txArray,fc,'Elevation',angs);

elseif strcmpi(type,'Azimuth')
    patRx = patternAzimuth(rxArray,fc,'Azimuth',angs);
    patTx = patternAzimuth(txArray,fc,'Azimuth',angs);
end

patVx = patTx+patRx;

fig = findobj(groot,'Type','figure','Name',[type ' Directivity']);
if isempty(fig) || ~any(ishghandle(fig))
    fig = figure('Name',[type ' Directivity'],'Visible','on');
end
fig = fig(1);
clf(fig);

ax = axes(fig);
p1 = plot(ax,angs,patRx);
hold(ax,'on');
p2 = plot(ax,angs,patTx);
p3 = plot(ax,angs,patVx);
xlabel(ax,[type ' (deg)']); ylabel(ax,'Directivity (dBi)');
Gmax = max([patRx(:);patTx(:);patVx(:)]);
grid(ax,'on'); grid(ax,'minor');
ylim(ax,[ceil(Gmax/5)*5-70 ceil(Gmax/5)*5+15]);
title(ax,[type ' Beam Directivity (dBi)']);

% Compute beamwidth resolution
thresh = -3; % dB
[Gmax,i2] = max(patVx(:));
i1 = find(angs(:)<0 & patVx(:)-Gmax>thresh,1);
i3 = find(angs(:)>0 & patVx(:)-Gmax<thresh,1);
pp = polyfit(angs([i1 i2 i3]),patVx([i1 i2 i3])-(Gmax+thresh),2);
fitAng = roots(pp);
bw = abs(diff(fitAng));

xmax = min(10*bw,angs(end)/2);
xlim(ax,xmax*[-1 1]);

fitVx = polyval(pp,fitAng)+(Gmax+thresh);

xvals = [fitAng;NaN;fitAng(1)*[1;1];NaN;fitAng(2)*[1;1]];
yvals = [fitVx;NaN;fitVx(1)+[-3;3];NaN;fitVx(2)+[-3;3]];
p4 = plot(ax,xvals,yvals,'k');

legend([p1 p2 p3 p4],'Receive array','Transmit array','Virtual array',sprintf('Beamwidth: %1.1f deg',bw));
end
