function varargout = SetupDisplay(ego,sensors)
%% Description:
% This function returns a plot of a driving scenario according to ego
% vehicle object and sensor configurations

%% Code
[plotters,bep] = setupBEP(ego,sensors);
varargout = {bep,plotters};

end

function [plotters,bep] = setupBEP(ego,sensors,fig)

if nargin<3
    fig = figure('Name','BEP');
end

velScale = 0.1;
detMrkrSize = 6;

fig.Visible = 'off';
clf(fig);
ax = axes(fig);
grid(ax,'on');
grid(ax,'minor');
bep = birdsEyePlot('XLim',[-3 150],'YLim',20*[-1 1],'Parent',ax);

plotters = struct();

lmPlotter = laneMarkingPlotter(bep,'DisplayName','Road');
lbPlotter = laneBoundaryPlotter(bep,'Tag','Lane boundaries');

pPlotter = detectionPlotter(bep,'DisplayName','Truth origin', ...
    'VelocityScaling',velScale,'Marker','o','MarkerFaceColor','k','MarkerSize',2);
olPlotter = outlinePlotter(bep,'Tag','Truth outlines');

plotters.PosePlotter = pPlotter;
plotters.OutlinePlotter = olPlotter;
plotters.LaneBoundaryPlotter = lbPlotter;
plotters.LaneMarkingPlotter = lmPlotter;

if nargin
    % Get the road boundaries and rectangular outlines
    rb = roadBoundaries(ego);
    % Get lane marking vertices and faces
    [lmv, lmf] = laneMarkingVertices(ego);
    % update the bird's-eye plotters with the road and actors
    if ~isempty(rb)
        plotLaneBoundary(plotters.LaneBoundaryPlotter, rb);
    end
    if ~isempty(lmv)
        plotLaneMarking(plotters.LaneMarkingPlotter, lmv, lmf);
    end

    tposes = targetPoses(ego);
    pos = cell2mat(arrayfun(@(p)p.Position(:)',tposes(:),'UniformOutput',false));
    vel = cell2mat(arrayfun(@(p)p.Velocity(:)',tposes(:),'UniformOutput',false));

    % Add the ego
    pos = [0 0 0;pos];
    vel = [0 0 0;vel];

    plotDetection(plotters.PosePlotter,pos(:,1:2),vel(:,1:2));

    [position, yaw, length, width, originOffset, color] = targetOutlines(ego);

    [bposition, byaw, blength, bwidth, boriginOffset, bcolor] = targetOutlines(ego,'Barriers');
    position = [position;bposition];
    yaw = [yaw;byaw];
    length = [length;blength];
    width = [width;bwidth];
    originOffset = [originOffset;boriginOffset];
    color = [color;bcolor];

    plotOutline(plotters.OutlinePlotter, position, yaw, length, width, ...
        'OriginOffset', originOffset, 'Color', color);
end

% Sensor plotters
if nargin>1
    if ~iscell(sensors)
        sensors = {sensors};
    end

    % Plot sensor coverage areas
    plotters.DetectionPlotters = repmat(struct('CoverageArea',[],'Plotter',[],'SensorIndex',[]),numel(sensors),1);
    for m = 1:numel(sensors)
        if isa(sensors{m},'visionDetectionGenerator')
            % Create coverage plotter
            if isprop(sensors{m},'SensorIndex') || isfield(sensors{m},'SensorIndex') && ~isempty(sensors{m}.SensorIndex)
                name = ['Vision ID' num2str(sensors{m}.SensorIndex) ' FoV'];
            else
                name = 'Vision FoV';
            end
            coveragePlotter = coverageAreaPlotter(bep,'DisplayName',name,'FaceColor','b');
            pos = sensors{m}.SensorLocation;
            rgMax = sensors{m}.MaxRange;
            yaw = sensors{m}.Yaw;
            azFov = sensors{m}.FieldOfView(1);
        else
            if isprop(sensors{m},'SensorIndex') || isfield(sensors{m},'SensorIndex') && ~isempty(sensors{m}.SensorIndex)
                name = ['Radar ID' num2str(sensors{m}.SensorIndex) ' FoV'];
            else
                name = 'Radar FoV';
            end
            coveragePlotter = coverageAreaPlotter(bep,'DisplayName',name,'FaceColor','r');
            if isa(sensors{m},'matlabshared.tracking.internal.fusion.BaseRadarDataGenerator')
                pos = sensors{m}.MountingLocation(1:2);
                rgMax = sensors{m}.RangeLimits(2);
                yaw = sensors{m}.MountingAngles(1);
                azFov = sensors{m}.FieldOfView(1);
            elseif isa(sensors{m},'fusion.internal.remotesensors.ScanningSensor')
                pos = sensors{m}.MountingLocation(1:2);
                rgMax = sensors{m}.ReferenceRange;
                yaw = sensors{m}.MountingAngles(1);
                azFov = sensors{m}.FieldOfView(1);
            else
                pos = sensors{m}.SensorLocation;
                rgMax = sensors{m}.MaxRange;
                yaw = sensors{m}.Yaw;
                azFov = sensors{m}.FieldOfView(1);
            end
        end

        % Plot coverage area
        plotCoverageArea(coveragePlotter, pos, rgMax, yaw, azFov);
        
        plotters.DetectionPlotters(m).CoverageArea = coveragePlotter;
    end
    
    % Plot detections/tracks
    for m = 1:numel(sensors)
        thisSensor = sensors{m};
        if isa(thisSensor,'visionDetectionGenerator')
            % Create detection plotter
            if isprop(thisSensor,'SensorIndex') || isfield(thisSensor,'SensorIndex') && ~isempty(thisSensor.SensorIndex)
                name = ['Vision ID' num2str(thisSensor.SensorIndex) ' detections'];
            else
                name = 'Vision detections';
            end
            pltr = trackPlotter(bep,'DisplayName',name, ...
                'HistoryDepth',0, ...
                'MarkerFaceColor','b','Marker','o','MarkerSize',detMrkrSize);
            if isprop(thisSensor,'SensorIndex') || isfield(thisSensor,'SensorIndex') && ~isempty(thisSensor.SensorIndex)
                plotters.DetectionPlotters(m).SensorIndex = thisSensor.SensorIndex;
            end
        else
            % Create detection plotter
            isTracks = isa(sensors{1},'matlabshared.tracking.internal.fusion.BaseRadarDataGenerator') ...
                && startsWith(lower(sensors{1}.TargetReportFormat),'T');
            if isTracks
                if isprop(thisSensor,'SensorIndex') || isfield(thisSensor,'SensorIndex') && ~isempty(thisSensor.SensorIndex)
                    name = ['Radar ID' num2str(thisSensor.SensorIndex) ' tracks'];
                else
                    name = 'Radar tracks';
                end
                pltr = trackPlotter(bep,'DisplayName',name, ...
                    'HistoryDepth',7, ...
                    'MarkerFaceColor','r');

                if isprop(thisSensor,'SensorIndex') || isfield(thisSensor,'SensorIndex') && ~isempty(thisSensor.SensorIndex)
                    plotters.DetectionPlotters(m).SensorIndex = thisSensor.SensorIndex;
                end
            else
                if isprop(thisSensor,'SensorIndex') || isfield(thisSensor,'SensorIndex')
                    name = ['Radar ID' num2str(thisSensor.SensorIndex) ' detections'];
                else
                    name = 'Radar detections';
                end
                pltr = trackPlotter(bep,'DisplayName',name, ...
                    'HistoryDepth',0, ...
                    'MarkerFaceColor','r','Marker','o','MarkerSize',detMrkrSize);
                if isprop(thisSensor,'SensorIndex') || isfield(thisSensor,'SensorIndex') && ~isempty(thisSensor.SensorIndex)
                    plotters.DetectionPlotters(m).SensorIndex = thisSensor.SensorIndex;
                end
            end
        end
        plotters.DetectionPlotters(m).Plotter = @(dets)plotDets(pltr,thisSensor,dets);
    end
end

legend(ax,'Location','northeast','AutoUpdate','off');
fig.Visible = 'on';
end

function plotDets(pltr,sensor,dets)

% Check if this is a 2D plotter
is2D = isa(pltr,'driving.birdsEyePlot.DetectionPlotter') || isa(pltr,'driving.birdsEyePlot.TrackPlotter');

% Plot on all of the copied axes too
hndls = findobj(groot,'Type','Line','DisplayName',pltr.DisplayName);
if isempty(dets)
    for m = 1:numel(hndls)
        this = hndls(m);
        if is2D
            % BEP is 2D
            set(this,'XData',zeros(1,0),'YData',zeros(1,0));
        else
            set(this,'XData',zeros(1,0),'YData',zeros(1,0),'ZData',zeros(1,0));
        end
    end

else
    [rot,off] = helperSensorFrame(sensor);
    if isa(sensor,'matlabshared.tracking.internal.fusion.BaseRadarDataGenerator')
        fmt = lower(char(sensor.TargetReportFormat));
        if fmt(1)=='t' % Tracks
            pos = cell2mat(arrayfun(@(t)t.State(1:2:end),dets(:)','UniformOutput',false));
            pos = pos(1:2:end,:);
        else % Detections
            dets = matlabshared.tracking.internal.fusion.objectDetection.rectangular(dets);
            pos = cell2mat(cellfun(@(d)d.Measurement,dets(:)','UniformOutput',false));
            crds = lower(char(sensor.DetectionCoordinates));
            if crds(1)=='s' % Sensor's frame
                pos = rot*pos(1:3,:) + off;
            end
        end
    else
        dets = matlabshared.tracking.internal.fusion.objectDetection.rectangular(dets);
        crds = lower(char(sensor.DetectionCoordinates));
        if crds(1)=='e' % Ego
            pos = cell2mat(cellfun(@(d)d.Measurement,dets(:)','UniformOutput',false));
        else
            pos = cell2mat(cellfun(@(d)d.Measurement,dets(:)','UniformOutput',false));
            pos = rot*pos(1:3) + off;
        end
    end
    
    for m = 1:numel(hndls)
        this = hndls(m);
        if is2D
            % BEP is 2D
            set(this,'XData',pos(1,:),'YData',pos(2,:));
        else
            set(this,'XData',pos(1,:),'YData',pos(2,:),'ZData',pos(3,:));
        end
    end
end
end
