function paths = PosesToPaths(tposes,txrx,scenario,rdr,bep)
%% Description:
% This function transport the target positions from the ego's body (vehicle object)
% frame to the sensor's mounted frame

%% Code:
rot = matlabshared.tracking.internal.fusion.rotZYX(txrx.MountingAngles);
off = txrx.MountingLocation(:);

frm = matlabshared.tracking.internal.fusion.objectFrame();
frm.OriginPosition(:) = off;
frm.Orientation(:,:) = rot;
frm.IsParentToChild(:) = false;

tposesMnt = matlabshared.tracking.internal.fusion.targetTruth.transport(tposes,frm);

% Sample target extent and compute RCS and viewed angles
rf = txrx.ReceiveAntenna.OperatingFrequency;
lambda = freq2wavelen(rf,txrx.ReceiveAntenna.PropagationSpeed);

% Profiles define the dimensions and RCS of the targets
profiles = actorProfiles(scenario);

% Compute the angle at which each target is viewed by the sensor
poses = matlabshared.tracking.internal.fusion.truthPose(tposesMnt);
viewangs = matlabshared.tracking.internal.fusion.targetTruth.viewedAngle(poses,profiles);
tids = matlabshared.tracking.internal.fusion.targetTruth.truthID(tposesMnt);
sigs = matlabshared.tracking.internal.fusion.targetTruth.signatures(tids,profiles);
numTgts = numel(tids);
rcsdBsm = NaN(numTgts,1);
for m = 1:numTgts
    thisSig = sigs{m}{1};
    rcsdBsm(m) = value(thisSig,viewangs(1,m),viewangs(2,m),rf);
end

map = containers.Map(tids,rcsdBsm);
rcsTbl = @(ids)arrayfun(@(id)map(id),ids); % Create a lookup table

% Map the targets to sets of points according to the sensor's resolution
res = NaN(1,3);
res(1) = rdr.AzimuthResolution;
if (isfield(rdr,'HasElevation') && rdr.HasElevation) || ...
        (~isfield(rdr,'HasElevation') && isfield(rdr,'ElevationResolution'))
    res(2) = rdr.ElevationResolution;
end
res(3) = rdr.RangeResolution;
if isfield(rdr,'RangeLimits')
    rgLim = [0 rdr.RangeLimits(2)];
elseif isfield(rdr,'MaxRange')
    rgLim = [0 rdr.MaxRange];
else
    rgLim = [0 inf];
end
fov = rdr.FieldOfView;

[posMnt,idxTgt,velMnt] = matlabshared.tracking.internal.fusion.targetTruth.sampleTargets(tposesMnt,profiles,res,fov,rgLim,true,10);
ids = matlabshared.tracking.internal.fusion.targetTruth.truthID(tposesMnt);
ids = ids(idxTgt);

rcsdBsm = rcsTbl(ids);

paths = FreespacePaths(posMnt,velMnt,lambda,rcsdBsm);
end
