function paths = FreespacePaths(destpos,destvel,lambda,rcsdBsm,startpos,startvel,isTwoWay)
%% Description:
% This function returns the free space paths structure

if nargin<2 || isempty(destvel)
    destvel = zeros(size(destpos));
end

if nargin<3 || isempty(lambda)
    lambda = 1;
end

if nargin<4 || isempty(rcsdBsm)
    rcsdBsm = 0;
end

if nargin<5 || isempty(startpos)
    startpos = zeros(size(destpos));
end

if nargin<6 || isempty(startvel)
    startvel = zeros(size(destpos));
end

if nargin<7
    isTwoWay = true;
end

Ntargets = size(destpos,2);

if size(startpos,2)==1 && Ntargets>1
    startpos = repmat(startpos,[1 Ntargets]);
end
if size(startvel,2)==1 && Ntargets>1
    startvel = repmat(startvel,[1 Ntargets]);
end

if isscalar(rcsdBsm)
    rcsdBsm = rcsdBsm*ones(Ntargets,1);
end
rcs = db2pow(rcsdBsm);
rgain = aperture2gain(rcs,lambda);

paths = repmat(struct(...
    'PathLength', zeros(1, 1), ...
    'PathLoss', zeros(1, 1), ...
    'ReflectionCoefficient', zeros(1,1), ...
    'AngleOfDeparture', zeros(2, 1), ...
    'AngleOfArrival', zeros(2, 1), ...
    'DopplerShift', zeros(1, 1)),...
    1,Ntargets);

for m = 1:Ntargets
    % Each target is already in the path length, angle
    [plength,tang] = rangeangle(destpos(:,m),startpos(:,m));
    
    % path loss
    ploss = fspl(plength,lambda);
    
    % doppler
    dop = speed2dop(radialspeed(destpos(:,m),destvel(:,m),startpos(:,m),startvel(:,m)),...
        lambda);
    
    if isTwoWay
        plength = 2*plength;
        ploss = 2*ploss;
        dop = 2*dop;
    end
    
    paths(m).PathLength = plength;
    paths(m).PathLoss = ploss;
    paths(m).ReflectionCoefficient = db2mag(rgain(m));
    paths(m).AngleOfDeparture = tang;
    paths(m).AngleOfArrival = tang;
    paths(m).DopplerShift = dop;
end
end
