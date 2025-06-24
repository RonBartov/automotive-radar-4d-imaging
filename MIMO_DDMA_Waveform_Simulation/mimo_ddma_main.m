close all
clear
clc

%% Description: 
% This MATLAB module simulates the generation of a Range-Doppler map
% while using rectangular 2D-MIMO 77GHz radar transceiver and maintaining 
% transmitters orthogonality by using Doppler Division Multiple Access (DDMA)

%% ----------------------Model a MIMO Radar Transceiver--------------------
% Creating a transceiver object
MIMOTransceiver = radarTransceiver();

% Defining FMCW waveform parameters
rgRes = 0.5; % range resolution (m)
rgMax = 150; % maximum range detection (m)
sweepBW = 750e6; % sweep BW of FMCW waveform (Hz)
Fs = 100e6; % sampling rate (Hz)
sweepTime = 20e-6; % duration of one chirp (s)
Ncp = 250; % Number of total chirps to transmit

% Creating FMCW waveform 
wfm = phased.FMCWWaveform( ...
    'SweepBandwidth',sweepBW, ...
    'SweepTime',sweepTime, ...
    'SampleRate',Fs);

% Define Tx and Rx parameters
fc = 77e9; % carrier frequency (Hz)
c = 299792458; % speed of light (m/s)
lambda = c./fc; % wavelength (m)
dr = lambda/2; % interval between adjacent receivers (m)
Ntx_in_edge = 2; % number of transmitters in rectangular edge
Nrx_in_edge = 7; % number of receivers in rectangular edge
Ntx = Ntx_in_edge * Ntx_in_edge; % total number of transmitters
Nrx = Nrx_in_edge  * Nrx_in_edge; % total number of receivers

% Generate positions for Tx and Rx arrays
[yPos,zPos] = meshgrid((0:(Ntx_in_edge-1))*(Nrx_in_edge-1),(0:(Ntx_in_edge-1))*(Nrx_in_edge-1));
txPos = [yPos(:) zPos(:)]';
txPos = sortrows(txPos',[2 1])';

[yPos,zPos] = meshgrid(0:(Nrx_in_edge-1),0:(Nrx_in_edge-1));
rxPos = [yPos(:) zPos(:)]';
rxPos = sortrows(rxPos',[2 1])';

% Create Tx object, Rx object and virtual array positions
[txArray,rxArray,vxPos] = VirtualArray(txPos,rxPos,lambda,Ntx_in_edge,Nrx_in_edge);
ViewArrays(txPos,rxPos,vxPos);

% Updating the MIMO transceiver object properties that obtained so far 
MIMOTransceiver.Waveform = wfm;
MIMOTransceiver.Receiver.SampleRate = Fs;
MIMOTransceiver.NumRepetitions = Ncp;

MIMOTransceiver.Receiver.NoiseFigure = 12;

MIMOTransceiver.TransmitAntenna.Sensor = txArray;
MIMOTransceiver.TransmitAntenna.OperatingFrequency = fc;
MIMOTransceiver.ReceiveAntenna.Sensor = rxArray;
MIMOTransceiver.ReceiveAntenna.OperatingFrequency = fc;

MIMOTransceiver.ElectronicScanMode = 'Custom'; % for using MIMO 
MIMOTransceiver.TransmitAntenna.WeightsInputPort = true;

MIMOTransceiver.Transmitter.PeakPower = db2pow(10);

%% --------Calculate Azimuth and Elevation BW Resolution-------------------
bwAz = CalcBWResolution(txArray,rxArray,fc, 'Azimuth');
bwEl = CalcBWResolution(txArray,rxArray,fc,'Elevation');

%% ------------------Define MIMO DDMA Waveform-----------------------------
% Number of doppler bins that are unused
Nvtx = 2;

[dopOffsets,Nv] = DDMAOffsets(Ntx,Nvtx);

% Recompute Ncp so that it is a multiple of Nv
Ncp = ceil(Ncp/Nv)*Nv;
MIMOTransceiver.NumRepetitions = Ncp;

% Select Ntx consecutive Doppler offsets for the DDMA weights
n = 1:Ncp;
wDDMA = exp(-1i*2*pi*dopOffsets(1:Ntx).*n); % size of Ntx x Ncp 

% Plot the DDMA Doppler offsets
plotDDMAOffsets(dopOffsets,Ntx);

%% --------------------Simulate Radar Data Cube----------------------------
% Create Driving Scenario
[scenario, egoVehicle] = CreateOverpassScenario("scenario1");

% Mount MIMO radar as forward-looking radar on the ego vehicle. Mounting
% location [x,y,z]
MIMOTransceiver.MountingLocation(1) = egoVehicle.Length-egoVehicle.RearOverhang;
MIMOTransceiver.MountingLocation(3) = 0.5;
MIMOTransceiver.MountingAngles(2) = -5; % Pitch radar upward, away from the road's surface

% Define the coverage configurations
fov = [85 25]; % [azimuth elevation] degrees

covCfg = struct( ...
    'SensorLocation',MIMOTransceiver.MountingLocation(1:2), ...
    'MaxRange',rgMax, ...
    'Yaw',MIMOTransceiver.MountingAngles(1), ...
    'FieldOfView',fov, ...
    'AzimuthResolution',bwAz, ...
    'ElevationResolution',bwEl, ...
    'RangeResolution',rgRes);

% Plot scenario
bep = SetupDisplay(egoVehicle,covCfg);
delete(bep.Plotters(end)); % if there is car 2 target, need to be change
axis(bep.Parent,[-2 110 70*[-1 1]]);

% Convert target poses to path structs
tposes = targetPoses(egoVehicle);
paths = PosesToPaths(tposes,MIMOTransceiver,scenario,covCfg);

%% ---------------Process MIMO Radar Data Cube-----------------------------
% Generate datacube
time = scenario.SimulationTime;
datacube = MIMOTransceiver(paths,time,conj(wDDMA));

% Generate range-doppler response object
wfm = MIMOTransceiver.Waveform;
rgdpProc = phased.RangeDopplerResponse( ...
    'RangeMethod','FFT', ...
    'SampleRate',wfm.SampleRate, ...
    'SweepSlope',wfm.SweepBandwidth/wfm.SweepTime, ...
    'DechirpInput',true, ...
    'RangeWindow','Hann', ...
    'DopplerWindow','Hann', ...
    'DopplerOutput','Frequency');

refSig = wfm(); % reference signal
[rgdpCube,rgBins,dpBins] = rgdpProc(datacube,refSig);

% Limit data to ranges of interest
inRg = rgBins>=0 & rgBins<=covCfg.MaxRange;
rgBins = rgBins(inRg);
rgdpCube = rgdpCube(inRg,:,:);
dpBins = -dpBins;

pwrCube = abs(rgdpCube).^2; % power of each cell in datacube

fpks = Ncp/Nv; % Distance between DDMA peaks

% Constant False Alarm Rate (CFAR) Detector Object
cfar = phased.CFARDetector( ...
    'NumGuardCells',even(ceil(fpks*0.5)), ... % Use 1/2 of space between DDMA Doppler peaks as guard cells
    'ProbabilityFalseAlarm',1e-6, ...
    'NoisePowerOutputPort',true);

cfar.NumTrainingCells = 2*(fpks-cfar.NumGuardCells); % Use remaining space for training

% Reshape data cube to perform CFAR averaging along the Doppler dimension
[~,Nrx,Ndp] = size(rgdpCube);
data = reshape(pwrCube,[],Ndp);
[detMtx,nPow] = cfar(data,1:size(data,1));

% Reshape the detection matrix to match the dimensions of the original
% range and Doppler processed data cube
detMtx = reshape(detMtx,size(pwrCube));

% Compute the signal-to-noise ratio (SNR) for the range-Doppler map
nPow = reshape(nPow,size(pwrCube));
snrCube = pwrCube./nPow;

% Identify the range and Doppler bins where the same target appears across
% the receive elements in the data cube
detMtx = squeeze(sum(detMtx,2)); 

% Plot the range and Doppler map. Expecting to get 'Ntx' doppler returns
prf = 1/MIMOTransceiver.Waveform.SweepTime;
PlotRangeDopplerDDMA(snrCube,rgBins,dpBins,prf,Ntx,Nv);

%% -----------------------Remove Doppler Ambiguities-----------------------
% Generate Doppler transmit test sequence
dPks = Ncp/Nv; % Number of Doppler bins between modulated DDMA target peaks
mfDDMA = zeros(dPks,Ntx);
mfDDMA(1,:) = 1;
mfDDMA = mfDDMA(:)';

% Plot matched filter
fig = findfig('DDMA Matched Filter'); ax = axes(fig);
stem(ax,mfDDMA);
xlabel(ax,'Doppler bin (k)'); ylabel(ax,'h_k');
title(ax,fig.Name);
grid(ax,'on'); grid(ax,'minor');

% Compute cyclical cross-correlation.
[Nrg,Ndp] = size(detMtx);

% Require the same target to appear across the receive channels.
corrMtx = detMtx>=Nrx*0.8;
corrMtx = ifft( fft(corrMtx,Ndp,2) .* fft(mfDDMA,Ndp,2), Ndp,2);
rdDetMtx = corrMtx>=Ntx*0.95;

% Isolate the local maxima for each group of detections in the range-Doppler image
snrDetMtx = squeeze(sum(snrCube,2));
snrDetMtx(~rdDetMtx) = NaN;
rdDetMtx = islocalmax(snrDetMtx,1);

% Identify the range and Doppler bins associated with localized detection maxima
iRD = find(rdDetMtx(:));
[iRgDet,iDpDet] = ind2sub(size(rdDetMtx),iRD);
rgDet = rgBins(iRgDet);

% Show the locations of the unambiguous detections corresponding to the
% signal returns from the first transmit element
ax = findaxes('Range-Doppler Map'); hold(ax,'on');
p = plot(ax,dpBins(iDpDet)/prf,rgDet,'rd','DisplayName','Unambiguous detections');
legend(p,'Unambiguous detections','Location','southwest');

% Apply Doppler correction for DDMA Doppler shift applied on the first
% transmit element
fd = (0:Ncp-1)/Ncp-0.5;
f1 = dopOffsets(1);
dpDet = mod(fd(iDpDet)+f1+0.5,1)-0.5;

% Compute unambiguous range-rate of detected Doppler bin
prf = 1/MIMOTransceiver.Waveform.SweepTime;
dpDet = dpDet*prf;
rrDet = dop2speed(dpDet,lambda)/2;

% Show the range and range-rate extracted from the detected target returns.
fig = findfig('Range-Doppler Detections'); ax = axes(fig);
plot(ax,rrDet,rgDet,'s');
xlabel(ax,'Range-rate (m/s)'); ylabel('Range (m)');
title(ax,'Range-Doppler Detections');
ax.YDir = 'normal';
grid(ax,'on'); grid(ax,'minor'); ax.Layer = 'top';
axis(ax,[dop2speed(prf/2,lambda)/2*[-1 1] 0 covCfg.MaxRange]);

%% -----------Virtual MIMO Array Beamspace Processing----------------------
% Assemble virtual array elements from range-Doppler detections.
dPks = Ncp/Nv; % Number of Doppler bins between modulated DDMA target peaks
iDpTx = mod(iDpDet(:)-(0:Ntx-1)*dPks-1,Ncp)'+1; % Transmit elements are isolated in Doppler
iRgTx = repmat(iRgDet,1,Ntx)';

% Plot the locations of the target returns to be extracted for the four
% transmit elements in the range and Doppler map
plotTxArrayReturns(rgBins,dpBins,iRgTx,iDpTx,Ntx,prf);

%% -------------------Supporting Functions---------------------------------
function y = even(x)
y = round(x/2)*2;
end

function ax = findaxes(name)
fig = findobj(groot,'Type','figure','Name',name);
if isempty(fig) || ~any(ishghandle(fig))
    fig = figure('Name',name);
    fig.Visible = 'on';
elseif any(ishghandle(fig))
    fig = fig(ishghandle(fig));
end
fig = fig(1);
ax = fig.CurrentAxes;
if isempty(ax)
    ax = axes(fig);
end
end

function plotTxArrayReturns(rgBins,dpBins,iRgTx,iDpTx,Ntx,prf)
ax = findaxes('Range-Doppler Map'); hold(ax,'on');
p = findobj(ax,'Type','line');
p = [p;plot(ax,dpBins(iDpTx(:))/prf,rgBins(iRgTx(:)),'k.','DisplayName','Tx elements')];
legend(p(1:2),'Unambiguous detections','Tx elements','Location','southwest','AutoUpdate','off');

for m = 1:Ntx
    text(ax,dpBins(iDpTx(m))/prf,rgBins(iRgTx(m)),sprintf('%i',m), ...
        'Color','w','FontWeight','bold','FontSize',10, ...
        'HorizontalAlignment','center','VerticalAlignment','bottom');
end
end