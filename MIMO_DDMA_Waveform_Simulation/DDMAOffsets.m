function [dopOff,Mv] = DDMAOffsets(Ntx,Nvtx)
%% Description:
% This function calculate the doppler offset frequencies for maintains 
% orthogonality in the frequency domain for Tx waveforms
% returns the doppler offset and number of used and unused doppler bins

%% Code:
% Ensure even number of bins to avoid a Doppler bin at fd = 0 or fd = 1
Mv = Ntx+Nvtx;
Mv = Mv+mod(Mv,2);
Nvtx = Mv-Ntx;

mdx = (1:Mv).';
dopOff = (mdx-0.5)/Mv;

% Shift modulated Doppler frequencies from -PRF/2 to PRF/2 and prevent
% separation of signals at ±PRF/2 into two parts.
dopOff = dopOff-1/2+Nvtx/Mv/2;
end