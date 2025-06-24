function [txArray,rxArray,vxPos] = VirtualArray(txPos,rxPos,lambda,Ntx_in_edge,Nrx_in_edge)
%% Description:
% This function generates the following:
% 1) Tx and Rx array objects with antennas of type 'IsotropicAntennaElement', 
% located according to txPos and rxPos.
% 2) virtual array positions (vxPos)

% Define Tx and Rx elements object type
txElmt = phased.IsotropicAntennaElement;
rxElmt = clone(txElmt);

% Generate the Rx and Tx arrays
txArray = phased.ConformalArray('Element',txElmt,'ElementPosition', ...
    [zeros(1,size(txPos,2));txPos]*lambda/2);

rxArray = phased.ConformalArray('Element',rxElmt,'ElementPosition', ...
    [zeros(1,size(rxPos,2));rxPos]*lambda/2);

% Compute virtual array element positions
vxMask = ones(Ntx_in_edge*Nrx_in_edge,Ntx_in_edge*Nrx_in_edge);
ind = find(vxMask~=0);
[row,col] = ind2sub(size(vxMask),ind);

% minus one to start from zero counting
vxPos = [row(:) col(:)] - 1; 

% sort vx pos as two rows mesh 
vxPos = sortrows(vxPos,[2 1])';
end
