function fig = findfig(name)
%% Description:
% This function return the figure that correspond to the name 'name' 
% If the figure doesnt exist, it creates a new one 

%% Code:
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