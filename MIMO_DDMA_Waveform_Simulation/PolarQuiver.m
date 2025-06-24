function h = PolarQuiver(ax,th,r,varargin)
%% Description:
% This function generate a polar plot for the complex values of vector th

%% Code:
if isscalar(r)
    r = repmat(r,size(th));
elseif isscalar(th)
    th = repmat(th,size(r));
end

% arrow head size
arrowheadSize = 1/15; % relative to radius
arrowheadAngle = 30; % degrees
dy = r(:).*(arrowheadSize*sind(arrowheadAngle));

x = r(:).*[0 (1-arrowheadSize) 1 (1-arrowheadSize) (1-arrowheadSize)];
y = [zeros(numel(r),3) dy -dy];
x = x.';
y = y.';

rot = zeros(5*numel(r),2,2,'like',th);
th = repmat(th(:).',5,1);
cth = cos(th(:)); sth = sin(th(:));
rot(:,1,1) = cth;
rot(:,1,2) = sth;
rot(:,2,1) = -sth;
rot(:,2,2) = cth;
verts = squeeze(sum(rot.*[x(:) y(:)],2));
faces = (5*((1:numel(r))-1)) + [1 2 4 3 5 2 1].';

pts = reshape(verts(faces(:),:),7,[],2);
[th,r] = cart2pol(pts(:,:,1),pts(:,:,2));

p = polarplot(ax,th(:,:),r(:,:),varargin{:},'Tag',mfilename);
if nargout
    h = p;
end
end
