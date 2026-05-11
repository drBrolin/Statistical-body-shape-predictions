function [inside, weight] = pointInEllipsoid(P, C, r)
% pointInEllipsoid  Test whether points lie inside an ellipsoid
%                   and compute proximity weights
%
% Inputs:
%   P : Nx3 array of query points
%   C : 1x3 center point of ellipsoid
%   r : 1x3 semi-axes radii [rx, ry, rz]
%
% Outputs:
%   inside : Nx1 logical array (true if point lies inside ellipsoid)
%   weight : Nx1 array (1 at center, 0 at surface, NaN outside)

    if size(P,2) ~= 3
        error('P must be an Nx3 matrix of 3-D points.');
    end
    if numel(C) ~= 3
        error('C must be a 1x3 vector representing the ellipsoid center.');
    end
    if numel(r) ~= 3
        error('r must be a 1x3 vector of semi-axes radii [rx, ry, rz].');
    end

    CP = P - C;
    d2 = (CP(:,1)/r(1)).^2 + (CP(:,2)/r(2)).^2 + (CP(:,3)/r(3)).^2;
    inside = d2 <= 1;
    weight = zeros(size(d2));
    weight(inside) = 1 - d2(inside);
end
