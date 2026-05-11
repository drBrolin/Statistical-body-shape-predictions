function inside = pointInSphere(P, C, r)
% pointInSphere  Test whether points lie inside a sphere
%
% Inputs:
%   P : Nx3 array of query points
%   C : 1x3 center point of sphere
%   r : sphere radius
%
% Output:
%   inside : Nx1 logical array (true if point lies inside sphere)

    if size(P,2) ~= 3
        error('P must be an Nx3 matrix of 3-D points.');
    end

    if numel(C) ~= 3
        error('C must be a 1x3 vector representing the sphere center.');
    end

    CP = P - C;
    dist2 = sum(CP.^2, 2);
    inside = dist2 <= r^2;
end
