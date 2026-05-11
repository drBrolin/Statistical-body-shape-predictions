function inside = pointInSweptSphere(P, A, B, r)
% pointInSweptSphere  Test whether points lie inside a swept sphere (capsule)
%
% Inputs:
%   P : Nx3 array of query points
%   A : 1x3 start point of sweep
%   B : 1x3 end point of sweep
%   r : sphere radius
%
% Output:
%   inside : Nx1 logical array (true if point lies inside swept sphere)

    AB = B - A;
    AB2 = dot(AB, AB);
    AP = P - A;
    t = (AP * AB.') / AB2;
    t = max(0, min(1, t));
    C = A + t .* AB;
    dist2 = sum((P - C).^2, 2);
    inside = dist2 <= r^2;
end
