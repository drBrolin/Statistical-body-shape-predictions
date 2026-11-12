% hingeWeights: per-vertex weights for bending a hinge joint (e.g. the elbow). s = signed distance of each vertex from
% the plane that bisects the joint angle (normal unit(uProx + uDist) through the pivot; s > 0 on the distal side);
% w = smoothstep over s in [-R, R]: 0 on the proximal side, 1 on the distal side, smooth within R of the joint.
%   uProx  unit direction of the proximal segment, pointing towards the joint (e.g. GH -> elbow)
%   uDist  unit direction of the distal segment, pointing away from the joint (e.g. elbow -> wrist)
%   [w, n] = hingeWeights(P, pivot, uProx, uDist, R)
function [w, n] = hingeWeights(P, pivot, uProx, uDist, R)
    n = (uProx + uDist)/norm(uProx + uDist);
    s = (P - pivot)*n';
    w = smoothstep01((s + R)/(2*R));
end

function w = smoothstep01(t)
    t = min(max(t, 0), 1);
    w = t.^2.*(3 - 2*t);
end
