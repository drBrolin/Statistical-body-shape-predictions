% twistWeights: per-vertex weights for a twist distributed along a segment (e.g. forearm pronation/supination).
% t = position of each vertex along p0 -> p1 (0 at p0, 1 at p1); w = smoothstep from tStart to tEnd: 0 before tStart
% (proximal), 1 after tEnd (distal, e.g. the hand), smooth in between.
%   w = twistWeights(P, p0, p1, tStart, tEnd)
function w = twistWeights(P, p0, p1, tStart, tEnd)
    u = p1 - p0;
    t = (P - p0)*u'/(u*u');
    t = min(max((t - tStart)/(tEnd - tStart), 0), 1);
    w = t.^2.*(3 - 2*t);
end
