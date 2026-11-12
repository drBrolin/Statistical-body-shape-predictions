% forearmTwistAngle: pronation/supination of the forearm relative to the humerus, in the clinical convention
% (0 = neutral, i.e. thumb forward with the arm hanging / palm towards the thigh; positive = supination, towards palm
% forward; negative = pronation). Soucie et al. 2011: ~80 deg pronation and ~85-90 deg supination from neutral.
%
%   axis f      pronation axis, from the radial head to the ulnar head. 'anatomical' (default): from the midpoint of
%               the Radiale landmark and the elbow joint centre to the midpoint of the UlnarStyloid landmark and the
%               wrist joint centre (the skin landmarks moved halfway towards the bone centres). 'joint': elbow JC -> wrist JC.
%   reference   lateral direction of the humerus = Lateral - Medial epicondyle, projected on the plane perpendicular to f
%   anterior    perpendicular to f and the reference, on the side away from the olecranon
%   forearm     Ulnar styloid -> Radial styloid, projected on the same plane; its angle from the lateral reference
%               towards anterior is psi (0 = radial styloid lateral = full supination, 90 = anterior = neutral)
%   supination  = 90 - psi
%   hand        the same angle from the hand: MetacarpalPhalV -> MetacarpalPhalII landmarks (info.supHand), and
%               pinky MCP -> index MCP joint centres (info.supHandJC)
%
%   [sup, info] = forearmTwistAngle(adjustedLM, jointCenter)
%   [sup, info] = forearmTwistAngle(adjustedLM, jointCenter, 'Rt', 'anatomical')
function [sup, info] = forearmTwistAngle(adjustedLM, jointCenter, side, axisType)
    if nargin < 3 || isempty(side), side = 'Rt'; end
    if nargin < 4 || isempty(axisType), axisType = 'anatomical'; end
    lm = lmIndices(); jc = jcIndices(); S = side;
    [p1, p2] = forearmAxisPoints(adjustedLM, jointCenter, S, axisType);
    f = (p2 - p1)/norm(p2 - p1);

    lat = adjustedLM(lm.([S 'HumeralLateralEpicn']),:); med = adjustedLM(lm.([S 'HumeralMedialEpicn']),:);
    ref = planar(lat - med, f);
    ant = cross(f, ref);
    antRef = (lat + med)/2 - adjustedLM(lm.([S 'Olecranon']),:);   % olecranon is posterior
    if dot(ant, antRef) < 0, ant = -ant; end

    styl = adjustedLM(lm.([S 'RadialStyloid']),:) - adjustedLM(lm.([S 'UlnarStyloid']),:);
    hand = adjustedLM(lm.([S 'MetacarpalPhalII']),:) - adjustedLM(lm.([S 'MetacarpalPhalV']),:);
    handJC = jointCenter(jc.([S 'IndexCarpal']),:) - jointCenter(jc.([S 'PinkyCarpal']),:);
    sup = 90 - psiOf(styl, f, ref, ant);
    info = struct('supHand', 90 - psiOf(hand, f, ref, ant), 'supHandJC', 90 - psiOf(handJC, f, ref, ant), ...
                  'axis', f, 'axisPoints', [p1; p2], 'lateral', ref, 'anterior', ant, 'axisType', axisType);
end

function psi = psiOf(v, f, ref, ant)
    v = planar(v, f);
    psi = atan2d(dot(v, ant), dot(v, ref));
end

function v = planar(v, f)
    v = v - dot(v, f)*f; v = v/norm(v);
end
