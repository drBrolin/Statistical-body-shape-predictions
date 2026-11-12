% forearmAxisPoints: two points on the forearm pronation/supination axis (radial head -> ulnar head).
%   'anatomical': midpoint of the Radiale landmark and the elbow joint centre -> midpoint of the UlnarStyloid landmark
%                 and the wrist joint centre (skin landmarks moved halfway towards the bone centres)
%   'joint':      elbow joint centre -> wrist joint centre
%   [p1, p2] = forearmAxisPoints(adjustedLM, jointCenter, 'Rt', 'anatomical')
function [p1, p2] = forearmAxisPoints(adjustedLM, jointCenter, S, axisType)
    lm = lmIndices(); jc = jcIndices();
    E = jointCenter(jc.([S 'Elbow']),:); W = jointCenter(jc.([S 'Wrist']),:);
    switch axisType
        case 'anatomical'
            p1 = (adjustedLM(lm.([S 'Radiale']),:) + E)/2;
            p2 = (adjustedLM(lm.([S 'UlnarStyloid']),:) + W)/2;
        case 'joint'
            p1 = E; p2 = W;
        otherwise
            error('forearmAxisPoints: unknown axis type %s', axisType);
    end
end
