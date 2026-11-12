% twistForearm: sets the forearm pronation/supination to a target angle, before the T-pose rotation.
% Called from mainInput.m after straightElbow and before rotateArmTpose (right arm only; rotateArmTpose mirrors it).
%
% Angle (forearmTwistAngle.m, clinical convention): 0 = neutral (thumb forward with the arm hanging, palm towards the
% thigh, which becomes palm down in the T-pose), positive = supination (towards palm forward), negative = pronation.
% Measured from the styloids (UlnarStyloid -> RadialStyloid) relative to the epicondyle axis, about the forearm axis.
%
% Rotation: about the pronation axis (forearmAxisPoints: radial head -> ulnar head, 'anatomical'), as the radius turns
% about the ulna. Because that axis is oblique to the forearm, the wrist joint centre moves a little too (in pronation
% the hand comes in line with the upper arm, which reduces the visible carrying angle).
%
% Mesh weight (utils/twistWeights, utils/rotateSegment): the twist is spread along the forearm, 0 up to twistStart and
% full from twistEnd (fractions of elbow JC -> wrist JC), smoothstep in between; the hand turns fully. Joint centres
% (wrist, all finger joints) and the styloid/metacarpal/dactylion landmarks turn rigidly with the full angle; the
% olecranon, radiale and epicondyles stay.
%
%   [meshModel, adjustedLM, jointCenter, info] = twistForearm(meshModel, adjustedLM, jointCenter, opts)
%   opts.targetDeg   target supination (deg), required, e.g. 0 = neutral
%   opts.side        'Rt' (default) or 'Lt'
%   opts.axisType    'anatomical' (default) or 'joint' (elbow JC -> wrist JC)
%   opts.reference   what the target refers to:
%                    'styloids' (default): anatomical supination (UlnarStyloid -> RadialStyloid relative to the
%                                epicondyle axis, clinical convention, see above)
%                    'palm':     palm direction in the T-pose (palm normal from the wrist / index MCP / pinky MCP joint
%                                centres, after rotateArmTpose, which is run on a copy): 0 = palm straight down,
%                                + = palm turned forward. Gives the same palm in every body; the anatomical angle then
%                                varies with the humerus rotation of the prediction. Only valid when rotateArmTpose
%                                follows directly (as in mainInput.m).
%   opts.twistStart  default 0.1, opts.twistEnd default 0.9
%   info             supination before/after (styloids and hand), applied angle, wrist JC shift, weights
function [meshModel, adjustedLM, jointCenter, info] = twistForearm(meshModel, adjustedLM, jointCenter, opts)
    if nargin < 4, opts = struct(); end
    if ~isfield(opts,'targetDeg') || isempty(opts.targetDeg), error('twistForearm: opts.targetDeg is required'); end
    if ~isfield(opts,'side'),       opts.side = 'Rt'; end
    if ~isfield(opts,'axisType'),   opts.axisType = 'anatomical'; end
    if ~isfield(opts,'twistStart'), opts.twistStart = 0.1; end
    if ~isfield(opts,'twistEnd'),   opts.twistEnd = 0.9; end
    if ~isfield(opts,'reference'),  opts.reference = 'styloids'; end
    r = meshRegions(); lm = lmIndices(); jc = jcIndices();
    S = opts.side;
    if strcmp(S,'Rt'), armVerts = r.rightArmHand; else, armVerts = r.leftArmHand; end
    jcMove = [jc.([S 'Wrist']), jc.([S 'IndexCarpal']):jc.([S 'ThumbDist'])];
    lmMove = [lm.([S 'RadialStyloid']), lm.([S 'UlnarStyloid']), lm.([S 'MetacarpalPhalII']), ...
              lm.([S 'MetacarpalPhalV']), lm.([S 'Dactylion'])];

    [supStyloidBefore, a0] = forearmTwistAngle(adjustedLM, jointCenter, S, opts.axisType);
    f = a0.axis; p1 = a0.axisPoints(1,:);
    [palmBefore, sgnPalm] = tposePalm(meshModel, adjustedLM, jointCenter, S);
    switch opts.reference
        case 'styloids'
            sgn = sign(dot(a0.anterior, cross(f, a0.lateral)));  % +angle about f turns lateral towards anterior if sgn > 0
            delta = sgn*(supStyloidBefore - opts.targetDeg);     % supination = 90 - psi: psi must change by sup - target
            before = supStyloidBefore;
        case 'palm'
            delta = sgnPalm*(opts.targetDeg - palmBefore);       % the arm rotation is rigid: the twist carries over
            before = palmBefore;
        otherwise
            error('twistForearm: unknown reference %s', opts.reference);
    end

    E = jointCenter(jc.([S 'Elbow']),:); W0 = jointCenter(jc.([S 'Wrist']),:);
    w = twistWeights(meshModel(armVerts,:), E, W0, opts.twistStart, opts.twistEnd);
    seg = struct('verts', armVerts, 'weights', w, 'pivot', p1, 'axis', f, 'angleDeg', delta, ...
                 'jcMove', jcMove, 'lmMove', lmMove);
    [meshModel, jointCenter, adjustedLM] = rotateSegment(meshModel, jointCenter, adjustedLM, seg);

    [supStyloidAfter, a1] = forearmTwistAngle(adjustedLM, jointCenter, S, opts.axisType);
    palmAfter = tposePalm(meshModel, adjustedLM, jointCenter, S);
    if strcmp(opts.reference, 'palm'), after = palmAfter; else, after = supStyloidAfter; end
    info = struct('side', S, 'targetDeg', opts.targetDeg, 'reference', opts.reference, ...
                  'before', before, 'after', after, ...
                  'supStyloidBefore', supStyloidBefore, 'supStyloidAfter', supStyloidAfter, ...
                  'palmBefore', palmBefore, 'palmAfter', palmAfter, ...
                  'supHandBefore', a0.supHand, 'supHandAfter', a1.supHand, 'delta', delta, ...
                  'wristShift', norm(jointCenter(jc.([S 'Wrist']),:) - W0), 'axis', f, 'pivot', p1, ...
                  'vertices', armVerts(:), 'weights', w, 'opts', opts);
end

function [a, sgn] = tposePalm(meshModel, adjustedLM, jointCenter, S)
    % Palm direction in the T-pose: rotateArmTpose is run on a copy (only its joint centres are used); palm normal from
    % the plane wrist / index MCP / pinky MCP, palm side = down; angle from straight down (-Z) towards anterior (+X).
    % 0 = palm down, + = palm turned forward. sgn: sign of a rotation about the forearm axis (radial head -> ulnar head,
    % i.e. towards the hand) that increases the angle: + for an arm pointing along -Y (right), - along +Y (left).
    jc = jcIndices();
    [~, ~, JT] = rotateArmTpose(meshModel, adjustedLM, jointCenter);
    W = JT(jc.([S 'Wrist']),:); I = JT(jc.([S 'IndexCarpal']),:); P = JT(jc.([S 'PinkyCarpal']),:);
    n = cross(I - W, P - W); n = n/norm(n); if n(3) > 0, n = -n; end
    a = atan2d(n(1), -n(3));
    sgn = sign(-(W(2) - JT(jc.([S 'Elbow']),2)));
end
