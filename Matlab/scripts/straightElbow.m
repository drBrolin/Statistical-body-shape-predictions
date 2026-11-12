% straightElbow: straightens (or sets) the elbow flexion of one arm to a target angle, before the T-pose rotation.
% Called from mainInput.m after jcPrediction (after the A-pose snapshot) and before rotateArmTpose. Only the right arm
% is needed there: rotateArmTpose mirrors the right arm, hand, joint centres and landmarks to the left afterwards.
%
% Elbow frame (from landmarks and joint centres):
%   pivot     elbow joint centre (jc.RtElbow, elbowAlt.m: on the trochlea/capitulum, near the epicondyle axis)
%   axis a    flexion axis = unit(Lateral - Medial humeral epicondyle landmark), oriented so that a positive
%             rotation about it is flexion (moves the forearm anteriorly; anterior = away from the olecranon)
%   u1, u2    upper arm GH -> elbow, forearm elbow -> wrist (unit vectors)
%   flexion   signed angle from u1 to u2, both projected on the plane perpendicular to a (0 = straight arm,
%             positive = flexed, negative = hyperextended). The out-of-plane part of the forearm (the carrying
%             angle) is kept, since the rotation is about the flexion axis itself.
%
% Mesh weight (utils/hingeWeights; each arm vertex is rotated by w * delta about a, through the pivot, utils/rotateSegment):
%   s = signed distance of the vertex from the plane that bisects the elbow angle (normal unit(u1 + u2), through
%   the pivot; s > 0 on the forearm side). w = smoothstep over s in [-R, R]: 0 above the blend zone (upper arm),
%   1 below it (forearm and hand), smooth in between. A vertex is therefore only partially rotated if it lies
%   within R of the elbow along the arm, i.e. inside a sphere-like zone around the joint. The bisecting plane
%   (not the plane perpendicular to the upper arm) keeps the inner crease and the olecranon side balanced.
%   Only vertices of the arm/hand region are candidates (r.rightArmHand / r.leftArmHand).
% Joint centres (wrist, all finger joints) and forearm/hand landmarks (olecranon, radiale, styloids,
% metacarpals, dactylion) move rigidly with the forearm (w = 1); the epicondyles stay with the humerus.
%
%   [meshModel, adjustedLM, jointCenter, info] = straightElbow(meshModel, adjustedLM, jointCenter, opts)
%   opts.targetDeg    target flexion (deg), required, e.g. 0 = straight, 5 = slightly flexed
%   opts.side         'Rt' (default) or 'Lt'
%   opts.radius       half-width R of the blend zone in mm, or 'auto' (default) = radiusScale x epicondyle width
%   opts.radiusScale  default 0.8 (epicondyle width ~60-75 mm -> R ~ 50-60 mm)
%   opts.onlyExtend   true (default): never flex an elbow that is already straighter than the target
%   info              flexion/carrying angle before and after, delta, R, the frame and the per-vertex weights
function [meshModel, adjustedLM, jointCenter, info] = straightElbow(meshModel, adjustedLM, jointCenter, opts)
    if nargin < 4, opts = struct(); end
    if ~isfield(opts,'targetDeg') || isempty(opts.targetDeg), error('straightElbow: opts.targetDeg is required'); end
    if ~isfield(opts,'side'),        opts.side = 'Rt'; end
    if ~isfield(opts,'radius'),      opts.radius = 'auto'; end
    if ~isfield(opts,'radiusScale'), opts.radiusScale = 0.8; end
    if ~isfield(opts,'onlyExtend'),  opts.onlyExtend = true; end
    r = meshRegions(); lm = lmIndices(); jc = jcIndices();
    S = opts.side;
    if strcmp(S,'Rt'), armVerts = r.rightArmHand; else, armVerts = r.leftArmHand; end

    % Forearm/hand joint centres and landmarks that move rigidly with the forearm
    jcMove = [jc.([S 'Wrist']), jc.([S 'IndexCarpal']):jc.([S 'ThumbDist'])];
    lmMove = [lm.([S 'RadialStyloid']), lm.([S 'Olecranon']), lm.([S 'Radiale']), lm.([S 'MetacarpalPhalII']), ...
              lm.([S 'Dactylion']), lm.([S 'UlnarStyloid']), lm.([S 'MetacarpalPhalV'])];

    %% Elbow frame
    lat = adjustedLM(lm.([S 'HumeralLateralEpicn']),:);
    med = adjustedLM(lm.([S 'HumeralMedialEpicn']),:);
    epiWidth = norm(lat - med);
    a = (lat - med)/epiWidth;
    E  = jointCenter(jc.([S 'Elbow']),:);
    u1 = unit(E - jointCenter(jc.([S 'Shoulder']),:));
    u2 = unit(jointCenter(jc.([S 'Wrist']),:) - E);
    antRef = (lat + med)/2 - adjustedLM(lm.([S 'Olecranon']),:);     % olecranon is posterior
    if dot(cross(a, u1), antRef) < 0, a = -a; end                    % positive rotation about a = flexion

    [flexBefore, carryBefore] = elbowAngles(u1, u2, a);
    delta = opts.targetDeg - flexBefore;                              % negative = extend
    if opts.onlyExtend && delta > 0, delta = 0; end

    if ischar(opts.radius), R = opts.radiusScale*epiWidth; else, R = opts.radius; end

    %% Weights across the plane bisecting the elbow angle; rotate: vertices by w*delta, forearm JCs/landmarks by delta
    [w, n] = hingeWeights(meshModel(armVerts,:), E, u1, u2, R);
    seg = struct('verts', armVerts, 'weights', w, 'pivot', E, 'axis', a, 'angleDeg', delta, ...
                 'jcMove', jcMove, 'lmMove', lmMove);
    [meshModel, jointCenter, adjustedLM] = rotateSegment(meshModel, jointCenter, adjustedLM, seg);

    u2after = unit(jointCenter(jc.([S 'Wrist']),:) - E);
    [flexAfter, carryAfter] = elbowAngles(u1, u2after, a);

    info = struct('side', S, 'targetDeg', opts.targetDeg, 'flexBefore', flexBefore, 'flexAfter', flexAfter, ...
                  'carryBefore', carryBefore, 'carryAfter', carryAfter, 'delta', delta, 'R', R, ...
                  'epicondyleWidth', epiWidth, 'axis', a, 'pivot', E, 'bisectorNormal', n, ...
                  'vertices', armVerts(:), 'weights', w, 'opts', opts);
end

function [flex, carry] = elbowAngles(u1, u2, a)
    % flexion about a (in the plane perpendicular to a), and the carrying angle (forearm out of that plane)
    p1 = u1 - dot(u1,a)*a; p2 = u2 - dot(u2,a)*a;
    flex  = atan2d(dot(cross(p1,p2),a), dot(p1,p2));
    carry = asind(max(min(dot(u2,a),1),-1));
end

function v = unit(v)
    v = v/norm(v);
end
