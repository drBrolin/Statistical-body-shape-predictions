% kneeMinFlexion: keeps the predicted standing knees from being hyperextended. Where the knee flexion is below
% minDeg, the knee joint centre is moved forward (anterior) until the flexion is exactly minDeg; the mesh, the
% landmarks and the posture stay as they are. Called from mainInput.m right after jcPrediction (before the A-pose
% snapshot), so the A- and T-pose joint centres both have it.
%
% Knee flexion (as Matlab/scripts/alternatives/checkKnee.m): thigh u1 = hip -> knee, shank u2 = knee -> ankle joint
% centres, both projected on the plane perpendicular to the femoral epicondyle axis (Lateral - Medial epicondyle
% landmark); signed angle from u1 to u2, + = flexed (shank swung posterior), - = hyperextended.
% Shift direction: the knee's own forward direction, i.e. global anterior (+X) made perpendicular to the epicondyle
% axis and kept horizontal (the knee keeps its height; thigh and shank lengths change by < 0.3 mm).
%
% A deliberate choice of posture over anatomy: the joint centre is otherwise the midpoint of the epicondyle landmarks
% (ISB), and musculoskeletal models put the knee axis 6-10 mm behind that midpoint (Rajagopal 2016, gait2392;
% alternatives/deriveOpenSimKnee.py). On the 36-body test grid the shift is 0-15 mm (young, slim bodies most).
%
%   [jointCenter, info] = kneeMinFlexion(jointCenter, adjustedLM)
%   [jointCenter, info] = kneeMinFlexion(jointCenter, adjustedLM, struct('minDeg', 1))
%   info(k): side, flexBefore, flexAfter, shift (mm), direction
function [jointCenter, info] = kneeMinFlexion(jointCenter, adjustedLM, opts)
    if nargin < 3, opts = struct(); end
    if ~isfield(opts, 'minDeg'), opts.minDeg = 1; end
    lm = lmIndices(); jc = jcIndices();
    info = struct('side', {}, 'flexBefore', {}, 'flexAfter', {}, 'shift', {}, 'direction', {});
    for S = {'Rt', 'Lt'}
        s = S{1};
        H = jointCenter(jc.([s 'Hip']),:); K = jointCenter(jc.([s 'Knee']),:); A = jointCenter(jc.([s 'Ankle']),:);
        lat = adjustedLM(lm.([s 'FemoralLateralEpicn']),:); med = adjustedLM(lm.([s 'FemoralMedialEpicn']),:);
        a = (lat - med)/norm(lat - med);
        e = [1 0 0] - a(1)*a; e(3) = 0; e = e/norm(e);           % forward, perpendicular to the axis, horizontal
        flex = @(k) kneeFlexion(H, k, A, a);
        f0 = flex(K); shift = 0;
        if f0 < opts.minDeg
            g = @(t) flex(K + t*e) - opts.minDeg;                % increases with t (knee forward = more flexed)
            hi = 10; while g(hi) < 0 && hi < 200, hi = 2*hi; end
            shift = fzero(g, [0 hi]);
            jointCenter(jc.([s 'Knee']),:) = K + shift*e;
        end
        info(end+1) = struct('side', s, 'flexBefore', f0, 'flexAfter', flex(jointCenter(jc.([s 'Knee']),:)), ...
                             'shift', shift, 'direction', e); %#ok<AGROW>
    end
end

function f = kneeFlexion(H, K, A, a)
    u1 = (K - H)/norm(K - H); u2 = (A - K)/norm(A - K);
    if dot(cross(a, u1), [-1 0 0]) < 0, a = -a; end              % + rotation about a swings the shank posterior
    p1 = u1 - dot(u1, a)*a; p2 = u2 - dot(u2, a)*a;
    f = atan2d(dot(cross(p1, p2), a), dot(p1, p2));
end
