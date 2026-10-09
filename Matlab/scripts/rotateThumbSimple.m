function [meshModel, jointCenter, info] = rotateThumbSimple(meshStart, meshEnd, jcChainTotal, meshModel, jointCenter, fingerDir, insideHand, opts)
    % A lighter thumb posture: ONE rotation about the thumb CMC, written like parallelFingers.
    % The thumb axis (CMC -> tip) is turned onto a target direction given by two angles, relative to the fingers:
    %   opts.spreadDeg  angle from fingerDir, measured in the hand plane (towards the thumb side)       default 25
    %   opts.lowerDeg   angle below the hand plane (palm/finger plane, away from opts.upDir)           default 0
    % The rotation is the shortest one from the current thumb axis to that direction.
    %
    % Which vertices move, and how much (weight w, vertex rotated by w*angle about the CMC):
    %   candidates  thumb-mask vertices, plus palm-mask vertices (palm and back of the hand) that are not in a
    %               finger mask and lie inside a swept sphere (capsule) along CMC -> MCP -> IP -> tip
    %   w = wAxial * wRadial
    %     wAxial   0 at the CMC rising linearly to 1 at the thumb MCP, along the thumb axis (opts.blend 'linear'),
    %              or 1 everywhere (opts.blend 'rigid')
    %     wRadial  1 within the thumb's own radius (median distance of thumb-only vertices to the joint line),
    %              smooth drop-off to 0 at opts.sweepRadius (mm) from the joint line; thumb-only vertices always 1
    %   opts.sweepRadius  outer radius of the swept sphere: a number (mm), or 'auto' (default) =
    %                     opts.sweepScale x thumb-tip displacement (thumb length CMC-MCP-IP-tip x rotation angle, rad),
    %                     at least 2 x the thumb radius. A larger rotation then spreads over more of the palm.
    %   opts.sweepScale   default 1.3 (spread 22.5/lowered 15: ~30 mm female, ~40-46 mm male; spread 25/lowered 12: 21-26 mm female, 30-43 mm male)
    %   opts.sweepStart   [] (default): swept sphere and axial weight start at the CMC. A point (1x3, e.g. the
    %                     wrist joint centre): the swept sphere gets an extra segment point -> CMC, and the
    %                     axial weight rises from 0 at that point (projected on the thumb axis) to 1 at the MCP
    %   opts.palmScale    outer radius on the palm side = palmScale x sweepRadius (default 1 = same on both sides);
    %                     the back of the hand keeps sweepRadius. Side = offset from the nearest point of the swept path
    %                     along the hand-plane normal, blended smoothly over +-1 thumb radius (no seam)
    %   opts.fullAt       'MCP' (default): axial weight reaches 1 at the thumb MCP; 'CMC': reaches 1 already at the CMC
    %                     (with sweepStart, the weight then rises from the sweepStart point to the CMC)
    %   opts.cmcDepth     [] (default, no drop) or a fraction of the middle finger length (MCP -> tip): first move the
    %                     whole thumb down (along -upDir) until the CMC is that deep below the finger MCP+PIP plane
    %                     (MakeHuman manikins: 0.30). Also opts.dropStart (e.g. wrist joint centre), opts.dropRadius,
    %                     opts.dropScale, see dropThumb. Returns info.cmcDepthBefore and info.cmcShift (mm)
    %   opts.twistDeg     0 (default) or the twist about the thumb axis after the rotation, nail side towards the thumb
    %                     side (the SBSP thumb is "nail up"); opts.twistFrom 'MCP' (default), 'CMC' or 0..1. See twistThumb
    %   opts.upDir        hand-plane normal on the BACK-of-hand side (default: plane through the finger MCP + PIP
    %                     joints, oriented to global +Z)
    %
    %   jcChainTotal = jc.RtThumbCarpal:jc.RtThumbDist, fingerDir = e.g. middle MCP -> middle tip
    %   insideHand   = insideHandTotalRight (columns 1 palm, 2 thumb, 3-6 index..pinky)
    if nargin < 8, opts = struct(); end
    if ~isfield(opts,'spreadDeg'), opts.spreadDeg = 25; end
    if ~isfield(opts,'lowerDeg'),  opts.lowerDeg = 0; end
    if ~isfield(opts,'blend'),     opts.blend = 'linear'; end
    jcPivot = jcChainTotal(1);   % CMC — rotation pivot
    jcTip   = jcChainTotal(end);
    jcStart = jointCenter(jcPivot, :);
    if ~isfield(opts,'upDir') || isempty(opts.upDir)
        % finger MCP and PIP rows: index..pinky chains lie 16..3 rows before the thumb CMC in jcIndices
        rows = jcPivot + [-16 -15 -12 -11 -8 -7 -4 -3];
        P = jointCenter(rows, :); [~, ~, V] = svd(P - mean(P, 1), 'econ'); opts.upDir = V(:,3)';
        if opts.upDir(3) < 0, opts.upDir = -opts.upDir; end
    end

    %% Optional drop: lower the whole thumb (CMC included) along -upDir before the rotation
    info.cmcDepthBefore = NaN; info.cmcShift = 0;
    if isfield(opts,'cmcDepth') && ~isempty(opts.cmcDepth)
        [meshModel, jointCenter, info] = dropThumb(meshStart, meshEnd, jcChainTotal, meshModel, jointCenter, insideHand, opts, info);
        jcStart = jointCenter(jcPivot, :);
    end

    %% Target direction and rotation
    uFinger = fingerDir / norm(fingerDir);
    uUp     = opts.upDir - dot(opts.upDir, uFinger)*uFinger;  uUp = uUp / norm(uUp);
    uThumb  = jointCenter(jcTip, :) - jcStart;                 uThumb = uThumb / norm(uThumb);
    uSide   = cross(uUp, uFinger); if dot(uSide, uThumb) < 0, uSide = -uSide; end   % in plane, towards the thumb
    angles  = @(u) [atan2d(dot(u, uSide), dot(u, uFinger)), -asind(dot(u, uUp))];    % [spread, lowered]
    info.before = angles(uThumb);
    target = cosd(opts.lowerDeg)*(cosd(opts.spreadDeg)*uFinger + sind(opts.spreadDeg)*uSide) - sind(opts.lowerDeg)*uUp;
    n = cross(uThumb, target);
    theta = atan2(norm(n), dot(uThumb, target));
    info.rotationDeg = rad2deg(theta);
    info.after = info.before;
    if norm(n) < 1e-10, return; end
    n = n / norm(n);

    %% Vertices to move and their weights
    vAll    = (meshStart:meshEnd)';
    mask    = insideHand(1:numel(vAll), :);
    thumbV  = mask(:,2) & ~mask(:,1);                              % thumb only
    candV   = mask(:,2) | (mask(:,1) & ~any(mask(:,3:6), 2));      % thumb, or palm/back of hand not on a finger
    joints  = jointCenter(jcChainTotal, :);
    if isfield(opts,'sweepStart') && ~isempty(opts.sweepStart), sweepPath = [opts.sweepStart; joints]; else, sweepPath = joints; end
    [d, h]  = distToPolyline(meshModel(vAll,:), sweepPath, uUp);  % h: offset along uUp (> 0 back of hand)
    info.thumbRadius = median(d(thumbV));
    if ~isfield(opts,'sweepRadius') || isempty(opts.sweepRadius), opts.sweepRadius = 'auto'; end
    if ~isfield(opts,'sweepScale'), opts.sweepScale = 1.3; end
    info.tipDisplacement = sum(vecnorm(diff(joints), 2, 2)) * theta;   % arc length the thumb tip moves, mm
    if ischar(opts.sweepRadius) || isstring(opts.sweepRadius)
        opts.sweepRadius = max(2*info.thumbRadius, opts.sweepScale * info.tipDisplacement);
    end
    if ~isfield(opts,'palmScale'), opts.palmScale = 1; end
    fPalm   = smooth01(0.5 - h / (2*info.thumbRadius));             % 1 on the palm side, 0 on the back
    rOuter  = opts.sweepRadius * (1 + (opts.palmScale - 1) * fPalm);
    wRadial = smooth01((rOuter - d) ./ max(rOuter - info.thumbRadius, eps));
    wRadial(thumbV) = 1;
    if strcmp(opts.blend, 'linear')
        lenMC  = dot(joints(2,:) - jcStart, uThumb);               % CMC -> MCP along the thumb axis
        if isfield(opts,'fullAt') && strcmpi(opts.fullAt,'CMC'), lenMC = 0; end   % weight 1 from the CMC on
        s0 = 0;                                                    % axial weight 0 here (CMC, or sweepStart)
        if isfield(opts,'sweepStart') && ~isempty(opts.sweepStart), s0 = min(0, dot(opts.sweepStart - jcStart, uThumb)); end
        if lenMC - s0 > 0
            wAxial = min(1, max(0, ((meshModel(vAll,:) - jcStart) * uThumb' - s0) / (lenMC - s0)));
        else
            wAxial = double((meshModel(vAll,:) - jcStart) * uThumb' >= 0);   % fullAt CMC without sweepStart: step at the CMC
        end
    else
        wAxial = ones(numel(vAll), 1);
    end
    w = wAxial .* wRadial .* candV;
    info.sweepRadius = opts.sweepRadius;
    info.nMoved = nnz(w > 0); info.nMovedPalm = nnz(w > 0 & ~mask(:,2));

    for i = find(w > 0)'
        p = meshModel(vAll(i), :) - jcStart;
        meshModel(vAll(i), :) = (axisRotation(n, w(i)*theta) * p')' + jcStart;
    end
    R = axisRotation(n, theta);
    jointCenter(jcChainTotal(2):jcTip, :) = (R * (jointCenter(jcChainTotal(2):jcTip, :) - jcStart)')' + jcStart;
    info.after = angles((jointCenter(jcTip, :) - jcStart) / norm(jointCenter(jcTip, :) - jcStart));

    %% Optional twist about the thumb's own axis (nail turning towards the thumb side)
    info.twistDeg = 0;
    if isfield(opts,'twistDeg') && opts.twistDeg ~= 0
        [meshModel, jointCenter, info.twistDeg] = twistThumb(meshStart, meshEnd, jcChainTotal, meshModel, jointCenter, insideHand, opts, uUp, uSide);
    end
end

function [meshModel, jointCenter, twistApplied] = twistThumb(meshStart, meshEnd, jcChainTotal, meshModel, jointCenter, insideHand, opts, uUp, uSide)
    % Twist the thumb about the line CMC -> tip by opts.twistDeg so that its back (the nail side, taken as the hand's
    % back-of-hand direction since the SBSP mesh is "nail up") turns towards the thumb side. The weight rises smoothly
    % from 0 at opts.twistFrom ('MCP' default, 'CMC', or a number: 0 = CMC, 1 = MCP) to 1 at the tip. Thumb-mask
    % vertices and the MCP/IP/tip joints are twisted by the weight at their own position along the axis, so the
    % joints keep their place inside the thumb.
    if ~isfield(opts,'twistFrom'), opts.twistFrom = 'MCP'; end
    cmc = jointCenter(jcChainTotal(1),:); tip = jointCenter(jcChainTotal(end),:);
    k = (tip - cmc) / norm(tip - cmc);
    nail = uUp - dot(uUp,k)*k; nail = nail / norm(nail);
    side = uSide - dot(uSide,k)*k;
    twistApplied = opts.twistDeg;
    th = deg2rad(opts.twistDeg); if dot(cross(k, nail), side) < 0, th = -th; end
    f = opts.twistFrom;
    if ischar(f) || isstring(f)
        switch upper(char(f)), case 'CMC', f = 0; case 'MCP', f = 1; otherwise, error('rotateThumbSimple:twistFrom', 'twistFrom must be ''CMC'', ''MCP'' or a number'); end
    end
    s0 = f * dot(jointCenter(jcChainTotal(2),:) - cmc, k); s1 = dot(tip - cmc, k);
    weight = @(P) smooth01(((P - cmc) * k' - s0) / (s1 - s0));
    vAll = (meshStart:meshEnd)'; v = vAll(insideHand(1:numel(vAll),2) == 1);
    w = weight(meshModel(v,:));
    for i = find(w > 0)'
        meshModel(v(i),:) = (axisRotation(k, w(i)*th) * (meshModel(v(i),:) - cmc)')' + cmc;
    end
    for j = jcChainTotal(2:end)
        jointCenter(j,:) = (axisRotation(k, weight(jointCenter(j,:))*th) * (jointCenter(j,:) - cmc)')' + cmc;
    end
end

function [meshModel, jointCenter, info] = dropThumb(meshStart, meshEnd, jcChainTotal, meshModel, jointCenter, insideHand, opts, info)
    % Move the whole thumb (CMC, MCP, IP, tip and the thumb mesh) along -upDir so that the CMC lies opts.cmcDepth x
    % (middle finger MCP -> tip length) below the plane of the finger MCP + PIP joints. Only moves down, never up.
    % Weights: thumb-only vertices 1; other thumb and palm/back-of-hand vertices (not on a finger) by
    %   wAxial  1 from the CMC on; 0 at opts.dropStart (e.g. the wrist, projected on the thumb axis) rising to 1 at
    %           the CMC; without dropStart the ramp is 2 thumb radii long, behind the CMC
    %   wRadial 1 within the thumb radius, smooth drop-off to 0 at opts.dropRadius from the joint line (+ dropStart)
    %   opts.dropRadius  mm, or 'auto' (default) = opts.dropScale x thumb radius; dropScale default 4.5 (~39 mm). A radius
    %                    tied to the shift was too abrupt for small shifts (16 mm at 4.5 mm shift: 2 flipped faces)
    jcPivot = jcChainTotal(1);
    rowsF = jcPivot + [-16 -15 -12 -11 -8 -7 -4 -3];                    % finger MCP + PIP joints
    c = mean(jointCenter(rowsF,:), 1); nUp = opts.upDir / norm(opts.upDir);
    Lmid = norm(jointCenter(jcPivot-9,:) - jointCenter(jcPivot-12,:));  % middle MCP -> middle tip
    cmc = jointCenter(jcPivot,:);
    info.cmcDepthBefore = -dot(cmc - c, nUp) / Lmid;
    shift = max(0, opts.cmcDepth - info.cmcDepthBefore) * Lmid;
    info.cmcShift = shift;
    if shift == 0, return; end

    vAll   = (meshStart:meshEnd)';
    mask   = insideHand(1:numel(vAll), :);
    thumbV = mask(:,2) & ~mask(:,1);
    candV  = mask(:,2) | (mask(:,1) & ~any(mask(:,3:6), 2));
    joints = jointCenter(jcChainTotal, :);
    hasStart = isfield(opts,'dropStart') && ~isempty(opts.dropStart);
    if hasStart, dropPath = [opts.dropStart; joints]; else, dropPath = joints; end
    d  = distToPolyline(meshModel(vAll,:), dropPath, nUp);
    rT = median(d(thumbV));
    if ~isfield(opts,'dropScale'), opts.dropScale = 4.5; end
    if ~isfield(opts,'dropRadius') || isempty(opts.dropRadius) || ischar(opts.dropRadius) || isstring(opts.dropRadius)
        opts.dropRadius = opts.dropScale * rT;
    end
    info.dropRadius = opts.dropRadius;
    uT = joints(end,:) - cmc; uT = uT / norm(uT);
    s  = (meshModel(vAll,:) - cmc) * uT';
    if hasStart, s0 = min(-eps, dot(opts.dropStart - cmc, uT)); else, s0 = -2*rT; end
    wAxial  = min(1, max(0, (s - s0) / -s0));
    wRadial = smooth01((opts.dropRadius - d) / max(opts.dropRadius - rT, eps));
    wRadial(thumbV) = 1;
    w = wAxial .* wRadial .* candV;
    t = -nUp * shift;
    meshModel(vAll,:) = meshModel(vAll,:) + w .* t;
    jointCenter(jcChainTotal,:) = jointCenter(jcChainTotal,:) + t;
    info.nMovedDrop = nnz(w > 0);
end

function [d, h] = distToPolyline(X, P, up)
    % Distance from every row of X to the polyline through the rows of P (= the swept-sphere test), and the
    % offset from the nearest polyline point along the direction up
    d = inf(size(X,1), 1); h = zeros(size(X,1), 1);
    for k = 1:size(P,1)-1
        a = P(k,:); ab = P(k+1,:) - a;
        t = min(1, max(0, (X - a) * ab' / dot(ab, ab)));
        v = X - (a + t.*ab); dk = vecnorm(v, 2, 2); closer = dk < d;
        d(closer) = dk(closer); h(closer) = v(closer,:) * up';
    end
end

function w = smooth01(s), s = min(1, max(0, s)); w = s.^2 .* (3 - 2*s); end

function R = axisRotation(n, theta)
    % Rotation matrix (axis–angle), same form as in straightFingers
    nx = n(1); ny = n(2); nz = n(3);
    c = cos(theta); s = sin(theta);
    R = [ ...
        c + nx^2*(1-c),     nx*ny*(1-c) - nz*s, nx*nz*(1-c) + ny*s;
        ny*nx*(1-c) + nz*s, c + ny^2*(1-c),     ny*nz*(1-c) - nx*s;
        nz*nx*(1-c) - ny*s, nz*ny*(1-c) + nx*s, c + nz^2*(1-c)
    ];
end
