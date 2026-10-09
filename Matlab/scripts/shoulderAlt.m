% shoulderAlt: shoulder-girdle joint centres (SC, AC, GH), called from jcPrediction.m.
% Places the sterno-clavicular (SC), acromio-clavicular (AC) and gleno-humeral (GH) joint centres on both sides
% from the skin landmarks, with offsets from the thoracoscapular shoulder model (Seth A, Dong M, Veloso A, Delp SL.
% Muscle contributions to upper-extremity movement and work from a musculoskeletal model of the human shoulder.
% Front Neurorobot 2019; 13:90. SimTK project "thoracoscapular", CC BY 4.0): subject model at rest (first frame,
% arm at the side, of the 18 IK trials), scaled per body:
%   s   = bony biacromial breadth / 323.8 mm (model), bony breadth = Acromion-landmark breadth - 2*tAcr
%   AC  = the model's AC joint:             Acromion + [6.6*s, medial (23.8*s + tAcr), 4.0*s]
%   GH  = model offset from that AC:        AC + [11.2*s, lateral 20.5*s, -32.8*s]
%   SC  = centre of the clavicle head:      Clavicale + [-(10.1*s + tClav), 0, -(9.1*s + tClav)]
% AC offsets are from the most lateral bony acromion point; SC offsets are the clavicle-head centre behind its front
% and below its top. Offsets are in the SBSP frame before export: X anterior, Y lateral (+Y = body's left), Z up, mm.
% tAcr = 1.7 mm is the soft tissue lateral of the acromion implied by SBSP's own prediction (landmark
% breadth vs caliper biacromial); tClav = 2 mm is assumed for the skin over the clavicle head.
% Use together with rotateArmTpose.m (atan2): the AC can lie above or below the acromion landmark.
%   [jointCenter, info] = shoulderAlt(jointCenter, adjustedLM)
%   [...] = shoulderAlt(jointCenter, adjustedLM, opts)   opts fields: see defaults below
function [jointCenter, info] = shoulderAlt(jointCenter, adjustedLM, opts)
    if nargin < 3, opts = struct(); end
    d = struct( ...
        'biacromialRef', 323.8, ...             % bony biacromial breadth of the reference model (mm)
        'tAcr',  1.7, ...                       % soft tissue lateral of the acromion bone (mm)
        'tClav', 2.0, ...                       % soft tissue over the clavicle head (mm)
        'acOffset', [6.6 23.8 4.0], ...         % AC from lateral acromion bone point: [anterior medial up] / s
        'ghOffset', [11.2 20.5 -32.8], ...      % GH from AC: [anterior lateral up] / s
        'scOffset', [10.1 9.1]);                % SC centre behind the front / below the top of the clavicle head / s
    f = fieldnames(d);
    for k = 1:numel(f), if ~isfield(opts, f{k}), opts.(f{k}) = d.(f{k}); end, end

    lm = lmIndices(); jc = jcIndices();
    bone = abs(adjustedLM(lm.LtAcromion,2) - adjustedLM(lm.RtAcromion,2)) - 2*opts.tAcr;
    s = bone / opts.biacromialRef;

    % Right side: medial = +Y. Left side: medial = -Y.
    sides = struct('acr', {lm.RtAcromion,   lm.LtAcromion}, ...
                   'cla', {lm.RtClavicale,  lm.LtClavicale}, ...
                   'sc',  {jc.RtSternoClavicular, jc.LtSternoClavicular}, ...
                   'ac',  {jc.RtAcromioClavicular, jc.LtAcromioClavicular}, ...
                   'gh',  {jc.RtShoulder,   jc.LtShoulder}, ...
                   'medial', {1, -1});
    for k = 1:2
        S = sides(k); m = S.medial;
        acr = adjustedLM(S.acr,:); cla = adjustedLM(S.cla,:);
        a = opts.acOffset; g = opts.ghOffset; c = opts.scOffset;
        AC = acr + [a(1)*s, m*(a(2)*s + opts.tAcr), a(3)*s];
        GH = AC  + [g(1)*s, -m*g(2)*s, g(3)*s];
        SC = cla + [-(c(1)*s + opts.tClav), 0, -(c(2)*s + opts.tClav)];
        jointCenter(S.ac,:) = AC; jointCenter(S.gh,:) = GH; jointCenter(S.sc,:) = SC;
    end

    info = struct('scale', s, 'bonyBiacromial', bone, 'opts', opts);
end
