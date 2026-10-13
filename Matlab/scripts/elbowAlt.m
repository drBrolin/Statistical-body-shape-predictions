% elbowAlt: elbow joint centres, called from jcPrediction.m after shoulderAlt (uses the GH joint centres).
% Places the elbow joint centre on both sides from the epicondyle landmarks, with the offset of the elbow joint from
% the epicondyle midpoint in the thoracoscapular shoulder model (Seth A, Dong M, Veloso A, Delp SL. Muscle
% contributions to upper-extremity movement and work from a musculoskeletal model of the human shoulder.
% Front Neurorobot 2019; 13:90. SimTK project "thoracoscapular", CC BY 4.0): subject model, elbow extended,
% scaled per body:
%   s     = epicondyle width (Lateral - Medial epicondyle landmark) / 69.0 mm (model's epicondyle markers)
%   frame   e_lat = unit(Lateral - Medial epicondyle), e_up = unit(GH - epicondyle midpoint) orthogonal to e_lat,
%           e_ant = anterior, perpendicular to both
%   Elbow = epicondyle midpoint + s*(5.7*e_ant - 5.7*e_up - 0.9*e_lat)
% The elbow joint lies on the trochlea/capitulum, anterior and distal of the epicondyle midpoint.
%   [jointCenter, info] = elbowAlt(jointCenter, adjustedLM)
%   [...] = elbowAlt(jointCenter, adjustedLM, opts)   opts fields: see defaults below
function [jointCenter, info] = elbowAlt(jointCenter, adjustedLM, opts)
    if nargin < 3, opts = struct(); end
    d = struct( ...
        'widthRef', 69.0, ...                   % epicondyle width of the reference model (mm)
        'offset', [5.7 -5.7 -0.9]);             % elbow from the epicondyle midpoint: [anterior up lateral] / s
    f = fieldnames(d);
    for k = 1:numel(f), if ~isfield(opts, f{k}), opts.(f{k}) = d.(f{k}); end, end

    lm = lmIndices(); jc = jcIndices();
    sides = struct('lat', {lm.RtHumeralLateralEpicn, lm.LtHumeralLateralEpicn}, ...
                   'med', {lm.RtHumeralMedialEpicn,  lm.LtHumeralMedialEpicn}, ...
                   'gh',  {jc.RtShoulder,            jc.LtShoulder}, ...
                   'el',  {jc.RtElbow,               jc.LtElbow}, ...
                   'side', {1, -1});
    s = zeros(1,2);
    for k = 1:2
        S = sides(k);
        lat = adjustedLM(S.lat,:); med = adjustedLM(S.med,:); mid = (lat + med)/2;
        w = norm(lat - med); e_lat = (lat - med)/w;
        up = jointCenter(S.gh,:) - mid; e_up = up - (up*e_lat')*e_lat; e_up = e_up/norm(e_up);
        e_ant = S.side*cross(e_up, e_lat);       % right: +Y medial, left: +Y lateral
        s(k) = w/opts.widthRef; o = opts.offset;
        jointCenter(S.el,:) = mid + s(k)*(o(1)*e_ant + o(2)*e_up + o(3)*e_lat);
    end

    info = struct('scale', s, 'opts', opts);
end
