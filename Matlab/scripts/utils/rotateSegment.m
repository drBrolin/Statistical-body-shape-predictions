% rotateSegment: rotates part of the body about an axis through a pivot. Mesh vertices turn by weight x angle (smooth
% deformation), the listed joint centres and landmarks turn rigidly by the full angle.
%   seg.verts     mesh vertex indices that may move (column or row vector)
%   seg.weights   one weight per vertex in seg.verts (0 = stays, 1 = full rotation)
%   seg.pivot     1x3 point on the rotation axis
%   seg.axis      1x3 unit axis (right-hand rule: positive angle = counter-clockwise looking against the axis)
%   seg.angleDeg  rotation angle (deg)
%   seg.jcMove    joint centre rows that turn rigidly (optional)
%   seg.lmMove    landmark rows that turn rigidly (optional)
%   [meshModel, jointCenter, adjustedLM] = rotateSegment(meshModel, jointCenter, adjustedLM, seg)
%   Q = rotateSegment(P, C, a, thetaRad)   plain Rodrigues rotation of the rows of P (theta scalar or one per row)
function varargout = rotateSegment(varargin)
    if nargin == 4 && isstruct(varargin{4})
        [meshModel, jointCenter, adjustedLM, seg] = varargin{:};
        th = deg2rad(seg.angleDeg);
        v = seg.verts(:);
        meshModel(v,:) = rodrigues(meshModel(v,:), seg.pivot, seg.axis, th*seg.weights(:));
        if isfield(seg,'jcMove') && ~isempty(seg.jcMove)
            jointCenter(seg.jcMove,:) = rodrigues(jointCenter(seg.jcMove,:), seg.pivot, seg.axis, th);
        end
        if isfield(seg,'lmMove') && ~isempty(seg.lmMove)
            adjustedLM(seg.lmMove,:) = rodrigues(adjustedLM(seg.lmMove,:), seg.pivot, seg.axis, th);
        end
        varargout = {meshModel, jointCenter, adjustedLM};
    else
        varargout = {rodrigues(varargin{:})};
    end
end

function Q = rodrigues(P, C, a, theta)
    V = P - C;
    c = cos(theta); s = sin(theta);
    Q = V.*c + cross(repmat(a,size(V,1),1), V, 2).*s + (V*a').*a.*(1 - c) + C;
end
