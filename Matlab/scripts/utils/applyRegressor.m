function points = applyRegressor(R, meshModel)
% applyRegressor  Reconstruct target points from a computeRegressor relation
%
% Each target point is rebuilt as the weighted average over its stored
% neighbours of (vertex position + stored offset), using the stored
% inverse-distance weights normalised per point. On the reference mesh this
% returns the original target points exactly; on a deformed mesh the point
% follows its neighbourhood while keeping its offset from the surface.
%
% Inputs:
%   R         : J x 5*k relation matrix from computeRegressor
%   meshModel : Nx3 mesh vertices, same ordering and units as the mesh the
%               relation was computed on
%
% Output:
%   points : Jx3 reconstructed point coordinates
%
% See also computeRegressor

    if mod(size(R,2), 5) ~= 0
        error('applyRegressor:badRelation', ...
              'R must have 5*k columns, got %d.', size(R,2));
    end

    k   = size(R,2) / 5;
    idx = R(:, 1:5:end);                       % JxK vertex indices
    w   = R(:, 2:5:end);                       % JxK weights
    w   = w ./ sum(w, 2);                      % normalise per target point

    points = zeros(size(R,1), 3);
    for dim = 1:3
        v = meshModel(:,dim);
        off = R(:, (2+dim):5:end);             % JxK offsets in this dimension
        points(:,dim) = sum(w .* (off + reshape(v(idx), size(idx))), 2);
    end
end
