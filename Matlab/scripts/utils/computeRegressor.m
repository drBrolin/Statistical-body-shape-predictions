function W = computeRegressor(meshModel, targetPoints, k)
% computeRegressor  Compute a sparse linear regressor from mesh vertices to target points
%
% For each target point, finds the k nearest mesh vertices and solves for
% interpolation weights such that W * meshModel approximates targetPoints.
%
% Inputs:
%   meshModel    : Nx3 source mesh vertices
%   targetPoints : Jx3 target points (joint centers, landmarks, or mesh vertices)
%   k            : number of nearest neighbours per target point
%
% Output:
%   W : JxN regressor matrix

    N = size(meshModel, 1);
    J = size(targetPoints, 1);
    W = zeros(J, N);

    for j = 1:J
        pt = targetPoints(j,:);
        dist = sum((meshModel - pt).^2, 2);
        [~, idx] = sort(dist);
        idx = idx(1:k);
        % V = meshModel(idx,:);
        % A = [V'; ones(1,k)];
        % b = [pt'; 1];
        % w = A \ b;

        d = dist(idx);
        if any(d == 0)
            w = double(d == 0);
        else
            w = 1 ./ d;
        end

        w = w / sum(w);
        W(j, idx) = w';
    end
end