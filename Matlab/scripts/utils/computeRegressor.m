function R = computeRegressor(meshModel, targetPoints, k)
% computeRegressor  Compute a sparse relation from mesh vertices to target points
%
% For each target point the k nearest mesh vertices are stored together with
% an inverse-distance weight AND the offset vector from the vertex to the
% target point. Storing the offset is what lets the relation represent points
% that do not lie on the mesh surface (joint centres inside the body, a target
% mesh with a different topology, ...). A pure weighted average of vertices,
% as used before, always collapses such points onto the surface hull.
%
% This is the same "distanceMeas" layout used by saveMeshTpose /
% selectedMeshMorph, so the result can be saved and reused the same way.
%
% Inputs:
%   meshModel    : Nx3 source (reference) mesh vertices
%   targetPoints : Jx3 target points (joint centres, landmarks, mesh vertices)
%   k            : number of nearest neighbours per target point
%
% Output:
%   R : J x 5*k relation matrix. For neighbour n = 1..k:
%         R(:,(n-1)*5+1)     vertex index into meshModel
%         R(:,(n-1)*5+2)     weight (1/distance)
%         R(:,(n-1)*5+3:5)   offset: targetPoint - meshModel(index,:)
%
% Reconstruct with applyRegressor(R, deformedMesh). Units of meshModel and
% targetPoints must match, and the mesh passed to applyRegressor must use the
% same vertex ordering and units as meshModel here.
%
% See also applyRegressor, saveMeshTpose

    N = size(meshModel, 1);
    J = size(targetPoints, 1);
    k = min(k, N);

    R = zeros(J, 5*k);

    for j = 1:J
        pt = targetPoints(j,:);
        diff = pt - meshModel;                 % Nx3, offsets vertex -> target
        dist = sqrt(sum(diff.^2, 2));

        [dSort, idx] = sort(dist);
        idx = idx(1:k);
        d = dSort(1:k);

        % A target sitting exactly on a vertex would give an infinite weight;
        % clamp so the weight stays finite and dominates instead.
        d = max(d, eps(class(d)));
        w = 1 ./ d;

        for n = 1:k
            R(j,(n-1)*5+1:(n-1)*5+5) = [idx(n) w(n) diff(idx(n),:)];
        end
    end
end
