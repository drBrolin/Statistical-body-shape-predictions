function meshModel = reduceCrotch(meshModel, adjustedLM)
    r = meshRegions(); lm = lmIndices();
    % Reduce anterior bulge in the crotch/groin area.
    %   Plane defined by the Crotch, LtASIS and RtASIS landmarks. Only vertices
    %   whose projection onto that plane falls inside the triangle spanned by
    %   the three landmarks are considered. Bulge is the perpendicular distance
    %   from the plane, measured along its normal (oriented anteriorly, +X).
    %   Vertices in front of the plane are pulled back along the normal, with
    %   the largest correction at the most extreme bulge point and progressively
    %   less for points further from it. The correction is scaled so the most
    %   extreme point ends up at the targetPercentile bulge distance of the affected
    %   vertices rather than flat on the plane.

    p1 = adjustedLM(lm.Crotch,:);
    p2 = adjustedLM(lm.LtASIS,:);
    p3 = adjustedLM(lm.RtASIS,:);

    normal = cross(p2-p1, p3-p1);
    normal = normal / norm(normal);
    if normal(1) < 0
        normal = -normal; % keep normal pointing in the +X (anterior) direction
    end

    % Constant terms for the barycentric inside-triangle test
    v0 = p2-p1;
    v1 = p3-p1;
    d00 = dot(v0,v0);
    d01 = dot(v0,v1);
    d11 = dot(v1,v1);
    denom = d00*d11 - d01*d01;

    bulgePoints = zeros(numel(r.pelvisLegs),2); % [vertex index, perpendicular distance from plane]
    idx = 0;
    for n=r.pelvisLegs
        perpDist = dot(meshModel(n,:) - p1, normal);
        if perpDist > 0
            v2 = (meshModel(n,:) - perpDist*normal) - p1; % vertex projected onto the plane, relative to p1
            d20 = dot(v2,v0);
            d21 = dot(v2,v1);
            b2 = (d11*d20 - d01*d21) / denom;
            b3 = (d00*d21 - d01*d20) / denom;
            if b2 >= 0 && b3 >= 0 && (b2+b3) <= 1 % inside the landmark triangle
                idx = idx+1;
                bulgePoints(idx,:) = [n perpDist];
            end
        end
    end
    bulgePoints = bulgePoints(1:idx,:);

    if isempty(bulgePoints)
        return;
    end

    targetPercentile = 60; % tuning knob: higher percentile = gentler correction
    targetPerpDist = prctile(bulgePoints(:,2), targetPercentile);

    [maxPerpDist, worstRow] = max(bulgePoints(:,2));
    worstPoint = meshModel(bulgePoints(worstRow,1),:);
    scale = (maxPerpDist - targetPerpDist) / maxPerpDist; % softens the correction so the worst point lands at the target bulge distance

    distToWorst = zeros(size(bulgePoints,1),1);
    for i=1:size(bulgePoints,1)
        distToWorst(i) = sqrt(sum((meshModel(bulgePoints(i,1),:) - worstPoint) .^ 2));
    end
    maxDistToWorst = max(distToWorst);

    for i=1:size(bulgePoints,1)
        n = bulgePoints(i,1);
        if maxDistToWorst > 0
            weight = 1 - (distToWorst(i)/maxDistToWorst)^2;
        else
            weight = 1;
        end
        meshModel(n,:) = meshModel(n,:) - bulgePoints(i,2)*weight*scale*normal; % pull bulge back along the plane normal, scaled by distance from worst point
    end
end
