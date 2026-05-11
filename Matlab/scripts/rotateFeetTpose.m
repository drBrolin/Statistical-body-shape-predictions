function [meshModel, adjustedLM, jointCenter] = rotateFeetTpose(meshModel, adjustedLM,jointCenter)
    r = meshRegions(); lm = lmIndices(); jc = jcIndices();
    % Rotate Feet to T-pose
    %   1. Isolate feet mesh coordinates, landmarks and joint centres in new matrices.
    %   2. Make ankle joint to origo
    %   3. Use toe and toe-tip joint to calulate rotation vector and rotate all arm coordinates
    %   into T-pose

    %%% FEET ROTATION %%%

    %%% LEFT FOOT %%%
    leftFootMesh = meshModel(r.leftFoot,:);
    leftFootJC = [jointCenter([jc.LtAnkle, jc.LtToe],:); jointCenter(jc.LtToeTip,:)];
    leftFootLM = adjustedLM(lm.LtMetatarsalPhalV:lm.LtDigitII,:);

    leftFootAngleTop = -1*(atan((leftFootJC(2,2)-leftFootJC(3,2))/(leftFootJC(2,1)-leftFootJC(3,1))));
    
    leftFootMeshCen = leftFootMesh-leftFootJC(1,:);
    leftFootJCCen = leftFootJC-leftFootJC(1,:);
    leftFootLMCen = leftFootLM-leftFootJC(1,:);

    A=[cos(leftFootAngleTop) -sin(leftFootAngleTop); sin(leftFootAngleTop) cos(leftFootAngleTop)]; % Rotate around Z to position foot straight out
    leftFootMeshCen_XY = [leftFootMeshCen(:,1),leftFootMeshCen(:,2)];
    leftFootMeshCen_XY_rot = A*leftFootMeshCen_XY';
    leftFootMeshCen_rot = [leftFootMeshCen_XY_rot',leftFootMeshCen(:,3)];

    leftFootJCCen_XY = [leftFootJCCen(:,1),leftFootJCCen(:,2)];
    leftFootJCCen_XY_rot = A*leftFootJCCen_XY';
    leftFootJCCen_rot = [leftFootJCCen_XY_rot',leftFootJCCen(:,3)];

    leftFootLMCen_XY = [leftFootLMCen(:,1),leftFootLMCen(:,2)];
    leftFootLMCen_XY_rot = A*leftFootLMCen_XY';
    leftFootLMCen_rot = [leftFootLMCen_XY_rot',leftFootLMCen(:,3)];

    leftFootMesh = leftFootMeshCen_rot+leftFootJC(1,:);
    leftFootJCCen = leftFootJCCen_rot+leftFootJC(1,:);
    leftFootLMCen = leftFootLMCen_rot+leftFootJC(1,:);

    meshModel(r.leftFoot,:) = leftFootMesh;
    jointCenter([jc.LtAnkle, jc.LtToe],:) = leftFootJCCen(1:2,:);
    jointCenter(jc.LtToeTip,:) = leftFootJCCen(3,:); % Toe-tip
    adjustedLM(lm.LtMetatarsalPhalV:lm.LtDigitII,:) = leftFootLMCen;

    %%% LOWER LEG ROTATION %%%   

    %%% LEFT LEG %%%
    leftLLMesh = meshModel(r.leftLowerLeg,:);
    maxDistanceToAnkle = sqrt(sum((leftFootJC(1,:) - adjustedLM(lm.LtKneeCrease,:)) .^ 2));
    belowKnee = leftLLMesh(:,3) < adjustedLM(lm.LtKneeCrease,3);
    distancesToAnkle = sqrt(sum((leftFootJC(1,:) - leftLLMesh) .^ 2, 2));
    lowerLegHeat = zeros(size(leftLLMesh,1),1);
    lowerLegHeat(belowKnee) = 1 - (distancesToAnkle(belowKnee) / maxDistanceToAnkle).^2;
    
    leftLLMeshCen = leftLLMesh-leftFootJC(1,:);
    leftLLMeshCen_XY = [leftLLMeshCen(:,1),leftLLMeshCen(:,2)];
    leftLLMeshCen_XY_rot = A*leftLLMeshCen_XY';
    leftLLMeshCen_XY_rotWeighted = leftLLMeshCen_XY_rot'.*lowerLegHeat+leftLLMeshCen_XY.*(1-lowerLegHeat);
    leftLLMeshCen_rot = [leftLLMeshCen_XY_rotWeighted,leftLLMeshCen(:,3)];
    leftLLMesh = leftLLMeshCen_rot+leftFootJC(1,:);
    meshModel(r.leftLowerLeg,:) = leftLLMesh;

    %% Could copy left to right side instead.

    %%% RIGHT FOOT %%%
    rightFootMesh = meshModel(r.rightFoot,:);
    rightFootJC = [jointCenter([jc.RtAnkle, jc.RtToe],:); jointCenter(jc.RtToeTip,:)];
    rightFootLM = adjustedLM(lm.RtMetatarsalPhalV:lm.RtDigitII,:);

    rightFootAngleTop = -1*(atan((rightFootJC(2,2)-rightFootJC(3,2))/(rightFootJC(2,1)-rightFootJC(3,1))));
    
    rightFootMeshCen = rightFootMesh-rightFootJC(1,:);
    rightFootJCCen = rightFootJC-rightFootJC(1,:);
    rightFootLMCen = rightFootLM-rightFootJC(1,:);

    A=[cos(rightFootAngleTop) -sin(rightFootAngleTop); sin(rightFootAngleTop) cos(rightFootAngleTop)]; % Rotate around Z to position foot straight out
    rightFootMeshCen_XY = [rightFootMeshCen(:,1),rightFootMeshCen(:,2)];
    rightFootMeshCen_XY_rot = A*rightFootMeshCen_XY';
    rightFootMeshCen_rot = [rightFootMeshCen_XY_rot',rightFootMeshCen(:,3)];

    rightFootJCCen_XY = [rightFootJCCen(:,1),rightFootJCCen(:,2)];
    rightFootJCCen_XY_rot = A*rightFootJCCen_XY';
    rightFootJCCen_rot = [rightFootJCCen_XY_rot',rightFootJCCen(:,3)];

    rightFootLMCen_XY = [rightFootLMCen(:,1),rightFootLMCen(:,2)];
    rightFootLMCen_XY_rot = A*rightFootLMCen_XY';
    rightFootLMCen_rot = [rightFootLMCen_XY_rot',rightFootLMCen(:,3)];

    rightFootMesh = rightFootMeshCen_rot+rightFootJC(1,:);
    rightFootJCCen = rightFootJCCen_rot+rightFootJC(1,:);
    rightFootLMCen = rightFootLMCen_rot+rightFootJC(1,:);

    meshModel(r.rightFoot,:) = rightFootMesh;
    jointCenter([jc.RtAnkle, jc.RtToe],:) = rightFootJCCen(1:2,:);
    jointCenter(jc.RtToeTip,:) = rightFootJCCen(3,:); % Toe-tip
    adjustedLM(lm.RtMetatarsalPhalV:lm.RtDigitII,:) = rightFootLMCen;
    
    %%% LOWER LEG ROTATION %%%  

    %%% RIGHT LEG %%%
    rightLLMesh = meshModel(r.rightLowerLeg,:);
    maxDistanceToAnkle = sqrt(sum((rightFootJC(1,:) - adjustedLM(lm.RtKneeCrease,:)) .^ 2));
    belowKnee = rightLLMesh(:,3) < adjustedLM(lm.RtKneeCrease,3);
    distancesToAnkle = sqrt(sum((rightFootJC(1,:) - rightLLMesh) .^ 2, 2));
    lowerLegHeat = zeros(size(rightLLMesh,1),1);
    lowerLegHeat(belowKnee) = 1 - (distancesToAnkle(belowKnee) / maxDistanceToAnkle).^2;
    
    rightLLMeshCen = rightLLMesh-rightFootJC(1,:);
    rightLLMeshCen_XY = [rightLLMeshCen(:,1),rightLLMeshCen(:,2)];
    rightLLMeshCen_XY_rot = A*rightLLMeshCen_XY';
    rightLLMeshCen_XY_rotWeighted = rightLLMeshCen_XY_rot'.*lowerLegHeat+rightLLMeshCen_XY.*(1-lowerLegHeat);
    rightLLMeshCen_rot = [rightLLMeshCen_XY_rotWeighted,rightLLMeshCen(:,3)];
    rightLLMesh = rightLLMeshCen_rot+rightFootJC(1,:);
    meshModel(r.rightLowerLeg,:) = rightLLMesh;

    % plot3(meshModel(r.rightFoot,1),meshModel(r.rightFoot,2),meshModel(r.rightFoot,3),'ko','MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',1); hold on;
    % figure(1);
    % grid;
    % xlabel('X'); ylabel('Y'); zlabel('Z')
    % axis equal; 
    
    %%% RIGHT FOOT TOE LIFTING %%%
    meshFootRight = meshModel(r.rightFoot,:);
    toePoints = load('insideFootRight.mat').rightToes;  % (N_feet x 5): logical mask per toe
    toePoints = toePoints(numel(r.leftFoot)+1:end, :);  % trim to right foot rows only

    toeMask       = any(toePoints, 2);
    footMask      = ~toeMask;
    toeMeshOrig   = meshFootRight(toeMask, :);
    footVertsOrig = meshFootRight(footMask, :);

    % Step 2: Staged lifting — align all toes to the same ground level
    toeMinZ = inf(5, 1);
    for n = 1:size(meshFootRight,1)
        for i = 1:5
            if toePoints(n,i) && meshFootRight(n,3) < toeMinZ(i)
                toeMinZ(i) = meshFootRight(n,3);
            end
        end
    end
    [sortedZ, sortOrder] = sort(toeMinZ);
    for step = 1:4
        toesInGroup = sortOrder(1:step);
        liftAmount  = sortedZ(step+1) - sortedZ(step);
        for n = 1:size(meshFootRight,1)
            if any(toePoints(n, toesInGroup))
                meshFootRight(n,3) = meshFootRight(n,3) + liftAmount;
            end
        end
        sortedZ(1:step) = sortedZ(1:step) + liftAmount;
    end

    % Step 3: IDW blend — propagate toe lift to nearby foot vertices (XYZ)
    toeMeshLifted = meshFootRight(toeMask, :);
    toeLift       = toeMeshLifted - toeMeshOrig;  % Nx3

    k = 10;
    [nearIdx, nearDist] = knnsearch(toeMeshOrig, footVertsOrig, 'K', k);

    w = 1 ./ (nearDist.^2 + eps);
    w = w ./ sum(w, 2);

    footLiftPredicted = zeros(size(footVertsOrig));
    for a = 3:3 %for a = 1:3
        toeLiftA = toeLift(:, a);
        footLiftPredicted(:, a) = sum(w .* toeLiftA(nearIdx), 2);
    end

    distToNearest = nearDist(:, 1);
    blendWeight   = exp(-distToNearest / median(distToNearest));
    meshFootRight(footMask, :) = footVertsOrig + blendWeight .* footLiftPredicted;

    % Ground clipping with smooth neighborhood propagation
    belowMask = meshFootRight(:, 3) < 0;
    if any(belowMask)
        belowVerts = meshFootRight(belowMask, :);
        belowLift  = -belowVerts(:, 3);

        meshFootRight(belowMask, 3) = 0;

        aboveMask  = ~belowMask;
        aboveVerts = meshFootRight(aboveMask, :);

        k = 8;
        [nearIdx, nearDist] = knnsearch(belowVerts, aboveVerts, 'K', min(k, sum(belowMask)));

        w = 1 ./ (nearDist.^2 + eps);
        w = w ./ sum(w, 2);

        liftPredicted = sum(w .* belowLift(nearIdx), 2);

        distToNearest = nearDist(:, 1);
        blendWeight   = exp(-distToNearest / median(distToNearest));

        meshFootRight(aboveMask, 3) = aboveVerts(:, 3) + blendWeight .* liftPredicted;
    end

    meshModel(r.rightFoot,:) = meshFootRight;

    % Copy right foot toe lifting to left foot via symmetry pairs
    pairs = readmatrix('statBodyModel/meshSymmetryPairs.txt', 'FileType', 'text');
    footPairMask = ismember(pairs(:,1), r.rightFoot) | ismember(pairs(:,2), r.rightFoot);
    footPairs = pairs(footPairMask, :);
    for i = 1:size(footPairs, 1)
        if meshModel(footPairs(i,1), 2) < 0
            meshModel(footPairs(i,2), 1) =  meshModel(footPairs(i,1), 1);
            meshModel(footPairs(i,2), 2) = -meshModel(footPairs(i,1), 2);
            meshModel(footPairs(i,2), 3) =  meshModel(footPairs(i,1), 3);
        elseif meshModel(footPairs(i,2), 2) < 0
            meshModel(footPairs(i,1), 1) =  meshModel(footPairs(i,2), 1);
            meshModel(footPairs(i,1), 2) = -meshModel(footPairs(i,2), 2);
            meshModel(footPairs(i,1), 3) =  meshModel(footPairs(i,2), 3);
        end
    end
end