function meshModel = meshSymmetry(meshModel, adjustedLM)
    r = meshRegions(); lm = lmIndices();
    pairs = readmatrix('statBodyModel/meshSymmetryPairs.txt','FileType','text');
    % Check for each point in mesh, find its pair to get the average value and assign that to both points.
    for i=1:size(pairs,1)
        meshModel(pairs(i,1),1) = (meshModel(pairs(i,1),1)+meshModel(pairs(i,2),1))/2; % average value of x-coordinate
        meshModel(pairs(i,1),2) = (meshModel(pairs(i,1),2)-meshModel(pairs(i,2),2))/2; % average value of y-coordinate, minus since left and right side have opposite values
        meshModel(pairs(i,1),3) = (meshModel(pairs(i,1),3)+meshModel(pairs(i,2),3))/2; % average value of z-coordinate
        meshModel(pairs(i,2),1) = meshModel(pairs(i,1),1); % set the left and right side to the same x-coordinate
        meshModel(pairs(i,2),2) = meshModel(pairs(i,1),2)*-1; % set the left and right side to different y-coordinate
        meshModel(pairs(i,2),3) = meshModel(pairs(i,1),3); % set the left and right side to the same z-coordinate
    end

    %% Fix meshpoints on midline between waist and cervicale. 
    %% For each point that are on midline, find closest symmetrical points and if X coord of midpoint is behind symmetrical X coord adjust X coord of midpoint in front equally much.
    midlineIdx = find(pairs(:,1) == pairs(:,2));
    pairsNoMid = find(pairs(:,1) ~= pairs(:,2));
    P1 = meshModel(pairs(pairsNoMid(:,1),1),:); % M x 3
%     M1 = meshModel(pairs(midlineIdx(:,1),1),:); % M x 3
    for k = 1:length(midlineIdx)
        i = pairs(midlineIdx(k),1);
        % Find meshpoints in back midline between waist and cervicale. 
        if meshModel(i,3) > adjustedLM(lm.WaistPreferredPost,3) && meshModel(i,3) < adjustedLM(lm.Suprasternale,3) && meshModel(i,1) < adjustedLM(lm.Cervicale,1)
%             plot3(meshModel(i,1),meshModel(i,2),meshModel(i,3),'mo','MarkerEdgeColor','m','MarkerFaceColor','m','MarkerSize',3); hold on;
            [~,j] = min(vecnorm(P1 - meshModel(i,:), 2, 2));
%             plot3(P1(j,1),P1(j,2),P1(j,3),'ro','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',3); hold on;
%             dM = vecnorm(M1 - meshModel(i,:), 2, 2);
%             [~,m] = min(dM(dM ~= 0));
            if P1(j,1) > meshModel(i,1)
                meshModel(i,1) = P1(j,1)*2-meshModel(i,1);
%                 meshModel(i,1) = P1(j,1);
            elseif P1(j,1) < meshModel(i,1)
                meshModel(i,1) = meshModel(i,1)-(meshModel(i,1)-P1(j,1))/2;
%                 meshModel(i,1) = P1(j,1);
            end
        end
    end
    %% Fix meshpoints on thigh that are on the other side of the midline. Find them and put on correct side of midline -1 mm
    meshRightLeg = readmatrix('statBodyModel/meshRightLeg.txt','FileType','text');
    meshStart = r.pelvisLegs(1);
    for i=1:size(meshRightLeg,1)
        if meshRightLeg(i) && meshModel(i+meshStart-1,2) > 0
%             plot3(meshModel(i+meshStart-1,1),meshModel(i+meshStart-1,2),meshModel(i+meshStart-1,3),'mo','MarkerEdgeColor','m','MarkerFaceColor','m','MarkerSize',2); hold on;
            meshModel(i+meshStart-1,2) = -1;
%             disp(i+meshStart-1);
            [pointRow,pointCol] = find(pairs == i+meshStart-1);
%             disp(pointRow);
            if pointCol == 1
                leftMesh = pairs(pointRow,2);
            else
                leftMesh = pairs(pointRow,1);
%                  disp('Second column!');
            end
%             plot3(meshModel(leftMesh,1),meshModel(leftMesh,2),meshModel(leftMesh,3),'ro','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',2); hold on;
            meshModel(leftMesh,2) = 1;            
        end
    end
%     figure(1);
%     grid;
%     xlabel('X'); ylabel('Y'); zlabel('Z')
%     axis equal; 
%     test = 1;

end