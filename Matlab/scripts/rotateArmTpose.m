function [meshModel, adjustedLM, jointCenter] = rotateArmTpose(meshModel, adjustedLM, jointCenter)
    r = meshRegions(); lm = lmIndices(); jc = jcIndices();
    % Rotate arms to T-pose
    %   1. Isolate arm mesh coordinates, landmarks and joint centres in new matrices.
    %   2. Make gleno-humeral (shoulder) joint to origo
    %   3. Use wrist joint to calulate rotation vector and rotate all arm coordinates into T-pose

    pairs = readmatrix('statBodyModel/meshSymmetryPairs.txt','FileType','text');

    %%% RIGHT ARM ROTATION %%%

    % To:From - 9663:11390	right arm and hand
    filePre = 'rightArmPoints';
    filePost = '.txt';
    fileName = [filePre filePost];
    rightArmPoints = readmatrix(fileName,'FileType','text','Delimiter',',');
    
    % Landmarks: 29, 30, 32, 11 (for rotation)
    acromionRightStart = adjustedLM(lm.RtAcromion,:);
    midPointArmPit = (adjustedLM(lm.RtAxillaAnt,:)+adjustedLM(lm.RtAxillaPost,:))/2;
    radiusSphere = sqrt(sum((adjustedLM(lm.RtAcromion,:) - midPointArmPit) .^ 2));
    acromionHeat = zeros(numel(r.torso),6);
    for n=r.torso
        if meshModel(n,2)<0 && meshModel(n,2)>adjustedLM(lm.RtAcromion,2)
            if ~ismember(n, rightArmPoints)
                distanceToAcromion = sqrt(sum((adjustedLM(lm.RtAcromion,:) - meshModel(n,:)) .^ 2));
                if distanceToAcromion < radiusSphere
                    acromionHeat(n-4678+1,:) = [n 1 1-(distanceToAcromion/radiusSphere)^2 meshModel(n,:)];
                end
%                 plot3(meshModel(n,1),meshModel(n,2),meshModel(n,3),'bo','MarkerEdgeColor','b','MarkerFaceColor','b','MarkerSize',2); hold on;
            end
        end
    end
    angAcrAlpha1 = -1*(atan((adjustedLM(lm.Suprasternale,2)-adjustedLM(lm.RtAcromion,2))/(adjustedLM(lm.Suprasternale,3)-adjustedLM(lm.RtAcromion,3)))); % Start angle for acromion
    
%     % ---------- Save debug data
%     test = acromionHeat(any(acromionHeat,2),:);
%     test = test(:,4:6);
%     %dlmwrite('debug/acromionHeatTest1.csv', test, 'precision', 15);
%     idx = acromionHeat(any(acromionHeat(:, 1), 2))' - 1;
%     %dlmwrite('debug/acromionHeatSavedIdxRight.csv', idx, 'precision', 10);
%     % ----------------------------

    armPointsExtra = zeros(numel(r.torso),2);
    for n=r.torso
        if meshModel(n,2)<0 && meshModel(n,2)<adjustedLM(lm.RtAcromion,2)
            if ~ismember(n, rightArmPoints)
                distanceToAcromion = sqrt(sum((adjustedLM(lm.RtAcromion,:) - meshModel(n,:)) .^ 2));
                if distanceToAcromion < radiusSphere
                    armPointsExtra(n-4678+1,1) = n;
                    armPointsExtra(n-4678+1,2) = distanceToAcromion;
                end
%                 plot3(meshModel(n,1),meshModel(n,2),meshModel(n,3),'bo','MarkerEdgeColor','b','MarkerFaceColor','b','MarkerSize',2); hold on;
            end
        end
    end
    armPointsAdd = armPointsExtra(any(armPointsExtra,2),:);
    allArmPoints = [rightArmPoints; armPointsAdd(:,1)];
    rightArmPoints = sort(allArmPoints);

%     % ---------- Save debug data
%     idx = armPointsAdd(:,1)' - 1;
%     dlmwrite('debug/armPointsAddRight.csv', idx, 'precision', 10);
%     % ----------------------------

    % if ~ismember(n, rightArmPoints) && ~ismember(n, acromionHeat)
    armPitPoints = zeros(numel(r.torso)+numel(r.leftNipple)+numel(r.rightNipple)+numel(r.abdomenDepth),3);
    for n=r.torso
        if meshModel(n,2)<0
            if ~ismember(n, rightArmPoints)
                distanceToMidPointArmPit = sqrt(sum((midPointArmPit - meshModel(n,:)) .^ 2));
                distanceToAcromion = sqrt(sum((adjustedLM(lm.RtAcromion,:) - meshModel(n,:)) .^ 2));
                armPitPoints(n-4678+1,1) = n;
                armPitPoints(n-4678+1,2) = distanceToMidPointArmPit;
                armPitPoints(n-4678+1,3) = distanceToAcromion;
%                 plot3(meshModel(n,1),meshModel(n,2),meshModel(n,3),'bo','MarkerEdgeColor','b','MarkerFaceColor','b','MarkerSize',2); hold on;
            end
        end
    end
    for n=[r.leftNipple, r.rightNipple, r.abdomenDepth] % Bust points and maximum abdominal depth
        if meshModel(n,2)<0
            if ~ismember(n, rightArmPoints)
                distanceToMidPointArmPit = sqrt(sum((midPointArmPit - meshModel(n,:)) .^ 2));
                distanceToAcromion = sqrt(sum((adjustedLM(lm.RtAcromion,:) - meshModel(n,:)) .^ 2));
                armPitPoints(n-r.leftNipple(1)+1+numel(r.torso),1) = n;
                armPitPoints(n-r.leftNipple(1)+1+numel(r.torso),2) = distanceToMidPointArmPit;
                armPitPoints(n-r.leftNipple(1)+1+numel(r.torso),3) = distanceToAcromion;
            end
        end
    end
    armPitPoints = armPitPoints(any(armPitPoints,2),:);
%     plot3(meshModel(armPitPoints(:,1),1),meshModel(armPitPoints(:,1),2),meshModel(armPitPoints(:,1),3),'bo','MarkerEdgeColor','b','MarkerFaceColor','b','MarkerSize',2); hold on;


%     % ---------- Save debug data
%     idx = armPitPoints(:, 1)' - 1;
%     dlmwrite('debug/armPitPointsRight.csv', idx, 'precision', 10);
%     % ----------------------------

    rightArmMesh = zeros(size(rightArmPoints,1),3);
    for n=1:size(rightArmPoints,1)
        rightArmMesh(n,1:3) = meshModel(rightArmPoints(n),1:3);
    end
%     plot3(rightArmMesh(:,1),rightArmMesh(:,2),rightArmMesh(:,3),'mo','MarkerEdgeColor','m','MarkerFaceColor','m','MarkerSize',1); hold on;

    % Acromion (landmark RtAcromion) relation to AC (RtAcromioClavicular) and GH (RtShoulder) joints
    distance = sqrt((jointCenter(jc.RtAcromioClavicular,2)-adjustedLM(lm.RtAcromion,2))^2+(jointCenter(jc.RtAcromioClavicular,3)-adjustedLM(lm.RtAcromion,3))^2);
    angGamma1 = -1*(atan((jointCenter(jc.RtAcromioClavicular,2)-adjustedLM(lm.RtAcromion,2))/(jointCenter(jc.RtAcromioClavicular,3)-adjustedLM(lm.RtAcromion,3))));
    angAlpha1 = -1*(atan((jointCenter(jc.RtAcromioClavicular,2)-jointCenter(jc.RtShoulder,2))/(jointCenter(jc.RtAcromioClavicular,3)-jointCenter(jc.RtShoulder,3))));
    angBeta = angGamma1-angAlpha1;

    % Set which point to rotate 
    rightArmJC = [jointCenter(jc.RtShoulder:jc.RtWrist,:); jointCenter(jc.RtIndexCarpal:jc.RtThumbDist,:)]; % Around GH the first time
    rightArmLM = [adjustedLM(lm.RtRadialStyloid,:); adjustedLM(lm.RtOlecranon:lm.RtMetacarpalPhalV,:)];
    rightArmMeshCen = rightArmMesh-rightArmJC(1,:);
    rightArmJCCen = rightArmJC-rightArmJC(1,:);
    rightArmLMCen = rightArmLM-rightArmJC(1,:);

    v_Z = atan((rightArmJCCen(3,1)-rightArmJCCen(1,1))/(rightArmJCCen(3,2)-rightArmJCCen(1,2)))+deg2rad(5); % rightArmJCCen(1,:) = 0;
    v_X = -1*(atan((rightArmJCCen(3,3)-rightArmJCCen(1,3))/(rightArmJCCen(3,2)-rightArmJCCen(1,2))));
    A=[cos(v_Z) -sin(v_Z); sin(v_Z) cos(v_Z)]; % Rotate around Z to position arms straight out
    rightArmMeshCen_XY = [rightArmMeshCen(:,1),rightArmMeshCen(:,2)];
    rightArmMeshCen_XY_rot = A*rightArmMeshCen_XY';
    rightArmMeshCen_rot = [rightArmMeshCen_XY_rot',rightArmMeshCen(:,3)];

    rightArmJCCen_XY = [rightArmJCCen(:,1),rightArmJCCen(:,2)];
    rightArmJCCen_XY_rot = A*rightArmJCCen_XY';
    rightArmJCCen_rot = [rightArmJCCen_XY_rot',rightArmJCCen(:,3)];

    rightArmLMCen_XY = [rightArmLMCen(:,1),rightArmLMCen(:,2)];
    rightArmLMCen_XY_rot = A*rightArmLMCen_XY';
    rightArmLMCen_rot = [rightArmLMCen_XY_rot',rightArmLMCen(:,3)];
    v_XDeg = 90+rad2deg(v_X);
    v_XSC_Start = -0.00001*v_XDeg^3 + 0.00185*v_XDeg^2 + 0.1*v_XDeg + 90; % SC angle at start position
    v_XSC_90 = -0.00001*90^3 + 0.00185*90^2 + 0.1*90 + 90; % SC angle at 90 degree abduction = T-pose
    v_XSCDeg = v_XSC_90-v_XSC_Start; % How much the SC joint will rotate and contribute to abduction
    v_XAC_GH = deg2rad(v_XSCDeg+rad2deg(v_X));

    v_XGH = v_XAC_GH*2/3;
    A=[cos(v_XGH) -sin(v_XGH); sin(v_XGH) cos(v_XGH)];
    rightArmMeshCen_YZ = [rightArmMeshCen_rot(:,2),rightArmMeshCen_rot(:,3)];
    rightArmMeshCen_YZ_rot = A*rightArmMeshCen_YZ';
    rightArmMeshCen_rot2 = [rightArmMeshCen_rot(:,1), rightArmMeshCen_YZ_rot'];

    rightArmJCCen_YZ = [rightArmJCCen_rot(:,2),rightArmJCCen_rot(:,3)];
    rightArmJCCen_YZ_rot = A*rightArmJCCen_YZ';
    rightArmJCCen_rot2 = [rightArmJCCen_rot(:,1), rightArmJCCen_YZ_rot'];

    rightArmLMCen_YZ = [rightArmLMCen_rot(:,2),rightArmLMCen_rot(:,3)];
    rightArmLMCen_YZ_rot = A*rightArmLMCen_YZ';
    rightArmLMCen_rot2 = [rightArmLMCen_rot(:,1), rightArmLMCen_YZ_rot'];

    rightArmMeshRot = rightArmMeshCen_rot2+rightArmJC(1,:);
    rightArmJCRot = rightArmJCCen_rot2+rightArmJC(1,:);
    rightArmLMRot = rightArmLMCen_rot2+rightArmJC(1,:);

    %%% AcromioClavicular joint rotation 1/3 of total abduction
    rightArmMeshCen = rightArmMeshRot-jointCenter(jc.RtAcromioClavicular,:);
    rightArmJCCen = rightArmJCRot-jointCenter(jc.RtAcromioClavicular,:);
    rightArmLMCen = rightArmLMRot-jointCenter(jc.RtAcromioClavicular,:);

    v_XAC = v_XAC_GH*1/3;
    A=[cos(v_XAC) -sin(v_XAC); sin(v_XAC) cos(v_XAC)];
    rightArmMeshCen_AC = [rightArmMeshCen(:,2),rightArmMeshCen(:,3)];
    rightArmMeshCen_AC_rot = A*rightArmMeshCen_AC';
    rightArmMeshCen_rot = [rightArmMeshCen(:,1), rightArmMeshCen_AC_rot'];

    rightArmJCCen_AC = [rightArmJCCen(:,2),rightArmJCCen(:,3)];
    rightArmJCCen_AC_rot = A*rightArmJCCen_AC';
    rightArmJCCen_rot = [rightArmJCCen(:,1), rightArmJCCen_AC_rot'];

    rightArmLMCen_AC = [rightArmLMCen(:,2),rightArmLMCen(:,3)];
    rightArmLMCen_AC_rot = A*rightArmLMCen_AC';
    rightArmLMCen_rot = [rightArmLMCen(:,1), rightArmLMCen_AC_rot'];

    rightArmMeshRot = rightArmMeshCen_rot+jointCenter(jc.RtAcromioClavicular,:);
    rightArmJCRot = rightArmJCCen_rot+jointCenter(jc.RtAcromioClavicular,:);
    rightArmLMRot = rightArmLMCen_rot+jointCenter(jc.RtAcromioClavicular,:);

    %%% SternoClavicular joint rotation in relation to total abduction
    rightArmJC = [jointCenter(jc.RtAcromioClavicular,:); rightArmJCRot];
    rightArmMeshCen = rightArmMeshRot-jointCenter(jc.RtSternoClavicular,:);
    rightArmJCCen = rightArmJC-jointCenter(jc.RtSternoClavicular,:);
    rightArmLMCen = rightArmLMRot-jointCenter(jc.RtSternoClavicular,:);

    v_XSC = deg2rad(-v_XSCDeg);
    A=[cos(v_XSC) -sin(v_XSC); sin(v_XSC) cos(v_XSC)];
    rightArmMeshCen_SC = [rightArmMeshCen(:,2),rightArmMeshCen(:,3)];
    rightArmMeshCen_SC_rot = A*rightArmMeshCen_SC';
    rightArmMeshCen_rot = [rightArmMeshCen(:,1), rightArmMeshCen_SC_rot'];

    rightArmJCCen_SC = [rightArmJCCen(:,2),rightArmJCCen(:,3)];
    rightArmJCCen_SC_rot = A*rightArmJCCen_SC';
    rightArmJCCen_rot = [rightArmJCCen(:,1), rightArmJCCen_SC_rot'];

    rightArmLMCen_SC = [rightArmLMCen(:,2),rightArmLMCen(:,3)];
    rightArmLMCen_SC_rot = A*rightArmLMCen_SC';
    rightArmLMCen_rot = [rightArmLMCen(:,1), rightArmLMCen_SC_rot'];

    rightArmMeshRot = rightArmMeshCen_rot+jointCenter(jc.RtSternoClavicular,:);
    rightArmJCRot = rightArmJCCen_rot+jointCenter(jc.RtSternoClavicular,:);
    rightArmLMRot = rightArmLMCen_rot+jointCenter(jc.RtSternoClavicular,:);

    for n=1:size(rightArmPoints,1)
        meshModel(rightArmPoints(n),1:3) = rightArmMeshRot(n,1:3);
    end

%     dlmwrite('debug/meshmodelTest1.csv', meshModel, 'precision', 15); % <-------

    jointCenter(jc.RtAcromioClavicular:jc.RtWrist,:) = rightArmJCRot(1:4,:);
    jointCenter(jc.RtIndexCarpal:jc.RtThumbDist,:) = rightArmJCRot(5:24,:);
    adjustedLM(lm.RtRadialStyloid,:) = rightArmLMRot(1,:);
    adjustedLM(lm.RtOlecranon:lm.RtMetacarpalPhalV,:) = rightArmLMRot(2:9,:);

    % Rotation of acromion
    angAlpha2 = -1*(atan((jointCenter(jc.RtAcromioClavicular,2)-jointCenter(jc.RtShoulder,2))/(jointCenter(jc.RtAcromioClavicular,3)-jointCenter(jc.RtShoulder,3))));
    angGamma2 = angBeta+angAlpha2;
    A=[cos(angGamma2) -sin(angGamma2); sin(angGamma2) cos(angGamma2)];
    acromionLM = [distance,0];
    acromionLM_rot = A*acromionLM';
    adjustedLM(lm.RtAcromion,2) = jointCenter(jc.RtAcromioClavicular,2)+acromionLM_rot(2);
    adjustedLM(lm.RtAcromion,3) = jointCenter(jc.RtAcromioClavicular,3)-acromionLM_rot(1);

    % Rotation of meshpoints close to acromion
    angAcrAlpha2 = -1*(atan((adjustedLM(lm.Suprasternale,2)-adjustedLM(lm.RtAcromion,2))/(adjustedLM(lm.Suprasternale,3)-adjustedLM(lm.RtAcromion,3))));
    angDelta = angAcrAlpha2-angAcrAlpha1;
    acromionHeatMesh = acromionHeat(any(acromionHeat,2),:);
    acromionHeatMeshCord = acromionHeatMesh(:,4:6)-adjustedLM(lm.Suprasternale,:);
    A=[cos(angDelta) -sin(angDelta); sin(angDelta) cos(angDelta)];
    acromionHeatMeshAcr = [acromionHeatMeshCord(:,2),acromionHeatMeshCord(:,3)];
    acromionHeatMeshAcr_rot = A*acromionHeatMeshAcr';
    acromionHeatMesh_rot = [acromionHeatMeshCord(:,1), acromionHeatMeshAcr_rot'];

    acromionHeatMeshCord = acromionHeatMesh_rot+adjustedLM(lm.Suprasternale,:);
    acromionHeatMeshWeight = (acromionHeatMeshCord-acromionHeatMesh(:,4:6)).*acromionHeatMesh(:,3)+acromionHeatMesh(:,4:6);

% 	dlmwrite('debug/acromionHeatMeshCordTest2.csv', acromionHeatMeshCord, 'precision', 15);
%     test = acromionHeat(any(acromionHeat,2),:);
%     test = test(:,4:6);
%     dlmwrite('debug/acromionHeatMeshTest2.csv', test, 'precision', 15);
%     dlmwrite('debug/acromionHeatMeshWeightTest2.csv', acromionHeatMeshWeight, 'precision', 15);																   
	
    for n=1:size(acromionHeatMesh,1)
        meshModel(acromionHeatMesh(n,1),1:3) = acromionHeatMeshWeight(n,1:3);
    end
% 	dlmwrite('debug/meshmodelTest2.csv', meshModel, 'precision', 15); % <-------																				  

    % Fix the extra arm points
    armPointsAddMesh = zeros(size(armPointsAdd,1),3);
    for n=1:size(armPointsAdd,1)
        armPointsAddMesh(n,1:3) = meshModel(armPointsAdd(n),1:3);
    end
    armPointsAddWeight = 1-(armPointsAdd(:,2)/max(armPointsAdd(:,2))).^2;
    armPointsAddMeshWeight = (adjustedLM(lm.RtAcromion,3)-armPointsAddMesh(:,3)).*armPointsAddWeight+armPointsAddMesh(:,3);
    for n=1:size(armPointsAdd,1)
        meshModel(armPointsAdd(n),3) = armPointsAddMeshWeight(n);
        armPointsAddMesh(n,3) = armPointsAddMeshWeight(n);
        if adjustedLM(lm.RtAcromion,2) < armPointsAddMesh(n,2)
            meshModel(armPointsAdd(n),2) = adjustedLM(lm.RtAcromion,2);
            armPointsAddMesh(n,2) = meshModel(armPointsAdd(n),2);
        end
    end
	%   dlmwrite('debug/meshmodelTest3.csv', meshModel, 'precision', 15); % <-------																				  

    % Fix the extra torso points
    armPitPointsMesh = zeros(size(armPitPoints,1),3);
    for n=1:size(armPitPoints,1)
        armPitPointsMesh(n,1:3) = meshModel(armPitPoints(n,1),1:3);
    end
    armPitDiffZ =  abs(midPointArmPit(3)-armPitPointsMesh(:,3));
    armPitPointsDist = 1-(armPitPoints(:,2)/max(armPitPoints(:,2))).^2;
    armPitPointsAcrDist = 1-(armPitPoints(:,3).^-1/max(armPitPoints(:,3).^-1));
    armPitPointsDiffZ = 1-(armPitDiffZ/max(armPitDiffZ)).^2;
    armPitPointsWeight = (armPitPointsDist.*armPitPointsDiffZ).^4.*armPitPointsAcrDist;
    acromionRaise = adjustedLM(lm.RtAcromion,:)-acromionRightStart;
    adjustedLM(lm.RtAxillaAnt,3) = adjustedLM(lm.RtAxillaAnt,3)+acromionRaise(3);
    adjustedLM(lm.RtAxillaPost,3) = adjustedLM(lm.RtAxillaPost,3)+acromionRaise(3);
    armPitPointsMeshWeight = acromionRaise(:,3).*armPitPointsWeight+armPitPointsMesh(:,3);
    for n=1:size(armPitPoints,1)
        meshModel(armPitPoints(n),3) = armPitPointsMeshWeight(n);
        armPitPointsMesh(n,3) = armPitPointsMeshWeight(n);
    end

	%   dlmwrite('debug/meshmodelTest4.csv', meshModel, 'precision', 15); % <-------
    %   dlmwrite('debug/ioLandmarksTest4.csv', adjustedLM, 'precision', 15); % <-------
     %   dlmwrite('debug/ioJointcenterTest4.csv', jointCenter, 'precision', 15); % <-------

    																				  
%%% LEFT ARM ROTATION  %%%
   
    % Copy JCs and LMs from right to left side
    jointCenter(jc.LtAcromioClavicular:jc.LtThumbDist,:) = [jointCenter(jc.RtAcromioClavicular:jc.RtWrist,:); jointCenter(jc.RtIndexCarpal:jc.RtThumbDist,:)];
    jointCenter(jc.LtAcromioClavicular:jc.LtThumbDist,2) = jointCenter(jc.LtAcromioClavicular:jc.LtThumbDist,2)*-1;
    adjustedLM(lm.LtAcromion:lm.LtMetacarpalPhalV,:) = adjustedLM(lm.RtAcromion:lm.RtMetacarpalPhalV,:);
    adjustedLM(lm.LtAcromion:lm.LtMetacarpalPhalV,2) = adjustedLM(lm.LtAcromion:lm.LtMetacarpalPhalV,2)*-1;

    %% Copy mesh points from right to left side
    % Check for each pair and check which point that is on the right side (negative Y value) 
    for i=1:size(pairs,1)
        if meshModel(pairs(i,1),2) < 0
            meshModel(pairs(i,2),1) = meshModel(pairs(i,1),1); % set the left side to right side x-coordinate
            meshModel(pairs(i,2),2) = meshModel(pairs(i,1),2)*-1; % set the left side to right side y-coordinate
            meshModel(pairs(i,2),3) = meshModel(pairs(i,1),3); % set the left side to right side z-coordinate 
        elseif meshModel(pairs(i,2),2) < 0
            meshModel(pairs(i,1),1) = meshModel(pairs(i,2),1); % set the left side to right side x-coordinate
            meshModel(pairs(i,1),2) = meshModel(pairs(i,2),2)*-1; % set the left side to right side y-coordinate
            meshModel(pairs(i,1),3) = meshModel(pairs(i,2),3); % set the left side to right side z-coordinate
        end
    end

    %% Align bust points to landmarks
    % 13124:13132;  left nipple; Left midpoint: 13127 % LM: 14	LtThelion/Bustpoint 
    % 13133:13141;  right nipple; Right midpoint: 13136 % LM: 13	RtThelion/Bustpoint
   
    adjustedLM(lm.RtThelion,:) = meshModel(r.rightNippleMid,:);
    adjustedLM(lm.LtThelion,:) = meshModel(r.leftNippleMid,:);

    %% ADD ADJUSTMENTS TO GET STRAIGHT FINGERS %%

    %% RIGHT HAND %%
    insideHandTotalRight = load('insideHandRight.mat').insideHandTotalRight; % insideHand = [1.insidePalm, 2.insideThumb, 3.insideIndex, 4.insideMiddle, 5.insideRing, 6.insidePinky];
    meshStart = r.rightArmHand(1);
    meshEnd = r.rightArmHand(end);
    jcChainTotal = jc.RtIndexCarpal:jc.RtIndexDist; %Right_Index
    [meshModel, jointCenter] = straightFingers(meshStart, meshEnd, jcChainTotal, meshModel, jointCenter, [jc.RtIndexCarpal,jc.RtMiddleCarpal], insideHandTotalRight(:,3));
    jcChainTotal = jc.RtMiddleCarpal:jc.RtMiddleDist; %Right_Middle
    [meshModel, jointCenter] = straightFingers(meshStart, meshEnd, jcChainTotal, meshModel, jointCenter, [jc.RtIndexCarpal,jc.RtRingCarpal], insideHandTotalRight(:,4));
    jcChainTotal = jc.RtRingCarpal:jc.RtRingDist; %Right_Ring
    [meshModel, jointCenter] = straightFingers(meshStart, meshEnd, jcChainTotal, meshModel, jointCenter, [jc.RtMiddleCarpal,jc.RtPinkyCarpal], insideHandTotalRight(:,5));
    jcChainTotal = jc.RtPinkyCarpal:jc.RtPinkyDist; %Right_Pinky
    [meshModel, jointCenter] = straightFingers(meshStart, meshEnd, jcChainTotal, meshModel, jointCenter, [jc.RtRingCarpal,jc.RtPinkyCarpal], insideHandTotalRight(:,6));
    jcChainTotal = jc.RtThumbCarpal:jc.RtThumbDist; %Right_Thumb
    [meshModel, jointCenter] = straightFingers(meshStart, meshEnd, jcChainTotal, meshModel, jointCenter, [jc.RtIndexCarpal,jc.RtThumbProx], insideHandTotalRight(:,2));

    %% Make fingers parallel, pointing in the Y direction
    jcChainTotal = jc.RtIndexCarpal:jc.RtIndexDist;
    [meshModel, jointCenter] = parallelFingers(meshStart, meshEnd, jcChainTotal, meshModel, jointCenter, insideHandTotalRight(:,3));
    jcChainTotal = jc.RtMiddleCarpal:jc.RtMiddleDist;
    [meshModel, jointCenter] = parallelFingers(meshStart, meshEnd, jcChainTotal, meshModel, jointCenter, insideHandTotalRight(:,4));
    jcChainTotal = jc.RtRingCarpal:jc.RtRingDist;
    [meshModel, jointCenter] = parallelFingers(meshStart, meshEnd, jcChainTotal, meshModel, jointCenter, insideHandTotalRight(:,5));
    jcChainTotal = jc.RtPinkyCarpal:jc.RtPinkyDist;
    [meshModel, jointCenter] = parallelFingers(meshStart, meshEnd, jcChainTotal, meshModel, jointCenter, insideHandTotalRight(:,6));


    %% Thumb posture
    thumbOpts.inPlaneDeg = 25;    % spread from the finger direction, in the palm plane, towards the thumb side
    thumbOpts.palmarDeg  = 3;     % lift out of the palm plane, towards the palm side
    thumbOpts.twistDeg   = 45;    % twist about the thumb axis, nail turning towards the thumb side
    thumbOpts.twistFrom  = 'MCP'; % where the twist begins: 'CMC', 'MCP' or a number (0 = CMC, 1 = MCP)

    hand     = handFrame(jointCenter, jc);
    thumbIdx = maskToMeshIdx(insideHandTotalRight(:,2), meshStart, meshEnd);
    palmIdx  = maskToMeshIdx(insideHandTotalRight(:,1) & ~insideHandTotalRight(:,2), meshStart, meshEnd);
    [meshModel, jointCenter] = positionThumb(meshModel, jointCenter, jc, hand, thumbIdx, palmIdx, thumbOpts);
    [meshModel, jointCenter] = twistThumb(meshModel, jointCenter, jc, hand, thumbIdx, thumbOpts);

    adjustedLM(lm.RtDactylion,:) = jointCenter(jc.RtMiddleDist,:);

    %% Copy right side values to left side
    jointCenter(jc.LtIndexCarpal:jc.LtThumbDist,:) = jointCenter(jc.RtIndexCarpal:jc.RtThumbDist,:);
    jointCenter(jc.LtIndexCarpal:jc.LtThumbDist,2) = jointCenter(jc.LtIndexCarpal:jc.LtThumbDist,2)*-1;
    adjustedLM(lm.LtDactylion,:) = adjustedLM(lm.RtDactylion,:);
    adjustedLM(lm.LtDactylion,2) = adjustedLM(lm.LtDactylion,2)*-1;
    pairs = readmatrix('statBodyModel/meshSymmetryPairs.txt','FileType','text');
    pairStart = 4921;  % first pair row covering the right arm/hand region
    pairEnd = 6644;    % last pair row covering the right arm/hand region
    meshStart = r.rightArmHand(1);

    for p=pairStart:pairEnd
        insideIdx = pairs(p,1) - meshStart + 1;
        if insideHandTotalRight(insideIdx,2) || insideHandTotalRight(insideIdx,3) || insideHandTotalRight(insideIdx,4) || insideHandTotalRight(insideIdx,5) || insideHandTotalRight(insideIdx,6) 
            meshModel(pairs(p,2),1) = meshModel(pairs(p,1),1); % set the left and right side to the same x-coordinate
            meshModel(pairs(p,2),2) = meshModel(pairs(p,1),2)*-1; % set the left and right side to different y-coordinate
            meshModel(pairs(p,2),3) = meshModel(pairs(p,1),3); % set the left and right side to the same z-coordinate
        end
    end
    
end

function [meshModel, jointCenter] = straightFingers(meshStart, meshEnd, jcChainTotal, meshModel, jointCenter, vecCarpal, insideFinger)
    %% Fix: Could include some sort of heat map as well.
    %%  LM vector (Y-axis) s
        vLM = jointCenter(vecCarpal(1),:) - jointCenter(vecCarpal(2),:);
        uvLM =  vLM / norm(vLM); % intial, corrected below
    %% 	AP vector (Z-axis)
    	vAP = jointCenter(jcChainTotal(1),:)-jointCenter(jcChainTotal(2),:);
        uvAP =  vAP / norm(vAP);  
    %% 	SI vector (X-axis)
        uvSI = cross(uvLM,uvAP);
        uvLM = cross(uvSI,uvAP);

    for k=1:(size(jcChainTotal,2)-2) %% Rotate around X-axis
        jcChain = jcChainTotal(k:size(jcChainTotal,2));
        vec = jointCenter(jcChain(2),:)-jointCenter(jcChain(3),:);
        uvec = vec / norm(vec);
        % --- Project vector into XZ plane of that system ---
        v_y = dot(uvec, uvLM);
        v_z = dot(uvec, uvAP);
        theta = -1*atan2(v_y, v_z); % --- Angle in that plane (relative to +X axis) ---
          
        % Plane normal (rotation axis)
        nx = uvSI(1);
        ny = uvSI(2);
        nz = uvSI(3);
        
        c = cos(theta);
        s = sin(theta);
        
        % Rotation matrix (axis–angle)
        R = [ ...
            c + nx^2*(1-c),     nx*ny*(1-c) - nz*s, nx*nz*(1-c) + ny*s;
            ny*nx*(1-c) + nz*s, c + ny^2*(1-c),     ny*nz*(1-c) - nx*s;
            nz*nx*(1-c) - ny*s, nz*ny*(1-c) + nx*s, c + nz^2*(1-c)
        ];
        jcStart = jointCenter(jcChain(2),:);
        jcEnd = jointCenter(jcChain(size(jcChain,2)),:);
%         inside = pointInSweptSphere(meshModel(meshStart:meshEnd,:), jcStart, jcEnd, radius-(k-1));
        inside = pointInSphere(meshModel(meshStart:meshEnd,:), jcEnd, norm(jcStart - jcEnd));
        for i=1:size(inside,1)
            if inside(i) && insideFinger(i)
                meshModel(i+meshStart-1,:) = (R * (meshModel(i+meshStart-1,:)-jcStart)')'+jcStart;  % Rotate mesh point around jcStart
            end
        end
        jointCenter(jcChain(3):jcChain(size(jcChain,2)),:) = (R * (jointCenter(jcChain(3):jcChain(size(jcChain,2)),:)-jcStart)')'+jcStart;  % Rotate JCs
    end
    for k=1:(size(jcChainTotal,2)-2) %% Rotate around Y-axis
        jcChain = jcChainTotal(k:size(jcChainTotal,2));
        vec = jointCenter(jcChain(2),:)-jointCenter(jcChain(3),:);
        uvec = vec / norm(vec);
        % --- Project vector into XZ plane of that system ---
        v_x = dot(uvec, uvSI);
        v_z = dot(uvec, uvAP);
        theta = atan2(v_x, v_z); % --- Angle in that plane (relative to +X axis) ---
          
        % Plane normal (rotation axis)
        nx = uvLM(1);
        ny = uvLM(2);
        nz = uvLM(3);
        
        c = cos(theta);
        s = sin(theta);
        
        % Rotation matrix (axis–angle)
        R = [ ...
            c + nx^2*(1-c),     nx*ny*(1-c) - nz*s, nx*nz*(1-c) + ny*s;
            ny*nx*(1-c) + nz*s, c + ny^2*(1-c),     ny*nz*(1-c) - nx*s;
            nz*nx*(1-c) - ny*s, nz*ny*(1-c) + nx*s, c + nz^2*(1-c)
        ];
        jcStart = jointCenter(jcChain(2),:);
        jcEnd = jointCenter(jcChain(size(jcChain,2)),:);
%         inside = pointInSweptSphere(meshModel(meshStart:meshEnd,:), jcStart, jcEnd, radius);
        inside = pointInSphere(meshModel(meshStart:meshEnd,:), jcEnd, norm(jcStart - jcEnd));
        for i=1:size(inside,1)
            if inside(i) && insideFinger(i)
                meshModel(i+meshStart-1,:) = (R * (meshModel(i+meshStart-1,:)-jcStart)')'+jcStart;  % Rotate mesh point around jcStart
            end
        end
        jointCenter(jcChain(3):jcChain(size(jcChain,2)),:) = (R * (jointCenter(jcChain(3):jcChain(size(jcChain,2)),:)-jcStart)')'+jcStart;  % Rotate JCs
    end
end

function [meshModel, jointCenter] = parallelFingers(meshStart, meshEnd, jcChainTotal, meshModel, jointCenter, insideFinger)
    % Rotate the entire finger around its carpal (MCP) joint, about the
    % global Z-axis only, so the finger becomes parallel to the YZ-plane
    % (its X-component is zeroed while its Z-elevation is preserved).
    jcPivot = jcChainTotal(1);   % Carpal joint — rotation pivot
    jcTip   = jcChainTotal(end); % Distal joint

    vec = jointCenter(jcTip, :) - jointCenter(jcPivot, :);

    sideSign = sign(jointCenter(jcPivot, 2));
    if sideSign == 0, sideSign = -1; end  % fallback: assume right hand

    if norm(vec(1:2)) < 1e-10
        return;  % already parallel to the YZ-plane
    end

    theta = sideSign*pi/2 - atan2(vec(2), vec(1));

    c = cos(theta);  s = sin(theta);
    R = [c -s 0; s c 0; 0 0 1];

    jcStart = jointCenter(jcPivot, :);
    jcEnd   = jointCenter(jcTip, :);
    inside  = pointInSphere(meshModel(meshStart:meshEnd, :), jcEnd, norm(jcStart - jcEnd));

    for i = 1:size(inside, 1)
        if inside(i) && insideFinger(i)
            meshModel(i+meshStart-1, :) = (R * (meshModel(i+meshStart-1, :) - jcStart)')' + jcStart;
        end
    end
    jointCenter(jcChainTotal(2):jcChainTotal(end), :) = (R * (jointCenter(jcChainTotal(2):jcChainTotal(end), :) - jcStart)')' + jcStart;
end

function hand = handFrame(jointCenter, jc)
    % Reference frame of the right hand, from the (already parallel) finger MCP and PIP joints:
    %   dorsal    - normal of the plane fitted through the MCP and PIP joints, pointing away from the thumb MCP
    %   fingerDir - mean MCP -> PIP direction, projected into that plane
    %   radialDir - in-plane direction perpendicular to fingerDir, pointing towards the thumb
    mcp = jointCenter([jc.RtIndexCarpal jc.RtMiddleCarpal jc.RtRingCarpal jc.RtPinkyCarpal],:);
    pip = jointCenter([jc.RtIndexProx jc.RtMiddleProx jc.RtRingProx jc.RtPinkyProx],:);
    P = [mcp; pip];
    [~, ~, V] = svd(P - mean(P,1), 'econ');
    dorsal = V(:,3)';
    if dot(jointCenter(jc.RtThumbProx,:) - mean(mcp,1), dorsal) > 0
        dorsal = -dorsal;
    end
    fingerDir = mean(pip - mcp, 1);
    fingerDir = unit(fingerDir - dot(fingerDir, dorsal)*dorsal);
    radialDir = cross(dorsal, fingerDir);
    if dot(radialDir, jointCenter(jc.RtThumbDist,:) - jointCenter(jc.RtThumbCarpal,:)) < 0
        radialDir = -radialDir;
    end
    hand = struct('dorsal', dorsal, 'fingerDir', fingerDir, 'radialDir', radialDir);
end

function [meshModel, jointCenter] = positionThumb(meshModel, jointCenter, jc, hand, thumbIdx, palmIdx, opts)
    % Rotate the right thumb about its CMC so that the thumb axis (CMC -> tip) points opts.inPlaneDeg from the
    % finger direction towards the thumb side, and opts.palmarDeg out of the palm plane towards the palm side.
    % The rotation is the smallest one taking the current thumb axis onto that target direction.
    % Thumb joints rotate rigidly. Mesh vertices rotate by weight*angle, where the weight rises smoothly from 0
    % at the CMC to 1 at the thumb MCP (along the thumb axis), so the skin at the MCP moves with the joints.
    % Palm vertices close to the thumb (within 0.3 x the CMC-MCP length) follow with a weight that fades to 0
    % with distance, keeping the thenar/web smooth.
    cmc = jointCenter(jc.RtThumbCarpal,:);
    thumbDir = unit(jointCenter(jc.RtThumbDist,:) - cmc);
    targetDir = cosd(opts.palmarDeg) * (cosd(opts.inPlaneDeg)*hand.fingerDir + sind(opts.inPlaneDeg)*hand.radialDir) ...
              - sind(opts.palmarDeg) * hand.dorsal;

    rotAxis = cross(thumbDir, targetDir);
    if norm(rotAxis) < 1e-9, return; end   % already on target
    angle = atan2(norm(rotAxis), dot(thumbDir, targetDir));
    rotAxis = unit(rotAxis);

    % Mesh: weight = distance fall-off (1 for thumb vertices) x position along the thumb axis
    lenMC = norm(jointCenter(jc.RtThumbProx,:) - cmc);
    sMCP = dot(jointCenter(jc.RtThumbProx,:) - cmc, thumbDir);   % MCP position along the thumb axis
    palmFall = max(0, 1 - minDistToSet(meshModel(palmIdx,:), meshModel(thumbIdx,:)) / (0.3*lenMC));
    idx = [thumbIdx; palmIdx];
    along = smooth01((meshModel(idx,:) - cmc) * thumbDir' / sMCP);
    w = [ones(numel(thumbIdx),1); palmFall] .* along;

    % Joints: MCP, IP and tip rotate rigidly about the CMC
    rows = [jc.RtThumbProx jc.RtThumbInter jc.RtThumbDist];
    jointCenter(rows,:) = rotateAboutAxis(jointCenter(rows,:), cmc, rotAxis, angle);
    meshModel(idx,:) = rotateAboutAxis(meshModel(idx,:), cmc, rotAxis, w*angle);
end

function [meshModel, jointCenter] = twistThumb(meshModel, jointCenter, jc, hand, thumbIdx, opts)
    % Twist the right thumb about its axis (line CMC -> tip) by opts.twistDeg, "outwards": the thumb's dorsal
    % (nail) side turns towards the thumb side of the hand. The mesh has no nail geometry, so the nail direction
    % is estimated as the hand's dorsal direction, made perpendicular to the thumb axis.
    % The twist weight rises smoothly from 0 at opts.twistFrom to 1 at the tip; mesh vertices and thumb joints
    % are twisted by the weight at their own position along the axis.
    if opts.twistDeg == 0, return; end
    cmc = jointCenter(jc.RtThumbCarpal,:);
    tip = jointCenter(jc.RtThumbDist,:);
    thumbDir = unit(tip - cmc);

    % Direction of the twist: positive if it turns the nail towards the thumb side
    nailDir = unit(hand.dorsal - dot(hand.dorsal, thumbDir)*thumbDir);
    radialPerp = hand.radialDir - dot(hand.radialDir, thumbDir)*thumbDir;
    twistAngle = deg2rad(opts.twistDeg);
    if dot(cross(thumbDir, nailDir), radialPerp) < 0
        twistAngle = -twistAngle;
    end

    % Start of the twist: 'CMC' (0), 'MCP' (1) or a number = fraction of the way from CMC to MCP
    startFrac = opts.twistFrom;
    if ischar(startFrac) || isstring(startFrac)
        switch upper(char(startFrac))
            case 'CMC', startFrac = 0;
            case 'MCP', startFrac = 1;
            otherwise, error('rotateArmTpose:twistFrom', 'twistFrom must be ''CMC'', ''MCP'' or a number');
        end
    end
    sStart = startFrac * dot(jointCenter(jc.RtThumbProx,:) - cmc, thumbDir);
    sTip = dot(tip - cmc, thumbDir);
    weight = @(P) smooth01(((P - cmc)*thumbDir' - sStart) / (sTip - sStart));

    meshModel(thumbIdx,:) = rotateAboutAxis(meshModel(thumbIdx,:), cmc, thumbDir, weight(meshModel(thumbIdx,:))*twistAngle);
    rows = [jc.RtThumbProx jc.RtThumbInter jc.RtThumbDist];
    jointCenter(rows,:) = rotateAboutAxis(jointCenter(rows,:), cmc, thumbDir, weight(jointCenter(rows,:))*twistAngle);
end

function P = rotateAboutAxis(P, origin, k, theta)
    % Rotate the rows of P about the line through origin with unit direction k (Rodrigues' formula).
    % theta (rad) is a scalar or a column with one angle per row.
    V = P - origin;
    c = cos(theta);
    P = origin + V.*c + cross(repmat(k, size(V,1), 1), V, 2).*sin(theta) + (V*k').*k.*(1 - c);
end

function idx = maskToMeshIdx(mask, meshStart, meshEnd)
    % Mesh vertex indices of a hand-region mask (mask row 1 = vertex meshStart)
    idx = find(mask) + meshStart - 1;
    idx = idx(idx <= meshEnd);
end

function d = minDistToSet(A, B)
    % Distance from each row of A to its nearest row of B
    d = min(vecnorm(permute(A, [1 3 2]) - permute(B, [3 1 2]), 2, 3), [], 2);
end

function v = unit(v), v = v / norm(v); end

function w = smooth01(s), s = min(1, max(0, s)); w = s.^2 .* (3 - 2*s); end
