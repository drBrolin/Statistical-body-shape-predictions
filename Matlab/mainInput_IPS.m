clearvars; 
clc; 
clf;
tic
sex = 0; % Female = 1, Male = 0
% Predictive anthropometric variables, possible to change. See measurement list below.
pred_id = [3 4 5]; % Age = 3, Weight = 4, Stature = 5
% pred_id = [3	4	5	6	7	8	9	10	16	17	18	19	20	21	23	24	26	29	33	35	39	40	41	42	43	44	53	55	56	58	59]; 

% pred_id = [3	5	19	21	29	33  39]; 
% % 03	Age (year)
% % 05	Stature (body height) (mm)
% % 19	Shoulder height, sitting (mm)
% % 21	Shoulder-elbow length (mm)
% % 29	Knee height (mm)
% % 33	Hand length (mm)
% % 39	Foot length (mm)
% Z_test = [40	1781    608 377 566 169 304];

% Z_test = [40	44.4	1502	1369	1189	882	804	675	807	687	588	511	205	308	346	390	333	461	157	76	264	89	182	144	109	534	482	813	632	482	348];
% Z_test = [40	67.9	1725	1595	1399	1035	959	809	900	778	665	586	223	366	377	439	401	545	172	82	297	98	190	147	116	555	594	924	732	585	350];
% Z_test = [40	76.4	1725	1590	1400	1031	965	806	905	778	669	586	223	366	390	486	368	545	172	88	297	102	194	152	118	561	594	991	832	595	387];
% Z_test = [40	101.8	1941	1814	1601	1186	1108	933	1000	873	747	658	251	411	420	530	424	629	187	95	330	111	202	155	124	580	687	1104	940	671	427];
% Z_test = [40	50	1650];
% Z_test = [41	73	1633];
% Z_test = [41	150	1710];
% Z_test = [41	93	1750];
Z_test = [41	82	1880];

age = Z_test(1); % (year) Ex. 41
weight = Z_test(2); % (kg) Ex. 73 or 83
stature = Z_test(3); % (mm) Ex. 1633 or 1775

% Z_test = [age weight stature]; % If predictive variables are changed, add them to this Z_test variable.

addpath('./scripts/');
addpath('./scripts/utils/');
r = meshRegions(); lm = lmIndices(); jc = jcIndices();
anthroMeasurements = anthroRegression(sex, pred_id, Z_test);
% weight = anthroMeasurements(2); % (kg)
landmarkCoordinates = landmarkPrediction(sex, anthroMeasurements);
meshModel = bodyShapePrediction(sex, anthroMeasurements, landmarkCoordinates);
adjustedLM = zeros(73,3);
for i=1:size(adjustedLM,1)
    adjustedLM(i,1:3) = landmarkCoordinates(1,(i*3-2):(i*3));
end
saveMesh(meshModel,'MeshModelDirect');
%%% Correction to get symmetry on both sides of sagittal plane %%%
meshModel = meshSymmetry(meshModel, adjustedLM); disp('Adjusted mesh to get symmetry.');
saveMesh(meshModel,'MeshModelSymmetry');

[jointCenter, meshModel] = jcPrediction(meshModel,adjustedLM,sex,weight,stature);

saveMesh(meshModel,'MeshModeljcPred');

% plot3(meshModel(:,1),meshModel(:,2),meshModel(:,3),'ko','MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',1); hold on;
% plot3(adjustedLM(:,1),adjustedLM(:,2),adjustedLM(:,3),'co','MarkerEdgeColor','c','MarkerFaceColor','c','MarkerSize',3); hold on;
% plot3(jointCenter(:,1),jointCenter(:,2),jointCenter(:,3),'ro','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',2); hold on;
% figure(1);
% grid;
% xlabel('X'); ylabel('Y'); zlabel('Z')
% axis equal; 
% t = datetime('now','TimeZone','local','Format','yyMMdd');
% if sex == 1
%     manikinSex = 'Female_';
% elseif sex == 0
%     manikinSex = 'Male_';
% end
% modelName = [manikinSex,'Manikin_',char(t),'_',int2str(age),'_',int2str(stature),'_',int2str(weight)]; %Automatically generates a suitable file-name.
% meshModelMirror = meshModel;
% meshModelMirror(:,2) = meshModelMirror(:,2)*-1;
% saveMesh(meshModel,modelName);
% saveMesh(meshModelMirror,[modelName,'_Mirror']);
% saveJC(jointCenter,modelName);
% % saveLM_Lua(adjustedLM,modelName);
% teststopp = 1;

% jointCenterT = jointCenter;
[meshModel,adjustedLM,jointCenterT] = rotateArmTpose(meshModel,adjustedLM,jointCenter);
% dlmwrite('debug/meshmodelTpose.csv', meshModelScaled, 'precision', 15);  % <-------------------------------
% dlmwrite('debug/landmarksTpose.csv', adjustedLMScaled, 'precision', 15); % <-------------------------------
% dlmwrite('debug/jointcenterTpose.csv', jointCenterT, 'precision', 15);   % <-------------------------------

saveMesh(meshModel,'MeshModelrotateArm');

[meshModel,adjustedLM,jointCenterT] = rotateFeetTpose(meshModel,adjustedLM,jointCenterT);   
% dlmwrite('debug/meshmodelTposeRotateFeet.csv', meshModelScaled, 'precision', 15);  % <-------------------------------
% dlmwrite('debug/landmarksTposeRotateFeet.csv', adjustedLMScaled, 'precision', 15); % <-------------------------------
% dlmwrite('debug/jointcenterTposeRotateFeet.csv', jointCenterT, 'precision', 15);   % <-------------------------------

saveMesh(meshModel,'MeshModelrotateFeet');

%%% SCALING AFTER JOINT CENTRE PREDICTION AND POSTURE ADJUSTMENT %%%
maxHt = max(meshModel(r.headTopScale,3));
scalingRatio = stature/maxHt;
meshModel = scalingRatio*meshModel;
adjustedLM = scalingRatio*adjustedLM;
jointCenterT = scalingRatio*jointCenterT;

%% Additional points, make into functions for specific usages (modelling and driver ergonomics etc.)
%%% Calculate midpoint for joints on lower arms and legs %%% 
% Maybe handled in later code of IPS?
jointCenterT(jc.RtUpperArmMid,:) = (jointCenterT(jc.RtShoulder,:)+jointCenterT(jc.RtElbow,:))/2;
jointCenterT(jc.LtUpperArmMid,:) = (jointCenterT(jc.LtShoulder,:)+jointCenterT(jc.LtElbow,:))/2;
jointCenterT(jc.RtLowerArmMid,:) = (jointCenterT(jc.RtElbow,:)+jointCenterT(jc.RtWrist,:))/2;
jointCenterT(jc.LtLowerArmMid,:) = (jointCenterT(jc.LtElbow,:)+jointCenterT(jc.LtWrist,:))/2;
jointCenterT(jc.RtLowerLegMid,:) = (jointCenterT(jc.RtKnee,:)+jointCenterT(jc.RtAnkle,:))/2;
jointCenterT(jc.LtLowerLegMid,:) = (jointCenterT(jc.LtKnee,:)+jointCenterT(jc.LtAnkle,:))/2;

%% Max Abdominal Depth %%
maxAbdominalDepth = mean(meshModel(r.abdomenDepth,:));
jointCenterT(jc.AbdominalDepth,:) = [maxAbdominalDepth(1)  0   maxAbdominalDepth(3)];

%%% Extra landmarks on widest hip-points %%%
widthHip = meshModel(r.pelvisLegs,:);
widthHip = widthHip(widthHip(:,3) < adjustedLM(lm.Crotch,3), :);
[maxHipBrth,indxmaxHipBrth] = max(widthHip(:,2));
[minHipBrth,indxminHipBrth] = min(widthHip(:,2));
hipHeight = (widthHip(indxmaxHipBrth,3)+widthHip(indxminHipBrth,3))/2;
hipX = (jointCenterT(jc.LtHip,1) - jointCenterT(jc.LtKnee,1))*(jointCenterT(jc.LtHip,3) - hipHeight)/(jointCenterT(jc.LtHip,3) - jointCenterT(jc.LtKnee,3));
jointCenterT(jc.RtHipWidth,:) = [(jointCenterT(jc.LtHip,1)-hipX) minHipBrth  hipHeight];
jointCenterT(jc.LtHipWidth,:) = [(jointCenterT(jc.LtHip,1)-hipX) maxHipBrth  hipHeight];
hipYBone = (jointCenterT(jc.LtHip,2) - jointCenterT(jc.LtKnee,2))*(jointCenterT(jc.LtHip,3) - hipHeight)/(jointCenterT(jc.LtHip,3) - jointCenterT(jc.LtKnee,3));
jointCenterT(jc.RtHipBone,:) = [jointCenterT(jc.LtHipWidth,1)    (jointCenterT(jc.RtHip,2)-hipYBone)  hipHeight];
jointCenterT(jc.LtHipBone,:) = [jointCenterT(jc.LtHipWidth,1)    (jointCenterT(jc.LtHip,2)+hipYBone)  hipHeight];

%%% Extra landmarks on buttocks surface
pelvLeg = meshModel(r.pelvisLegs,:);
[minButtockRt,~] = min(pelvLeg(pelvLeg(:,2) < 0, 1));
[minButtockLt,~] = min(pelvLeg(pelvLeg(:,2) > 0, 1));
jointCenterT(jc.RtButtock,:) = [minButtockRt jointCenterT(jc.RtHip,2:3)];
jointCenterT(jc.LtButtock,:) = [minButtockLt jointCenterT(jc.LtHip,2:3)];

% Add T8 as landmark.
T8T9 = [0.920215*(jointCenterT(jc.T6T7,1)-jointCenterT(jc.L5S1,1))+jointCenterT(jc.L5S1,1)  0  0.856305*(jointCenterT(jc.T6T7,3)-jointCenterT(jc.L5S1,3))+jointCenterT(jc.L5S1,3)];
T8   = [0.953711*(jointCenterT(jc.T6T7,1)-jointCenterT(jc.L5S1,1))+jointCenterT(jc.L5S1,1)  0  0.892418*(jointCenterT(jc.T6T7,3)-jointCenterT(jc.L5S1,3))+jointCenterT(jc.L5S1,3)];
T7T8 = [0.981990*(jointCenterT(jc.T6T7,1)-jointCenterT(jc.L5S1,1))+jointCenterT(jc.L5S1,1)  0  0.928733*(jointCenterT(jc.T6T7,3)-jointCenterT(jc.L5S1,3))+jointCenterT(jc.L5S1,3)];

T8Vec = T7T8-T8T9;
kT8 = T8Vec(3)/T8Vec(1);
kT8Inv = -1/kT8;
mT8Inv = T8(3)-kT8Inv*T8(1);

% Torso 4678:5446
closeT8 = zeros(numel(r.torso),4);
findClose = 1;
for n=r.torso
    if meshModel(n,1)<T8(1) && meshModel(n,2)>-10 && meshModel(n,2)<10 && (meshModel(n,3)<T8(3)+40) && (meshModel(n,3)>T8(3)-40)
        closeT8(findClose,:) = [n meshModel(n,:)];
        findClose = findClose+1;
%         plot3(meshModel(n,1),meshModel(n,2),meshModel(n,3),'bo','MarkerEdgeColor','b','MarkerFaceColor','b','MarkerSize',2); hold on;
    end
end
closeT8 = closeT8(1:findClose-1,:);
backT8 = mean(closeT8(:,2));
adjustedLM(lm.T8,:) = [backT8    0   kT8Inv*backT8+mT8Inv];

% FULL BODY PLOT
% dist = zeros(size(meshModel,1),1);
% for k = 1:length(dist)
%     dist(k) = k;
% end
% scatter3(meshModel(:,1), meshModel(:,2), meshModel(:,3), 10, dist, 'filled'); hold on;
% colorbar; 

% 5447:7585;    feet;                       2139

% plot3(meshModel(:,1),meshModel(:,2),meshModel(:,3),'ko','MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',1); hold on;
% % plot3(adjustedLM(:,1),adjustedLM(:,2),adjustedLM(:,3),'co','MarkerEdgeColor','c','MarkerFaceColor','c','MarkerSize',3); hold on;
% % % plot3(jointCenter(:,1),jointCenter(:,2),jointCenter(:,3),'ro','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',2); hold on;
% % plot3(jointCenterT(:,1),jointCenterT(:,2),jointCenterT(:,3),'ro','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',2); hold on;
% % plot3(meshModel(r.rightFoot,1),meshModel(r.rightFoot,2),meshModel(r.rightFoot,3),'mo','MarkerEdgeColor','m','MarkerFaceColor','m','MarkerSize',2); hold on;
% % plot3(meshModel(9663:11390,1),meshModel(9663:11390,2),meshModel(9663:11390,3),'ko','MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',1); hold on;
% % plot3(adjustedLM(37:40,1),adjustedLM(37:40,2),adjustedLM(37:40,3),'co','MarkerEdgeColor','c','MarkerFaceColor','c','MarkerSize',3); hold on;
% % % % plot3(jointCenter(:,1),jointCenter(:,2),jointCenter(:,3),'ro','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',2); hold on;
% % plot3(jointCenterT(48:67,1),jointCenterT(48:67,2),jointCenterT(48:67,3),'ro','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',2); hold on;
% % 9663:11390;   right arm and hand;         1728
% 
% % meshFoot = meshModel(5447:7585,:);
% % dist = zeros(size(meshFoot,1),1);
% % for k = 1:length(dist)
% %     dist(k) = k;
% % end
% % scatter3(meshFoot(:,1), meshFoot(:,2), meshFoot(:,3), 10, dist, 'filled'); hold on;
% % colorbar; 
% 
% figure(1);
% grid;
% xlabel('X'); ylabel('Y'); zlabel('Z')
% axis equal; 

t = datetime('now','TimeZone','local','Format','yyMMdd');
if sex == 1
    manikinSex = 'Female_';
elseif sex == 0
    manikinSex = 'Male_';
end
modelName = [manikinSex,'Manikin_',char(t),'_',int2str(age),'_',int2str(stature),'_',int2str(weight)]; %Automatically generates a suitable file-name.

saveMesh(meshModel,[modelName,'_noRot']);

%% CENTRE MANIKIN AROUND L5/S1 %%
meshModel = meshModel-jointCenterT(jc.L5S1,:);
adjustedLM = adjustedLM-jointCenterT(jc.L5S1,:);
jointCenterT = jointCenterT-jointCenterT(jc.L5S1,:);

%% ROTATE MANIKIN TO Y=UP %%
% meshModelScaled_StatA = [meshModelScaled_StatA(:,1) meshModelScaled_StatA(:,3) meshModelScaled_StatA(:,2)];
% meshModel = [meshModel(:,1) meshModel(:,3) -meshModel(:,2)]; % Does not work with shadows in the obj-file.
meshModel = [meshModel(:,1) meshModel(:,3) -meshModel(:,2)];
jointCenterT = [jointCenterT(:,1) jointCenterT(:,3) -jointCenterT(:,2)];
adjustedLM = [adjustedLM(:,1) adjustedLM(:,3) -adjustedLM(:,2)];

% dist = zeros(size(meshModel,1),1);
% for k = 1:length(dist)
%     dist(k) = k;
% end
% scatter3(meshModel(:,1), meshModel(:,2), meshModel(:,3), 10, dist, 'filled'); hold on;
% colorbar; 

plot3(meshModel(:,1),meshModel(:,2),meshModel(:,3),'ko','MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',1); hold on;

figure(1);
grid;
xlabel('X'); ylabel('Y'); zlabel('Z')
axis equal; 

saveMesh(meshModel,modelName);
saveXYZ(adjustedLM,modelName,1);
saveJC(jointCenterT,modelName);
saveXYZ(jointCenterT,modelName,0);
saveLM_Lua(adjustedLM,modelName);
saveLM(adjustedLM, meshModel, jointCenterT,modelName);
% FIX and REDO matched meshes.
morphedMatchedMesh = saveMeshTpose(meshModel,modelName,sex); % Now returns the matchedMesh

%% Proof-Of-Concept: RIB CAGE MODEL %%
% ribCageModel = parametrizedRibCage(age, stature/1000, weight, abs(sex-1));
% ribCageModel = alignRibCage(ribCageModel, adjustedLM, jointCenterT);
% saveRibCage(ribCageModel, modelName);
% % Plot the rib mesh with vertex indices labeled for identification
%   figure; hold on;
%   plot3(ribCageModel.ribVertices(:,1), ribCageModel.ribVertices(:,2), ribCageModel.ribVertices(:,3), 'k.', 'MarkerSize', 4);
%   % Zoom into the 10th rib area (lower-lateral region, Y roughly -80 to -130mm after centering)
%   idx = find(ribCageModel.ribVertices(:,2) < -70 & ribCageModel.ribVertices(:,2) > -140 & abs(ribCageModel.ribVertices(:,1)) > 80);
%   text(ribCageModel.ribVertices(idx,1), ribCageModel.ribVertices(idx,2), ribCageModel.ribVertices(idx,3), num2str(idx));
%   grid;
%   axis equal; xlabel('X'); ylabel('Y'); zlabel('Z');

% Meas id   Measurement/variable
% 03	Age (year)
% 04	Body mass (weight) (kg)
% 05	Stature (body height) (mm)
% 06	Eye height (mm)
% 07	Shoulder height (mm)
% 08	Elbow height (mm)
% 09	Iliac spine height, standing (mm)
% 10	Crotch height (mm)
% 16	Sitting height (erect) (mm)
% 17	Eye height, sitting (mm)
% 18	Cervicale height, sitting (mm)
% 19	Shoulder height, sitting (mm)
% 20	Elbow height, sitting (mm)
% 21	Shoulder-elbow length (mm)
% 23	Shoulder (biacromial) breadth (mm)
% 24	Shoulder (bideoid) breadth (mm)
% 26	Hip breadth, sitting (mm)
% 29	Knee height (mm)
% 33	Hand length (mm)
% 35	Hand breadth at metacarpals (mm)
% 39	Foot length (mm)
% 40	Foot breadth (mm)
% 41	Head length (mm)
% 42	Head breadth (mm)
% 43	Face length (nasion-menton) (mm)
% 44	Head circumference (mm)
% 53	Buttock-knee length (mm)
% 55	Chest circumference (mm)
% 56	Waist circumference (mm)
% 58	Thigh circumference (mm)
% 59	Calf circumference (mm)
% 60	Acromion-Radiale length (mm)
% 61	Ankle Circumference (mm)
% 62	Axilla height (mm)
% 63	Bispinous breadth (mm)
% 64	Bizygomatic Breadth (mm)
% 65	Bustpoint breadth (mm)
% 66	Hip Circumference, Maximum (mm)
% 67	Hip Circ Max Height (mm)
% 68	Cervicale height (mm)
% 69	Chest Girth at Scye (Chest Circumference at Scye)(mm)
% 70	Chest height Stand (mm)
% 71	Total Crotch Length (Crotch Length) (mm)
% 72	Hand Circumference (mm)
% 73	Inter-pupillary distance (mm)
% 74	Ankle height (Malleolus, Lateral) (mm)
% 75	Radiale-Stylion length (mm)
% 76	Armscye Circumference (Scye Circ Over Acromion) (mm)
% 77	Suprasternale height (mm)
% 78	Trochanterion height (mm)

toc
elapsedTime = toc;
