clearvars; clc;
clf;
tic
sex = 0; % Female = 1, Male = 0
% Predictive anthropometric variables, possible to change. See measurement list below.
pred_id = [3 4 5]; % Age = 3, Weight = 4, Stature = 5

age = 41; % (year) Ex. 41
stature = 1880; % (mm) Ex. 1633 
weight = 82; % (kg) Ex. 73 

Z_test = [age weight stature]; % If predictive variables are changed, add them to this Z_test variable.

addpath('./scripts/');
addpath('./scripts/utils/');
r = meshRegions(); lm = lmIndices(); jc = jcIndices();
anthroMeasurements = anthroRegression(sex, pred_id, Z_test);

t = datetime('now','TimeZone','local','Format','yyMMdd');
if sex == 1
    manikinSex = 'Female_';
elseif sex == 0
    manikinSex = 'Male_';
end
modelName = [manikinSex,char(t),'_',int2str(anthroMeasurements(1)),'_',int2str(anthroMeasurements(2)),'_',int2str(anthroMeasurements(3))]; %Automatically generates a suitable file-name.

landmarkCoordinates = landmarkPrediction(sex, anthroMeasurements);
meshModel = bodyShapePrediction(sex, anthroMeasurements, landmarkCoordinates);
adjustedLM = zeros(73,3);
for i=1:size(adjustedLM,1)
    adjustedLM(i,1:3) = landmarkCoordinates(1,(i*3-2):(i*3));
end
% saveMesh(meshModel,'MeshModelDirect'); % Meshmodel after initial body shape prediction.

meshModel = meshSymmetry(meshModel, adjustedLM); disp('Adjusted mesh to get symmetry.'); % Correction to get symmetry on both sides of sagittal plane
% saveMesh(meshModel,[modelName,'_Symmetry']); % Meshmodel after body shape have been made symmetrical.

[jointCenter, meshModel] = jcPrediction(meshModel,adjustedLM,sex,weight,stature);
% saveMesh(meshModel,[modelName,'_jcPred']); % Meshmodel after prediction of joint centres and adjustment of hand mesh.

[meshModel,adjustedLM,jointCenterT] = rotateArmTpose(meshModel,adjustedLM,jointCenter);
% saveMesh(meshModel,[modelName,'_rotateArm']); % Meshmodel after arms have been rotated into a T-pose.

[meshModel,adjustedLM,jointCenterT] = rotateFeetTpose(meshModel,adjustedLM,jointCenterT);
% saveMesh(meshModel,[modelName,'_rotateFeet']); % Meshmodel after feet have been rotated pointing straight forward and meshpoints on the underside of feet have been adjusted to 0 level.

%%% SCALING To GET INPUT STATURE AFTER JOINT CENTRE PREDICTION AND POSTURE ADJUSTMENT %%%
maxHt = max(meshModel(r.headTopScale,3));
scalingRatio = stature/maxHt;
meshModel = scalingRatio*meshModel;
adjustedLM = scalingRatio*adjustedLM;
jointCenterT = scalingRatio*jointCenterT;

saveMesh(meshModel,[modelName,'_Scaled']); % Meshmodel after scaling based on stature value.

%% Create XYZ files for landmarks and joint centres %%
saveXYZ(adjustedLM,modelName,1);
saveXYZ(jointCenterT,modelName,0);

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
