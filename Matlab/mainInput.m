clear all; clc; 
clf;
tic
sex = 1; % Female = 1, Male = 0
% Predictive anthropometric variables, possible to change. See measurement list below.
pred_id = [3 4 5]; % Age = 3, Weight = 4, Stature = 5

age = 41; % (year) Ex. 41
stature = 1633; % (mm) Ex. 1633 
weight = 73; % (kg) Ex. 73 

Z_test = [age weight stature]; % If predictive variables are changed, add them to this Z_test variable.

addpath('./scripts/');
anthroMeasurements = anthroRegression(sex, pred_id, Z_test);
landmarkCoordinates = landmarkPrediction(sex, anthroMeasurements);
bodyShapePrediction(sex, anthroMeasurements, landmarkCoordinates);

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
