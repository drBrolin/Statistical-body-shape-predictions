% [1] Hara, R. et al. (2016). Predicting the location of the hip joint centres, impact of age group and sex.
% Coordinate system: X = 1 (back/forth), Y = 2 (left/right), Z = 3 (up/down)

function [jointCenter, meshModel] = jcPrediction(meshModel, adjustedLM,sex,bWeight,bStature)
    r = meshRegions(); lm = lmIndices(); jc = jcIndices();
%     BMI = bWeight/(bStature/1000)^2;
    jointCenter = zeros(82,3);
    %%% JOINT CENTER PREDICTION %%%
    % Inner (CT scan derived) pelvic depth (PD) is less than outer (body scan derived) PD. Especially for higher BMIs. Adjustment is applied based on SOURCE (REF).
    % Hip joints % RtASIS=17 LtASIS=19 RtPSIS=26 LtPSIS=27
    pdPred = 8.992808703+0.076688072*bStature+0.118264886*bWeight; % Regression analysis based on data [1]
    if sex==0 % Male
        pdPred = 18.11998533+0.067587949*bStature+0.107625933*bWeight; % Regression analysis based on data [1]
    end
    ASISmid = (adjustedLM(lm.RtASIS,:)+adjustedLM(lm.LtASIS,:))/2;
    PSISmid = (adjustedLM(lm.RtPSIS,:)+adjustedLM(lm.LtPSIS,:))/2;
    pdMeas = sqrt(sum((ASISmid - PSISmid).^2)); % Body scan derived PD
    if pdPred > pdMeas
        pdPred = pdMeas;
    end
    pdDiff = pdMeas - pdPred;
    
    % Origin of the pelvis is set to the mid-point between the Left and right ASIS. [1] 
    % Assumption that 2/3 of the difference is in the front of the pelvis.
    ASISmidCT = (PSISmid-ASISmid)*(pdDiff*2/3)/pdMeas+ASISmid; % Skeleton ASIS
    PSISmidCT = (PSISmid-ASISmid)*(pdMeas-pdDiff*1/3)/pdMeas+ASISmid; % Skeleton PSIS

    iliacSpVec = (PSISmidCT - ASISmidCT)/norm(PSISmidCT - ASISmidCT); % vector with length 1
    iliacSpVecN = [-iliacSpVec(3) iliacSpVec(2) iliacSpVec(1)];
    
    %    iaMeas: Inter ASIS distance, IA, Distance between left and right ASIS
    iaMeas = sqrt(sum((adjustedLM(lm.RtASIS,:)-adjustedLM(lm.LtASIS,:)) .^ 2));
    iaPred = 101.7456555+0.065259054*bStature+0.152209529*bWeight;
    if iaPred > iaMeas
        iaPred = iaMeas;
    end
    
%	Total Pelvic Width (mm): Intercept	49.7221519, Height (mm)	0.03868476, Inter ASIS distance (mm)	0.683445989
    tpwPred = 49.7221519+0.03868476*bStature+0.683445989*iaPred;

    % Left hip: Regression analysis based on data [1]
    %	L_HJCx (mm) Intercept	25.10635219, Pelvic Depth (mm)	-0.489291275
    %     Origin of the pelvis is set to the mid-point between the Left and right ASIS
    %     Transverse plane of the pelvis contain the A/P SIS
    hjcXdir = ASISmidCT - (25.10635219-0.489291275*pdPred) * iliacSpVec; % Along a vector from ASISmidCT to PSISmidCT
    hjcZdir = hjcXdir - (-14.16719072-0.038316849*bStature) * iliacSpVecN; % Along a vector normal to the line ASISmidCT to PSISmidCT from hjcXdir to find the final coordinate hjcZdir
    jointCenter(jc.LtHip,1) = hjcZdir(1);
    %	L_HJCy (mm) Intercept	16.43140609, Total Pelvic Width (mm)	0.12316064, Pelvic Depth (mm)	0.251066068
    jointCenter(jc.LtHip,2) = ASISmidCT(2)+16.43140609+0.12316064*tpwPred+0.251066068*pdPred;
    %	L_HJCz (mm) Intercept	-14.16719072, Height (mm)	-0.038316849
    jointCenter(jc.LtHip,3) = hjcZdir(3);

    % Right hip: Copy from left hip
    jointCenter(jc.RtHip,1) = jointCenter(jc.LtHip,1);
    jointCenter(jc.RtHip,2) = -1*jointCenter(jc.LtHip,2);
    jointCenter(jc.RtHip,3) = jointCenter(jc.LtHip,3);
    
    % Lumbosacral joint, X = 0.7006*pdPred, Z = 0.0349*iaPred
    lsjcXdir = ASISmidCT + (0.7006*pdPred) * iliacSpVec; % Along a vector from ASISmidCT to PSISmidCT
    lsjcZdir = lsjcXdir - (0.0349*iaPred) * iliacSpVecN; % Along a vector normal to the line ASISmidCT to PSISmidCT from lsjcXdir to find the final
    
    jointCenter(jc.L5S1,:) = [lsjcZdir(1) 0 lsjcZdir(3)];
    
    % Lower neck joint, C7T1
    C = sqrt((adjustedLM(lm.Suprasternale,1)-adjustedLM(lm.Cervicale,1))^2+(adjustedLM(lm.Suprasternale,3)-adjustedLM(lm.Cervicale,3))^2);
    angleLM = atan((adjustedLM(lm.Cervicale,3)-adjustedLM(lm.Suprasternale,3))/(adjustedLM(lm.Cervicale,1)-adjustedLM(lm.Suprasternale,1)));
    angleTot = angleLM+8*pi()/180;
    jointC7T1 =  zeros(1,3);
    jointC7T1(1) = adjustedLM(lm.Cervicale,1)+0.55*C*cos(angleTot);
    jointC7T1(2) = 0;
    jointC7T1(3) = adjustedLM(lm.Cervicale,3)+0.55*C*sin(angleTot);
    
    % T12L1 joint
    alfa = -0.065344916;
    ratio = 0.3868544;
    
    xDiff = jointC7T1(1)-jointCenter(jc.L5S1,1);
    yDiff = jointC7T1(3)-jointCenter(jc.L5S1,3);
    hypo = sqrt(xDiff^2+yDiff^2);
    alfa1 = asin(xDiff/hypo);
    alfa2 = alfa + alfa1;
    hypo2 = hypo*ratio;
    jointCenter(jc.T12L1,1) = hypo2*sin(alfa2)+jointCenter(jc.L5S1,1);
    jointCenter(jc.T12L1,2) = 0;
    jointCenter(jc.T12L1,3) = hypo2*cos(alfa2)+jointCenter(jc.L5S1,3);

    % L3L4 joint
    alfa = 0.27532;
    ratio = 0.415827;

    xDiff = jointCenter(jc.T12L1,1)-jointCenter(jc.L5S1,1);
    yDiff = jointCenter(jc.T12L1,3)-jointCenter(jc.L5S1,3);
    hypo = sqrt(xDiff^2+yDiff^2);
    alfa1 = asin(xDiff/hypo);
    alfa2 = alfa + alfa1;
    hypo2 = hypo*ratio;
    jointCenter(jc.L3L4,1) = hypo2*sin(alfa2)+jointCenter(jc.L5S1,1);
    jointCenter(jc.L3L4,2) = 0;
    jointCenter(jc.L3L4,3) = hypo2*cos(alfa2)+jointCenter(jc.L5S1,3);
    
     % T6T7 joint
    alfa = -0.219756;
    ratio = 0.570058;

    xDiff = jointC7T1(1)-jointCenter(jc.T12L1,1);
    yDiff = jointC7T1(3)-jointCenter(jc.T12L1,3);
    hypo = sqrt(xDiff^2+yDiff^2);
    alfa1 = asin(xDiff/hypo);
    alfa2 = alfa + alfa1;
    hypo2 = hypo*ratio;
    jointCenter(jc.T6T7,1) = hypo2*sin(alfa2)+jointCenter(jc.T12L1,1);
    jointCenter(jc.T6T7,2) = 0;
    jointCenter(jc.T6T7,3) = hypo2*cos(alfa2)+jointCenter(jc.T12L1,3);

    % T1T2 joint
    alfa = -0.031017623;
    ratio = 0.93397206;

    xDiff = jointC7T1(1)-jointCenter(jc.T12L1,1);
    yDiff = jointC7T1(3)-jointCenter(jc.T12L1,3);
    hypo = sqrt(xDiff^2+yDiff^2);
    alfa1 = asin(xDiff/hypo);
    alfa2 = alfa + alfa1;
    hypo2 = hypo*ratio;
    jointCenter(jc.T1T2,1) = hypo2*sin(alfa2)+jointCenter(jc.T12L1,1);
    jointCenter(jc.T1T2,2) = 0;
    jointCenter(jc.T1T2,3) = hypo2*cos(alfa2)+jointCenter(jc.T12L1,3);
        
    % Upper neck joint, AtlantoAxial
    A = sqrt((adjustedLM(lm.RtInfraorbitale,1)-adjustedLM(lm.RtTragion,1))^2+(adjustedLM(lm.RtInfraorbitale,3)-adjustedLM(lm.RtTragion,3))^2);
    angleLM = atan((adjustedLM(lm.RtInfraorbitale,3)-adjustedLM(lm.RtTragion,3))/(adjustedLM(lm.RtInfraorbitale,1)-adjustedLM(lm.RtTragion,1)));
    angleTot = angleLM-117*pi()/180;
    jointCenter(jc.AtlantoAxial,1) = adjustedLM(lm.RtTragion,1)+0.31*A*cos(angleTot);
    jointCenter(jc.AtlantoAxial,2) = 0;
    jointCenter(jc.AtlantoAxial,3) = adjustedLM(lm.RtTragion,3)+0.31*A*sin(angleTot);

    % C6C7 joint
    alfa = 0.153440233;
    ratio = 0.168899352;

    xDiff = jointCenter(jc.AtlantoAxial,1)-jointC7T1(1);
    yDiff = jointCenter(jc.AtlantoAxial,3)-jointC7T1(3);
    hypo = sqrt(xDiff^2+yDiff^2);
    alfa1 = asin(xDiff/hypo);
    alfa2 = alfa + alfa1;
    hypo2 = hypo*ratio;
    jointCenter(jc.C6C7,1) = hypo2*sin(alfa2)+jointC7T1(1);
    jointCenter(jc.C6C7,2) = 0;
    jointCenter(jc.C6C7,3) = hypo2*cos(alfa2)+jointC7T1(3);

    % C4C5 joint
    alfa = 0.06499;
    ratio = 0.480274;

    xDiff = jointCenter(jc.AtlantoAxial,1)-jointC7T1(1);
    yDiff = jointCenter(jc.AtlantoAxial,3)-jointC7T1(3);
    hypo = sqrt(xDiff^2+yDiff^2);
    alfa1 = asin(xDiff/hypo);
    alfa2 = alfa + alfa1;
    hypo2 = hypo*ratio;
    jointCenter(jc.C4C5,1) = hypo2*sin(alfa2)+jointC7T1(1);
    jointCenter(jc.C4C5,2) = 0;
    jointCenter(jc.C4C5,3) = hypo2*cos(alfa2)+jointC7T1(3);

    jointCenter(jc.Head,:) = adjustedLM(lm.Sellion,:);
    
    % Knee joints
    jointCenter(jc.RtKnee,:) = (adjustedLM(lm.RtFemoralLateralEpicn,:)+adjustedLM(lm.RtFemoralMedialEpicn,:))/2;
    jointCenter(jc.LtKnee,:) = (adjustedLM(lm.LtFemoralLateralEpicn,:)+adjustedLM(lm.LtFemoralMedialEpicn,:))/2;
    % Ankle joints
    jointCenter(jc.RtAnkle,:) = (adjustedLM(lm.RtLateralMalleolus,:)+adjustedLM(lm.RtMedialMalleolus,:))/2;
    jointCenter(jc.LtAnkle,:) = (adjustedLM(lm.LtLateralMalleolus,:)+adjustedLM(lm.LtMedialMalleolus,:))/2;
    % Toe joints
    jointCenter(jc.RtToe,:) = (adjustedLM(lm.RtMetatarsalPhalV,:)+adjustedLM(lm.RtMetatarsalPhalI,:))/2;
    jointCenter(jc.LtToe,:) = (adjustedLM(lm.LtMetatarsalPhalV,:)+adjustedLM(lm.LtMetatarsalPhalI,:))/2;
    jointCenter(jc.RtToeTip,:) = adjustedLM(lm.RtDigitII,:);
    jointCenter(jc.LtToeTip,:) = adjustedLM(lm.LtDigitII,:);

    % Sterno-clavicular joints
    jointCenter(jc.RtSternoClavicular,1) = adjustedLM(lm.RtClavicale,1)-(adjustedLM(lm.Suprasternale,1)-adjustedLM(lm.RtClavicale,1))/2; % Going into the body half the distance as between Suprasternale and clavicle
    jointCenter(jc.RtSternoClavicular,2) = adjustedLM(lm.RtClavicale,2); % Otherwise the same position as Clavicle LM
    jointCenter(jc.RtSternoClavicular,3) = adjustedLM(lm.RtClavicale,3);

    jointCenter(jc.LtSternoClavicular,1) = adjustedLM(lm.LtClavicale,1)-(adjustedLM(lm.Suprasternale,1)-adjustedLM(lm.LtClavicale,1))/2;
    jointCenter(jc.LtSternoClavicular,2) = adjustedLM(lm.LtClavicale,2);
    jointCenter(jc.LtSternoClavicular,3) = adjustedLM(lm.LtClavicale,3);
    
    acZR = (meshModel(5144,:)+meshModel(5145,:)+meshModel(5158,:)+meshModel(5254,:))/4;
    acZL = (meshModel(4764,:)+meshModel(4765,:)+meshModel(4778,:)+meshModel(4874,:))/4;
    acZ = (acZR+acZL)/2;

    % AcromioClavicular joints
    jointCenter(jc.RtAcromioClavicular,1) = adjustedLM(lm.RtAcromion,1); % Same position as Acromion
    jointCenter(jc.RtAcromioClavicular,2) = adjustedLM(lm.RtAcromion,2)-0.22*(adjustedLM(lm.RtAcromion,2)-adjustedLM(lm.RtClavicale,2)); % Distance from Acromion in relation to Clavicle -0.175 originally from estimation on plastic skeleton
    jointCenter(jc.RtAcromioClavicular,3) = acZ(3);

    jointCenter(jc.LtAcromioClavicular,1) = adjustedLM(lm.LtAcromion,1);
    jointCenter(jc.LtAcromioClavicular,2) = adjustedLM(lm.LtAcromion,2)-0.22*(adjustedLM(lm.LtAcromion,2)-adjustedLM(lm.LtClavicale,2));
    jointCenter(jc.LtAcromioClavicular,3) = acZ(3);

    % GlenoHumeral (Shoulder) joints
    angleLM = atan((adjustedLM(lm.Cervicale,3)-adjustedLM(lm.Suprasternale,3))/(adjustedLM(lm.Cervicale,1)-adjustedLM(lm.Suprasternale,1)));
    angleTot = angleLM-67*pi()/180;
    jointCenter(jc.RtShoulder,1) = 0.48*C*cos(angleTot)+adjustedLM(lm.RtAcromion,1); % 0.42*C in Reed (~2000)
    jointCenter(jc.RtShoulder,2) = adjustedLM(lm.RtAcromion,2)-(adjustedLM(lm.RtAcromion,2)-jointCenter(jc.RtAcromioClavicular,2))/2; % Right side
    jointCenter(jc.RtShoulder,3) = 0.48*C*sin(angleTot)+adjustedLM(lm.RtAcromion,3);

    jointCenter(jc.LtShoulder,1) = jointCenter(jc.RtShoulder,1);
    jointCenter(jc.LtShoulder,2) = adjustedLM(lm.LtAcromion,2)-(adjustedLM(lm.LtAcromion,2)-jointCenter(jc.LtAcromioClavicular,2))/2; % Left side
    jointCenter(jc.LtShoulder,3) = jointCenter(jc.RtShoulder,3);

    % Elbow joints
%     midPoint(1:3) = (adjustedLM(lm.RtHumeralLateralEpicn,:)+adjustedLM(lm.RtHumeralMedialEpicn,:))/2;
%     jointCenter(jc.RtElbow,:) = (adjustedLM(lm.RtOlecranon,:)+midPoint(1:3))/2;
    jointCenter(jc.RtElbow,:) = (adjustedLM(lm.RtHumeralLateralEpicn,:)+adjustedLM(lm.RtHumeralMedialEpicn,:))/2;

%     midPoint(1:3) = (adjustedLM(lm.LtHumeralLateralEpicn,:)+adjustedLM(lm.LtHumeralMedialEpicn,:))/2;
%     jointCenter(jc.LtElbow,:) = (adjustedLM(lm.LtOlecranon,:)+midPoint(1:3))/2;
    jointCenter(jc.LtElbow,:) = (adjustedLM(lm.LtHumeralLateralEpicn,:)+adjustedLM(lm.LtHumeralMedialEpicn,:))/2;

    % Wrist joints
    jointCenter(jc.RtWrist,:) = (adjustedLM(lm.RtRadialStyloid,:)+adjustedLM(lm.RtUlnarStyloid,:))./2;
    jointCenter(jc.LtWrist,:) = (adjustedLM(lm.LtRadialStyloid,:)+adjustedLM(lm.LtUlnarStyloid,:))./2;

    %%% HAND JOINT CENTRES %%% <------ Adds joints in both hands and adjust mesh and joints to be aligned and symmetrical.
    load_handJointRegressors = load('statBodyModel/rightHandJointRegressors.mat');
    W = load_handJointRegressors.W;
    handMesh = meshModel(r.rightArmHand,:);
    jointCenter(jc.RtIndexCarpal:jc.RtThumbDist,:) = W * handMesh;
%     % Copy JCs from RIGHT to LEFT side
    jointCenter(jc.LtIndexCarpal:jc.LtThumbDist,:) = jointCenter(jc.RtIndexCarpal:jc.RtThumbDist,:);
    jointCenter(jc.LtIndexCarpal:jc.LtThumbDist,2) = jointCenter(jc.LtIndexCarpal:jc.LtThumbDist,2)*-1;
   
    %% insideHand = [1.insidePalm, 2.insideThumb, 3.insideIndex, 4.insideMiddle, 5.insideRing, 6.insidePinky];
    insideHandTotalRight = load('insideHandRight.mat').insideHandTotalRight;
    insideHandTotalLeft = load('insideHandLeft.mat').insideHandTotalLeft;
    
    % timePinkyFixStart = tic;
    %% Fixing pinky top (left & right), time approx. 0.0045 sec
    % Fixing left pinky top
    meshStart = r.leftArmHand(1);
    inside = insideHandTotalLeft(:,6);
    maxLength = sqrt(sum((jointCenter(jc.LtPinkyDist,:) - jointCenter(jc.LtPinkyProx,:)).^ 2));
    minDistId = 1331; % Found earlier.
    fingerDiff = jointCenter(jc.LtPinkyDist,:) - meshModel(minDistId+meshStart-1,:);
    for i=1:size(inside,1)
        if inside(i)
            fingerTipDist = sqrt(sum((meshModel(minDistId+meshStart-1,:) - meshModel(i+meshStart-1,:)) .^ 2)); % Calculate how close to the mesh finger tip.
            heatIndxPoint = 1-(fingerTipDist/maxLength);
            if heatIndxPoint>0
                meshModel(i+meshStart-1,:) = meshModel(i+meshStart-1,:) + fingerDiff*heatIndxPoint;
            end
        end
    end
    
    % Fixing right pinky top
    meshStart = r.rightArmHand(1);
    inside = insideHandTotalRight(:,6);
    maxLength = sqrt(sum((jointCenter(jc.RtPinkyDist,:) - jointCenter(jc.RtPinkyProx,:)).^ 2));
    minDistId = 1129; % Found earlier.
    fingerDiff = jointCenter(jc.RtPinkyDist,:) - meshModel(minDistId+meshStart-1,:);
    for i=1:size(inside,1)
        if inside(i)
            fingerTipDist = sqrt(sum((meshModel(minDistId+meshStart-1,:) - meshModel(i+meshStart-1,:)) .^ 2)); % Calculate how close to the mesh finger tip.
            heatIndxPoint = 1-(fingerTipDist/maxLength);
            if heatIndxPoint>0
                meshModel(i+meshStart-1,:) = meshModel(i+meshStart-1,:) + fingerDiff*heatIndxPoint;
            end
        end
    end  
    
    %% insideHand = [1.insidePalm, 2.insideThumb, 3.insideIndex, 4.insideMiddle, 5.insideRing, 6.insidePinky];
    fingerDiff = adjustedLM(lm.RtDactylion,:) - jointCenter(jc.RtMiddleDist,:);  % Check distance between middle distal and dactylion
    % This computes the delta for the middle finger and applies the same vector to INDEX, MIDDLE, and RING fingers. It is intentional (treating dactylion as a hand-wide reference)
    % Pinky and Thumb are skipped since they do not need the adjustment
    
    %% For each finger, create heat map from distal JC as zero point with maxlength = carpal - distal.
    % Fixing right indx top
    meshStart = r.rightArmHand(1);
    inside = insideHandTotalRight(:,3); % 3.insideIndex
    fingerTip = jointCenter(jc.RtIndexDist,:);
    fingerCarpal = jointCenter(jc.RtIndexCarpal,:);
    maxLength = sqrt(sum((fingerTip - fingerCarpal).^ 2));
    for i=1:size(inside,1)
        if inside(i)
            fingerTipDist = sqrt(sum((fingerTip - meshModel(i+meshStart-1,:)) .^ 2)); % Calculate how close to the finger tip.
            heatIndxPoint = 1-(fingerTipDist/maxLength);
            if heatIndxPoint>0
                meshModel(i+meshStart-1,:) = meshModel(i+meshStart-1,:) + fingerDiff*heatIndxPoint;
            end
        end
    end
    for jcRow=jc.RtIndexProx:jc.RtIndexDist
        fingerTipDist = sqrt(sum((fingerTip - jointCenter(jcRow,:)) .^ 2));
        heatIndxPoint = 1-(fingerTipDist/maxLength);
        jointCenter(jcRow,:) = jointCenter(jcRow,:) + fingerDiff*heatIndxPoint;
    end
    % test = 1;
    inside = insideHandTotalRight(:,4); % 4.insideMiddle
    fingerTip = jointCenter(jc.RtMiddleDist,:);
    fingerCarpal = jointCenter(jc.RtMiddleCarpal,:);
    maxLength = sqrt(sum((fingerTip - fingerCarpal).^ 2));
    for i=1:size(inside,1)
        if inside(i)
            fingerTipDist = sqrt(sum((fingerTip - meshModel(i+meshStart-1,:)) .^ 2)); % Calculate how close to the finger tip.
            heatIndxPoint = 1-(fingerTipDist/maxLength);
            if heatIndxPoint>0
                meshModel(i+meshStart-1,:) = meshModel(i+meshStart-1,:) + fingerDiff*heatIndxPoint;
            end
        end
    end
    for jcRow=jc.RtMiddleProx:jc.RtMiddleDist
        fingerTipDist = sqrt(sum((fingerTip - jointCenter(jcRow,:)) .^ 2));
        heatIndxPoint = 1-(fingerTipDist/maxLength);
        jointCenter(jcRow,:) = jointCenter(jcRow,:) + fingerDiff*heatIndxPoint;
    end
    % test = 1;
    inside = insideHandTotalRight(:,5); % 5.insideRing
    fingerTip = jointCenter(jc.RtRingDist,:);
    fingerCarpal = jointCenter(jc.RtRingCarpal,:);
    maxLength = sqrt(sum((fingerTip - fingerCarpal).^ 2));
    for i=1:size(inside,1)
        if inside(i)
            fingerTipDist = sqrt(sum((fingerTip - meshModel(i+meshStart-1,:)) .^ 2)); % Calculate how close to the finger tip.
            heatIndxPoint = 1-(fingerTipDist/maxLength);
            if heatIndxPoint>0
                meshModel(i+meshStart-1,:) = meshModel(i+meshStart-1,:) + fingerDiff*heatIndxPoint;
            end
        end
    end
    for jcRow=jc.RtRingProx:jc.RtRingDist
        fingerTipDist = sqrt(sum((fingerTip - jointCenter(jcRow,:)) .^ 2));
        heatIndxPoint = 1-(fingerTipDist/maxLength);
        jointCenter(jcRow,:) = jointCenter(jcRow,:) + fingerDiff*heatIndxPoint;
    end

    %% insideHand = [1.insidePalm, 2.insideThumb, 3.insideIndex, 4.insideMiddle, 5.insideRing, 6.insidePinky];
    %% Copy to left side %%
    jointCenter(jc.LtIndexCarpal:jc.LtPinkyDist,:) = jointCenter(jc.RtIndexCarpal:jc.RtPinkyDist,:);
    jointCenter(jc.LtIndexCarpal:jc.LtPinkyDist,2) = -1*jointCenter(jc.LtIndexCarpal:jc.LtPinkyDist,2);
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

    %% Align bust points to landmarks
    % 13124:13132;  left nipple; Left midpoint: 13127
    % 13133:13141;  right nipple; Right midpoint: 13136
    % LM: 13	RtThelion/Bustpoint
    % LM: 14	LtThelion/Bustpoint   
    bustTipMesh = meshModel(r.leftNippleMid,:);
    bustTipLM = adjustedLM(lm.LtThelion,:);
    bustDiff = bustTipLM - bustTipMesh;
    maxLength = abs(bustTipMesh(2)); % Includes all points around bustpoint to mid sagittal plane.
    insideChest = pointInSphere(meshModel, bustTipMesh, maxLength);
    for i=1:size(insideChest,1)
        if insideChest(i)
            bustTipDist = sqrt(sum((bustTipMesh - meshModel(i,:)) .^ 2)); % Calculate how close to the bust tip.
            heatIndxPoint = 1-(bustTipDist/maxLength);
            if heatIndxPoint>0
                meshModel(i,:) = meshModel(i,:) + bustDiff*heatIndxPoint;
            end
        end
    end
    bustTipMesh = meshModel(r.rightNippleMid,:);
    bustTipLM = adjustedLM(lm.RtThelion,:);
    bustDiff = bustTipLM - bustTipMesh;
    maxLength = abs(bustTipMesh(2));
    insideChest = pointInSphere(meshModel, bustTipMesh, maxLength);
    for i=1:size(insideChest,1)
        if insideChest(i)
            bustTipDist = sqrt(sum((bustTipMesh - meshModel(i,:)) .^ 2)); % Calculate how close to the bust tip.
            heatIndxPoint = 1-(bustTipDist/maxLength);
            if heatIndxPoint>0
                meshModel(i,:) = meshModel(i,:) + bustDiff*heatIndxPoint;
            end
        end
    end 
end

% From:To;      BodyPart;                   NrOfPoints
% 1:4677;       head and neck;              4677
% 4678:5446;    torso;                      769
% 5447:7585;    feet;                       2139
% 7586:8276;    top of head;                691
% 8277:9662;    pelvis and legs;            1386
% 9663:11390;   right arm and hand;         1728
% 11391:13116;  left arm and hand;          1726
% 13117:13123;  buttocks line;              7
% 13124:13132;  left nipple;                9
% 13133:13141;  right nipple;               9
% 13142:13143;  maximum abdominal depth;    2
