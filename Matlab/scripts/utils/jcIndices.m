function jc = jcIndices()
% jcIndices  Named row indices into the 82-row jointCenter/jointCenterT array.
%
% Joints 1-69 are predicted by jcPrediction.m.
% Joints 70-82 are computed as midpoints / surface points in mainInput_IPS.m.
%
% Coordinate system (pre-output): X = anterior/posterior, Y = lateral
%   (positive Y = left, negative Y = right), Z = superior/inferior.
%
% Usage:
%   jc = jcIndices();
%   hipLeft = jointCenter(jc.LtHip, :);

    % ---- Spine ----
    jc.L5S1            = 1;
    jc.L3L4            = 2;
    jc.T12L1           = 3;
    jc.T6T7            = 4;
    jc.T1T2            = 5;
    jc.C6C7            = 6;
    jc.C4C5            = 7;
    jc.AtlantoAxial    = 8;
    jc.Head            = 9;   % set to Sellion landmark (LM 1)

    % ---- Left leg ----
    jc.LtHip           = 10;
    jc.LtKnee          = 11;
    jc.LtAnkle         = 12;
    jc.LtToe           = 13;  % midpoint of LtMetatarsalPhalV and LtMetatarsalPhalI

    % ---- Right leg ----
    jc.RtHip           = 14;
    jc.RtKnee          = 15;
    jc.RtAnkle         = 16;
    jc.RtToe           = 17;  % midpoint of RtMetatarsalPhalV and RtMetatarsalPhalI

    % ---- Right arm ----
    jc.RtSternoClavicular   = 18;
    jc.RtAcromioClavicular  = 19;
    jc.RtShoulder           = 20;  % GlenoHumeral
    jc.RtElbow              = 21;
    jc.RtWrist              = 22;

    % ---- Left arm ----
    jc.LtSternoClavicular   = 23;
    jc.LtAcromioClavicular  = 24;
    jc.LtShoulder           = 25;  % GlenoHumeral
    jc.LtElbow              = 26;
    jc.LtWrist              = 27;

    % ---- Left hand (mirror of right, 28:47) ----
    jc.LtIndexCarpal   = 28;  jc.LtIndexProx   = 29;  jc.LtIndexInter  = 30;  jc.LtIndexDist   = 31;
    jc.LtMiddleCarpal  = 32;  jc.LtMiddleProx  = 33;  jc.LtMiddleInter = 34;  jc.LtMiddleDist  = 35;
    jc.LtRingCarpal    = 36;  jc.LtRingProx    = 37;  jc.LtRingInter   = 38;  jc.LtRingDist    = 39;
    jc.LtPinkyCarpal   = 40;  jc.LtPinkyProx   = 41;  jc.LtPinkyInter  = 42;  jc.LtPinkyDist   = 43;
    jc.LtThumbCarpal   = 44;  jc.LtThumbProx   = 45;  jc.LtThumbInter  = 46;  jc.LtThumbDist   = 47;

    % ---- Right hand (from mesh regressor, 48:67) ----
    jc.RtIndexCarpal   = 48;  jc.RtIndexProx   = 49;  jc.RtIndexInter  = 50;  jc.RtIndexDist   = 51;
    jc.RtMiddleCarpal  = 52;  jc.RtMiddleProx  = 53;  jc.RtMiddleInter = 54;  jc.RtMiddleDist  = 55;
    jc.RtRingCarpal    = 56;  jc.RtRingProx    = 57;  jc.RtRingInter   = 58;  jc.RtRingDist    = 59;
    jc.RtPinkyCarpal   = 60;  jc.RtPinkyProx   = 61;  jc.RtPinkyInter  = 62;  jc.RtPinkyDist   = 63;
    jc.RtThumbCarpal   = 64;  jc.RtThumbProx   = 65;  jc.RtThumbInter  = 66;  jc.RtThumbDist   = 67;

    % ---- Toe tips ----
    jc.RtToeTip        = 68;  % RtDigitII landmark
    jc.LtToeTip        = 69;  % LtDigitII landmark

    % ---- Segment midpoints (added in mainInput_IPS.m) ----
    jc.RtUpperArmMid   = 70;  % midpoint of RtShoulder and RtElbow
    jc.LtUpperArmMid   = 71;  % midpoint of LtShoulder and LtElbow
    jc.RtLowerArmMid   = 72;  % midpoint of RtElbow and RtWrist
    jc.LtLowerArmMid   = 73;  % midpoint of LtElbow and LtWrist
    jc.RtLowerLegMid   = 74;  % midpoint of RtKnee and RtAnkle
    jc.LtLowerLegMid   = 75;  % midpoint of LtKnee and LtAnkle

    % ---- Surface / width points (added in mainInput_IPS.m) ----
    jc.AbdominalDepth  = 76;  % max anterior abdominal point (sagittal midline)
    jc.RtHipWidth      = 77;  % widest right hip surface point
    jc.LtHipWidth      = 78;  % widest left hip surface point
    jc.RtHipBone       = 79;  % right iliac crest surface point
    jc.LtHipBone       = 80;  % left iliac crest surface point
    jc.RtButtock       = 81;  % posteriormost right buttock surface point
    jc.LtButtock       = 82;  % posteriormost left buttock surface point
end
