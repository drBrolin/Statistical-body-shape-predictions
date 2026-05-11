function lm = lmIndices()
% lmIndices  Named row indices into the 74-row adjustedLM landmark array.
%
% Row order matches labelLandmarks.txt (rows 1-73 predicted, row 74 = T8
% added in mainInput_IPS.m).
%
% Usage:
%   lm = lmIndices();
%   cervicale = adjustedLM(lm.Cervicale, :);

    % ---- Head ----
    lm.Sellion                = 1;
    lm.RtInfraorbitale        = 2;
    lm.LtInfraorbitale        = 3;
    lm.Supramenton            = 4;
    lm.RtTragion              = 5;
    lm.RtGonion               = 6;
    lm.LtTragion              = 7;
    lm.LtGonion               = 8;
    lm.Nuchale                = 9;

    % ---- Thorax / clavicle ----
    lm.RtClavicale            = 10;
    lm.Suprasternale          = 11;
    lm.LtClavicale            = 12;
    lm.RtThelion              = 13;   % RtThelion/Bustpoint
    lm.LtThelion              = 14;   % LtThelion/Bustpoint
    lm.Substernale            = 15;
    lm.Rt10thRib              = 16;
    lm.RtASIS                 = 17;
    lm.Lt10thRib              = 18;
    lm.LtASIS                 = 19;
    lm.RtIliocristale         = 20;
    lm.RtTrochanterion        = 21;
    lm.LtIliocristale         = 22;
    lm.LtTrochanterion        = 23;
    lm.Cervicale              = 24;
    lm.Rib10MidSpine          = 25;   % 10thRibMidspine
    lm.RtPSIS                 = 26;
    lm.LtPSIS                 = 27;
    lm.WaistPreferredPost     = 28;

    % ---- Right arm ----
    lm.RtAcromion             = 29;
    lm.RtAxillaAnt            = 30;
    lm.RtRadialStyloid        = 31;
    lm.RtAxillaPost           = 32;
    lm.RtOlecranon            = 33;
    lm.RtHumeralLateralEpicn  = 34;
    lm.RtHumeralMedialEpicn   = 35;
    lm.RtRadiale              = 36;
    lm.RtMetacarpalPhalII     = 37;
    lm.RtDactylion            = 38;
    lm.RtUlnarStyloid         = 39;
    lm.RtMetacarpalPhalV      = 40;

    % ---- Left arm ----
    lm.LtAcromion             = 41;
    lm.LtAxillaAnt            = 42;
    lm.LtRadialStyloid        = 43;
    lm.LtAxillaPost           = 44;
    lm.LtOlecranon            = 45;
    lm.LtHumeralLateralEpicn  = 46;
    lm.LtHumeralMedialEpicn   = 47;
    lm.LtRadiale              = 48;
    lm.LtMetacarpalPhalII     = 49;
    lm.LtDactylion            = 50;
    lm.LtUlnarStyloid         = 51;
    lm.LtMetacarpalPhalV      = 52;

    % ---- Right leg / foot ----
    lm.RtKneeCrease           = 53;
    lm.RtFemoralLateralEpicn  = 54;
    lm.RtFemoralMedialEpicn   = 55;
    lm.RtMetatarsalPhalV      = 56;
    lm.RtLateralMalleolus     = 57;
    lm.RtMedialMalleolus      = 58;
    lm.RtSphyrion             = 59;
    lm.RtMetatarsalPhalI      = 60;
    lm.RtCalcaneousPost       = 61;
    lm.RtDigitII              = 62;

    % ---- Left leg / foot ----
    lm.LtKneeCrease           = 63;
    lm.LtFemoralLateralEpicn  = 64;
    lm.LtFemoralMedialEpicn   = 65;
    lm.LtMetatarsalPhalV      = 66;
    lm.LtLateralMalleolus     = 67;
    lm.LtMedialMalleolus      = 68;
    lm.LtSphyrion             = 69;
    lm.LtMetatarsalPhalI      = 70;
    lm.LtCalcaneousPost       = 71;
    lm.LtDigitII              = 72;

    % ---- Other ----
    lm.Crotch                 = 73;
    lm.T8                     = 74;   % added in mainInput_IPS.m (not in base prediction)
end
