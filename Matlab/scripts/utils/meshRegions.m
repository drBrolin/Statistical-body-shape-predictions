function r = meshRegions()
% meshRegions  Named vertex index ranges and special vertex indices for the
%              13,143-vertex body mesh.
%
% Usage:
%   r = meshRegions();
%   meshFoot = meshModel(r.feet, :);

    % ---- Body-part ranges ----
    r.headNeck      = 1:4677;
    r.torso         = 4678:5446;
    r.feet          = 5447:7585;
    r.headTop       = 7586:8276;
    r.pelvisLegs    = 8277:9662;
    r.rightArmHand  = 9663:11390;
    r.leftArmHand   = 11391:13116;
    r.buttocksLine  = 13117:13123;
    r.leftNipple    = 13124:13132;
    r.rightNipple   = 13133:13141;
    r.abdomenDepth  = 13142:13143;

    % ---- Foot and lower-leg sub-ranges ----
    r.leftFoot      = 5447:6517;   % left foot vertices within feet region
    r.rightFoot     = 6518:7585;   % right foot vertices within feet region
    r.leftLowerLeg  = 8277:8969;   % left lower leg (used in rotateFeetTpose)
    r.rightLowerLeg = 8970:9662;   % right lower leg (used in rotateFeetTpose)

    % ---- Other sub-ranges ----
    r.headTopScale  = 7684:8375;   % top-of-head vertices for stature scaling

    % ---- Individual special vertices ----
    r.leftNippleMid  = 13127;  % midpoint of left nipple cluster  (LM 14 LtThelion)
    r.rightNippleMid = 13136;  % midpoint of right nipple cluster (LM 13 RtThelion)
end
