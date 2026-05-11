function adjustedLM = landmarkAdjust(regressionRow)
    % Pairs for aligning landmarks
    pairs = [2	3;
    5	7;
    6	8;
    10	12;
    13	14;
    16	18;
    17	19;
    20	22;
    21	23;
    26	27;
    29	41;
    30	42;
    31	43;
    32	44;
    33	45;
    34	46;
    35	47;
    36	48;
    37	49;
    38	50;
    39	51;
    40	52;
    53	63;
    54	64;
    55	65;
    56	66;
    57	67;
    58	68;
    59	69;
    60	70;
    61	71;
    62	72];

    % Midpoints for aligning landmarks
    midPoints = [1, 4, 9, 11, 15, 24, 25, 28, 73];

    landmarks = zeros(73,3);
    for i=1:size(landmarks,1)
        landmarks(i,1:3) = regressionRow(1,(i*3-2):(i*3));
    end

    landmarksAdj = landmarks;
    for i=1:size(pairs,1)
        landmarksAdj(pairs(i,1),1) = (landmarks(pairs(i,1),1)+landmarks(pairs(i,2),1))/2; % average value of x-coordinate
        landmarksAdj(pairs(i,1),2) = (landmarks(pairs(i,1),2)-landmarks(pairs(i,2),2))/2; % average value of y-coordinate, minus since left and right side have opposite values
        landmarksAdj(pairs(i,1),3) = (landmarks(pairs(i,1),3)+landmarks(pairs(i,2),3))/2; % average value of z-coordinate
        landmarksAdj(pairs(i,2),1) = landmarksAdj(pairs(i,1),1); % set the left and right side to the same x-coordinate
        landmarksAdj(pairs(i,2),2) = landmarksAdj(pairs(i,1),2)*-1; % set the left and right side to different y-coordinate
        landmarksAdj(pairs(i,2),3) = landmarksAdj(pairs(i,1),3); % set the left and right side to the same z-coordinate
    end

    for i=1:size(midPoints,2)
        landmarksAdj(midPoints(i),2) = 0;
    end
    adjustedLM = regressionRow;
    for i=1:size(landmarksAdj,1)
        adjustedLM(1,(i-1)*3+1:(i-1)*3+3) = landmarksAdj(i,1:3);
    end
end