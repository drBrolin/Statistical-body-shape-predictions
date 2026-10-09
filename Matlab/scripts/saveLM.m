function saveTrue = saveLM(adjustedLM, meshModel, jointCenterT, modelName)
    % % Landmark labels can be found in file: labelLandmarks.txt
    % filename = ['output/','BM_', modelName, '_landmarks.xyz'];
    % if ~exist('output', 'dir'); mkdir('output'); end
    % fileID = fopen(filename,'w');
    % if fileID < 0; error('Could not open file for writing: %s', filename); end
    % for i=1:size(adjustedLM,1)
    %     fprintf(fileID, '%5.5f ', adjustedLM(i,1:3)/1000);
    %     fprintf(fileID,'\n');
    % end
    % fclose(fileID);
    addpath('./scripts/utils/');
    r = meshRegions(); lm = lmIndices(); jc = jcIndices();

        % Landmark labels can be found in file: labelLandmarks.txt
    filename = ['output/','BM_', modelName, '_LMs.lm'];
    if ~exist('output', 'dir'); mkdir('output'); end
    fileID = fopen(filename,'w');
    if fileID < 0; error('Could not open file for writing: %s', filename); end
    listLM = zeros(23,3);    
    % TopOfHead
    [~,topOfHead] = max(meshModel(r.headTopScale,2));
    listLM(1,:) = meshModel(r.headTopScale(topOfHead),:);
    listLM(2,:) = jointCenterT(jc.RtIndexDist,:); 
    listLM(3,:) = jointCenterT(jc.RtMiddleDist,:);
    listLM(4,:) = jointCenterT(jc.RtRingDist,:);
    listLM(5,:) = jointCenterT(jc.RtPinkyDist,:);
    listLM(6,:) = jointCenterT(jc.RtThumbDist,:);
    listLM(7,:) = jointCenterT(jc.LtIndexDist,:); 
    listLM(8,:) = jointCenterT(jc.LtMiddleDist,:);
    listLM(9,:) = jointCenterT(jc.LtRingDist,:);
    listLM(10,:) = jointCenterT(jc.LtPinkyDist,:);
    listLM(11,:) = jointCenterT(jc.LtThumbDist,:);
    listLM(12,:) = adjustedLM(lm.LtThelion,:);
    listLM(13,:) = adjustedLM(lm.RtThelion,:);

    meshFootRight = meshModel(r.rightFoot,:);
    toePoints = load('insideFootRight.mat').rightToes;  % (N_feet x 5): logical mask per toe
    toePoints = toePoints(numel(r.leftFoot)+1:end, :);  % trim to right foot rows only
    for i = 1:5
        maxX = 0;
        maxId = 0;
        for n = 1:size(meshFootRight,1)
            if toePoints(n,i) && meshFootRight(n,1) > maxX
                maxX = meshFootRight(n,1);
                maxId = n;
            end
        end
        listLM(13+i,:) = meshFootRight(maxId,:);
    end
    for i = 1:5
        listLM(18+i,:) = listLM(13+i,:);
        listLM(18+i,3) = -1*listLM(18+i,3);
    end

    for i=1:size(listLM,1)
        fprintf(fileID, '%5.5f ', listLM(i,1:3)/1000);
        fprintf(fileID,'\n');
    end


    fclose(fileID);
    saveTrue = 1;
end