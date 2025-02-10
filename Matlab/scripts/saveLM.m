function saveTrue = saveLM(adjustedLM, modelName)
    % Landmark labels can be found in file: labelLandmarks.txt
    filename = ['output\','BM_', modelName, '_landmarks.xyz'];
    fileID = fopen(filename,'w');
    for i=1:size(adjustedLM,1)
        fprintf(fileID, '%5.5f ', adjustedLM(i,1:3)/1000);
        fprintf(fileID,'\n');
    end
    fclose(fileID);
    saveTrue = 1;
end