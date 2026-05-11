function saveTrue = saveXYZ(xyzCoord, modelName, LMtrue)
    if LMtrue == 1
       filename = ['output/','BM_', modelName, '_landmarks.xyz'];
    else
       filename = ['output/','BM_', modelName, '_jointcenters.xyz'];
    end
    if ~exist('output', 'dir'); mkdir('output'); end
    fileID = fopen(filename,'w');
    if fileID < 0; error('Could not open file for writing: %s', filename); end
    for i=1:size(xyzCoord,1)
        fprintf(fileID, '%5.5f ', xyzCoord(i,1:3)/1000);
        fprintf(fileID,'\n');
    end
    fclose(fileID);
    saveTrue = 1;
end