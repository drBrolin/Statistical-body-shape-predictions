function saveTrue = saveMesh(meshModel, modelName)
    filename = ['output\', modelName, '_mesh_Stat.obj'];
    fileID = fopen(filename,'w');
    for i=1:size(meshModel,1)
        fprintf(fileID,'v ');
        fprintf(fileID, '%5.5f ', meshModel(i,1:3)/1000);
        fprintf(fileID,'\n');
    end
    objFileEnd = readmatrix('objFileEnd.txt');
    for i=1:size(objFileEnd,1)
        fprintf(fileID,'f ');
        fprintf(fileID, '%5.0f ', objFileEnd(i,[1,3,2]));
        fprintf(fileID,'\n');
    end
    
    fclose(fileID);
    saveTrue = 1;
end