function matchedMesh = saveMeshTpose(meshModel, modelName,sex)
    meshModelMeter = meshModel(:,1:3)/1000;
    % loadRegressors = load('statBodyModel/translation30_MakeHuman.mat');
    loadRelation = load('statBodyModel/translationMakeHuman8points.mat');
    loadRelationEyes = load('statBodyModel/translationEyes8points.mat');
    
    %% Tried this, not successfull yet.
    % W = loadRegressors.W;
    % matchedMesh = W * meshModelMeter;

    %% This is the old verison, works well
    distanceMeas = loadRelation.distanceMeas;
    matchedMesh = zeros(size(distanceMeas,1),3);
    nrPoints = 8;
    for i=1:size(distanceMeas,1)
        distSum = 0;
        for n=1:nrPoints
            distSum = distSum + distanceMeas(i,(n-1)*5+2);
        end
        for n=1:nrPoints
            matchedMesh(i,:) = matchedMesh(i,:) + (distanceMeas(i,(n-1)*5+3:(n-1)*5+5)+meshModelMeter(distanceMeas(i,(n-1)*5+1),:))*(distanceMeas(i,(n-1)*5+2)/distSum);
        end
    end

    % Save symmetrized mesh using original OBJ structure with replaced vertex positions
    fid_in = fopen('utils/makeHumanPos.obj', 'r');
    allLines_orig = textscan(fid_in, '%s', 'Delimiter', '\n', 'WhiteSpace', '');
    fclose(fid_in);
    allLines_orig = allLines_orig{1};
    
    filename = ['output/', modelName, '_mesh_IAMU.obj'];
    if ~exist('output', 'dir'); mkdir('output'); end
    fid_out = fopen(filename,'w');
    if fid_out < 0; error('Could not open file for writing: %s', filename); end
    vCount = 0;
    for i = 1:numel(allLines_orig)
        line = allLines_orig{i};
        if numel(line) >= 2 && line(1) == 'v' && line(2) == ' '
            vCount = vCount + 1;
            fprintf(fid_out, 'v %.7f %.7f %.7f\n', matchedMesh(vCount,1), matchedMesh(vCount,2), matchedMesh(vCount,3));
        else
            fprintf(fid_out, '%s\n', line);
        end
    end
    fclose(fid_out);
    fprintf('Saved body mesh (%d vertices)\n', vCount);

    meshModelMeter = matchedMesh(:,1:3);

    distanceMeasEyes = loadRelationEyes.distanceMeas;
    meshEyes = zeros(size(distanceMeasEyes,1),3);
    nrPoints = 8;
    for i=1:size(distanceMeasEyes,1)
        distSum = 0;
        for n=1:nrPoints
            distSum = distSum + distanceMeasEyes(i,(n-1)*5+2);
        end
        for n=1:nrPoints
            meshEyes(i,:) = meshEyes(i,:) + (distanceMeasEyes(i,(n-1)*5+3:(n-1)*5+5)+meshModelMeter(distanceMeasEyes(i,(n-1)*5+1),:))*(distanceMeasEyes(i,(n-1)*5+2)/distSum);
        end
    end

    % Save symmetrized mesh using original OBJ structure with replaced vertex positions
    fid_in = fopen('utils/exportMakeHumanEyes.obj', 'r');
    allLines_orig = textscan(fid_in, '%s', 'Delimiter', '\n', 'WhiteSpace', '');
    fclose(fid_in);
    allLines_orig = allLines_orig{1};
    
    filename = ['output/', modelName, '_mesh_Eyes.obj'];
    if ~exist('output', 'dir'); mkdir('output'); end
    fid_out = fopen(filename,'w');
    if fid_out < 0; error('Could not open file for writing: %s', filename); end
    vCount = 0;
    for i = 1:numel(allLines_orig)
        line = allLines_orig{i};
        if numel(line) >= 2 && line(1) == 'v' && line(2) == ' '
            vCount = vCount + 1;
            fprintf(fid_out, 'v %.7f %.7f %.7f\n', meshEyes(vCount,1), meshEyes(vCount,2), meshEyes(vCount,3));
        else
            fprintf(fid_out, '%s\n', line);
        end
    end
    fclose(fid_out);
    fprintf('Saved eyes mesh (%d vertices)\n', vCount);

end