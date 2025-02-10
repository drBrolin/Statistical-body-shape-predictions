function bodyShapePrediction(sex, anthroMeasurements, landmarkCoordinates)
    if sex == 1
        manikinSex = 'Female_';
        load_A_mesh = load('statBodyModel\A_mesh_female.mat');
        load_dStat_Y_mesh = load('statBodyModel\dStat_Y_mesh_female.mat');
        load_dStat_Z_mesh = load('statBodyModel\dStat_Z_mesh_female.mat');
        load_G_p_mesh = load('statBodyModel\G_p_mesh_female.mat');
    elseif sex == 0
        manikinSex = 'Male_';
        load_A_mesh = load('statBodyModel\A_mesh_male.mat');
        load_dStat_Y_mesh = load('statBodyModel\dStat_Y_mesh_male.mat');
        load_dStat_Z_mesh = load('statBodyModel\dStat_Z_mesh_male.mat');
        load_G_p_mesh = load('statBodyModel\G_p_mesh_male.mat');
    end
    A_mesh = load_A_mesh.A;
    dStat_Y_mesh = load_dStat_Y_mesh.dStatAll;
    dStat_Z_mesh = load_dStat_Z_mesh.dStat_Z;
    G_p_mesh = load_G_p_mesh.G_p_All;
    disp('Read matrices.');

    age = anthroMeasurements(1);
    weight = anthroMeasurements(2);
    stature = anthroMeasurements(3);
     
    adjustedLM = landmarkCoordinates;
    
    %%% MESH PREDICTION %%%
    pred = size(dStat_Z_mesh,2);
    resp = size(dStat_Y_mesh,2);
    Z_test = [age weight stature adjustedLM];
  
    Z_PCs =  size(G_p_mesh,1);
    Y_test_PC = zeros(resp,1)';
    Z_value = zeros(1,pred);
    for i=1:pred
         Z_value(i) = (Z_test(i)-dStat_Z_mesh(1,i))/dStat_Z_mesh(2,i);
    end
    
    %%% PCA regression %%%
    W_value = Z_value*A_mesh;
    W_p = W_value(:,1:Z_PCs);
    Y_value_S_PC = W_p*G_p_mesh;
    
    for i=1:resp
        Y_test_PC(i) = Y_value_S_PC(i)*dStat_Y_mesh(2,i)+dStat_Y_mesh(1,i);
    end
    
    predicted_mesh = Y_test_PC;
    disp('Predicted mesh data with regression analysis.');
    meshModel = zeros(resp/3,3);
    for i=1:resp/3
          meshModel(i,1:3) = predicted_mesh(1,(i-1)*3+1:(i-1)*3+3);
    end
        
    landmarksAdj = zeros(73,3);
    for i=1:size(landmarksAdj,1)
        landmarksAdj(i,1:3) = adjustedLM(1,(i*3-2):(i*3));
    end
    adjustedLM = landmarksAdj;
    maxHt = max(meshModel(23052/3:25125/3,3));
    scalingRatio = stature/maxHt;
    meshModelScaled = scalingRatio*meshModel;
    adjustedLMScaled = scalingRatio*adjustedLM;
       
    
    % FULL BODY PLOT
    plot3(meshModelScaled(:,1),meshModelScaled(:,2),meshModelScaled(:,3),'co','MarkerEdgeColor','c','MarkerFaceColor','c','MarkerSize',1); hold on;
    plot3(adjustedLMScaled(:,1),adjustedLMScaled(:,2),adjustedLMScaled(:,3),'mo','MarkerEdgeColor','m','MarkerFaceColor','m','MarkerSize',4); hold on;
    figure(1);
    grid;
    xlabel('X'); ylabel('Y'); zlabel('Z')
    axis equal; 

    t = datetime('now','TimeZone','local','Format','yyMMdd');
    modelName = [manikinSex,'Manikin_',char(t),'_',int2str(age),'_',int2str(stature),'_',int2str(weight)]; %Automatically generates a suitable file-name.

    saveMesh(meshModelScaled,modelName);
    saveLM(adjustedLMScaled,modelName);

end

