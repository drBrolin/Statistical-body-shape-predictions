function meshModel = bodyShapePrediction(sex, anthroMeasurements, landmarkCoordinates)
    if sex == 1
        load_A_mesh = load('statBodyModel/A_mesh_female.mat');
        load_dStat_Y_mesh = load('statBodyModel/dStat_Y_mesh_female.mat');
        load_dStat_Z_mesh = load('statBodyModel/dStat_Z_mesh_female.mat');
        load_G_p_mesh = load('statBodyModel/G_p_mesh_female.mat');
    elseif sex == 0
        load_A_mesh = load('statBodyModel/A_mesh_male.mat');
        load_dStat_Y_mesh = load('statBodyModel/dStat_Y_mesh_male.mat');
        load_dStat_Z_mesh = load('statBodyModel/dStat_Z_mesh_male.mat');
        load_G_p_mesh = load('statBodyModel/G_p_mesh_male.mat');
    end
    A_mesh = load_A_mesh.A;
    dStat_Y_mesh = load_dStat_Y_mesh.dStatAll;
    dStat_Z_mesh = load_dStat_Z_mesh.dStat_Z;
    G_p_mesh = load_G_p_mesh.G_p_All;
    disp('Read matrices.');

    age = anthroMeasurements(1);
    weight = anthroMeasurements(2);
    stature = anthroMeasurements(3);
         
    %%% MESH PREDICTION %%%
    % pred = size(dStat_Z_mesh,2);
    % resp = size(dStat_Y_mesh,2);
    Z_test = [age weight stature landmarkCoordinates];
  
    Z_PCs =  size(G_p_mesh,1);
    % Y_test_PC = zeros(resp,1)';
    Z_value = (Z_test - dStat_Z_mesh(1,:)) ./ dStat_Z_mesh(2,:);
    
    %%% PCA regression %%%
    W_value = Z_value*A_mesh;
    W_p = W_value(:,1:Z_PCs);
    Y_value_S_PC = W_p*G_p_mesh;
    
    predicted_mesh = Y_value_S_PC .* dStat_Y_mesh(2,:) + dStat_Y_mesh(1,:);
    disp('Predicted mesh data with regression analysis.');
    meshModel = reshape(predicted_mesh, 3, [])';
end

