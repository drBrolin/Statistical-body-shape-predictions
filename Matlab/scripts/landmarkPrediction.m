function landmarkCoordinates = landmarkPrediction(sex, anthroMeasurements)
    if sex == 1
        load_A_LM = load('statBodyModel\A_LM_female.mat');
        load_dStat_Y_LM = load('statBodyModel\dStat_Y_LM_female.mat');
        load_dStat_Z_LM = load('statBodyModel\dStat_Z_LM_female.mat');
        load_G_p_LM = load('statBodyModel\G_p_LM_female.mat');

    elseif sex == 0
        load_A_LM = load('statBodyModel\A_LM_male.mat');
        load_dStat_Y_LM = load('statBodyModel\dStat_Y_LM_male.mat');
        load_dStat_Z_LM = load('statBodyModel\dStat_Z_LM_male.mat');
        load_G_p_LM = load('statBodyModel\G_p_LM_male.mat');
    end
    A_LM = load_A_LM.A;
    dStat_Y_LM = load_dStat_Y_LM.dStat_Y;
    dStat_Z_LM = load_dStat_Z_LM.dStat_Z;
    G_p_LM = load_G_p_LM.G_p;
    disp('Read matrices.');


    %%% LANDMARK PREDICTION %%%
    pred_id = [3	4	5	6	7	8	9	10	16	17	18	19	20	21	23	24	26	29	33	35	39	40	41	42	43	44	53	55	56	58	59	60	61	62	63	64	65	66	67	68	69	70	71	72	73	74	75	76	77	78];
    Z_test_LM = anthroMeasurements;
     
    pred = size(pred_id,2);
    resp = size(dStat_Y_LM,2);
    Z_PCs =  size(G_p_LM,1);
    Y_test_PC = zeros(resp,1)';
    Z_value = zeros(1,pred);
    for i=1:pred
         Z_value(i) = (Z_test_LM(i)-dStat_Z_LM(1,i))/dStat_Z_LM(2,i);
    end
    
    %%% PCA regression %%%
    W_value = Z_value*A_LM;
    W_p = W_value(:,1:Z_PCs);
    Y_value_S_PC = W_p*G_p_LM;
    
    for i=1:resp
        Y_test_PC(i) = Y_value_S_PC(i)*dStat_Y_LM(2,i)+dStat_Y_LM(1,i);
    end
    predicted_LM = Y_test_PC;
    disp('Predicted landmark data with regression analysis.');
    
    %%% Correction to get symmetry on both sides of sagittal plane %%%
    adjustedLM = landmarkAdjust(predicted_LM);
    disp('Adjusted landmark to get symmetry.');

    landmarkCoordinates = adjustedLM;
end

