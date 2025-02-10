function anthroMeasurements = anthroRegression(sex, pred_id, Z_test)
    pred = size(pred_id,2);
    measList =  [3	4	5	6	7	8	9	10	16	17	18	19	20	21	23	24	26	29	33	35	39	40	41	42	43	44	53	55	56	58	59	60	61	62	63	64	65	66	67	68	69	70	71	72	73	74	75	76	77	78];
    % pred + resp = size(measList,2)
    resp = size(measList,2) - pred;
    resp_id = zeros(1,resp);
    respIns = 1;
    for i=1:size(measList,2)
        if ~ismember(measList(i),pred_id)
            resp_id(respIns) = measList(i);
            respIns = respIns + 1;
        end
    end
   
    if sex == 1
        load_StatAntro = load('statBodyModel\StatAntro_female.mat');
        load_CorAntro = load('statBodyModel\CorAntro_female.mat');
    elseif sex == 0
        load_StatAntro = load('statBodyModel\StatAntro_male.mat');
        load_CorAntro = load('statBodyModel\CorAntro_male.mat');
    end
    StatAntro = load_StatAntro.StatData;
    CorAntro = load_CorAntro.CorVal;
    disp('Read matrices.');

    %%% CONDITIONAL REGRESSION ANALYSIS OF ANTHROPOMETRIC MEASUREMENTS %%%
    cut_off = 0.01; % Cut off value (Average eigenvalue is 1)
    
    %%% SORT DATA %%% 
    M_Z = zeros(1,pred); SD_Z = zeros(1,pred);
    for i=1:pred
         M_Z(i) = StatAntro(1,pred_id(i));
         SD_Z(i) = StatAntro(2,pred_id(i));
    end
    
    M_Y = zeros(1,resp); SD_Y = zeros(1,resp);
    for i=1:resp
         M_Y(i) = StatAntro(1,resp_id(i));
         SD_Y(i) = StatAntro(2,resp_id(i));
    end

    COR = CorAntro;
    dim = 78;
    
    COR_YZ_ZZ = zeros(dim,pred);
    for i=1:pred
         COR_YZ_ZZ(:,i) = COR(:,pred_id(i));
    end
    COR_YY_ZY = zeros(dim,resp);
    for i=1:resp
         COR_YY_ZY(:,i) = COR(:,resp_id(i));
    end
    
    COR_YY = zeros(resp,resp);
    COR_ZZ = zeros(pred,pred);
    COR_ZY = zeros(pred,resp);
    COR_YZ = zeros(resp,pred);
    
    for i=1:pred
        COR_ZZ(i,:) = COR_YZ_ZZ(pred_id(i),:);
        COR_ZY(i,:) = COR_YY_ZY(pred_id(i),:);
    end
    for i=1:resp
        COR_YY(i,:) = COR_YY_ZY(resp_id(i),:);
        COR_YZ(i,:) = COR_YZ_ZZ(resp_id(i),:);
    end
    %%% SORT DATA - END %%%

    %%% CALCULATES THE MATRIX OF BETA REGRESSION COEFFICIENTS %%%
    B = pinv(COR_ZZ)*COR_ZY; %B = COR_ZZ\COR_ZY;
    %B = (COR_YZ/COR_ZZ)'; % or   B = COR_ZZ\COR_ZY; %    B = COR_YZ*inv(COR_ZZ); 

    %%% CALCULATES THE SINGULAR VALUE DECOMPOSITION %%%
    [~,L,A] = svd(COR_ZZ); % [U,L,A] = svd(COR_ZZ);
    PC_Z = diag(L);

    %%% CALCULATES THE MATRIX OF GAMMA REGRESSION COEFFICIENTS %%%
    G = A'*B; 

    count = size(PC_Z,1);
    test_PC = PC_Z(count);
    while test_PC <= cut_off % Cut off value (Average eigenvalue is 1)
        count = count-1;
        test_PC = PC_Z(count);
    end
    Z_PCs(1) = count;
%     Z_CUM(1) = 1-(sum(PC_Z((Z_PCs+1):size(PC_Z,1)))/sum(PC_Z));

    G_p = G(1:Z_PCs,:);

    % Prediction
    Y_test_PC = zeros(resp,1)';

    Z_value = zeros(1,pred);
    for i=1:pred
         Z_value(i) = (Z_test(i)-M_Z(i))/SD_Z(i);
    end
    
    % PCA regression
    W_value = Z_value*A; %     Non PCA regression: Y_value_S = Z_value*B;
    W_p = W_value(:,1:Z_PCs);
    Y_value_S_PC = W_p*G_p;
    
    for i=1:resp
        Y_test_PC(i) = Y_value_S_PC(i)*SD_Y(i)+M_Y(i);
    end
    regressionRow = Y_test_PC;

    allAnthro = zeros(1,size(measList,2));
    predIns = 1;
    respIns = 1;
    for i=1:size(measList,2)
        if ismember(measList(i),pred_id)
            allAnthro(i) = Z_test(predIns);
            predIns = predIns + 1;
        else
            allAnthro(i) = regressionRow(respIns);
            respIns = respIns + 1;
        end
    end
    anthroMeasurements = allAnthro;
    %pred_id_Antro = pred_id;
end

