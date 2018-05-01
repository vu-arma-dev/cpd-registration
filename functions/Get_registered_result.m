function data_requested = ...
    Get_registered_result(PSM_or_Cartesian,Rob_or_Laser,varargin)
%%  Get the registered results
%   By Long Wang, 2016/10/9
%   This func fetches the registered results
%%  Required Inputs:
%   PSM_or_Cartesian = {'PSM', 'Cartesian'}
%   Rob_or_Laser = {'robot', 'laser'}
%%  Optional Inputs:
DataType = 'RegCurve';
% ALL DataType include:
% 'RegCurve' - registered curve, output as (Nx3) matrix
% 'RegPtCloud' - registered model (point cloud), output as pointCloud type
% 'ExplrPtCloud' - explored point cloud (Not acitive yet)
% 'ptApr' - apriori model (Not acitive yet)
% 'curveApr' - priori curve (Not acitive yet)
% 'CorrespPtInExplr' -  The corresponding points of the registered point
%                           cloud in the original data set
% 'CorrespCurveInExplr' - The corresonpoing points of the registered curve
%                       in the original data set
% 'GroundTruthDigitized' -  The ground truth digitization. Note that this is
%                           not obtained by transforming any apriori
%                           or exploration. This is true % measurement if
%                           digitization.
%   EXAMPLE USAGE: Get_registered_result('PSM','laser','data type','GroundTruthDigitized')
iter_num = 100;  % this is used only for cases of PSM
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'data type')
            DataType = propertyValue;
        elseif strcmp(propertyName,'iter num')
            iter_num = propertyValue;
        end
    end
end
%% Start fetching data requested
if strcmp(PSM_or_Cartesian,'PSM')
    %%  In the case of PSM
    PC_folder_path = [getenv('UDPREGJHU'),'\PSM_Data\PointCloudData\'];
    if strcmp(Rob_or_Laser,'robot')
        ResultFolder = [PC_folder_path,'RegAprToRobot\'];
    elseif strcmp(Rob_or_Laser,'laser')
        ResultFolder = [PC_folder_path,'RegAprToLaser\'];
    else
        fprintf(['Wrong input for "Rob_or_Laser"',...
            ' in func "Get_Registered_result"\n']);
        pause;
    end
    %   load the result
    %   T is the transformation that thansforms the apriori to the
    %   exploration data.
    %   C is the corresponding list where each element of C is the index
    %   pointing to the exploration data.
    filename = [ResultFolder,'iter_',num2str(iter_num)];
    ResultData = load(filename);
    T = ResultData.T;
    C = ResultData.C;
    ptApriori = ResultData.ptApriori;
    CurveIncpFitted = ResultData.CurveIncpFitted;
    if strcmp(Rob_or_Laser,'robot')
        ptExplr = ResultData.ptRobotExplr;
    else
        ptExplr = ResultData.ptLaserScan;
    end
    %   extract the requested data
    switch DataType
        case 'RegCurve'
            CurveCurrentModel = T.Y(ptApriori.Count+1:end,:);
            data_requested = CurveCurrentModel;
        case 'RegPtCloud'
            ptCurrentModel = pointCloud(T.Y(1:ptApriori.Count,:));
            data_requested = ptCurrentModel;
        case 'ExplrPtCloud'
            data_requested = ptExplr;
        case 'ptApr'
            data_requested = ptApriori;
        case 'curveApr'
            data_requested = CurveIncpFitted;
        case 'CorrespPtInExplr'
            CorrespPointCloudInExplr = ...
                ptExplr.Location(C(1:ptApriori.Count),:);
            data_requested = pointCloud(CorrespPointCloudInExplr);
        case 'UniqueCorrespPtInExplr'
            %   Identify the duplicate correspondance
            %   Select one of the duplicated group, the one with the
            %   smallest error
            unique_indices_in_X = unique(C);
            N_unique_points_pair = length(unique_indices_in_X);
            unique_indices_in_Y = zeros(N_unique_points_pair,1);
            for i = 1:N_unique_points_pair
                points_in_Y_corr_i_th_unique = (C==unique_indices_in_X(i));
                Num_pt_group_i = sum(points_in_Y_corr_i_th_unique);
                Error_Temp_in_i_th_group = ...
                    ptExplr.Location(points_in_Y_corr_i_th_unique,:)... 
                - repmat(T.Y(i,:),Num_pt_group_i,1)
                a = 0;
            end
        case 'CorrespCurveInExplr'
            CorrespCurveInExplr = ...
                ptExplr.Location(C(ptApriori.Count+1:end),:);
            data_requested = CorrespCurveInExplr;
        case 'GroundTruthDigitized'
            if strcmp(Rob_or_Laser,'robot')
                DeformedCurve_GroundTruth_Data = ...
                    load([PC_folder_path,...
                    'Deformed_curve_robot_digitized_PSM']);
                DeformedCurve = DeformedCurve_GroundTruth_Data.DigitizedPoints;
                clear DeformedCurve_GroundTruth_Data;
                ptGroundTruthDigitized = pointCloud(DeformedCurve);
            elseif strcmp(Rob_or_Laser,'laser')
                FLY_folder_path = [getenv('UDPREGJHU'),'\PSM_Data\PLY\'];
                ptGroundTruthDigitized = pcread([FLY_folder_path,...
                    'deformed_digitized_curve_by_Faro_PSM_adjusted.ply']);
            else
                fprintf('Wrong input for Rob_or_Laser\n');
                pause;
            end
            [GroundTruthFitted,FittedRMS] = ...
                FitPointCloudToCurve(ptGroundTruthDigitized);
            data_requested = struct(...
                'FittedPt',GroundTruthFitted,'FittedRMS',FittedRMS,...
                'OriginalPt',ptGroundTruthDigitized.Location);
        case 'frame_rob2laser'
            %   get the frame transformation from robot to laser
            Frames_transformation_folder = [PC_folder_path,'\RegFrames_rob2laser\'];
            Loaded_data = load([Frames_transformation_folder,'iter_',num2str(iter_num)]);
            data_requested = Loaded_data.T;
        otherwise
            fprintf('Wrong input for "data type"\n');
            pause;
    end
elseif strcmp(PSM_or_Cartesian,'Cartesian')
    %%  In the case of Cartesian
    %   The difference in how to fetch the data here is because
    %   this code was done earlier than the PSM exploration
    %   Different iteration numbers were not considered or implemented
    %   Could or should implement them later for thesis
    PC_folder_path = [getenv('UDPREGJHU'),'\Cartesian_Data\PointCloudData\'];
    if strcmp(Rob_or_Laser,'robot')
        FileName_workspace = [PC_folder_path,'\stl2robot_ws'];
        Name_RegCurve = 'curveRobot';
    elseif strcmp(Rob_or_Laser,'laser')
        FileName_workspace = [PC_folder_path,'\stl2laser_ws'];
        Name_RegCurve = 'curveLaser';
    else
        fprintf(['Wrong input for "Rob_or_Laser"',...
            ' in func "Get_Registered_result"\n']);
        pause;
    end
    DataName_RegCurve = [PC_folder_path,'\',Name_RegCurve];
    %   Load the reg result
    WS_Loaded = load(FileName_workspace,'T','C','curveSTL','X','Y');
    %   'T' - resulted transform
    %   'C' - resulted corresponding list
    %   'curveSTL' - fitted/incorporated a-priori curve
    %   'X' - exploration/laser data
    %   'Y' - a-priori STL point cloud
    T = WS_Loaded.T;
    C = WS_Loaded.C;
    X = WS_Loaded.X;
    Y = WS_Loaded.Y;
    curveSTL = WS_Loaded.curveSTL;
    RegPointCloud_NumPts = length(C) - length(curveSTL);
    %   load the result
    switch DataType
        case 'RegCurve'
            Data_Loaded = load(DataName_RegCurve);
            data_requested = Data_Loaded.(Name_RegCurve);
        case 'RegPtCloud'
            ptCurrentModel = pointCloud(T.Y(1:RegPointCloud_NumPts,:));
            data_requested = ptCurrentModel;
        case 'ExplrPtCloud'
            data_requested = pointCloud(X);
        case 'ptApr'
            data_requested = pointCloud(Y);
        case 'curveApr'
            data_requested = curveSTL;
        case 'CorrespPtInExplr'
            CorrespPointCloudInExplr = X(C(1:RegPointCloud_NumPts),:);
            data_requested = pointCloud(CorrespPointCloudInExplr);
        case 'CorrespCurveInExplr'
            CorrespCurveInExplr = X(C(RegPointCloud_NumPts+1:end),:);
            data_requested = CorrespCurveInExplr;
        case 'GroundTruthDigitized'
            %   In the case of Cartesian robot experiments,
            %   The ground truth is only collected using Faro Arm
            Data_Loaded = load([PC_folder_path,'\DeformedTargetCurve']);
            ptGroundTruthDigitized = ...
                pointCloud(Data_Loaded.TargetCurveData);
            [GroundTruthFitted,FittedRMS] = ...
                FitPointCloudToCurve(ptGroundTruthDigitized,...
                'figure','on','poly order',[8,2],'variable mode','[y,z]=f(x)');
            data_requested = struct(...
                'FittedPt',GroundTruthFitted,'FittedRMS',FittedRMS,...
                'OriginalPt',ptGroundTruthDigitized.Location);
        case 'frame_rob2laser'
            %   get the frame transformation from robot to laser
            Loaded_data = load([PC_folder_path,'\RegFrames_rob2laser']);
            data_requested = Loaded_data.T;
        otherwise
            fprintf('Wrong input for "data type"\n');
            pause;  
    end
else
    fprintf('Wrong input for "PSM_or_Cartesian"\n');
    pause;
end

end

