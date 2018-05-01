function APP_RegAprToRobot_PSM(varargin)
%%  Register the Apriori model to the Robot Exploration Data
%   By Long Wang, 2016/10/5
%   This is consistent with Long's JMR draft.
%   The Apriori model include the STL model assoiciated with the
%   incorporated Vritual Fixture Curve.
%%  parse optional arguments
OnlyLoadResult = 'no';
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'only load result')
            OnlyLoadResult = propertyValue;
            %   This option will skip the computation only show result
        end
    end
end
%%  Load data files
Setup_Dir_DeformableReg;
PLY_path_PSM = [getenv('UDPREGJHU'),'\PSM_Data\PLY\'];
PC_path_PSM = [getenv('UDPREGJHU'),'\PSM_Data\PointCloudData\'];
Apriori_Name= 'STL_model_1mm_PSM_use.ply';
IncpCurveName = 'PSMCurveIncorpApriori';
RobotExplrPtName = 'PSMExplrPtCloud.ply';
ptApriori = pcread([PLY_path_PSM,Apriori_Name]);
CurveIncpFittedData = load([PC_path_PSM,IncpCurveName]);
CurveIncpFitted = CurveIncpFittedData.CurveIncpFittedProjected;
clear CurveIncpFittedData;
ptRobotExplr = pcread([PLY_path_PSM,RobotExplrPtName]);
%% register
SaveResultsFolder = [PC_path_PSM,'RegAprToRobot\'];
AprioriMerge = [ptApriori.Location;CurveIncpFitted];
if ~exist(SaveResultsFolder,'dir')
    mkdir(SaveResultsFolder);
end
for k = 11
    [T,C]= deformReg(ptRobotExplr.Location,AprioriMerge,'max iter',k);
    save([SaveResultsFolder,'iter_',num2str(k)],...
        'T','C','ptRobotExplr','ptApriori','CurveIncpFitted');
    close all;
end
end

