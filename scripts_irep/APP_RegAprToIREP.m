function APP_RegAprToIREP(varargin)
%%  Register the Apriori model to the Robot Exploration Data
%   By Long Wang, 2016/10/5
%   This is consistent with Long's JMR draft.
%   The Apriori model include the STL model assoiciated with the
%   incorporated Vritual Fixture Curve.
%%  parse optional arguments
Max_Iter = 100;
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'iter max')
            Max_Iter= propertyValue;
        end
    end
end
%%  Load data files
Setup_Dir_DeformableReg;
PLY_path_IREP = [getenv('UDPREGJHU'),'\IREP_Data\PLY\'];
PC_path_IREP = [getenv('UDPREGJHU'),'\IREP_Data\PointCloudData\']; 
% PC_path_PSM = [getenv('UDPREGJHU'),'\PSM_Data\PointCloudData\'];
Apriori_Name = 'kidney_apriori_topsurf.ply';
% RobotExplrPtName = ...
%     {'IREPExplrPtCloud_Oct19tasktest.ply',...
%     'IREPExplrPtCloud_Oct20nothing.ply'};
RobotExplrPtName = 'IREPExplrPtCloud_Oct26explore.ply';
ptApriori = pcread([PLY_path_IREP,Apriori_Name]);
ptApriori.Color = ptApriori.Color*0.5;
% ptApriori = pcdownsample(ptApriori,'gridAverage',0.5);
%%  rotate the apriori to match the exploration better
R = axang2rotm([1 0 0 pi]);
A = [R,zeros(3,1); ...
     0 0 0 1];
tform = affine3d(A);
ptApriori = pctransform(ptApriori,tform);
% The following selection works for dataset Oct19
% xSlection = [-10,30];
% ySlection = [-35,35];
% The following selection works for dataset Oct26explore
xSlection = [5,30];
ySlection = [-20,35];
selectAprioriRegion = ...
    (ptApriori.Location(:,1)>xSlection(1)) & ...
    (ptApriori.Location(:,1)<xSlection(2)) & ...
    (ptApriori.Location(:,2)>ySlection(1)) & ...
    (ptApriori.Location(:,2)<ySlection(2));
R = axang2rotm([0 0 1 pi/2]);
A = [R,zeros(3,1); ...
     0 0 0 1];
tform = affine3d(A);
ptApriori = pctransform(ptApriori,tform); 
ptApriori = pointCloud(ptApriori.Location(selectAprioriRegion,:));
if iscell(RobotExplrPtName)
    ptRobotExplr = mergePointCloud(PLY_path_IREP,RobotExplrPtName,1);
else
    ptRobotExplr = pcread([PLY_path_IREP,RobotExplrPtName]);
end
ptRobotExplr = pcdownsample(ptRobotExplr,'gridAverage',1);
%% register
SaveResultsFolder = [PC_path_IREP,'RegAprToRobotIREP\'];
AprioriMerge = [ptApriori.Location;];
if ~exist(SaveResultsFolder,'dir')
    mkdir(SaveResultsFolder);
end
close all;
[T,C]= deformReg(ptRobotExplr.Location,AprioriMerge,'max iter',Max_Iter);
save([SaveResultsFolder,'reg_result'],...
    'T','C','ptRobotExplr','ptApriori');
figure(3);
set_Axis;
end
function set_Axis()
title('');
axis equal;
grid on;
box on;
view(61,19);
end
function mergedData = mergePointCloud(PLY_path_IREP,fileNames,gridSize)
    N_files = length(fileNames);
    for i=1:N_files
       if i==1
           mergedData = pcread([PLY_path_IREP,fileNames{i}]);
       else
           pointCloudData = pcread([PLY_path_IREP,fileNames{i}]);
           mergedData = pcmerge(mergedData,pointCloudData,gridSize);
       end
    end
end