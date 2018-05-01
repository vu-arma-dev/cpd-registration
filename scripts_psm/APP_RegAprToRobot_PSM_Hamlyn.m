function APP_RegAprToRobot_PSM_Hamlyn(varargin)
%%  Register the Apriori model to the Robot Exploration Data
%   By Long Wang, 2016/10/5
%   This is consistent with Long's JMR draft.
%   The Apriori model include the STL model assoiciated with the
%   incorporated Vritual Fixture Curve.
%%  parse optional arguments
Max_Iter = 200;
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
PLY_path_PSM = [getenv('UDPREGJHU'),'\PSM_Data\PLY\'];
PC_path_PSM = [getenv('UDPREGJHU'),'\PSM_Data\PointCloudData\'];
Apriori_Name = 'kidney_and_base_2mm_aligned.ply';
RobotExplrPtName = 'PSMExplrPtCloud_Hamlyn.ply';
ptApriori = pcread([PLY_path_PSM,Apriori_Name]);
ptApriori = pcdownsample(ptApriori,'gridAverage',2);
%%  rotate the apriori to match the exploration better
R = axang2rotm([1 0 0 pi]);
A = [R,zeros(3,1); ...
     0 0 0 1];
tform = affine3d(A);
ptApriori = pctransform(ptApriori,tform);
ptRobotExplr = pcread([PLY_path_PSM,RobotExplrPtName]);
ptRobotExplr = pcdownsample(ptRobotExplr,'gridAverage',2);
%% register
SaveResultsFolder = [PC_path_PSM,'RegAprToRobotHamlyn\'];
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
view(120,20);
end
