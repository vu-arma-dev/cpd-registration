function Generate_Mesh_User_Study(phantomModelName)
%%  Generate registered mesh and stl files for user study
%   By Long Wang, 2018/4/4
if nargin<1
    phantomModelName = 'A';
end
%%  Path setting
cpdBasePath= [fileparts(fileparts(mfilename('fullpath'))) filesep];
PC_path_UserStudy = [cpdBasePath filesep 'userstudy_data' filesep 'PointCloudData'];
LoadResultsFolder = [PC_path_UserStudy filesep 'RegAprToCT'];
PLY_path_Apriori = [cpdBasePath filesep 'psm_data' filesep 'PLY'];
Apriori_Name= 'kidney_and_base_2mm_aligned.ply';

flipping_norms='yes';
% flipping_norms='no';
RegResultName =[ 'Kidney_' num2str(phantomModelName) '_iter_100_NoOpt'];
meshFileName = ['Kidney_' num2str(phantomModelName) '_UserStudy'];
%%  Loading the results
LoadData = load([LoadResultsFolder,filesep,RegResultName]);
T = LoadData.T;
meshReg = T.Y;

%   please note that the fiducial location unit is in [mm]
meshReg_SI = meshReg/1000;

%% get triangle indices
ptApriori = pcread([PLY_path_Apriori,filesep, Apriori_Name]);
DT_Apriori = delaunayTriangulation(double(ptApriori.Location(:,1:2)));
TR = triangulation(DT_Apriori.ConnectivityList,meshReg_SI);
output_path = [PC_path_UserStudy,filesep 'OutputMesh'];
if ~exist(output_path,'dir')
    mkdir(output_path);
end
if strcmp(flipping_norms,'yes')
    fn = -faceNormal(TR);
    ConnList_to_STL = TR.ConnectivityList(:,[1,3,2]);
else
    fn = faceNormal(TR);
    ConnList_to_STL = TR.ConnectivityList;
end
MeshSave(meshReg_SI, ...
    TR.ConnectivityList, fn, [output_path, filesep ,meshFileName,'.mesh']);
stlwrite([output_path,filesep,meshFileName,'.stl'],ConnList_to_STL,...
    meshReg_SI,'FACECOLOR',repmat([255,255,0],...
    length(TR.ConnectivityList),3));

%%  compute the averaged direction
[~,~,V] = svd(fn);
fn_avg = V(:,1);
dot_product = fn*fn_avg;
if sum(dot_product)<0
    fn_avg = -fn_avg;
end
fprintf('The averaged direction of the phantom is:\n');
disp(fn_avg);

% %%  plot the mesh and the curve
% figure;
% hold on;
% axis equal;
% grid on;
% box on;
% h = trimesh(TR);
% alpha(h,0.5);
% plot3(curveReg_SI(:,1),curveReg_SI(:,2),curveReg_SI(:,3),...
%     '-k','LineWidth',3);
end

