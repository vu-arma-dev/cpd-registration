function Generate_Registered_Mesh_User_Study(phantomModelName)
%%  Generate registered mesh and stl files for user study
%   By Long Wang, 2018/4/4
flipping_norms = 'yes';
if nargin<1
    phantomModelName = 'A';
end
%%  Path setting
PC_path_UserStudy = [getenv('cpdreg'),filesep 'userstudy_data' filesep 'PointCloudData' filesep];
switch phantomModelName
    case 'A'
        RegResultName = 'iter_100';
        CurveName = 'CT_KidneyA_vein_curve';
        meshFileName = 'Kidney_A_UserStudy';
        curveFileName = 'curve_A_UserStudy';
        Apriori_Name= 'kidney_and_base_2mm_aligned.ply';
        Fiducial_with_PC = 'CT_KidneyA_fiducials';
        Fiducial_target = 'Fiducial_2018-05-01';
        LoadResultsFolder = [PC_path_UserStudy,'RegAprToCT' filesep];
        FIDUCIAL_path = [getenv('cpdreg'),filesep 'userstudy_data' filesep 'FiducialLocations' filesep];
        PLY_path_Apriori = [getenv('cpdreg'), filesep 'psm_data' filesep 'PLY' filesep];
end

%%  Loading the results
LoadData = load([LoadResultsFolder,RegResultName]);
T = LoadData.T;
LoadData = load([PC_path_UserStudy,CurveName]);
curveReg = LoadData.Positions;
meshReg = T.Y;
%%  Using fiducial to transform the registered point cloud
LoadData = load([FIDUCIAL_path,Fiducial_with_PC]);
F_with_PC = LoadData.Fiducial_Positions';
LoadData = load([FIDUCIAL_path,Fiducial_target]);
F_target = (LoadData.FiducialPositions')*1000;
%   please note that the fiducial location unit is in [mm]
[meshReg,~] = TF_Fiducial(meshReg,F_with_PC,F_target);
[curveReg,~] = TF_Fiducial(curveReg,F_with_PC,F_target);
meshReg_SI = meshReg/1000;
curveReg_SI = curveReg/1000;
%% get triangle indices
ptApriori = pcread([PLY_path_Apriori,Apriori_Name]);
DT_Apriori = delaunayTriangulation(double(ptApriori.Location(:,1:2)));
TR = triangulation(DT_Apriori.ConnectivityList,meshReg_SI);
output_path = [getenv('cpdreg'), 'OutputMesh' filesep];
if exist(output_path,'dir')
    mkdir(output_path);
end
if strcmp(flipping_norms,'yes')
    fn = -faceNormal(TR);
    ConnList_to_STL = TR.ConnectivityList(:,[1,3,2]);
else
    fn = faceNormal(TR);
    ConnList_to_STL = TR.ConnectivityList;
end
if ~exist(output_path,'dir')
    mkdir(output_path);
end
MeshSave(meshReg_SI, ...
    TR.ConnectivityList, fn, [output_path,meshFileName,'.mesh']);
stlwrite([output_path,meshFileName,'.stl'],ConnList_to_STL,...
    meshReg_SI,'FACECOLOR',repmat([255,255,0],...
    length(TR.ConnectivityList),3));
%%  write curve to ".pcd" file
%   cut off the beginning N_begin points and N_end points
N_begin = 50;
N_end = 0;
curveReg_SI([1:N_begin,end-N_end:end],:) = [];
curveReg_SI_Pt = pointCloud(curveReg_SI);
pcwrite(...
    curveReg_SI_Pt,...
    [output_path,curveFileName,'.pcd'],...
    'Encoding','ascii');
%%  compute the averaged direction
[~,~,V] = svd(fn);
fn_avg = V(:,1);
dot_product = fn*fn_avg;
if sum(dot_product)<0
    fn_avg = -fn_avg;
end
fprintf('The averaged direction of the phantom is:\n');
disp(fn_avg);
%%  plot the mesh and the curve
figure;
hold on;
axis equal;
grid on;
box on;
h = trimesh(TR);
alpha(h,0.5);
plot3(curveReg_SI(:,1),curveReg_SI(:,2),curveReg_SI(:,3),...
    '-k','LineWidth',3);
end

