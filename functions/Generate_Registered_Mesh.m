function Generate_Registered_Mesh(varargin)
%%  Generate registered mesh and stl files
%   By Long Wang, 2016/11
%   This func currently are used in the following experiments/demos
%   1) JMR submission
%   2) Hamlyn 2017 challenge
ExperimentName = 'Hamlyn2017';
flipping_norms = 'no';
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'Experiment')
            ExperimentName = propertyValue;
        elseif strcmp(propertyName,'flipping norms')
            flipping_norms = propertyValue;
        end
    end
end
%%  Path setting
PC_path_PSM = [getenv('UDPREGJHU'),'\PSM_Data\PointCloudData\'];
%%  Setup for a particular experiment
switch ExperimentName
    case 'JMR'
        ResultName = 'iter_50';
        OutFileName = 'deformedRobotPSM';
        Apriori_Name= 'STL_model_1mm_PSM_use.ply';
        Fiducial_with_PC = [];
        Fiducial_target = [];
        LoadResultsFolder = [PC_path_PSM,'RegAprToRobot\'];
        PLY_path_Apriori = [getenv('UDPREGJHU'),'\PSM_Data\PLY\'];
    case 'Hamlyn2017'
        ResultName = 'reg_result';
        OutFileName = 'deformedRobotPSM_Hamlyn17_v4';
        Apriori_Name= 'kidney_and_base_2mm_aligned.ply';
        Fiducial_with_PC = 'FiducialHamlyn2017VU';
        Fiducial_target = 'FiducialHamlyn2017JHU';
        LoadResultsFolder = [PC_path_PSM,'RegAprToRobotHamlyn\'];
        PLY_path_Apriori = [getenv('UDPREGJHU'),'\PSM_Data\PLY\'];
    case 'HamlynVUTest'
        ResultName = 'reg_result';
        OutFileName = 'deformedRobotPSM_Hamlyn17_VU';
        Apriori_Name= 'kidney_and_base_2mm_aligned.ply';
        Fiducial_with_PC = 'FiducialHamlyn2017VU';
        %         Fiducial_target = 'FiducialHamlyn2017VU_170810';
        Fiducial_target = 'FiducialHamlyn2017VU_rashid';
        LoadResultsFolder = [PC_path_PSM,'RegAprToRobotHamlyn\'];
        PLY_path_Apriori = [getenv('UDPREGJHU'),'\PSM_Data\PLY\'];
    case 'Kidney'
        ResultName = 'reg_result';
        OutFileName = 'deformedRobotPSM_Kidney';
        Apriori_Name= 'kidney_and_base_2mm_aligned.ply';
        Fiducial_with_PC = [];
        Fiducial_target = [];
        LoadResultsFolder = [PC_path_PSM,'RegAprToRobotKidney\'];
        PLY_path_Apriori = [getenv('UDPREGJHU'),'\PSM_Data\PLY\'];
end

%%  Loading the results
LoadData = load([LoadResultsFolder,ResultName]);
T = LoadData.T;
if strcmp(ExperimentName,'JMR')
    curveSTL = LoadData.CurveIncpFitted;
else
    curveSTL = [];
end
YNoCurve = T.Y(1:length(T.Y)-length(curveSTL),:);
%%  Using fiducial to transform the registered point cloud
if ~isempty(Fiducial_with_PC)
    FIDUCIAL_path = [getenv('UDPREGJHU'),'\PSM_Data\FiducialLocations\'];
    LoadData = load([FIDUCIAL_path,Fiducial_with_PC]);
    F_with_PC = [LoadData.A,LoadData.B,LoadData.C,LoadData.D];
    LoadData = load([FIDUCIAL_path,Fiducial_target]);
    F_target = [LoadData.A,LoadData.B,LoadData.C,LoadData.D];
    %   please note that the fiducial location unit is in [mm]
    [YNoCurve,TF_fiducial] = TF_Fiducial(YNoCurve,F_with_PC,F_target);
end
YNoCurve_SI = YNoCurve./1000.0;
%% get triangle indices
ptApriori = pcread([PLY_path_Apriori,Apriori_Name]);
ptApriori = pcdownsample(ptApriori,'gridAverage',2); % NOTE, this nubmer of downsampling has to match the other
DT_Apriori = delaunayTriangulation(double(ptApriori.Location(:,1:2)));
TR = triangulation(DT_Apriori.ConnectivityList,YNoCurve_SI);
output_path = [getenv('UDPREGJHU'),'\OutputMesh\'];
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
MeshSave(YNoCurve_SI, ...
    TR.ConnectivityList, fn, [output_path,OutFileName,'.mesh']);
fv.vertices = YNoCurve_SI;
fv.faces = ConnList_to_STL;
stlwrite([output_path,OutFileName,'.stl'],ConnList_to_STL,...
    YNoCurve_SI,'FACECOLOR',repmat([255,255,0],...
    length(TR.ConnectivityList),3));
%%  compute the averaged direction
[~,~,V] = svd(fn);
fn_avg = V(:,1);
dot_product = fn*fn_avg;
if sum(dot_product)<0
    fn_avg = -fn_avg;
end
fprintf('The averaged direction of the phantom is:\n')
disp(fn_avg);
end

