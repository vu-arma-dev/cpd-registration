function Vein_Fiducial_validation()
%%  Validating the segmented curve to the CT-segmented mesh
%   Long & Colette, 2018/4/4

%%  load the 3D mesh
PLY_path_CT = [getenv('UDPREGJHU'),'\UserStudy_Data\PLY\'];
CT_Vein_Mesh_PtName = 'CT_veinA_mesh.ply';
ptCTVein = pcread([PLY_path_CT,CT_Vein_Mesh_PtName]);
ptCTVein = pcdownsample(ptCTVein,'gridAverage',2);
ptCTVein.Color = repmat(uint8([0 255 0]),ptCTVein.Count,1);
CT_Scan_PtName = 'CT_kidneyA_mesh.ply';
ptCTScan = pcread([PLY_path_CT,CT_Scan_PtName]);
ptCTScan = pcdownsample(ptCTScan,'gridAverage',2);
ptCTScan.Color = repmat(uint8([0 0 0]),ptCTScan.Count,1);
%%  load the segmented 3D curve
PC_path_CT = [getenv('UDPREGJHU'),'\UserStudy_Data\PointCloudData\'];
RegisteredCurvName = 'CT_KidneyA_vein_curve';
RegCurveDataLoaded = load([PC_path_CT,RegisteredCurvName]);
RegCurve = RegCurveDataLoaded.Positions;
%%  load the segmented fiducials
Fiducial_Path = [getenv('UDPREGJHU'),'\UserStudy_Data\FiducialLocations\'];
FiducialFileName = 'CT_KidneyA_fiducials';
FidcucialDataLoaded = load([Fiducial_Path,FiducialFileName]);
Fiducials = FidcucialDataLoaded.Fiducial_Positions;

%%  plot
figure;
hold on;
colors = ['r','g','b','m'];
for i=1:length(Fiducials)
    scatter3(Fiducials(i,1),Fiducials(i,2),Fiducials(i,3),100,...
        'filled','MarkerFaceColor',colors(i));
end
pcshow(ptCTScan);
pcshow(ptCTVein);
plot3(RegCurve(:,1),RegCurve(:,2),RegCurve(:,3),'-r','LineWidth',3);
legend('A','B','C','D','Kidney-CT','Vein-CT','Vein-centerline')
end

