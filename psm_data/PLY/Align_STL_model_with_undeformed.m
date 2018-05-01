function Align_STL_model_with_undeformed()
%%  This func aligns the STL a-priori model to the laser scan of undeformed
%   Load point clouds
NewFileName = 'STL_model_1mm_PSM_use.ply';
PLY_folder_path = [getenv('UDPREGJHU'),'\','PLY\'];
DefaultModelFileName = 'model_1mm.ply';
LaserScanUndeformedName = 'undeformed_model_PSM_downsampled.ply';
ptLaser = pcread([PLY_folder_path,LaserScanUndeformedName]);
ptSTL = pcread([PLY_folder_path,DefaultModelFileName]);
%   Set the trasformation
R = axang2rotm([1 0 0 -pi/2])*axang2rotm([0 0 1 pi/2]);
T = [R,zeros(3,1);0 0 0 1];
tf = affine3d(T);
%   Rotate and save
ptSTL_aligned = pctransform(ptSTL,tf);
figure;
draw_coordinate_system(5*ones(3,1), eye(3), [0, 0, 0], 'rgb');
pcshow(ptSTL_aligned);
figure;
draw_coordinate_system(5*ones(3,1), eye(3), [0, 0, 0], 'rgb');
pcshow(ptLaser);
pcwrite(ptSTL_aligned,[PLY_folder_path,NewFileName],'PLYFormat','binary');
end