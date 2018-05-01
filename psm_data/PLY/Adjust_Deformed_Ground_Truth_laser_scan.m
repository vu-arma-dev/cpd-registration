function Adjust_Deformed_Ground_Truth_laser_scan()
%%  This func aligns the STL a-priori model to the laser scan of undeformed
%   Load point clouds
LaserNewFileName = 'Deformed_GroundTruth_PSM_adjusted.ply';
DigitizeNewFileName = 'deformed_digitized_curve_by_Faro_PSM_adjusted.ply';
PLY_folder_path = [getenv('UDPREGJHU'),'\','PLY\'];
LaserScanDeformedName = 'Deformed_GroundTruth_PSM.ply';
LaserCurveDeformedName = 'deformed_digitized_curve_by_Faro_PSM.ply';
ptLaser = pcread([PLY_folder_path,LaserScanDeformedName]);
ptCurve = pcread([PLY_folder_path,LaserCurveDeformedName]);
% demean
meanPos = mean(ptLaser.Location);
ptLaser = pointCloud(ptLaser.Location - ...
    repmat(meanPos,ptLaser.Count,1)); 
ptCurve = pointCloud(ptCurve.Location - ...
    repmat(meanPos,ptCurve.Count,1)); 
%   Set the trasformation
R = axang2rotm([0 0 1 -70/180*pi]);
T = [R,zeros(3,1);0 0 0 1];
tf = affine3d(T);
%   Rotate and save
ptLaser_aligned = pctransform(ptLaser,tf);
ptCurve_aligned = pctransform(ptCurve,tf);
figure;
hold on;
draw_coordinate_system(5*ones(3,1), eye(3), [0, 0, 0], 'rgb');
pcshow(ptLaser_aligned);
scatter3(ptCurve_aligned.Location(:,1),...
    ptCurve_aligned.Location(:,2),...
    ptCurve_aligned.Location(:,3),'filled')
pcwrite(ptLaser_aligned,[PLY_folder_path,LaserNewFileName],'PLYFormat','binary');
pcwrite(ptCurve_aligned,[PLY_folder_path,DigitizeNewFileName],'PLYFormat','binary');
end