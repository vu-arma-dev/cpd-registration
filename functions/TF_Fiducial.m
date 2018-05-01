function [point_cloud_TF, TF_fiducial] = TF_Fiducial(point_cloud, F_with_PC, F_target)
%%  Transform a point cloud according to fidual correspondance
%   By Long Wang, 2017/5/29
%%  INPUTS:
%   point_cloud - the point cloud that needs to be transformed
%   F_with_PC - Fiducial with the point cloud (file name)
%   F_target - Fiducial on the target site (file name)
%%  Rigid transformation between fiducial pairs
[R,t] = rigidPointRegistration(F_with_PC,F_target);
TF_fiducial = [R,t;...
                0 0 0 1];
point_cloud_TF = R*point_cloud' + repmat(t,1,size(point_cloud,1));
point_cloud_TF = point_cloud_TF';
end

