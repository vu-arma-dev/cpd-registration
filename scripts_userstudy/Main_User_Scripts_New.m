% Same as Main_User_Scripts - performs segmentation and saving of organ
% stls/ply/mesh files
% Different from Main_User_Scripts in that is processes the new red-dyed
% organs numbered 10-12 and 20-22
CPD_DIR=fileparts(fileparts(mfilename('fullpath')));
addpath([CPD_DIR filesep 'functions']);

oList=[10:12,20:22];

%% Generate CPD registration of new organs, save the data as a mat file
for jj=oList
    APP_RegAprToCT(jj,CPD_DIR);
end

%% Plot results to verify
for jj=oList
    load([fileparts(fileparts(mfilename('fullpath'))) filesep 'userstudy_data' filesep 'PointCloudData' filesep 'RegAprToCT' filesep 'Kidney_' num2str(jj) '_iter_100_NoOpt.mat'])
    figure
    scatter3(ptCTScan.Location(:,1),ptCTScan.Location(:,2),ptCTScan.Location(:,3))
%     figure
    hold on
    scatter3(T.Y(:,1),T.Y(:,2),T.Y(:,3));
    axis equal
end
% close all

%% Save CPD results to mesh/stl files
for jj=oList
    Generate_Mesh_User_Study(jj);
end

%% Save fiducial locations
Organ_Registration_New %TODO: add FidA.ply to each kidney output folder in git, only saved at VU ARMAlab host - output is ok, saved in FiducialLocations

%% Test Organ Registration with robot data
% TODO

%% Perform segmentation of sphere points and save mat file/pcd of spheres for ground truth
for jj=oList
    SpherePt3D(jj,'Save',1,'projectMethod','organ','Plot',0);
end

% for jj=oList
%     SpherePt3D(jj,'Save',0,'projectMethod','closest','Plot',1);
% end