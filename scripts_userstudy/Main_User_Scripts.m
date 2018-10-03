% Validate_CT_Registration
labels={'A','B','C','D','E','F'};

%% Perform segmentation of artery points and save mat file/pcd of artery curves
for i=[1,3,4]
    ArteryCentroid3D(labels{i},'Save',0,'projectOnSurface',1,'Plot',1);
end

%% Perform segmentation of sphere points and save mat file/pcd of spheres
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=[2,6]
    SpherePt3D(labels{i},'Save',0,'projectMethod','closest');
end
i=5;
SpherePt3D(labels{i},'Save',1,'projectMethod','organ');

%% Generate CPD registration of organs, save the data as a mat file
for i=1:6
    APP_RegAprToCT(labels{i});
end

%% Plot Results of CPD registration to verify
for i=1:6
    load(['R:\Robots\CPD_Reg.git\userstudy_data\PointCloudData\RegAprToCT\Kidney_' labels{i} '_iter_100_NoOpt.mat'])
    figure(i+6)
    scatter3(ptCTScan.Location(:,1),ptCTScan.Location(:,2),ptCTScan.Location(:,3))
    hold on
    scatter3(T.Y(:,1),a.T.Y(:,2),T.Y(:,3));
    axis equal
end

%% Perform mesh generation of mat files, output .stl and .mesh files
for i=1:6
    Generate_Mesh_User_Study(labels{i});
end

%% Test fiducial-based registration for registering the mesh/stl to robot data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO - not finished
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cpd_dir=getenv('CPDREG');

load([cpd_dir filesep 'userstudy_data' filesep 'Fiducial_2018-05-01'])
% fiducial_Robot=FiducialPositions'*1000;
fiducial_Robot=stlPoints;

for i=1:6 %test each organ for funzies
    load([cpd_dir filesep 'userstudy_data' filesep 'FiducialLocations' filesep 'FiducialLocations_' num2str(i)]);
    fiducial_CT=FidLoc;
    
    [R,t] = rigidPointRegistration(fiducial_CT,fiducial_Robot);
    % R*Fiducial_CT+t ~~ Fiducial_Robot
    % save to file R and t
    quat=rotm2quat(R(1:3,1:3));
    errorMean(i)=mean(sqrt(sum((R*fiducial_CT+t - fiducial_Robot).^2)));
end