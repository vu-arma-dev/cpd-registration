% Validate_CT_Registration
labels={'A','B','C','D','E','F'};

%% Perform segmentation of artery points and save mat file/pcd of artery curves
for i=[1,3,4]
    ArteryCentroid3D(labels{i},'Save',1);
end


%% Perform segmentation of sphere points and save mat file/pcd of spheres
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=[2,5,6]
    SpherePt3D(labels{i},'Save',1); %QUESTION: what are we saving? Center, closest on mesh? Closest to mesh on sphere?
end

%% Generate CPD registration of organs, save the data as a mat file
for i=3:6
    APP_RegAprToCT(labels{i});
end

%% Plot Results of CPD registration to verify
for i=1:6
    load(['R:\Robots\CPD_Reg.git\userstudy_data\PointCloudData\RegAprToCT\Kidney_' labels{i} '_iter_100.mat'])
    figure(i)
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
% TODO
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
