function Vein_Fiducial_segmentation()
%%  Vein & Fiducial segmentation from Dicom images
%   By Long Wang & Colette Abah, 2018/4/3
%   The Vein is segmented by calculating the centroid of Dicom images and
%   the fiducials are segmented manually by recording the pixel positions
%   in Dicom images
%%  Calculating fidicual locations in CT
Fiducial_result_file_name = 'CT_KidneyA_fiducials';
%   Manual segmentaion of surface centers of the dimples
centers = [... % each row is a position, A, B, C, D
    5.85,	125.3,	100.5;
    4.1,	124,	49.8;
    156.8,	130.9,	43.3;
    158.7,	132.4,	93 ...
    ];
%   Manual segmentaion of pivot tips
pivot_tips = [... % each row is a position, A, B, C, D
    5.8,	129.4,	100.5;
    4.05,	128,	49.8;
    156.7,	135.7,	43.3;
    158.65,	136.5,	93 ...
    ];
%   offset the raidus of probe ball
%       According to the spec of dimples, 45 degree slop counter-sink
HalfCounterSinkAngle = pi/2/2;

probeBall_R = 6.35/2; % unit in [mm]
N_Fiducials = length(centers);
Fiducial_LPI_coordinates = zeros(N_Fiducials,3);
for i=1:N_Fiducials
    offset_dir = normr(centers(i,:)-pivot_tips(i,:));
    Fiducial_LPI_coordinates(i,:) = ...
        pivot_tips(i,:) + ...
        (probeBall_R/sin(HalfCounterSinkAngle))*offset_dir;
end
Fiducial_RAS = diag([1,1,-1])*Fiducial_LPI_coordinates';
Fiducial_RAS = Fiducial_RAS';
%%  Dicom image information
Vein_file_name = 'CT_KidneyA_vein_curve';
folderName = 'C:\Users\long\Desktop\Data\UserStudy\KidneyA_vein\ScalarVolume_33';
filePrefix = 'IMG';
fileSuffix = '.dcm';
N_images = 300;
%%  get center coordinates from each image
figure;
hold on;
Positions = zeros(N_images,3);
Areas = zeros(N_images,1);
for i = 1:N_images
    %   load one image
    idName = sprintf('%04d',i);
    imageName = [folderName,filesep,filePrefix,idName,fileSuffix];
    dicomIm = dicomread(imageName);
    dicomIm_bw = (dicomIm>0);
    dicomInfo = dicominfo(imageName);
    imageCornerPos = dicomInfo.ImagePositionPatient;
    px_Spacing = dicomInfo.PixelSpacing;
    %   process the image
    region_info = regionprops(dicomIm_bw,'centroid','area');
    [area,idx] = max([region_info.Area]);
    center_im = region_info(idx).Centroid;
    %   show figure result
    cla;
    imshow(dicomIm_bw);
    hold on;
    plot(center_im(1),center_im(2),'g+','LineWidth',2);
    drawnow;
    if i==1
        pause;
    end
    %   store results and calculate positions
    Positions(i,1:2) = imageCornerPos(1:2,1)' + center_im*diag(px_Spacing);
    Positions(i,3) = imageCornerPos(3);
    Areas(i) = area;
end

%%  detele points below threshold area amount
std_area = std(Areas);
mean_area = mean(Areas);
min_area = mean_area-3*std_area;
idx_to_del = Areas<min_area;
Positions(idx_to_del,:) = [];

%%  Transform the points into "RAS" frame
%%  Background
%   The 3D slicer generates 3D model of segmentation in a different frame
%   than when using the Dicom images.

%   A curve has been segmented using the Dicom image, and needs to be
%   transformed to the frame of the 3D mesh generated in Slicer.

%   This transformation can be otained from Slicer as:
%       Data->Transform Hierarchy -> Node information -> IJKtoRASDirections
M = [...
    -1, 0, 0, 0;...
    0, -1, 0, 0;...
    0,  0, 1, 0;...
    0,  0, 0, 1 ...
    ];
RegCurveHomog = inv(M)*[Positions';ones(1,length(Positions))];
Positions = transpose(RegCurveHomog(1:3,:));
Fiducial_Positions_Homog = inv(M)*[Fiducial_RAS';ones(1,length(Fiducial_RAS))];
Fiducial_Positions = transpose(Fiducial_Positions_Homog(1:3,:));
%%  Save the data
PC_path_CT = [getenv('UDPREGJHU'),'\UserStudy_Data\PointCloudData\'];
save([PC_path_CT,Vein_file_name],'Positions');
Fiducial_Path = [getenv('UDPREGJHU'),'\UserStudy_Data\FiducialLocations\'];
save([Fiducial_Path,Fiducial_result_file_name],'Fiducial_Positions');
fprintf('segmented vein and fiducials saved \n');
end