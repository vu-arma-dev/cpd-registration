% This is a function to register the fiducial markers of the kidney markers
clear
close all
clc
addpath('R:\Projects\NRI\PalpationSTL\User_Study\Output\ypml110-dbscan-clustering\YPML110 DBSCAN Clustering\DBSCAN Clustering');
addpath(genpath(getenv('ARMA_CL')))

kidneyLabels={'A','B','C','D','E','F'};
baseFolder='R:\Projects\NRI\PalpationSTL\User_Study\Output\';
folderNames{length(kidneyLabels)}='';
fiducialNames{length(kidneyLabels)}='';
for i=1:length(kidneyLabels)
    folderNames{i}=['Kidney' kidneyLabels{i} '_output\'];
    fiducialNames{i}=['Kidney' kidneyLabels{i} 'Fids.ply'];
    fidsCloud{i}=pcread([baseFolder folderNames{i} fiducialNames{i}]);
    [F{i},V{i}]=stlread(['R:\Projects\NRI\PalpationSTL\User_Study\Output\Kidney' kidneyLabels{i} ...
        '_output\Kidney' kidneyLabels{i} '_TopOnly_Reduced.stl']);
end

%%
for i=1:5
    figure(i)
    pcshow(fidsCloud{i})
end

%%
epsilon=0.5;
MinPts=10;

for i=1:6
    X=double(fidsCloud{i}.Location);
    IDX{i}=DBSCAN(X,epsilon,MinPts);
end

%%

for organ=1:6
    
    figure(organ)
    for i=1:12
        myPoint{i}=fidsCloud{organ}.Location(IDX{organ}==i,:);
        plot3(myPoint{i}(:,1),myPoint{i}(:,2),myPoint{i}(:,3),'.')
        centerPoint{organ}(1:3,i)=double(mean(myPoint{i}));
        hold on
        mytext{i}=num2str(i);
    end

    text(centerPoint{organ}(1,:),centerPoint{organ}(2,:),centerPoint{organ}(3,:)+10,mytext)
    patch('Faces',F{organ},'Vertices',V{organ},'FaceColor','red')
    xlabel('x');ylabel('y');zlabel('z')

end

%% Circle fitting for partial scans of organ 2
centerPtNew=[];
Base_Fiducial_Location
for index=[1 3 8 9]
    pt=fidsCloud{2}.Location(IDX{2}==index,:);
    pt=double(pt);
    [myplane.n,myplane.p]=fitPlane(pt');
    ptInPlane=proj_onto_a_plane(myplane,pt);
    k=convhull(ptInPlane(:,1),ptInPlane(:,3));
    circPts=ptInPlane(k,:);
    circPts(circPts(:,1)>0,:)=[];
    [px,pz,R]=circfit(circPts(:,1),circPts(:,3));

    centerPtNew=[centerPtNew;[px,circPts(1,2),pz]];
    centerPoint{2}(:,index)=[px,circPts(1,2),pz];
end

%% Find missing points of organ B
figure(2)
% regPoints2=centerPoint{2}(:,[7 6 10 2 4 5]);
regPoints2=centerPoint{2}(:,[1 3 8 9 7 6 10 2 4 5]);
[R,p]=rigidPointRegistration(stlPointList([1:2, 4:5, 7:end],:)',regPoints2);
transformedPoints=R*stlPointList'+p;
plot3(transformedPoints(1,:),transformedPoints(2,:),transformedPoints(3,:),'rx','MarkerSize',15);

A=[14.06 -127.8 -97.54];
B=[14.38 -126.4 -47.56];

centerPoint{2}(:,11)=A';
centerPoint{2}(:,12)=B';
%% Find missing points of organ E
figure(5)
regPoints5=centerPoint{5}(:,[1 2 6 7 8 9 10 3 4 5]);
[R,p]=rigidPointRegistration(stlPointList([1:2,4:5,7:end],:)',regPoints5);
transformedPoints=R*stlPointList'+p;
plot3(transformedPoints(1,:),transformedPoints(2,:),transformedPoints(3,:),'rx','MarkerSize',15);

A=[1.933 -128.8 -100.5];
B=[3.875 -127.1 -50.59];

centerPoint{5}(:,11)=A';
centerPoint{5}(:,12)=B';
%% Find missing points of organ F
figure(6)
regPoints6=centerPoint{6}(:,[8 9 10 3 4 5 1 2 6 7]);
[R,p]=rigidPointRegistration(stlPointList([1:8,10:11],:)',regPoints6);
transformedPoints=R*stlPointList'+p;
plot3(transformedPoints(1,:),transformedPoints(2,:),transformedPoints(3,:),'rx','MarkerSize',15);

C=[5.14 -127.8 -82.61];
D=[5.693 -126.3 -32.64];
centerPoint{6}(:,11)=C';
centerPoint{6}(:,12)=D';
%%
ABCDMap=[3 9 12 6
         11 12 10 5
         3 9 12 6
         6 11 12 5
         11 12 10 5
         10 5 11 12];
%   offset the raidus of probe ball
%       According to the spec of dimples, 45 degree slop counter-sink
HalfCounterSinkAngle = pi/2/2;

probeBall_R = 6.35/2; % unit in [mm]
% N_Fiducials = length(centers);
% Fiducial_LPI_coordinates = zeros(N_Fiducials,3);

% offset_dir = normr(centers(i,:)-pivot_tips(i,:));
%     Fiducial_LPI_coordinates(i,:) = ...
%         pivot_tips(i,:) + ...
%         (probeBall_R/sin(HalfCounterSinkAngle))*offset_dir;
% end

for i=1:6
    ABCD{i}=centerPoint{i}(:,ABCDMap(i,:));
    FidLoc=ABCD{i};
    offset_dir=fitPlane(FidLoc);
    FidLoc=FidLoc-offset_dir*(probeBall_R/sin(HalfCounterSinkAngle));
    save(['../userstudy_data/FiducialLocations/FiducialLocations_' num2str(i)],'FidLoc');
end
     
