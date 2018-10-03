
function ptOutput=SpherePt3D(kidneyLetter,varargin)
if nargin<1
    kidneyLetter='F';
end
plotOption=0;
% close all
projectMethod='closest';

saveData=0;
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'Plot') % 0 or 1, to plot data
            plotOption = propertyValue;
        elseif strcmpi(propertyName,'Save') % 0 or 1, 1 to save output
            saveData = propertyValue;
        elseif strcmpi(propertyName,'projectMethod')
            projectMethod = propertyValue;
            % Options:
            % 'z' : project vertically in the z direction
            % 'organ' : project in the average norm of the
            % 'closest' : find the closest point on the mesh
        end
    end
end
switch kidneyLetter
    case 'B'
        stlName='KidneyB_Spheres_reduced.stl';
        orgName='Kidney_B_UserStudy.stl';
    case 'E'
        stlName='KidneyE_Spheres_reduced.stl';
        orgName='Kidney_E_UserStudy.stl';
    case 'F'
        stlName='KidneyF_Spheres_reduced.stl';
        orgName='Kidney_F_UserStudy.stl';
end
FolderName='R:\Robots\CPD_Reg.git\userstudy_data\STL\';
[F,V]=stlread([FolderName stlName]);
V=V'/1000;

index=kmeans(V',2);
s1=index==1;
s2=index==2;

ball1=V(:,s1);
ball2=V(:,s2);
% plot3(ball1(1,:),ball1(2,:),ball1(3,:))
% hold on;plot3(ball2(1,:),ball2(2,:),ball2(3,:))


%%
% Find the center of the sphere
centerV=mean(V,2);
FolderName='R:\Robots\CPD_Reg.git\userstudy_data\PointCloudData\OutputMesh\';
[Forg,Vorg]=stlread([FolderName orgName]);
P1=Vorg(Forg(:,1),:);
P2=Vorg(Forg(:,2),:);
P3=Vorg(Forg(:,3),:);
intersect=zeros(size(P1,1),1);
xhist=zeros(1,3);

switch projectMethod
    case 'z'
        ballpt1=mean(ball1,2);
        dir=[0;0;1];
        
        [intersect,~,~,~,xcoor] = TriangleRayIntersection(ballpt1,dir, P1,P2,P3,'lineType','line','border','inclusive','eps',1e-8);
        ptOutput(:,1)=xcoor(intersect,:)';
        
        ballpt2=mean(ball2,2);
        [intersect,~,~,~,xcoor] = TriangleRayIntersection(ballpt2,dir, P1,P2,P3,'lineType','line','border','inclusive','eps',1e-8);
        ptOutput(:,2)=xcoor(intersect,:)';
        
    case 'organ'
        ballpt1=mean(ball1,2);
        [n,~]=fitPlane(Vorg');
        dir=n;
        
        [intersect,~,~,~,xcoor] = TriangleRayIntersection(ballpt1,dir, P1,P2,P3,'lineType','line','border','inclusive','eps',1e-8);
        ptOutput(:,1)=xcoor(intersect,:)';
        
        ballpt2=mean(ball2,2);
        [intersect,~,~,~,xcoor] = TriangleRayIntersection(ballpt2,dir, P1,P2,P3,'lineType','line','border','inclusive','eps',1e-8);
        ptOutput(:,2)=xcoor(intersect,:)';
        
    case 'closest'
        % Find the distance between each point on the sphere and the mesh,
        % use a simple upsampling method
        upsampling=1;
        
        P = upsamplemesh(Forg,Vorg,upsampling);
        [k,d] = dsearchn(P,ball1');
        [~,i]=min(d);
        ptOutput(:,1)=P(k(i),:);
        ballpt1=ball1(:,i);
        
        [k,d] = dsearchn(P,ball2');
        [~,i]=min(d);
        ptOutput(:,2)=P(k(i),:);
        ballpt2=ball2(:,i);
        
end

if plotOption
    % Plot the original artery and the smoothed output
    figure
    trisurf(Forg,Vorg(:,1),Vorg(:,2),Vorg(:,3), intersect*1.0,'FaceAlpha', 0.9)
    hold on
    plot3(ballpt1(1),ballpt1(2),ballpt1(3),'yo')
    plot3(ballpt2(1),ballpt2(2),ballpt2(3),'yo')
%     plot3(ball1(1,:),ball1(2,:),ball1(3,:))
%     plot3(ball2(1,:),ball2(2,:),ball2(3,:))
    plot3(ptOutput(1,:),ptOutput(2,:),ptOutput(3,:),'rx')
end

if saveData
    cpdDir=getenv('CPDREG');
    save([cpdDir filesep 'userstudy_data/PLY/Kidney_' num2str(kidneyLetter) '_Sphere_Pts'],'ptOutput');
    cloud=pointCloud(ptOutput');
    pcwrite(cloud,[cpdDir filesep 'userstudy_data/PLY/Kidney_' num2str(kidneyLetter) '_Sphere_Pts.ply'],'Encoding','ascii');
    pcwrite(cloud,[cpdDir filesep 'userstudy_data/PLY/Kidney_' num2str(kidneyLetter) '_Sphere_Pts.pcd'],'Encoding','ascii');
end
end