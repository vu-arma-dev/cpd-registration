
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
            % 'organ' : project in the average norm of the organ
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
if kidneyLetter >=10 && kidneyLetter<=22
    orgNum = num2str(kidneyLetter);
    stlName = ['Sphere' orgNum '_clean.stl'];    
    orgName = ['Kidney_' orgNum '_UserStudy.stl'];
end
FolderName='R:\Robots\CPD_Reg.git\userstudy_data\STL\';
[F,V]=stlread([FolderName stlName]);
V=V'/1000;


if kidneyLetter>=20 && kidneyLetter <=22
    Nball=3;
else
    Nball=2;
end
index=kmeans(V',Nball);
for ii=1:Nball
    sIndex{ii}=index==ii;
    balls{ii}=V(:,sIndex{ii});
end

%%
% Read the organ
FolderName='R:\Robots\CPD_Reg.git\userstudy_data\PointCloudData\OutputMesh\';
[Forg,Vorg]=stlread([FolderName orgName]);
P1=Vorg(Forg(:,1),:);
P2=Vorg(Forg(:,2),:);
P3=Vorg(Forg(:,3),:);
intersect=zeros(size(P1,1),1);

switch projectMethod
    case 'z'
        
        for jj=1:Nball
            ballpt{jj}=mean(balls{jj},2);
            dir=[0;0;1];
            [intersect,~,~,~,xcoor] = TriangleRayIntersection(ballpt{jj},dir, P1,P2,P3,'lineType','line','border','inclusive','eps',1e-8);
            ptOutput(:,jj)=xcoor(intersect,:)';
        end
        
    case 'organ'
        [n,~]=fitPlane(Vorg');
        dir=n;
        for jj=1:Nball
            ballpt{jj}=mean(balls{jj},2);
            [intersect,~,~,~,xcoor] = TriangleRayIntersection(ballpt{jj},dir, P1,P2,P3,'lineType','line','border','inclusive','eps',1e-8);
            ptOutput(:,jj)=xcoor(intersect,:)';
        end
        
    case 'closest'
        % Find the distance between each point on the sphere and the mesh,
        % use a simple upsampling method
        upsampling=1;
        
        P = upsamplemesh(Forg,Vorg,upsampling);
        for jj=1:Nball
            [k,d] = dsearchn(P,balls{jj}');
            [~,i]=min(d);
            ptOutput(:,jj)=P(k(i),:);
            ballpt{jj}=mean(balls{jj},2);
        end        
end

if plotOption
    % Plot the original artery and the smoothed output
    figure
    trisurf(Forg,Vorg(:,1),Vorg(:,2),Vorg(:,3), intersect*1.0,'FaceAlpha', 0.9)
    hold on
    for jj=1:Nball
        plot3(ballpt{jj}(1),ballpt{jj}(2),ballpt{jj}(3),'yo')
    end
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