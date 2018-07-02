
function ptOutput=ArteryCentroid3D(kidneyLetter,varargin)
if nargin<1
    kidneyLetter='A';
end
plotOption=0;
N=200;
cutoff=[5;10];
smoothParam=0.01;
saveData=0;
if numel(varargin)
    for i = 1:2:numel(varargin)
        propertyName = varargin{i};
        propertyValue = varargin{i+1};
        if strcmp(propertyName,'Plot') % 0 or 1, to plot data
            plotOption = propertyValue;
        elseif strcmp(propertyName,'N') % Number of spline points
            N = propertyValue;
        elseif strcmp(propertyName,'Cutoff') % 2 elements for cut off begin/end of spline
            cutoff = propertyValue;
        elseif strcmp(propertyName,'Spline Smooth') % Between 0 and 1, 0 is smoother
            smoothParam = propertyValue;
        elseif strcmpi(propertyName,'Save') % Between 0 and 1, 0 to save output
            saveData = propertyValue;
        end
    end
end
switch kidneyLetter
    case 'A'
        stlName='KidneyA_Artery_reduced.stl';
    case 'C'
        stlName='KidneyC_Artery_reduced.stl';
    case 'D'
        stlName='KidneyD_Artery_reduced.stl';
end
FolderName='R:\Robots\CPD_Reg.git\userstudy_data\STL\';
[F,V]=stlread([FolderName stlName]);
V=V'/1000;


%%
% Use PCA/SVD to fit a line for V, after demeaning
centerV=mean(V,2);
Vdemean=(V-centerV);
[U,~,~]=svd(Vdemean,'econ');
mainVec=U(:,1);
Omega=mainVec*mainVec';
Vlinear=Omega*Vdemean;

% Find a line representing the distances from one end of the line to the other
startLine = min(Vlinear,[],2);
endLine = max(Vlinear,[],2);
lineVec = endLine-startLine;
fromStartV=Vlinear-startLine;
lineDistance=normc(lineVec)'*fromStartV;
lineDistance=lineDistance/max(lineDistance);

% From the "beginning" of that line to the "end" of that line, split into N chunks.
% figure
fitPlane.n=normc(lineVec);
for i=1:N
    relevantLines=Vlinear(:,lineDistance<i/N & lineDistance >= (i-1)/N);
    relevantV=Vdemean(:,lineDistance<i/N & lineDistance >= (i-1)/N);
    %     Find the center of each chunk
    fitPlane.p=relevantV(:,1);
    planeData=proj_onto_a_plane(fitPlane,relevantV)';
    centerArtery(:,i)=mean(planeData,2);
%     plot3(relevantV(1,:),relevantV(2,:),relevantV(3,:))
%     hold on
end
% Fit a smoothed spline to the data
p = csaps(1:length(centerArtery),centerArtery,smoothParam);
smoothed=fnval(p,1:length(centerArtery));
centerArtery=centerArtery+centerV;
smoothed = smoothed+centerV;

% Cut off a piece of the spline
cutStart=cutoff(1);
cutEnd=cutoff(2);
beginI=1+cutStart;
endI=N-cutEnd;

ptOutput=smoothed(:,beginI:endI);

if plotOption
    % Plot the original artery and the smoothed output
    figure
    plot3(V(1,:),V(2,:),V(3,:));
    ax1=gca;
    figure
    plot3(centerArtery(1,:),centerArtery(2,:),centerArtery(3,:),'kx')
    hold on
    plot3(ptOutput(1,:),ptOutput(2,:),ptOutput(3,:),'r')
    axis([ax1.XLim, ax1.YLim, ax1.ZLim])
end
if saveData
    save(['../userstudy_data/PLY/Kidney_' num2str(kidneyLetter) '_Artery_Pts'],'ptOutput');
    cloud=pointCloud(ptOutput');
    pcwrite(cloud,['../userstudy_data/PLY/Kidney_' num2str(kidneyLetter) '_Artery_Pts.ply'],'Encoding','ascii');
    pcwrite(cloud,['../userstudy_data/PLY/Kidney_' num2str(kidneyLetter) '_Artery_Pts.pcd'],'Encoding','ascii');
end
end