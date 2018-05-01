function MeshSave( Vertices, Triangles, Normals, meshFile )
% saves a mesh to file
%  MeshSave(Vertices, Triangles, Normals, MeshFileName) 
% 
%  assumes file format:
%
%   POINTS numPoints
%   px py pz
%    ...
%   px py pz
%   TRIANGLES numTriangles
%   vx1 vx2 vx3
%    ...
%   vx1 vx2 vx3
%   NORMALS numTriangles
%   nx ny nz
%    ...
%   nx ny nz
%
%  NOTE: vertex indices are stored in zero-based (C++) indexing format
%        before saving, these indices are converted from Matlab (one-based)
%        to C++ indexing format
%

%file = 'C:/Data/Images/2010-12-01_CIRSPhantomSurfaceModel/CIRSPhantomCT_LowRes.mesh'

numVertices = size(Vertices,1);
numTriangles = size(Triangles,1);
numNormals = size(Normals,1);
if (numNormals ~= numTriangles)
  error('ERROR: number of normals does not match number of triangles');
end

% convert triangles to C++ (zero-base) indexing
Triangles = Triangles - ones(numTriangles,3);

disp(['Saving mesh to file: ', meshFile]);

f = fopen(meshFile,'w');
if f == -1
    error(['ERROR: failed to open file: ', meshFile])
end

% write vertices to file
fprintf(f,'POINTS %d\n',numVertices);
fprintf(f,'%f %f %f\n',Vertices');

% write triangles to file
fprintf(f,'TRIANGLES %d\n',numTriangles);
fprintf(f,'%d %d %d\n',Triangles');

% write normals to file
fprintf(f,'NORMALS %d\n',numNormals);
fprintf(f,'%f %f %f\n',Normals');

fclose(f);

msg = sprintf(' ...mesh save complete');
disp(msg);
