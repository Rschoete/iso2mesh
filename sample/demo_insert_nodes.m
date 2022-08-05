%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   demo script to insert new nodes to a tetrahedral mesh
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all
load rat_head.mat

[node,elem,face]=vol2mesh(volimage>0.05,1:size(volimage,1),1:size(volimage,2),...
                          1:size(volimage,3),2,2,1);

plotmesh(node,face);

x = 1:size(volimage,1);
y = 1:size(volimage,2);
z = 1:size(volimage,3);

xslice = [length(x)/2,length(x)/4,3/4*length(x)];    % location of y-z planes
yslice = 25;              % location of x-z plane
zslice = [10,30];         % location of x-y planes
[x,y,z] = meshgrid(y,x,z);
figure()
slice(x,y,z,volimage,xslice,yslice,zslice)

dt=pi/10;
r=3;
x0=30;
y0=28;
theta=dt:dt:2*pi;
x=x0+r*cos(theta);
y=y0+r*sin(theta);

p0=[x;y;ones(1,length(theta))*50]';
v0=[0,0,-1];

[t,u,v,idx,xnode]=raysurf(p0,v0,node,face(:,1:3));

hold on
plotmesh(p0,'r+');

goodpt=find(~isnan(xnode(:,1)) & ~isnan(xnode(:,2)) & ~isnan(xnode(:,3)));
[newnode,newelem,newface]=meshrefine(node,elem,face,xnode(goodpt,:));

figure
plotmesh(newnode,newface);
hold on
plotmesh(p0,'r+');
plotmesh(xnode,'r.')



%% test raytrace instead raysurf
len=size(p0,1);
t2 = cell(1,len);
u2 = cell(1,len);
v2 = cell(1,len);
idx2 = cell(1,len);
xnode2 = [];
for i=1:len
   [ti,ui,vi,id]=raytrace(p0(i,:),v0,node,face(:,1:3));
   if(isempty(id)) continue; end
   ti=ti(id);
   tpid=find(ti>=0);
   if(isempty(tpid)) continue; end
   t2{i}=ti(tpid);
   u2{i}=ui(id(tpid));
   v2{i}=vi(id(tpid));
   idx2{i}=id(tpid);
   for it=t2{i}(:)
       ixnode=p0(i,:)+repmat(it,1,3).*v0;
       xnode2 = [xnode2; ixnode];
   end
end


hold on
plotmesh(p0,'r+');

goodpt2=find(~isnan(xnode2(:,1)) & ~isnan(xnode2(:,2)) & ~isnan(xnode2(:,3)));
[newnode2,newelem2,newface2]=meshrefine(node,elem,face,xnode2(goodpt2,:));

figure
plotmesh(newnode2,newface2);
hold on
plotmesh(p0,'r+');
plotmesh(xnode2,'r.')
