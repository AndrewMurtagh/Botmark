%{
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% 
%   Copyright (C) 2017 Trinity Robotics Group.
%
%
%   FILENAME:   prm.m 
%
%
%   DATE: 20/02/2017
%
%
%   DESCRIPTION: program to compute a path by probabilistic roadmap method.
%
%
%   AUTHOR: Andrew Murtagh, 
%           Trinity Robotics Group, Trinity College Dublin.
%
%
%   NOTES:  -map representation is a b/w image.
%           -uses A* to search graph.
%
%
%   VERSION: v1
%
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%}




clear; close all; clc;

%--------------------------------------------------------------------------
%parameters
%--------------------------------------------------------------------------
map = im2bw(imread('../Data/office.png'));
source=[45 140]; %Y X
goal=[460 460]; 
k=150; 
display=true; 




%--------------------------------------------------------------------------
% add start and goal poses
%--------------------------------------------------------------------------
if ~collisionFreePoint(source,map)
    disp('source lies on an obstacle or outside map'); 
end
if ~collisionFreePoint(goal,map)
    disp('goal lies on an obstacle or outside map'); 
end

imshow(map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k')

if display, title('Add start and goal positions'), end
vertices=[source;goal]; 
if display, rectangle('Position',[vertices(1,2)-5,vertices(1,1)-5,10,10],'Curvature',[1,1],'FaceColor','g'); end
if display, rectangle('Position',[vertices(2,2)-5,vertices(2,1)-5,10,10],'Curvature',[1,1],'FaceColor','g'); end
if display, disp('press any key'), waitforbuttonpress, end


%--------------------------------------------------------------------------
% add randomly sampled points
%--------------------------------------------------------------------------
if display, title('Add randomly sampled collision free points'), end
while length(vertices)<k+2 % 
    pos=double(int32(rand(1,2) .* size(map)));
    if collisionFreePoint(pos,map), 
        vertices=[vertices;pos]; 
        if display, rectangle('Position',[pos(2)-5,pos(1)-5,10,10],'Curvature',[1,1],'FaceColor','r'), end
    end
end
if display, disp('press any key'), waitforbuttonpress, end





%--------------------------------------------------------------------------
% add edges
%--------------------------------------------------------------------------
if display, title('Add collision free edges'), end
edges = cell(length(vertices),1);

for i=1:length(vertices)
 
    for j=i+1:length(vertices)
        if collisionFreeEdge(vertices(i,:),vertices(j,:),map);
            edges{i}=[edges{i};j];
            edges{j}=[edges{j};i];
            if display, line([vertices(i,2);vertices(j,2)],[vertices(i,1);vertices(j,1)]); end
        end
    end
    
end
if display, disp('press any key'), waitforbuttonpress, end








%--------------------------------------------------------------------------
% find path with A* 
%--------------------------------------------------------------------------
if display, title('A* pathfinding'), end
%       index of node in vertices,  historic cost,                          heuristic cost,                 total cost,                         parent index in closed list (-1 for source) 
open=[  1                           0                                       heuristic(vertices(1,:),goal)   0+heuristic(vertices(1,:),goal)     -1]; 
closed=[]; 
pathFound=false;


while size(open,1)>0
    %get lowest cost node
    lowestCostNodeIndex = 1;
    for i=1:size(open,1)
        if open(i,4) < open(lowestCostNodeIndex,4)
            lowestCostNodeIndex = i;
        end
    end
    lowestTotalCostNode = open(lowestCostNodeIndex,:);
    open(lowestCostNodeIndex,:) = [];
    
    %if lowest cost node is goal
     if lowestTotalCostNode(1)==2                                                                 
         pathFound=true;
         break;
     end
     
     
     for nodechildindex=1:length(edges{lowestTotalCostNode(1),1})      
         %is in closed
         childindexinvertices=edges{lowestTotalCostNode(1),1}(nodechildindex);
         inClosed = false;
         for i=1:size(closed,1)
             if closed(i,1) == childindexinvertices
                 inClosed = true;
             end
         end
         
         if length(closed)==0 || ~inClosed                      
             historicCost=lowestTotalCostNode(2)+historic(vertices(lowestTotalCostNode(1),:),vertices(childindexinvertices,:));
             heuristicCost=heuristic(vertices(childindexinvertices,:),goal);
             totalCost=historicCost+heuristicCost;
             addtoopen=true;          
             
             %is already in open     
             existingOpenNodeIndex = -1;
             for i=1:size(open,1)
                 if open(i,1) == childindexinvertices
                     existingOpenNodeIndex = i;
                 end
             end
             
             if existingOpenNodeIndex ~= -1
                 if open(existingOpenNodeIndex,4)<totalCost
                    addtoopen=false;
                 else
                    open(existingOpenNodeIndex,:) = [];
                    addtoopen=true;
                 end
             end
             if addtoopen
                 newnode = [childindexinvertices historicCost heuristicCost totalCost size(closed,1)+1];
                 open=[open; newnode];   
             end
         end           
     end
     closed=[closed; lowestTotalCostNode];                                                            

end




%--------------------------------------------------------------------------
% construct path
%--------------------------------------------------------------------------
if pathFound
    disp('path found');
    path=[vertices(lowestTotalCostNode(1), :)]; 
    parentIndexInClosed=lowestTotalCostNode(5);
    while parentIndexInClosed>0
        parentIndexInVertices = closed(parentIndexInClosed,1);
        path=[vertices(parentIndexInVertices, :); path];
        parentIndexInClosed=closed(parentIndexInClosed,5);
    end

line(path(:,2),path(:,1),'color','g','LineWidth',3);
else 
    disp('no path found');
end








%--------------------------------------------------------------------------
% define functions
%--------------------------------------------------------------------------
function feasible=collisionFreePoint(point,map)
    feasible=true;
    if ~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 && point(2)<=size(map,2) && map(point(1),point(2))==1)
        feasible=false;
    end
end


function feasible=collisionFreeEdge(n,newPos,map)
    feasible=true;
    dir=atan2(newPos(1)-n(1),newPos(2)-n(2));
    for r=0:0.5:sqrt(sum((n-newPos).^2))
        posCheck=n+r.*[sin(dir) cos(dir)];
        if ~(collisionFreePoint(ceil(posCheck),map) && collisionFreePoint(floor(posCheck),map) && ... 
                collisionFreePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && collisionFreePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
            feasible=false;break;
        end
        if ~collisionFreePoint(newPos,map), feasible=false; end
    end
end

function h=heuristic(X,goal)
    h = sqrt(sum((X-goal).^2));
end

function h=historic(a,b)
    h = sqrt(sum((a-b).^2));
end
