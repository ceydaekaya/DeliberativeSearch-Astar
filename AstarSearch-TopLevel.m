clear all
clc
scaling = 10;

load('OccupancyMap_v2.mat');

GoalLocation = [5, 5];

pathLength = 0;

%0 for no obstacle 1 for obstacle
obstacleEncountered = 0;

PlotOccupancyMap(OccupancyMap, GoalLocation);

RobotLocation = [98,98];
PlotRobotLocation(RobotLocation);

% Astar calculate heuristic for each cell - distance between cell and goal
msize=size(OccupancyMap);
h=[];%heuristic values of every cell. estimated as the straight distance between the goal and cell
for i=1:msize(1)
    for j=1:msize(2)
        h(i,j)=sqrt((i-GoalLocation(1))^2+(j-GoalLocation(2))^2);
        if OccupancyMap(i,j)==1
            h(i,j)=h(i,j)+Inf; %if it is an obstacle, set the cost really high so the node is avoided
        end
    end
end
% Initialise
open=[RobotLocation(1),RobotLocation(2),h(RobotLocation(1),RobotLocation(2)),-1,-1];
%open nodes,X(x coordinate of the cell/column),Y( y coordinate of the
%cell/row),f(sum of weight of all previous nodes and included this+heuristic), x
%(previous cell),y (previous cell)]
closed=[];%explored nodes
[m n] = min(open(:,3));
current=open(n,:);

% open min and extract
while ~isequal(current(1:2),GoalLocation)
    children= expand_node(current,h, msize(1), msize(2));
    %place the children with the lower cost in the open list if they are
    %redundant in that list
    if current(3)>10000 || h(current(1),current(2))>10000
        pause();
    end
    closed=[closed;current];%place the explored node in the closed list
    open=open([1:n-1,n+1:end],:); %deleting the current node from the open list
    if isempty(children)==0 %if the children list is NOT empty
        for i=1:length(children(:,1))
            if BinA(open(:,1:2),children(i,1:2))~=0 %if the child is already present in the open list
                if(children(i,3)<open(BinA(open(:,1:2),children(i,1:2)),3))%if the cost of the child is less than the already present one in the open list
                    open(BinA(open(:,1:2),children(i,1:2)),:)=children(i,:);%replace the element in the open list with the child that has less cost
                end
            else %if the child is not in the open list already
                if BinA(closed(:,1:2),children(i,1:2))==0%if the child is not already in the closed list (previously explored node)
                    open=[open ; children(i,:)];%then add it to the open list, to be explored later
                else %if it is already in the closed list
                    if (children(i,3)<closed(BinA(closed(:,1:2),children(i,1:2)),3))%if the childs cost is less than the one in the closed list
                        closed(BinA(closed(:,1:2),children(i,1:2)),:)=children(i,:);%replace the one in the closed list with the child (lower cost)
                    end
                end
            end
        end
    end
    [m n] = min(open(:,3));% find a node from the open list to explore that has the least cost
    current=open(n,:);
    %for dramatic effect
    pause(.01);
    PlotRobotLocation([current(2),current(1)]);
end
closed=[closed ;current];
%reiterate to start to find path
node=current;
path=current(1:2);
while ~isequal(node(4:5),(RobotLocation))
    path=[node(4:5); path];
    node=closed(BinA(closed(:,1:2),node(4:5)),:);
    pause(0.01)
    PlotRobotLocation([node(5),node(4)]);
end

if(isequal(RobotLocation, GoalLocation))
    disp('Goal Reached! YAY!');
else
    disp('Goal Not Achieved... :(');
end
disp(strcat('Path Length =  ', num2str(pathLength)));

