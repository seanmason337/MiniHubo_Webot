%% FindAstarPath_sam337
%Input: qstart ([1x2] vector of the starting coordinate)
%       qgoal ([1x2] vector of the goal coordinate)
%       map (A [nxm] matrix representing a map, where 1's indicate obstacles and
%       0's indicate free space)
%Output: path (a [nx2] matrix that consists of the points traveled from the
%       start to the goal using th Astar search algorithm)
%
%Method: The Astar search algorithm finds the path to the goal

function path = findAstarPath_sam337(qstart, qgoal, map)

%% Plots my initial map
axis([0,size(map,1)+1,0,size(map,2)+1]); %set the size of the plot to completely show the map
hold on;                                 %sets all plotting to be on the same plot

plot(qstart(1),qstart(2),'r*');  %plot the start position
plot(qgoal(1),qgoal(2),'rd');    %plot the goal position
for i = 1:size(map,1)            %draws the freespace and obstacles
    for j = 1:size(map,2)
        if map(i,j) ==1
            plot(i,j,'x')
        else
            plot(i,j,'yo')
        end
    end
end
%% Define my heuristic function, open, and closed list
% The open and closed list consist of [xCurrent, yCurrent, xParent, yParent, steps taken along current path (g), heurisitc function (h), g+h (f)]
h = norm(qstart - qgoal);       %defines my heuristic function to be the eucledian distance from the position to the goal
open = [qstart,1,-1,0,h,h];     %contains all of my elements that have been looked at but not completely expanded
closed = [];                    %contains all of my expanded elements

%% Runs the A* algorithm
while ~isempty(open)
    
    [currentN, open] = popOffStack(open);    %pop my currentNode to look at off the open list
    plot(currentN(1),currentN(2),'rs');      %mark this node as completely expanded on the plot
    closed = addToStack(currentN, closed);   %add the node to the closed list
    
    if currentN(6) == 0                 %if my heuristic = 0 then I am at the goal
        display('goal');
        break;
    end
    
    for i = -1:1         %Check all of the 8 surrounding points
        for k = -1:1
            
            %variables updated
            x = currentN(1) + i;
            y = currentN(2) + k;
            g = currentN(5)+ norm(currentN(1:2)-[x,y]); %distance to new point is path to old point + the distance between the two
            h = norm([x,y]-qgoal);
            f = g+h;
            
            [isInList, index] = inList([x,y], open);  %tells me if my current node is in the open list and the index if so
            if x>0 && x<=size(map,1) && y>0 && y<=size(map,2) %sets my map boundries so I do not look outside my map
                if map(x,y) == 1   %obstacle found
                    %display('obstacle');
                    plot(x,y,'ks'); %plot the obstacle as being found
                elseif inList([x,y], closed) %checks to see if the node is already closed
                    %display('Already closed')
                elseif isInList
                    if g < open(index,5) %checks to see if the path to this point is better than the current one
                        open(index,:) = [x,y,currentN(1),currentN(2),g,h,f];
                        %display('better path');
                    else
                        %display('already in list');
                    end
                else
                    open = addToStack([x,y,currentN(1),currentN(2),g,h,f],open); %add current point and its associated details to the open list
                    plot(x,y,'gs'); %plot the node as being looked at
                end
            end
        end
    end
    
    open = sortrows(open, 7);%sort the open lists by the lowest f value
    %display(open);
    
end

if ~isempty(open)
    
    %% Calculate the path taken by looking at the end point in the closed list,
    %% its parent, and repeating the process.
    position = qgoal;
    path = [];
    while 1
        [bool, index] = inList(position, closed);
        
        path = [closed(index,1:2);path];
        position = closed(index, 3:4);
        if path(1,:) == qstart
            break
        end
    end
    plot(path(:,1),path(:,2),'k-');
    
    hold off;
    display(path);
    figure(1);
    
else
    path = [];
    display('No path found');
end


% inList
%take in the element [1x2] you are looking for, and the list you are looking through
%It outputs a 1 if the element exists in the list and a 0 if not. In
%addition it ouputs the index of the element if found.
function [bool, index] = inList(element, list)
bool = 0;
if isempty(list)
    bool = 0;
    index = 1;
else
    for i = 1:size(list, 1)
        if element == list(i,1:2)
            bool = 1;
            index =i;
            break;
        else
            bool = 0;
            index = 1;
        end
        
    end
end

% AddToStack
% adds newElement (1xM row vector) to the originalStack (N x M array)
function outStack = addToStack(newElement, originalStack)
outStack = [newElement; originalStack];

% popOffStack
% helper function to remove top element of a stack
function [outElement, newStack] = popOffStack(originalStack)
[n,m] = size(originalStack);
outElement = originalStack(1,:);
newStack = originalStack(2:n,:);






