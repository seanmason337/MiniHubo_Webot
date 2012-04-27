%% FindAstarPath_sam337
%Input: qstart ([1x2] vector of the starting coordinate)
%       qgoal ([1x2] vector of the goal coordinate)
%       Q (A [nxm] matrix representing a Q, where 1's indicate obstacles and
%       0's indicate free space)
%Output: path (a [nx2] matrix that consists of the points traveled from the
%       start to the goal using th Astar search algorithm)
%
%Method: The Astar search algorithm finds the path to the goal

function path = findAstarPath_Q(state,Q)

Q = [Q(1:end-1,1:end-5),repmat(Q(1:end-1,end-5),1,5)];  %Take out junk data
Q = Q/max(max(Q));                          %Normalize matrix
[stateActions totalSteps] = size(Q);
Q = [ones(1,totalSteps);Q;ones(1,totalSteps)];
[stateActions totalSteps] = size(Q);
states = (stateActions)/3;
Q = Q';

qstart = [1,state];
qgoal = [totalSteps,state];

%% Plots my initial Q
axis([0,totalSteps,0,states]); %set the size of the plot to completely show the Q
p = 0;
if p==1
    hold on;                                 %sets all plotting to be on the same plot
    plot(qstart(1),qstart(2),'r*');  %plot the start position
    plot(qgoal(1),qgoal(2),'rd');    %plot the goal position
end

%% Define my heuristic function, open, and closed list
% The open and closed list consist of [xCurrent, yCurrent, xParent, yParent, steps taken along current path (g), heurisitc function (h), g+h (f)]
hgain = 10^-2;
h = hgain*norm(qstart - qgoal);       %defines my heuristic function to be the eucledian distance from the position to the goal
open = [qstart,1,-1,0,h,h];     %contains all of my elements that have been looked at but not completely expanded
closed = [];                    %contains all of my expanded elements


%% Runs the A* algorithm
while ~isempty(open)
    
    [currentN, open] = popOffStack(open);    %pop my currentNode to look at off the open list
    if p == 1
        plot(currentN(1),currentN(2),'rs');      %mark this node as completely expanded on the plot
    end
    closed = addToStack(currentN, closed);   %add the node to the closed list
    
    if currentN(6) == 0                 %if my heuristic = 0 then I am at the goal
        display('goal');
        break;
    end
    
    for k = -1:1         %Check connecting points to the right
        i = 1;

        %variables updated
        x = currentN(1) + i;
        y = currentN(2) + k;

        [isInList, index] = inList([x,y], open);  %tells me if my current node is in the open list and the index if so
        if x>0 && x<=totalSteps && y>0 && y<=states %sets my Q boundries so I do not look outside my Q
            g = currentN(5)+ Q(currentN(1),currentN(2)*3-1+k); %distance to new point is path to old point + the distance between the two
            h = hgain*norm([x,y]-qgoal)/norm(qstart - qgoal);
            f = g+h;
            if inList([x,y], closed) %checks to see if the node is already closed
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
                if p == 1
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
    
    %hold off;
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






