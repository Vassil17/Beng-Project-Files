function path=A_Star_unexplored()
startingNode = Node(start,NaN);
endingNode = Node(ending,NaN);
openList = {};
closedList = {};

openList{1} = startingNode;



end

% Check adjacent node closest to goal
% If free, go to that node
% Repeat
% If obstacle is seen, check next best node
% Repeat