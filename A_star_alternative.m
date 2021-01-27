function path = Astar(maze,start,ending)
flag = 0;
startingNode = Node(start,NaN);
endingNode = Node(ending,NaN);

openList = {};
closedList = {};

openList{1} = startingNode;
q=startingNode;
qIndex = 1;


while size(openList,2) > 0
    % go through each open entry
    for index=1:size(openList,2)
        % get lowest f value
       f(index)=openList{index}.f;     
    end
    % find index of node with lowest f
    index = find(f==min(f));
    % if more than 1 node with lowest f exists, pick the first one
    index = min(index);
    q = openList{index};
    qIndex = index;
    % remove q from open list
    openList(qIndex) = [];
    % and add it to closed list;
    closedList{end+1} = q;
    % Generate children to q:
    children={};
    adjacent = [0 1;1 1;1 0;1 -1;0 -1;-1 -1; -1 0; -1 1];
    
    for i=1:8
       children{i}.position = q.position + adjacent(i,:);
       children{i}.parent = q;
    end
    % For each child:
    for i=1:8
        % if any child node is out of bounds
        if any((children{i}.position) < 1) || any((children{i}.position) > 10)
            continue;
        end
        
        % if obstructed
        if maze(children{i}.position(1),children{i}.position(2)) == 1
           continue; 
        end
        
        
        
        if children{i}.position == endingNode.position
            path={};          
            currentNode = children{i}.parent;
            path{1} = currentNode.position;
            current = 2;
            while ~isequal(path{end},startingNode.position)
                path{current} = currentNode.position;
                currentNode = currentNode.parent;
                current=current+1;
            end
            flag = 1;
            break;       
        end
       % find distance from current node to child
        distance = pdist([q.position;children{i}.position],'euclidean');
        children{i}.g = q.g + distance;
        % find distance from child to goal
        distanceToEnd = pdist([children{i}.position;endingNode.position],'euclidean');
        children{i}.h = distanceToEnd;
        children{i}.f = children{i}.g + children{i}.h;
        for k=1:size(openList,2)
        % if position of child already exists
        if  children{i}.position == openList{k}.position
            if openList{k}.f < children{i}.f
                continue;
            end
        end
        end
        for k=1:size(closedList,2)
        % if position of child already exists
        if  children{i}.position == closedList{k}.position
            if closedList{k}.f < children{i}.f
                continue;
            else
                openList{end+1} = children{i}; 
            end
        else
           openList{end+1} = children{i};      
        end
        end
    end % end children for loop
    if flag == 1; break; end
    closedList{end+1} = q;
end