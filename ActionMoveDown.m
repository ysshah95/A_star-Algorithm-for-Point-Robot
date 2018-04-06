function [StatusD, NewNodeD] = ActionMoveDown(CurrentNode)
x = CurrentNode(1);
y = CurrentNode(2);
if y == 0
    StatusD = false;
    NewNodeD = [];
else
    newX = x;
    newY = y-1;
    StatusD = true;
    NewNodeD = [newX, newY];
        
end
