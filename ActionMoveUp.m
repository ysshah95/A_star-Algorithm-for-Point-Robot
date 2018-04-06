function [StatusU, NewNodeU] = ActionMoveUp(CurrentNode)
x = CurrentNode(1);
y = CurrentNode(2);
if y == 150
    StatusU = false;
    NewNodeU = [];
else
    newX = x;
    newY = y+1;
    StatusU = true;
    NewNodeU = [newX, newY];

end
