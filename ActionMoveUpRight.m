function [StatusUR, NewNodeUR] = ActionMoveUpRight(CurrentNode)
x = CurrentNode(1);
y = CurrentNode(2);
if y == 150 || x == 250
    StatusUR = false;
    NewNodeUR = [];
else
    newX = x+1;
    newY = y+1;
    StatusUR = true;
    NewNodeUR = [newX, newY];
end
