function [StatusR, NewNodeR] = ActionMoveRight(CurrentNode)
x = CurrentNode(1);
y = CurrentNode(2);
if x == 250
    StatusR = false;
    NewNodeR = [];
else
    newX = x+1;
    newY = y;
    StatusR = true;
    NewNodeR = [newX, newY];
end
