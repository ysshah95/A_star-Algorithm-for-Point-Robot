function [in] = insidepoly_halfplane(c,d)

% Square 

xs1 = 55;
ys1 = 112.5;

xs2 = 55;
ys2 = 67.5;

xs3 = 105;
ys3 = 67.5;

xs4 = 105;
ys4 = 112.5;

% xs = c;
% ys = d;
% 
% yss1 = ys1 + (((ys2-ys1)*(xs-xs1))/(xs2-xs1));
% % xss1 = xs1 + (((xs2-xs1)/(ys2-ys1))*(ys-ys1));
% yss2 = ys2 + (((ys3-ys2)*(xs-xs2))/(xs3-xs2));
% % xss3 = xs4 + (((xs3-xs4)/(ys3-ys4))*(ys-ys4));
% yss3 = ys3 + (((ys4-ys3)*(xs-xs3))/(xs4-xs3));
% yss4 = ys1 + (((ys4-ys1)*(xs-xs1))/(xs4-xs1));

if ( c>=55 && c<=105 ) && (d<=112.5 && d>=67.5)
% if d<=yss1 && d>=yss2 && d<=yss3 && d<=yss4
    ins = true;
else
    ins = false;
end

% polygon

x1 = 120;
y1 = 55;

x2 = 145;
y2 = 14;

x3 = 158;
y3 = 51;

x4 = 165;
y4 = 89;

x5 = 188;
y5 = 51;

x6 = 168;
y6 = 14;

x = c;
yy1 = y1 + ((y2-y1)*(x-x1))/(x2-x1);
yy2 = y1 + ((y3-y1)*(x-x1))/(x3-x1);
yy3 = y3 + ((y2-y3)*(x-x3))/(x2-x3);
yy4 = y3 + ((y4-y3)*(x-x3))/(x4-x3);
yy5 = y4 + ((y5-y4)*(x-x4))/(x5-x4);
yy6 = y5 + ((y6-y5)*(x-x5))/(x6-x5);
yy7 = y6 + ((y2-y6)*(x-x6))/(x2-x6);
yy8 = y3 + ((y5-y3)*(x-x3))/(x5-x3);
yy9 = y3 + ((y6-y3)*(x-x3))/(x6-x3);

if (d>=yy1 && d<=yy2 && d>=yy3) | (d<=yy3 && d>=yy7 && d<=yy9) |  (d<=yy4 && d>=yy8 && d<yy5) || (d>=yy9 && d>=yy6 && d<=yy8)
    inp = true;
else
    inp = false;
end

% circle 

h = 180;
k = 120;

y = ((c-h)^2 + (d-k)^2)^(0.5);

if y <= 15
    inc = true;
else
    inc = false;
end

in = ins | inp | inc;

end 

        
