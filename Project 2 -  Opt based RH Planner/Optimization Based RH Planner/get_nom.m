function [x,y] = get_nom()

%initial values
a = [0 0];
b = [10 10];

%x = linspace(a(1),b(1),4);
x = [0 1.5 2 2.5 3 3.5 6 6.5 7 7.5 10];

%y = linspace(a(2),b(2),4);
y = [0 0.5 2 5 1 7 7 7 2 1 10];

t = 0:0.1:10;

c1 = [BernsteinPoly(x, t); BernsteinPoly(y, t)];
 
 %plot(c1(1, :), c1(2, :));
end

%m1 = plot(c1(1, 1), c1(2, 1), 'o', 'MarkerSize', 10);


