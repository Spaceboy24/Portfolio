function x0 = init_guess(CONSTANTS)

N = CONSTANTS.N; 

x1 = linspace(CONSTANTS.pinit(1,1),CONSTANTS.pfin(1,1),N+1);
y1 = linspace(CONSTANTS.pinit(1,2),CONSTANTS.pfin(1,2),N+1);
x2 = linspace(CONSTANTS.pinit(2,1),CONSTANTS.pfin(2,1),N+1);
y2 = linspace(CONSTANTS.pinit(2,2),CONSTANTS.pfin(2,2),N+1);
x3 = linspace(CONSTANTS.pinit(3,1),CONSTANTS.pfin(3,1),N+1);
y3 = linspace(CONSTANTS.pinit(3,2),CONSTANTS.pfin(3,2),N+1);
T = 2;

x0 = [x1';y1';x2';y2';x3';y3';T];

end

