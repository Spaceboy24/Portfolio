function [c,ceq] = nonlcon(x,CONSTANTS)
%NONLCON Summary of this function goes here
%   Detailed explanation goes here
N = CONSTANTS.N; 

x1 = x(1:N+1);
y1 = x(N+2:2*N+2);
x2 = x(2*N+3:3*N+3);
y2 = x(3*N+4:4*N+4);
x3 = x(4*N+5:5*N+5);
y3 = x(5*N+6:6*N+6); 
T = x(end);

[~,~,Diff] = BeBOT(N,T);
 
x1dot = x1'*Diff;
y1dot = y1'*Diff;
x2dot = x2'*Diff;
y2dot = y2'*Diff;
x3dot = x3'*Diff;
y3dot = y3'*Diff;

x1ddot = x1dot*Diff;
y1ddot = y1dot*Diff;
x2ddot = x2dot*Diff;
y2ddot = y2dot*Diff;
x3ddot = x3dot*Diff;
y3ddot = y3dot*Diff;

speed1 = BernsteinProduct(x1dot, x1dot) + BernsteinProduct(y1dot, y1dot);
speed2 = BernsteinProduct(x2dot, x2dot) + BernsteinProduct(y2dot, y2dot);
speed3 = BernsteinProduct(x3dot, x3dot) + BernsteinProduct(y3dot, y3dot);
speed = [speed1, speed2, speed3]; %add speed3

angrate1 = (BernsteinProduct(x1ddot,y1dot)-BernsteinProduct(x1dot,y1ddot))./(speed1);
angrate2 = (BernsteinProduct(x2ddot,y2dot)-BernsteinProduct(x2dot,y2ddot))./(speed2);
angrate3 = (BernsteinProduct(x3ddot,y3dot)-BernsteinProduct(x3dot,y3ddot))./(speed3);
angrate = [angrate1, angrate2, angrate3]; %add angrate3

dist12 = BernsteinProduct(x1-x2, x1-x2) + BernsteinProduct(y1-y2, y1-y2);%add 3rd vehicle
dist13 = BernsteinProduct(x1-x3, x1-x3) + BernsteinProduct(y1-y3, y1-y3);
dist23 = BernsteinProduct(x2-x3, x2-x3) + BernsteinProduct(y2-y3, y2-y3);
distobs = BernsteinProduct(x1-CONSTANTS.pobs(1),x1-CONSTANTS.pobs(1)) + BernsteinProduct(y1-CONSTANTS.pobs(2),y1-CONSTANTS.pobs(2)); %add 3rd vehicle

%c = [speed' - CONSTANTS.vmax^2; -speed' + CONSTANTS.vmin^2; angrate' - CONSTANTS.omegamax; -angrate' - CONSTANTS.omegamax; -dist' + CONSTANTS.mindist^2];
c = [
    speed' - CONSTANTS.vmax^2;
    -speed' + CONSTANTS.vmin^2;
    angrate' - CONSTANTS.omegamax^2;
    -angrate' - CONSTANTS.omegamax^2;
    -dist12' + CONSTANTS.mindist^2;
    -dist13' + CONSTANTS.mindist^2;
    -dist23' + CONSTANTS.mindist^2;
    -distobs' + CONSTANTS.mindist^2];
ceq = [x1(1) - CONSTANTS.pinit(1,1); y1(1) - CONSTANTS.pinit(1,2); x2(1) - CONSTANTS.pinit(2,1); y2(1) - CONSTANTS.pinit(2,2);x3(1) - CONSTANTS.pinit(3,1); y1(3) - CONSTANTS.pinit(3,2); x1(end) - CONSTANTS.pfin(1,1); y1(end) - CONSTANTS.pfin(1,2); x2(end) - CONSTANTS.pfin(2,1); y2(end) - CONSTANTS.pfin(2,2); x3(end) - CONSTANTS.pfin(3,1); y3(end) - CONSTANTS.pfin(3,2);];
end

