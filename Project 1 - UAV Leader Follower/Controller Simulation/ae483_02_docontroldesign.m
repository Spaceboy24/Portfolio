function [K,Kint,C, x_e, u_e] = ae483_02_docontroldesign(h, x, u, h_num, params)

% Choose an equilibrium point
x_e = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
u_e = [0; 0; 0; params.m * params.g];

% Linearize about equilibrium point
A = double( subs( jacobian(h, x), [x; u], [x_e; u_e] ) );

B = double( subs( jacobian(h, u), [x; u], [x_e; u_e] ) );
C = [1 0 0 0 0 0 0 0 0 0 0 0; 0 1 0 0 0 0 0 0 0 0 0 0; 0 0 1 0 0 0 0 0 0 0 0 0]%; 0 0 0 1 0 0 0 0 0 0 0 0];
D = 0;

% LQR for implementing linear state feedback
Q = diag([75, 75, 1150, 5, 35, 35, 15,15, 600, 8, 8, 1, 0.50, 0.50, 15]);
R = 150*diag([1, 1, 1, 1]);
sys = ss(A,B,C,D);
temp = lqi(sys,Q,R);
K = temp(:,1:12);
Kint = temp(:,13:15);


%K = lqr(A, B, Q, R);
end

