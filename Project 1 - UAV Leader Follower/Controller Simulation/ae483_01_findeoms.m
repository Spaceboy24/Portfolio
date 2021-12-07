function [h, x, u, h_num, params] = ae483_01_findeoms()

% Define states and state vector
syms o1 o2 o3 hy hp hr v1 v2 v3 w1 w2 w3 real 
o = [o1; o2; o3];
v = [v1; v2; v3];
w = [w1; w2; w3];
x = [o; hy; hp; hr; v; w];

% Define Input Vector
syms u1 u2 u3 u4 real
u = [u1; u2; u3; u4];

% Define all parameter values. 
m = 0.3000;                     % mass
g = 9.81;                       % acceleration of gravity
J = 0.004 * diag([1, 1, 2]);    % moment of inertia matrix 

syms kF kM l real

P = [0,             0,          kF * l,     -kF * l;
     kF * l,        -kF * l,    0,          0;
     -kM,           -kM,        kM,         kM;
     kF,            kF,         kF,         kF];

% Define struct to package parameters
params = struct;
params.m = m;
params.g = g;
params.J = J;
params.kF = 7.49e-6;
params.kM = 1.309e-7;
params.l = 0.1725;
params.M = double(subs(inv(P), {kF, kM, l}, {params.kF, params.kM, params.l}));
params.alpha = 3.8467;
params.beta = 100.5216;
params.s_min = (params.alpha * 1 + params.beta)^2;      % minimum squared spin rate of each rotor
params.s_max = (params.alpha * 200 + params.beta)^2;    % maximum squared spin rate of each rotor

% Find applied force (from gravity and rotors) in the room frame.
f = [0; 0; m * g] + GetR_ZYX(hy, hp, hr) * [0; 0; -u4];

% Find applied torque (from rotors) in the body frame.
tau = [u1; u2; u3];

% Find equations of motion
h = [v;
     GetN_ZYX(hy, hp, hr) * w;
     (1 / m) * f;
     inv(J) * (tau - wedge(w) * J * w) ];
h_num = matlabFunction(h, 'vars', {x, u});

end


% Rotation matrix correspoding to yaw pitch roll
function R = GetR_ZYX(hy, hp, hr)
R = Rz(hy) * Ry(hp) * Rx(hr);
end

function N = GetN_ZYX(hy, hp, hr)
R_1inB = Rx(hr);
R_BinA = Ry(hp);
N = inv([(R_BinA * R_1inB)'*[0; 0; 1] (R_1inB)'*[0; 1; 0] [1; 0; 0]]);
end

% Skew symmetric matrix to cross product
function A = wedge(a)
    A = [0 -a(3) a(2); a(3) 0 -a(1); -a(2) a(1) 0];
end

% Returns the rotation matrix corresponding to a rotation by an angle h
% about the x axis.
function R = Rx(h)
c = cos(h);
s = sin(h);
R = [ 1  0  0;
      0  c -s;
      0  s  c];
end

% Returns the rotation matrix corresponding to a rotation by an angle h
% about the y axis.
function R = Ry(h)
c = cos(h);
s = sin(h);
R = [ c  0  s;
      0  1  0;
     -s  0  c];
end

% Returns the rotation matrix corresponding to a rotation by an angle h
% about the z axis.
function R = Rz(h)
c = cos(h);
s = sin(h);
R = [ c -s  0;
      s  c  0;
      0  0  1];
end
