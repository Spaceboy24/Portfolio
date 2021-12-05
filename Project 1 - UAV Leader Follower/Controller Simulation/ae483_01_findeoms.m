function [h, x, u, h_num, params] = ae483_01_findeoms()

% Define all 12 states as symbolic variables. The "real" tag at the end
% tells MATLAB to assume that all of these variables are real-valued (as
% opposed to compled-valued, for example).
syms o1 o2 o3 hy hp hr v1 v2 v3 w1 w2 w3 real 

% For convenience, define the position (a point), the linear velocity (a
% vector), and the angular velocity (a vector) in coordinates, in terms of
% the state variables. Note that we do NOT group the yaw, pitch, and roll
% angles into a 3x1 matrix, to emphasize that these angles are NOT the
% coordinates of any point or vector.
o = [o1; o2; o3];
v = [v1; v2; v3];
w = [w1; w2; w3];

% For convenience, define the set of all states as a 12x1 matrix "x". The
% order in which we choose to list the states is up to us - we just need to
% be consistent everywhere else.
x = [o; hy; hp; hr; v; w];

% Define all 4 inputs as real-valued symbolic variables.
syms u1 u2 u3 u4 real

% For convenience, define the set of all inputs as a 4x1 matrix "u".
u = [u1; u2; u3; u4];

% Define all parameter values. (I made the numbers up. You all will have
% your own estimates of these numbers that you found from data.)
m = 0.3000;%0.7092;                        % mass
g = 9.81;                       % acceleration of gravity
J = 0.004 * diag([1, 1, 2]);    % moment of inertia matrix (diagonal, because we
                                %   assume that the axes of the body frame are
                                %   aligned with the principal axes)
% From sdk.h:
%
%  * motor[0]: front
%  * motor[1]: rear
%  * motor[2]: left
%  * motor[3]: right

syms kF kM l real

P = [0,             0,          kF * l,     -kF * l;
     kF * l,        -kF * l,    0,          0;
     -kM,           -kM,        kM,         kM;
     kF,            kF,         kF,         kF];

% It is helpful to package all parameter values into a struct that can be
% stored for use later in control design.
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

% Find equations of motion in symbolic form. This is a 12x1 matrix h, each
% entry of which is a function of symbolic states and inputs. In our notes
% from class, we would write:
%
%   xdot = h(x, u)
%
h = [v;
     GetN_ZYX(hy, hp, hr) * w;
     (1 / m) * f;
     inv(J) * (tau - wedge(w) * J * w) ];

% Find equations of motion in numeric form. This is a MATLAB function h_num
% that can called like any other function as h_num(x, u), with real-valued
% arguments x (a 12x1 matrix of numbers) and u (a 4x1 matrix of numbers).
h_num = matlabFunction(h, 'vars', {x, u});

end


% Returns the rotation matrix R that corresponds to yaw, pitch, and roll
% angles of hy, hp, and hr (assuming a ZYX Euler Angle sequence).
function R = GetR_ZYX(hy, hp, hr)
R = Rz(hy) * Ry(hp) * Rx(hr);
end

% Given yaw, pitch, and roll angles of hy, hp, and hr (assuming a ZYX Euler
% Angle sequence), returns the matrix N for which
%
%   [hydot; hpdot; hrdot] = N * w
%
% where w is the angular velocity and where hydot, hpdot, and hrdot are the
% angular rates.
function N = GetN_ZYX(hy, hp, hr)
R_1inB = Rx(hr);
R_BinA = Ry(hp);
N = inv([(R_BinA * R_1inB)'*[0; 0; 1] (R_1inB)'*[0; 1; 0] [1; 0; 0]]);
end

% Returns the matrix A for which the matrix multiplication A * b implements
% the cross product of two vectors a and b (both written in coordinates).
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