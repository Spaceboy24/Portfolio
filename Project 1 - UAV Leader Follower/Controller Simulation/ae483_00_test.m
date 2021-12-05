%% Clear workspace
clear
clc

%% Find equations of motion
[h, x, u, h_num, params] = ae483_01_findeoms();

%% Do control design
[K, Kint,C, x_e, u_e] = ae483_02_docontroldesign(h, x, u, h_num, params);

%% Do planner design
waypoints = struct;
waypoints.xdes = [  0.0,  0.0,  0.0,   0.0,  1.0,   1.0,  0.0,   0.0,  1.0,   1.0,  0.0,   0.0,  1.0,   1.0,  0.0,   0.0,  1.0,   1.0,  0.0,   0.0, -1.0,  -1.0,  0.0,   0.0, -1.0,  -1.0,  0.0,   0.0, -1.0,  -1.0,  0.0,   0.0, -1.0,  -1.0,  0.0,   0.0,  0.0];
waypoints.ydes = [  0.0,  0.0,  0.0,   0.0,  1.0,   1.0,  0.0,   0.0, -1.0,  -1.0,  0.0,   0.0,  1.0,   1.0,  0.0,   0.0, -1.0,  -1.0,  0.0,   0.0,  1.0,   1.0,  0.0,   0.0, -1.0,  -1.0,  0.0,   0.0,  1.0,   1.0,  0.0,   0.0, -1.0,  -1.0,  0.0,   0.0,  0.0];
waypoints.zdes = [  0.0,  0.0, -1.0,  -1.0, -1.5,  -1.5, -1.0,  -1.0, -1.5,  -1.5, -1.0,  -1.0, -0.5,  -0.5, -1.0,  -1.0, -0.5,  -0.5, -1.0,  -1.0, -1.5,  -1.5, -1.0,  -1.0, -1.5,  -1.5, -1.0,  -1.0, -0.5,  -0.5, -1.0,  -1.0, -0.5,  -0.5, -1.0,  -1.0,  0.0];
waypoints.tdes = [  0.0,  5.0,  8.0,  10.0, 13.0,  15.0, 18.0,  20.0, 23.0,  25.0, 28.0,  30.0, 33.0,  35.0, 38.0,  40.0, 43.0,  45.0, 48.0,  50.0, 53.0,  55.0, 58.0,  60.0, 63.0,  65.0, 68.0,  70.0, 73.0,  75.0, 78.0,  80.0, 83.0,  85.0, 88.0,  90.0, 93.0];
waypoints.ndes = length(waypoints.xdes);
% waypoints.tdes(2:end) = waypoints.tdes(2:end)+5;
%% Simulate
% - choose final time (i.e., the time at which the simulation will stop)
tStop = 30;
% - choose initial state
x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
% - run simulation
[data] = ae483_03_simulate(h_num, K,Kint, C, x_e, u_e, params, tStop, x0, waypoints);

%% Visualize
% - get and clear figure
figure(1);
clf;
% - parse data from simulation
t = data.t;
o = data.x(1:3, :);
hy = data.x(4, :);
hp = data.x(5, :);
hr = data.x(6, :);
o_desired = data.o_desired;
o_x_des = o_desired(1,:);
o_y_des = o_desired(2,:);
o_z_des = o_desired(3,:);
% - display
moviefile = [];     % <--- could give the name of a file to save a movie
                    %           of the visualization to, like 'test.mp4'
ae483_04_visualize(t, o, hy, hp, hr, o_desired, params, moviefile);


%% 
figure(1)
p1 = plot(t,o(1,:),'r','linewidth',2);
hold on
p2 = plot(t,o(2,:),'g','linewidth',2);
p3 = plot(t,o(3,:),'b','linewidth', 2);
p4 = plot(t,o_x_des,'--r');
p5 = plot(t,o_y_des,'--g');
p6 = plot(t,o_z_des,'--b');
hold off

%legend([p1, p2, p3],{'X position', 'Y position', 'Z position'})
%title('Position Control', 'fontweight', 'bold', 'fontsize', 16)
xlabel('Time [s]')
ylabel('Position [m]')
