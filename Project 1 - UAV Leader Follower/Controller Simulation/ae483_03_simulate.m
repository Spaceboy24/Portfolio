function [data] = ae483_03_simulate(h_num, K,Kint,C, x_e, u_e, params, tStop, x0, waypoints)
%
% sampleNumber
%   
%   An integer number of time steps to run the simulation for (i.e.,
%   samples to take). The total simulation time is then
%
%       num_samples / sample_rate
%
% x0
%
%   The initial state (i.e., the state at time zero, at the very start
%   of the simulation). To compare your results with an experiment, it
%   would be very important to match the initial state here with the one
%   you used in your experiment (so you should get it from your data).

% Choose a sample rate. This is the number of times per second that the
% controller will run. This is also the number of times per second that
% data will be collected (at least, in the simulation I've created here -
% as you know, things are different in the hardware experiments, where the
% onboard controller and the offboard data logger run at different rates).
sampleRate = 50;

% Compute the time step. This is the length of time between one sample and
% the next.
tStep = 1 / sampleRate;

% Compute sample number. This is the number of time steps (samples) to
% simulate.
sampleNumber = ceil(tStop / tStep);

% Define variables to keep track of the time, state, and input during the
% simulation. I like to put these variables all together in a struct - that
% makes it easy to store and pass the data around, and also to add more
% things to the data later on if I want (rotor spin rates, for example).
%
% I initialize the time and state with their initial values. I initialize
% the input as an empty matrix - no inputs have been chosen yet.
data = struct('t', 0, 'x', x0, 'u', [], 'mu', [], 'o_desired', x0(1:3));

xint = [0; 0; 0]; %Initialize pseudo states

% Loop through all time steps.
for i = 1:sampleNumber
    
    % The "current time" is at time step "i". So, the current time and
    % state are in the i'th column of data.t and data.x.
    t = data.t(:, i);
    x = data.x(:, i);
    
    %%%%%%%%%%%%%%%%%
    % PLANNER (choose desired position)
    
    if (t <= waypoints.tdes(1))
        o_desired = [waypoints.xdes(1);
                     waypoints.ydes(1);
                     waypoints.zdes(1)];
    elseif (t >= waypoints.tdes(waypoints.ndes))
        o_desired = [waypoints.xdes(waypoints.ndes);
                     waypoints.ydes(waypoints.ndes);
                     waypoints.zdes(waypoints.ndes)];
    else
        for j=2:waypoints.ndes
            if (t <= waypoints.tdes(j))
                ratio = (t - waypoints.tdes(j-1)) / (waypoints.tdes(j) - waypoints.tdes(j-1));
                ojminus = [waypoints.xdes(j-1);
                           waypoints.ydes(j-1);
                           waypoints.zdes(j-1)];
                oj = [waypoints.xdes(j);
                      waypoints.ydes(j);
                      waypoints.zdes(j)];
                o_desired = (1 - ratio) * ojminus + ratio * oj;
                break;
            end
        end
    end
    
    % Store desired position in data structure.
    data.o_desired(:, i + 1) = o_desired;
    
    %
    %%%%%%%%%%%%%%%%%
    
    
    %%%%%%%%%%%%%%%%%
    % CONTROLLER (choose input)
    
    % 
    % - Track desired position
    x_e(1:3, :) = o_desired; %[waypoints.xdes(j);
                             % waypoints.ydes(j);
                             % waypoints.zdes(j); 0]
                             
    % - Compute state of linear system.
    x_c = x - x_e;
    
    % Compute integral of pseudo states
%     flag = 0;
%     if(i>=2)
%         flag = 1;
%     end
%     
%     if(flag==1)
%         v = v + (tStep/2)*((C*data.x(:, i-1) - data.o_desired(:, i)) + (C*data.x(:, i) - data.o_desired(:, i + 1)));
%     end
    
    xint = xint + ((C*x) - x_e(1:3,:))*(tStep/2);

    % - Compute input of linear system (by state feedback).
    u_c = -K * x_c + Kint * xint;
    % - Compute input of original, nonlinear system.
    u = u_c + u_e;

    
    
    
    % - Compute squared spin rates
    s = params.M * u;
    % - Bound squared spin rates
    for j = 1:length(s)
        if s(j) < params.s_min
            s(j) = params.s_min;
        elseif s(j) > params.s_max
            s(j) = params.s_max;
        end
    end
    % - Compute input of original, nonlinear system (again).
    u = params.M \ s; 
    % - Compute motor commands
    mu = (sqrt(s) - params.beta) / params.alpha;
    
    % Store input and motor commands in data structure.
    data.u(:, i) = u;
    data.mu(:, i) = mu;
    
    %
    %%%%%%%%%%%%%%%%%
    
    
    % Numerically integrate equations of motion to compute what the next
    % state will be given the current state and input.
    %
    %   @(t, x) h_num(x, u)     This is a anonymous function that ode45
    %                           will call to find xdot as a function of t
    %                           and x. The syntax means that when ode45
    %                           calls this anonymous function with (t, x),
    %                           this function will return the value
    %                           h_num(x, u). Note that the "x" comes from
    %                           ode45 and is changing all the time, while
    %                           the "u" is what we computed above, from the
    %                           controller, and is constant over each time
    %                           step. For help on anonymous functions, do:
    %
    %                               doc anonymous
    %
    %   [t, t + tStep]          This is the time interval to simulate. The
    %                           state (i.e., current) time is t, the final
    %                           (i.e., next) time is t + tStep.
    %
    %   x                       This is the state at the state of the time
    %                           interval that we want to simulate. In other
    %                           words, it is the current state.
    %
    [t_sol, x_sol] = ode45(@(t, x) h_num(x, u), [t, t + tStep], x);
    
    % Parse the solution that is returned by ode45 and store the next time
    % and state in our data structure. These will become the *current* time
    % and state the next time through our loop.
    %
    % Here is what ode45 returns:
    %
    %   t_sol       m x 1 matrix of times
    %   x_sol       m x n matrix of states, where the state time t_sol(i)
    %               is given by the 1 x n matrix x_sol(i, :)
    %
    % So, the "next time" is the last element of t_sol, and the "next
    % state" is the last row of x_sol. I always prefer to represent the
    % state as an n x 1 matrix instead of as a 1 x n matrix, so I take the
    % transpose of the result before storing it in data.
    data.t(:, i + 1) = t_sol(end, :);
    data.x(:, i + 1) = x_sol(end, :)';
end

end

function obst = AddObstacle_Sphere(obst, p, s)
    obst{end+1} = struct('type', 1, 'p', p, 's', s);
end

function obst = ...
    AddObstacle_RandomSpheres(obst, ...
                              n, smin, smax, dx, dy, dz, ...
                              o_start, o_goal, params)

	% n = number of spheres to add
    % smin = minimum radius
    % smax = maximum radius
    % o_start = start position
    % o_goal = goal position
    % (-dx, dx), (-dy, dy), (-dz, 0) = room dimensions

    % Loop to add n spheres one at a time
    for i=1:n

        % Keep trying until we add a sphere that does not interfere
        % with the start or goal position of the quadrotor.
        while(1)
            % Random center position in the room
            p = [dx;dy;dz].*([-1;-1;-1]+[2;2;1].*rand(3,1));
            % Random radius
            s = smin+(smax-smin)*rand;
            % Check for non-interference
            ds = norm(p-o_start)-(params.r+s);
            dg = norm(p-o_goal)-(params.r+s);
            if ((ds>0)&&(dg>0.5))
                obst = AddObstacle_Sphere(obst, p, s);
                break;
            end
        end

    end
end