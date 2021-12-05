function ae483_04_visualize(t, o, hy, hp, hr, o_desired, params, moviefile)
%
%   t           time (1xM)
%   o           position of body frame in coordinates of room frame (3xM)
%   hy, hp, hr  ZYX Euler Angles (yaw, pitch, roll) describing orientation
%               of body frame in coordinates of room frame (each 1xM)
%   moviefile   a filename (e.g., 'movie.mp4') where a movie of the
%               simulation will be saved - if this filename is empty, i.e.,
%               if it is [], then no movie will be saved

% Set flag if we are making a movie
makemovie = ~isempty(moviefile);

% Load a description of the quadrotor (points and faces that make up
% a triangular mesh)
[pQuad_InBody, fQuad] = GetQuadModel('quadmodel.mat');

% Scale the quadrotor to match the spar length (model is scaled to l=0.5)
pQuad_InBody = pQuad_InBody*(params.l / 0.5);

% Load a description of the mocap system (one point for each camera)
pMocap_InRoom = GetMocapModel('mocapmodel.mat');

% Create a description of the room frame and the body frame (four points -
% at the origin and then at the end of the x, y, z unit vectors)
pRoomFrame_InRoom = [zeros(3,1) eye(3)];
pBodyFrame_InBody = [zeros(3,1) eye(3)];

% Create an empty figure
fig = [];

% Start making movie, if necessary.
if (makemovie)
    myV = VideoWriter(moviefile,'MPEG-4');
    myV.Quality = 100;
    myV.FrameRate = 30;
    open(myV);
    movie_nframes = 0;
    movie_tstep = 1/myV.FrameRate;
end

% Draw figures for the first time (hack - assumes t starts at 0)
i = 1;
% - Update geometry
[pQuad_InRoom, pBodyFrame_InRoom] = ...
    UpdateGeometry(t(i), o(:, i), hy(i), hp(i), hr(i), ...
                   pQuad_InBody, pBodyFrame_InBody);
% - Update figure
fig = UpdateFigure(fig, t(i), o(:, i), o_desired(:, i), params, ...
                        pMocap_InRoom, pQuad_InRoom, fQuad, ...
                        pRoomFrame_InRoom, pBodyFrame_InRoom);

% Loop over all time
% pause;              % <-- uncomment this line if you want the sim to wait
                      %     for a keypress (return/enter) before starting
tic;
while (1)
    
    % Get the current time
    if (makemovie)
        t_cur = movie_nframes * movie_tstep;
    else
        t_cur = toc;
    end
    
    % Find the last time step that has time less than the current time
    i = find(t_cur >= t, 1, 'last');
    
    % Update geometry
    [pQuad_InRoom, pBodyFrame_InRoom] = ...
        UpdateGeometry(t(i), o(:, i), hy(i), hp(i), hr(i), ...
                       pQuad_InBody, pBodyFrame_InBody);
    
    % Update figure
    fig = UpdateFigure(fig, t(i), o(:, i), o_desired(:, i), params, ...
                            pMocap_InRoom, pQuad_InRoom, fQuad, ...
                            pRoomFrame_InRoom, pBodyFrame_InRoom);
    
    % If making a movie, store the current figure as a frame
    if (makemovie)
        frame = getframe(gcf);
        writeVideo(myV,frame);
        movie_nframes = movie_nframes + 1;
    end
    
    % Check if done
    if (i == length(t))
        break;
    end
end

% Close and save the movie, if necessary.
if (makemovie)
    for i=1:myV.FrameRate
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    close(myV);
end

end

function [pQuad_InRoom, pBodyFrame_InRoom] = ...
    UpdateGeometry(t, o, hy, hp, hr, pQuad_InBody, pBodyFrame_InBody)

o_BodyInRoom = o;
R_BodyInRoom = R_ZYX(hy, hp, hr);

for i = 1:size(pQuad_InBody, 2)
    pQuad_InRoom(:, i) = o_BodyInRoom + R_BodyInRoom * pQuad_InBody(:, i);
end

for i = 1:size(pBodyFrame_InBody, 2)
    pBodyFrame_InRoom(:, i) = o_BodyInRoom + R_BodyInRoom * pBodyFrame_InBody(:, i);
end

end


function [fig] = UpdateFigure(fig, t, o, o_desired, params, pMocap_InRoom, pQuad_InRoom, fQuad, pRoomFrame_InRoom, pBodyFrame_InRoom)

if isempty(fig)
    
    % Create figure
    clf;
    set(gcf,'renderer','opengl');
    set(gcf,'color','w');
    
    % Create an axis for text (it's important this is in the back,
    % so you can rotate the view and other stuff!)
    axes('position',[0 0 1 1]);
    axis([0 1 0 1]);
    hold on;
    axis off;
    fig.text=text(0.05,0.1,sprintf('t = %6.2f\n',t),'fontsize',10,'verticalalignment','top','fontname','monaco');
    
    % Create an axis for the view from the room frame
    axes();
    axis equal;
    axis([-4 4 -4 4 -3.5 0.1]);
    axis manual;
    hold on;
    
    % Reverse the y and z axes to get the "z down" view
    set(gca, 'ydir', 'reverse');
    set(gca, 'zdir', 'reverse');
    
    % Draw lights and labels
    lighting flat
    light('Position',[0 -2 -1])
    light('Position',[0 -2 1])
    xlabel('x');
    ylabel('y');
    zlabel('z');
    
    % Draw the room frame and the body frame
    fig.roomframe = DrawFrame([], pRoomFrame_InRoom);
    fig.bodyframe = DrawFrame([], pBodyFrame_InRoom);
    
    % Draw the floor
    fig.floor = rectangle('position', [-2.5 -2.5 5 5], 'facecolor', 0.9*[1 1 1]);
    
    % Draw the mocap system (put a point at the location of each camera)
    fig.mocap = scatter3(pMocap_InRoom(1,:), ...
                         pMocap_InRoom(2,:), ...
                         pMocap_InRoom(3,:), ...
                         15, 'k', 'filled');
    
    % Draw the quadrotor
    fig.quad = DrawMesh([], pQuad_InRoom, fQuad, 'y', 0.6);
    
    % Draw trace of desired position
    fig.odes = DrawTrace([], nan(3,1), 'k', 200);
    
    % Draw trace of position
    fig.o = DrawTrace([], nan(3,1), [1, 0.6, 0], 50);
    
    % Create unit sphere
    [fig.xSphere,fig.ySphere,fig.zSphere]=sphere(16);
    [m,n]=size(fig.xSphere);
    
    % Create template array for sphere color
    c = ones(m,n,3);
    
    % Make everything look pretty
    h = findobj('Type','surface');
    set(h,'FaceLighting','gouraud',...
          'FaceColor','interp',...
          'EdgeColor',[.4 .4 .4],...
          'LineStyle','none',...
          'BackFaceLighting','lit',...
          'AmbientStrength',0.4,...
          'DiffuseStrength',0.6,...
          'SpecularStrength',0.5);
    material default
else
    
    % Update figure
    % - update time
    set(fig.text,'string',sprintf('t = %6.2f', t));
    % - update body frame
    fig.bodyframe = DrawFrame(fig.bodyframe, pBodyFrame_InRoom);
    % - update quadrotor
    fig.quad = DrawMesh(fig.quad, pQuad_InRoom);
    % - update traces
    fig.odes = DrawTrace(fig.odes, o_desired);
    fig.o = DrawTrace(fig.o, o);
end

% Tell MATLAB to update the figure window so we see what we just drew
% on the screen immediately
drawnow;

end

% Creates or updates a triangular mesh
function mesh = DrawMesh(mesh, p, f, color, alpha)
if isempty(mesh)
    mesh = patch('Vertices',p','Faces',f,...
                 'FaceColor',color,'FaceAlpha',alpha,'EdgeAlpha',alpha);
else
    set(mesh,'vertices',p');
end
end

% Creates or updates three lines that describe a frame
function frame = DrawFrame(frame, p)
if isempty(frame)
    frame.x = plot3(p(1,[1 2]),p(2,[1 2]),p(3,[1 2]),'r-','linewidth',3);
    frame.y = plot3(p(1,[1 3]),p(2,[1 3]),p(3,[1 3]),'g-','linewidth',3);
    frame.z = plot3(p(1,[1 4]),p(2,[1 4]),p(3,[1 4]),'b-','linewidth',3);
else
    set(frame.x,'xdata',p(1,[1 2]),'ydata',p(2,[1 2]),'zdata',p(3,[1 2]));
    set(frame.y,'xdata',p(1,[1 3]),'ydata',p(2,[1 3]),'zdata',p(3,[1 3]));
    set(frame.z,'xdata',p(1,[1 4]),'ydata',p(2,[1 4]),'zdata',p(3,[1 4]));
end
end

% Creates or updates position trace
function trace = DrawTrace(trace,p,c,n)
if (isempty(trace))
    trace = line(p(1), p(2), p(3), ...
                 'color', c, 'marker', '.', 'markersize', 12, ...
                 'linestyle', 'none', 'UserData', n);
else
    x = get(trace,'xdata');
    y = get(trace,'ydata');
    z = get(trace,'zdata');
    n = get(trace,'UserData');
    if (length(x)>=n)
        x = x(2:end);
        y = y(2:end);
        z = z(2:end);
    end
    x(:,end+1) = p(1);
    y(:,end+1) = p(2);
    z(:,end+1) = p(3);
    set(trace,'xdata',x,'ydata',y,'zdata',z);
end
end

% Loads quadrotor geometry from a file
function [p, f] = GetQuadModel(filename)
load(filename);
end

% Loads mocap geometry from a file
function p = GetMocapModel(filename)
load(filename);
end



function R = Rx(h)
c = cos(h);
s = sin(h);
R = [ 1  0  0;
      0  c -s;
      0  s  c];
end

function R = Ry(h)
c = cos(h);
s = sin(h);
R = [ c  0  s;
      0  1  0;
     -s  0  c];
end

function R = Rz(h)
c = cos(h);
s = sin(h);
R = [ c -s  0;
      s  c  0;
      0  0  1];
end

function R = R_ZYX(hy, hp, hr)
R = Rz(hy) * Ry(hp) * Rx(hr);
end

