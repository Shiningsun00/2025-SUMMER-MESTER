function drawAircraft(uu)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);           
    u        = uu(4);       
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);       % roll rate
    q        = uu(11);       % pitch rate     
    r        = uu(12);       % yaw rate    
    t        = uu(13);       % time

    % define persistent variables 
    persistent Aircraft_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1), clf
        [Vertices, Faces, facecolors] = defineAircraftBody;
        Aircraft_handle = drawAircraftBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Aircraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis([-50, 50, -30, 200, -50, 50]);
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawAircraftBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           Aircraft_handle);
    end
end

  
%=======================================================================
% drawSpacecraft
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawAircraftBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  V = rotate(V', phi, theta, psi)';  % rotate spacecraft
  V = translate(V', pn, pe, pd)';  % translate spacecraft
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = V*R;
  
  if isempty(handle)
  handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat');
  axis equal
  else
    set(handle,'Vertices',V,'Faces',F);
  end
  drawnow
end

%%%%%%%%%%%%%%%%%%%%%%%
function XYZ=rotate(XYZ,phi,theta,psi)
  % define rotation matrix
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;
  % rotate vertices
  XYZ = R*XYZ;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)
  XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define spacecraft vertices and faces
function [V,F,colors] = defineAircraftBody()
    % Define the vertices (physical location of vertices
    V = [...
        3.5   0    0.7;... % point 1
        1.7,   1    -1.5;... % point 2
        1.7   -1    -1.5;... % point 3
        1.7   -1    1.5;... % point 4
        1.7    1   1.5;... % point 5
        -8   0   0;... % point 6
        0   5   0;... % point 7
        -4    5   0;... % point 8
        -4    -5  0;... % point 9
        0     -5  0;... % point 10
        -6    3  0;... % point 11
        -8     3  0;... % point 12
        -8    -3  0;... % point 13
        -6    -3  0;... % point 14
        -6     0  0;... % point 15
        -8     0  0;... % point 16
    ];
    % define faces as a list of vertices numbered above
    F = [...
        1, 2, 3, NaN;...
        1, 3, 4, NaN;...
        1, 4, 5 ,NaN;...  
        1, 2, 5 ,NaN;... 
        2, 3, 6 ,NaN;...
        3, 4, 6 ,NaN;...
        4, 5, 6 ,NaN;...
        5, 2, 6 ,NaN;...
        7, 8, 9 ,10;...
        11, 12, 13, 14;...
        6, 15, 16 ,NaN;...
        ];
     % define colors for each face    
    myred = [1, 0, 0];
    mygreen = [0, 1, 0];
    myblue = [0, 0, 1];
    myyellow = [1, 1, 0];
    mycyan = [0, 1, 1];

    colors = [...
        myred;...
        myred;...
        myred;...
        myred;...
        myyellow;... % front
        myyellow;... % front
        myyellow;... % front
        myyellow;... % front
        mygreen;...
        myblue;...        mygreen;...
        mycyan;...
        ];
end
  