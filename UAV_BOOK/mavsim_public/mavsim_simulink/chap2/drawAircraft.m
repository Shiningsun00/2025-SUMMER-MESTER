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
    persistent gif_frame_idx
    persistent gif_filename
    persistent h_fig1
    
    % if isempty(gif_frame_idx)
    %     gif_frame_idx = 1;
    %     gif_filename = 'aircraft_motion.gif'; % 저장할 파일명
    % end
    % first time function is called, initialize plot and persistent vars
    if t==0
        h_fig1=figure(1);
        clf
        [Vertices, Faces, facecolors] = defineAircraftBody;
        Aircraft_handle = drawAircraftBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Aircraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis equal
        hold on
        grid on
        gif_frame_idx = 1;
        gif_filename = 'aircraft_motion.gif'; % 저장할 파일명


    % at every other time step, redraw base and rod
    else
        set(0, 'CurrentFigure', h_fig1);
        drawAircraftBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           Aircraft_handle);
        
    end
    set(0, 'CurrentFigure', h_fig1);
    frame = getframe(h_fig1);         % 현재 figure 캡처
    im = frame2im(frame);           % image 데이터로 변환
    [imind, cm] = rgb2ind(im,256);  % gif용으로 index color 변환

    if gif_frame_idx == 1
        imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
    else
        imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    end
    gif_frame_idx = gif_frame_idx + 1;
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
    range = 20; % 보여줄 범위
    axis([pn-range, pn+range, pe-range, pe+range, -pd-range, -pd+range]);
    hold on
    grid on
  else
      set(handle, 'Vertices', V, 'Faces', F);
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
        3, 0, 0;...%1
        1, 1, -1;...%2
        1, -1, -1;...%3
        1, -1, 1;...%4
        1, 1, 1;...%5
        -6, 0, 0;...%6
        0, 4, 0;...%7
        -2, 4, 0;...%8
        -2, -4, 0;...%9
        0, -4, 0;...%10
        -5, 2, 0;...%11
        -6, 2, 0;...%12
        -6, -2, 0;...%13
        -5, -2, 0;...%14
        -5, 0, 0;...%15
        -6, 0, -2;...%16
    ];

    % define faces as a list of vertices numbered above
    F = [...
        1, 2, 5, NaN;
        1, 2, 3, NaN;
        1, 3, 4, NaN;
        1, 4, 5, NaN;
        2, 5, 6, NaN;
        2, 3, 6, NaN;
        3, 4, 6, NaN;
        4, 5, 6, NaN;
        6, 15, 16, NaN;
        7, 8, 9, 10;
        11, 12, 13, 14;
        ];

     % define colors for each face    
    myred = [1, 0, 0];
    mygreen = [0, 1, 0];
    myblue = [0, 0, 1];
    myyellow = [1, 1, 0];
    mycyan = [0, 1, 1];

    colors = [...
        myred;...    % front
        myred;...  % back
        myred;...   % right
        myred;... % left
        mygreen;...   % top
        mygreen;...
        mygreen;...
        mygreen;...
        myyellow;...
        myblue;...
        mycyan;...
        ];
end
  