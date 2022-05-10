function [V,F,colors] = defineaircraftBody()
    % Define the vertices (physical location of vertices
%     V = [...
%         1    1    0;... % point 1
%         1   -1    0;... % point 2
%         -1   -1    0;... % point 3
%         -1    1    0;... % point 4
%         1    1   -2;... % point 5
%         1   -1   -2;... % point 6
%         -1   -1   -2;... % point 7
%         -1    1   -2;... % point 8
%         1.5  1.5  0;... % point 9
%         1.5 -1.5  0;... % point 10
%         -1.5 -1.5  0;... % point 11
%         -1.5  1.5  0;... % point 12
%     ];
% 
%     % define faces as a list of vertices numbered above
%     F = [...
%         1, 2,  6,  5;...  % front
%         4, 3,  7,  8;...  % back
%         1, 5,  8,  4;...  % right 
%         2, 6,  7,  3;...  % left
%         5, 6,  7,  8;...  % top
%         9, 10, 11, 12;... % bottom
%         ];
    V = [...
        1.5 0 0;...   % pt 1
        1 .25 -0.25;... % pt 2
        1 -.25 -0.25;...   % pt 3
        1 -.25 0.25;...  % pt 4
        1 0.25 0.25;...  % pt 5
        -4 0 0;...   % pt 6
        0 -3 0;...      % pt 7
        -1.5 -3 0;...   % pt 8
        -1.5 3 0;...    % pt 9
        0 3 0;...       % pt 10     
        -3 1.5 0;...   % pt 11
        -4 1.5 0;... % pt 12
        -4 -1.5 0;...   % pt 13
        -3 -1.5 0;...  % pt 14
        -3 0 0;...  % pt 15
        -4 0 -1.5;...   % pt 16
        ]';

      V=diag([50,50,50])*V;

    % define faces as a list of vertices numbered above
      F = [...
            7, 8, 9, 10;...  % wing
            11, 12, 13, 14;...  % tail
            16, 15, 6, NaN;...  % tail 2
            1, 2, 3, NaN;...  % front 1
            1, 3, 4, NaN;...  % front 2
            1, 4, 5, NaN;...  % front 3
            1, 5, 2, NaN;...  % front 4
            2, 3, 6, NaN;...  % mid 1
            3, 4, 6, NaN;...  % mid 2
            6, 4, 5, NaN;...  % mid 3
            6, 2, 5, NaN;...  % mid 4
            ];
    % define colors for each face    
    myred = [1, 0, 0];
    mygreen = [0, 1, 0];
    myblue = [0, 0, 1];
    myyellow = [1, 1, 0];
    mycyan = [0, 1, 1];

%     colors = [...
%         myred;...    % front
%         mygreen;...  % back
%         myblue;...   % right
%         myyellow;... % left
%         mycyan;...   % top
%         mycyan;...   % bottom
%         ];
    colors = [...
      mygreen;...  % wing
      mygreen;...  % tail
      mycyan;...  % tail 2
      myyellow;...  % front 1
      myyellow;...  % front 2
      myyellow;...  % front 3
      myyellow;...  % front 4
      myblue;...  % mid 1
      myblue;...  % mid 2
      myred;...  % mid 3
      myblue;...  % mid 4
    ];
end