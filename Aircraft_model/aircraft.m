function aircraft(uu)

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

%     % process inputs to function
%     pn       = 1;       % inertial North position     
%     pe       = 1;       % inertial East position
%     pd       = 1;           
%     u        = 0;       
%     v        = 0;       
%     w        = 0;       
%     phi      = 0;       % roll angle         
%     theta    = 0;       % pitch angle     
%     psi      = 0;       % yaw angle     
%     p        = 0;       % roll rate
%     q        = 0;       % pitch rate     
%     r        = 0;       % yaw rate    
%     t        = 0;       % time

    % define persistent variables 
    persistent aircraft_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1), clf
        [Vertices, Faces, facecolors] = defineaircraftBody;
        aircraft_handle = drawaircraftBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Aircraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis([-1000,1000,-1000,1000,-500,1000]);
        grid on
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawaircraftBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           aircraft_handle);
    end
end

  

