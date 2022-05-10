%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,uu, MAV)

    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi  = x(7);
    theta= x(8);
    psi  = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    
    % specifying parameters
    mass = MAV.mass;
    Jx = MAV.Jx;
    Jy = MAV.Jy;
    Jz =MAV.Jz;
    Jxz = MAV.Jxz;
    
    G = Jx*Jz-Jxz^2;
    G1 = Jxz*(Jx-Jy+Jz)/G;
    G2 = (Jz*(Jz-Jy)+ Jxz^2)/G;
    G3 = Jz/G;
    G4 = Jxz/G;
    G5 = (Jz - Jx)/Jy;
    G6 = Jxz/Jy;
    G7 = ((Jx-Jy)*Jx+Jxz^2)/G;
    G8 = Jx/G;
    
    pndot = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)]*[u, v, w]';
    pedot = [cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)]*[u, v, w]';
    pddot = [-sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)]*[u, v, w]';
    udot = r*v - q*w +1/mass*fx;
    vdot = p*w-r*u + 1/mass*fy;
    wdot = q*u-p*v + 1/mass*fz;
    phidot = [1 sin(phi)*tan(theta) cos(phi)*tan(theta)]*[p, q, r]';
    thetadot = [0 cos(phi) -sin(phi)]*[p, q, r]';
    psidot = [0 sin(phi)/cos(theta) cos(phi)/cos(theta)]*[p, q, r]';
    pdot = G1*p*q - G2*q*r + G3*ell+G4*n; 
    qdot = G5*p*r - G6*(p^2-r^2) + 1/Jy*m;
    rdot = G7*p*q - G1*q*r + G4*ell + G8*n;

        

sys = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];
%end
% end mdlDerivatives
