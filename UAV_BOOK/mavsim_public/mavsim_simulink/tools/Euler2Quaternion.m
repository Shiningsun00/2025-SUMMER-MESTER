
function quaternion = Euler2Quaternion(phi, theta, psi)
    % converts euler angles to a quaternion
    cy = cos(psi/2);
    sy = sin(psi/2);
    cp = cos(theta/2);
    sp = sin(theta/2);
    cr = cos(phi/2);
    sr = sin(phi/2);
    e0 = cr*cp*cy + sr*sp*sy;
    e1 = sr*cp*cy - cr*sp*sy;
    e2 = cr*sp*cy + sr*cp*sy;
    e3 = cr*cp*sy - sr*sp*cy; 
    quaternion = [e0; e1; e2; e3];
end
