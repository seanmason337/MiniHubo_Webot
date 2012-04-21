function out = hZ(theta, x,y,z)
    out = [cosd(theta)     -sind(theta)     0   x;
              sind(theta)     cosd(theta)   0   y;
              0               0             1   z;
              0               0             0   1];