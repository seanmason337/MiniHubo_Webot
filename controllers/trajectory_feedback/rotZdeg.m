function out = rotZdeg(theta)
    out =     [cosd(theta)     -sind(theta)  0;
              sind(theta)     cosd(theta)   0;
              0               0             1];
          