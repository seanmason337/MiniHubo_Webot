function out = rotYdeg(theta)
    out =    [cosd(theta)     0             sind(theta);
              0               1             0           ;
              -sind(theta)     0             cosd(theta)];
          