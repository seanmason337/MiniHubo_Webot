function coor = footPlot(theta,tx,ty)
    % Foot Dimensions
    l = 110;    %length
    w = 64;     %width
    of = .3;     %center offset in the backwards direction (%/100)
    
    coor = [-of*l       -w/2    0 ; %Coordinates of corner points
            (1-of)*l    -w/2    0 ;
            (1-of)*l    w/2     0 ;
            -of*l       w/2     0 ;
            -of*l       -w/2    0 ]';
    coor = [coor; 1 1 1 1 1];       %Add an additional matrix to multiply with the homogenous transfromation (4x4)
    coor = hZ(theta,tx,ty,0)*coor;
    coor = coor(1:2,:);
    