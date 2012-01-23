function coor = footPlot(theta)
    % Foot Dimensions
    w = 4;
    h = 2;
    tx = 2;
    ty = 1; 
    theta = 40;
    %Relative foot corner position
    coor = [-w/2 -h/2 0 ;
            w/2 -h/2 0 ;
            w/2 h/2 0 ;
            -w/2 h/2 0 ;
            -w/2 -h/2 0 ]'
    coor = [coor; 1 1 1 1 1];
    h = hZ(30,2,2,0);
    coor = hZ(theta,tx,ty,0)*coor;
