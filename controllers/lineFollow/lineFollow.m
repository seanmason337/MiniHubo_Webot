
% uncomment the next two lines if you want to use
% MATLAB's desktop and interact with the controller
%desktop;
%keyboard;

% control step
TIME_STEP=32;


% get the display (not a real e-puck device !)
%display = wb_robot_get_device('display');
%wb_display_set_color(display, [0 0 0]);

% get and enable camera
camera = wb_robot_get_device('camera');
wb_camera_enable(camera,TIME_STEP);


% avoid dummy values on first device query
step = 0;
samples = 0;

while wb_robot_step(TIME_STEP) ~= -1

  step = step + 1;
  
  % get camera RGB image
  % this function return an image in true color (RGB) uint8 format
  rgb = wb_camera_get_image(camera);

  % display camera image
  figure(1);
  image(rgb);
  title('RGB Camera');

  % flush graphics
  drawnow;
  
  % actuate wheels
  wb_differential_wheels_set_speed(50, -50);
end
