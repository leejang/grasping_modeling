% This function generates a cloud of N points along a sphere with center
% given by object's matrix H, with gauge G. Then creates an array of Homogeneous
% transformation matrices for hand pose
% P is a 4*4*N matrix Sigma-World R.F.

function P = YCBgenerateCloud(struct,G,N)

 switch struct.type
     case 'sph'
         G = struct.radius+G;
     case 'cyl'
         %dist = norm(struct.p(:,1,1) - struct.center);
         %G = dist + G;
         G = struct.radius+G;
     case 'cube'
         %G = norm(0.5*(struct.dim)) + G;
         G = struct.dim(2)/2+G
     otherwise
         error('bad input argument')
 end

%disp('use it');

% These are realizations of Random Variables ~U[0,2*pi]
%x_angle = pi/6.*rand(1,N); % parameter for SGrotx 
%y_angle = pi/2.*ones(1,N);  % parameter for SGroty
%z_angle = pi/6.*rand(1,N); % parameter for SGrotz

rot_m = struct.Htr(1:3, 1:3);
axis_angles = rotm2axang(rot_m);
%disp(axis_angles);

rot_x = axis_angles(1) * axis_angles(4);
rot_y = axis_angles(2) * axis_angles(4);
rot_z = axis_angles(3) * axis_angles(4);
%disp(rot_x);
%disp(rot_y);
%disp(rot_z);

x_angle = [];
y_angle = [];
z_angle = [];

for i=1:N

    switch struct.type
        case 'cyl'
            if rem(i,2) == 0
                x_angle = [x_angle, pi*(2/9)*rand(1) - pi/9]; % parameter for SGrotx 
                y_angle = [y_angle, -pi/2 + rot_y]; % parameter for SGroty
                z_angle = [z_angle, pi*(2/9)*rand(1) + pi*(8/9)]; % parameter for SGrotz
            else
                x_angle = [x_angle, pi*(2/9)*rand(1) - pi/9]; % parameter for SGrotx 
                y_angle = [y_angle, pi/2 + rot_y]; % parameter for SGroty
                z_angle = [z_angle, pi*(2/9)*rand(1) - pi/9]; % parameter for SGrotz
            end
        case 'cube'
            if rem(i,2) == 0
                x_angle = [x_angle, pi/2]; % parameter for SGrotx 
                %y_angle = [y_angle, -pi/2 + rot_y]; % parameter for SGroty
                y_angle = [y_angle, rot_z]; % parameter for SGroty
                z_angle = [z_angle, -pi/2]; % parameter for SGrotz
            else
                x_angle = [x_angle, pi/2]; % parameter for SGrotx 
                %y_angle = [y_angle, pi/2 + rot_y]; % parameter for SGroty
                y_angle = [y_angle, rot_z]; % parameter for SGroty
                z_angle = [z_angle, -pi/2]; % parameter for SGrotz
            end

    end
end

%disp (y_angle);
%disp (z_angle);

%x_angle = 2*pi.*rand(1,N); % parameter for SGrotx 
%y_angle = 2*pi.*rand(1,N);  % parameter for SGroty
%z_angle = 2*pi.*rand(1,N);    % parameter for SGrotz

%Rotations:
Rx = zeros(3,3,N);
Ry = Rx;
Rz = Rx;
H_tmp = zeros(4,4,N);
P = H_tmp;
for i=1:N
    H_tmp(:,4,i) = [struct.center;1];
    Rx(:,:,i) = SGrotx(x_angle(i));
    Ry(:,:,i) = SGroty(y_angle(i));
    Rz(:,:,i) = SGrotz(z_angle(i));
    H_tmp(1:3,1:3,i) = Rx(:,:,i)*Ry(:,:,i)*Rz(:,:,i);
    if strcmp(struct.type, 'cube')
        H_tmp(:,:,i) = H_tmp(:,:,i)*SGtransl([(struct.dim(3)/2)*rand(1)-struct.dim(3)/4, 0 ,G]);
        % rotate
        %R_tmp = [eye(3),ones(3,1);zeros(1,3),1];
        R_center = SGrotz(0)*H_tmp(1:3,4,i);
        H_tmp(:,4,i) = [R_center;1];
    else
        H_tmp(:,:,i) = H_tmp(:,:,i)*SGtransl([0,0,G]);
    end
    %H_tmp(:,:,i) = H_tmp(:,:,i)*SGtransl([0,0,0]);
    P(:,:,i) = H_tmp(:,:,i);
end
