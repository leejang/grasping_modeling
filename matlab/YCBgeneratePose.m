%    YCBgeneratePose.m
%    Modified GraspPlannerExample.m by Jangwon Lee

close all
clear all
clc

total = input('set number of poses to generate:  ');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial Set up
%
%

%% Select a hand
disp('use paradigmatic hand')
hand = SGparadigmatic;

%% select an object
% height(mm), radius(mm), number of points around circumference
% pringles original can
%obj = SGcylinder(H,250,37.5,100);
%obj = SGcylinder(H,200,40,50);
%obj = SGcylinder(H,200,40,100);
% flat screwdriver
%obj = SGcylinder(H,198,11.5,100);
% size of a hammer handle, 30 mm diameter (test)
%obj = SGcylinder(H,100*rand(1)+100, 10*rand(1)+10,100);
% default
%obj = SGcylinder(H,80,40,50);

%% Define the number of possible starting configuration and the object to grasp (sphere, cube or cylinder)
N = 5;
disp('set type of the object:')

obj_type = input('(cylinder,cube):  ','s');
switch obj_type
    case 'cylinder'
        disp('use cylinder type of object');
    case 'cube'
        disp('use cube type of object');
    otherwise
        error('bad object definition');
end

%% Select quality measure
%disp('use ''mev'' for the grasp planner')
%type = 'mev';
disp('use ''dtsc'' for the grasp planner')
type = 'dtsc';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:total

    %% randomize object size, center and rotation
    switch obj_type
        case 'cylinder'
            % for cylinder type of object
            R = SGroty(-pi/3*rand(1));
            H =  [R,zeros(3,1);zeros(1,3),1];
            % object center (x,y,z: -10 ~ 10)
            H(1:3,4) = [20*rand(1)-10,20*rand(1)-10,20*rand(1)-10];
            % create the object 
            % height(mm), radius(mm), number of points around circumference
            obj = SGcylinder(H,100*rand(1)+120, 10*rand(1)+10,100);
        case 'cube'
            %% cube type object
            %R = SGrotz(0);
            %R = SGrotz(pi/2);
            R = SGrotz(2*pi*rand(1));
            H = [R,zeros(3,1);zeros(1,3),1];
            % object center (x,y,z: -10 ~ 10)
            H(1:3,4) = [20*rand(1)-10,20*rand(1)-10,20*rand(1)-10];
            % size of x, y, z (mm)
            % cereal box: Cheerios
            obj = SGcube(H,80,195,295);
        otherwise
            error('bad object definition');
    end

    %% Grasp planning
    [hand_c,object,index] = YCBgraspPlanner(hand,obj,N,i,type);

    %% Plot of the best grasp
    figure()
    SGplotHand(hand_c);
    axis auto
    hold on
    SGplotSolid(obj);
    title('best obtained grasp')
    hold on
    SGplotContactPoints(hand_c,10,'o')

    %% save current plot
    file_figure = strcat('generated_pose/fig_',num2str(i),'.jpg');
    saveas(gcf, file_figure);

    %% save hand structure and parameters
    disp(sprintf('save generated hand pose: %d', i));
    %% whole hand structure
    file_hand = strcat('generated_pose/hand_c_',num2str(i), '.json');
    %save(file_hand, 'hand_c');
    savejson('hand_c', hand_c, file_hand);

    %% finger
    file_finger = strcat('generated_pose/finger_',num2str(i), '.json');
    %save(file_finger, '-struct', 'hand_c', 'F');
    savejson('fingers', hand_c.F, file_finger);

    %% object
    file_object = strcat('generated_pose/object_',num2str(i), '.json');
    %save(file_object, '-struct', 'hand_c', 'objectbase');
    savejson('object', obj, file_object);
end
1
