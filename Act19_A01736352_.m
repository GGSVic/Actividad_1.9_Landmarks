%% EXAMPLE: Differential drive vehicle following waypoints using the 
% Pure Pursuit algorithm
%
% Copyright 2018-2019 The MathWorks, Inc.

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.05;               % Sample time [s]
tVec = 0:sampleTime:34.2;        % Time array
%{
Ejercicio 1: tVec = 0:sampleTime:34.2; 
Ejercicio 2: tVec = 0:sampleTime:19;
Ejercicio 3: tVec = 0:sampleTime:14;  
Dog: tVec = 0:sampleTime:143;  
Flower: tVec = 0:sampleTime:131; 
Cherry: tVec = 0:sampleTime:83.4; 
%}


initPose = [4;4;0]; % Initial pose (x y theta) %apple =  [9;5;3*pi/4]; 
%{
Ejercicio 1: initPose = [4;4;0];  
Ejercicio 2: initPose = [2;5;0];
Ejercicio 3: initPose = [-3;4;0];
Dog: initPose = [7;8;pi/4];
Flower: initPose = [4; 6; 0];
Cherry: initPose = [9; 5; 3*pi/4]; 
%}

pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

waypoints = [4,4; -10,8; 8,-1; -7,-6; 0,5; -3,0,; 2,-5; 0,0];% Define waypoints
%{
Ejercicio 1: waypoints = [4,4; -10,8; 8,-1; -7,-6; 0,5; -3,0,; 2,-5; 0,0];
Ejercicio 2: waypoints = [2,5; -5,3; -5,-2; 2,-5; 5,2; -3,2,; -4,-4; 4,-3];
Ejercicio 3: waypoints = [-3,4; 3,3; 1,-3; -1,-1; 1,4; -4,-3,; 2,-1];



Dog: waypoints = [7,8; 8,9; 7,12; 7,10; 7,11; 6,11; 5,10.75; 5,12; 6,11; 3,10; 
             3,9; 4,9; 4,10; 5,10; 4,9; 2,9; 1,9; 1,8; 2,9; 1,9; 1,8; 2,6;  
             3,7; 4,7; 3,7; 2,6; 5,6; 6,5; 6,4; 11,6; 10,7; 6,5; 6,2; 7,0; 
             12,0; 12,6; 11,6; 10,7; 9,8; 7,12]; 

Flower:  waypoints = [4,6; 5,6; 5,8; 3,8; 3,6; 4,6; 4,1; 6,3; 6,5; 8,5; 
                      6,7; 8,9; 6,9; 6,11; 4,9; 2,11; 2,9; 0,9; 2,7; 0,5;
                      2,5; 2,3; 4,5; 6,3; 8,3; 6,1; 2,1; 0,3; 2,3; 4,1]; 

Cherry: waypoints = [9,5; 7,7; 5,7; 3,5; 3,3; 5,1; 7,1; 9,3; 9,5;
                    8,5; 7,5; 6,6; 7,5; 10,9; 8,11; 6,11; 4,9; 10,9];

%}

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.4; 
controller.DesiredLinearVelocity = 3; 
controller.MaxAngularVelocity = 5; 

%{
%Ejercicio 1 / Ejercicio 2: 
controller.LookaheadDistance = 0.4; 
controller.DesiredLinearVelocity = 3; 
controller.MaxAngularVelocity = 5; 

Ejercicio 3:
controller.LookaheadDistance = 0.35;
controller.DesiredLinearVelocity = 3;
controller.MaxAngularVelocity = 8;

Ejercicio Dog: 
controller.LookaheadDistance = 0.35; 
controller.DesiredLinearVelocity = 0.53;  
controller.MaxAngularVelocity = 10; 

Ejercicio Flower: 
controller.LookaheadDistance = 0.2; 
controller.DesiredLinearVelocity = 0.53;  
controller.MaxAngularVelocity = 10; 

Ejercicio Cherry: 
controller.LookaheadDistance = 0.2; 
controller.DesiredLinearVelocity = 0.53;  
controller.MaxAngularVelocity = 30; 
%}

%% Simulation loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef); 
  
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints)
    waitfor(r);
    
end
