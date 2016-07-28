function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

% persistent waypoints0 traj_time d0
% if nargin > 2   %Initialisation
%     d = waypoints(:,2:end) - waypoints(:,1:end-1);
%     d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);   %distance per segment
%     traj_time = [0, cumsum(d0)];
%     waypoints0 = waypoints;
% else        %Simulation time
%     if(t > traj_time(end))
%         t = traj_time(end);
%     end
%     t_index = find(traj_time >= t,1);   %find the index of the closest tabulated trajectory time corresponding to the current time
%     if(t_index > 1) %Exception for the first time point
%         t = t - traj_time(t_index-1);
%     end         
%     if(t == 0)  %Exception for time=0
%         desired_state.pos = waypoints0(:,1);
%     else    %For any other time, make a scalar progression between points
%         scale = t/d0(t_index-1);
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%     end
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
% end
% 


%% Fill in your code here

% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;


persistent waypoints0 traj_time d0
if nargin > 2   %Initialisation
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);   %distance per segment
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    
    [Cx,Cy,Cz]=computeCoefs(waypoints0,d0);
    
else        %Simulation time
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);   %find the index of the closest tabulated trajectory time corresponding to the current time
    if(t_index > 1) %Exception for the first time point
        t = t - traj_time(t_index-1);
    end         
    if(t == 0)  %Exception for time=0
        desired_state.pos = waypoints0(:,1);
    else    %For any other time, make a scalar progression between points
        scale = t/d0(t_index-1);
        desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
    end
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
end

%%Functions
function [Cx,Cy,Cz]=computeCoefs(w,d0)
%computes the coefficients for the polynomials that defines the trajectory.
%The problem to solve is A*Coeffs=b, where A contains the constraints, b
%contains the waypoints and Coeffs is the matrix of coefficients. It must
%be taken into account that it has to be solved for x, y and z.

n=size(w,1)-1; %Number of polynomials -1

%initialise matrixes:
b=zeros(3,8*n); %dimension is 3x8 coz we have to solve for x,y and z and for a snap trajectory (8 coefficients per polinomial)
A=zeros(3,8*n,8*n);

b=w;

%Construction of A


for i=1:3
    for j=1:n
        
        %position constraint: P(t=0)
        A(i,j,((j-1)*8)+1:j*8) = polynom(8,0,0);
        
        
        
        
        
end

end




















