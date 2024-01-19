%%%%%%%%
%Based on the position (data name: posEE_XX) and orientation (data name: thetaSum_XX) data of the end-effector of the manipulator obtained using mechanics, 
% the position and orientation data of the bending and yawing joints according to the motor resolution will be obtained through interpolation. 
%%%%%%%%

%Load the position (data name: posEE_XX) and orientation (data name: thetaSum_XX) data of the end-effector of the manipulator obtained using mechanics%

load('C:\Users\LG\Documents\카카오톡 받은 파일\bend_base_ee.mat')
load('C:\Users\LG\Documents\카카오톡 받은 파일\oriEE_b.mat')
load('C:\Users\LG\Documents\카카오톡 받은 파일\posEE_yaw.mat')
load('C:\Users\LG\Documents\카카오톡 받은 파일\thetaSum_bend.mat')
load('C:\Users\LG\Documents\카카오톡 받은 파일\thetaSum_yaw.mat')
load('C:\Users\LG\Documents\카카오톡 받은 파일\posEE_bend.mat')

oriEE_b=transpose(oriEE_b);
thetaSum_bend=transpose(thetaSum_bend);
thetaSum_yaw=transpose(thetaSum_yaw);

bend_base_ee(1,:)=[0,169];

% Given data from actuator and instrument 

Disk_radius = 6; % [mm]
motor_res = deg2rad((300/1023));  % [rad] (maximum deg of motor rotaion)/(maximum motor steps)*(deg to rad)


tendon_disp_res = Disk_radius*motor_res; % [mm] resolution of tendon displacement due to motor resolution 
% total_theta_deg=65; % [deg]
% total_theta=deg2rad(total_theta_deg);

lenBen = 75; % [mm]  % length of bending 
lenYaw = 25; % [mm]  % length of yawing 

OD = 1.95; % [mm] % Outer diameter of instrument 
ID = OD/2; % [mm] % Inner diameter 


num_bend= floor((max(thetaSum_bend)*ID)/(Disk_radius*motor_res)+1); 
num_yaw = floor((max(thetaSum_yaw)*ID)/(Disk_radius*motor_res)+1);


bending_theta_per_tendon_disp_res = zeros(num_bend,1); % motor rotation angle [rad] per tendon displacement
yawing_theta_per_tendon_disp_res = zeros(num_yaw,1); % motor rotation angle [rad] per tendon displacement


%The bending angle of joint (thetaSum of the joint) when input is applied while the motor increments the rotation angle by the resolution value
for i=2:1:num_bend

    bending_theta_per_tendon_disp_res(i) = (Disk_radius/ID)*(motor_res*(i-1));

end


%The yawing angle of joint (thetaSum of the joint) when input is applied while the motor increments the rotation angle by the resolution value
for i=2:1:num_yaw

    yawing_theta_per_tendon_disp_res(i) = (Disk_radius/ID)*(motor_res*(i-1));

end



% Matrix for (bending_pos / yawing_pos)


%Organize data in ascending order for interpolation

%for bending
Mat_Bend = [thetaSum_bend,posEE_bend]; %combine orientation and position data of bending joint EE from manipulator mechanics function %given data 
Mat_res_Bend = [bending_theta_per_tendon_disp_res,zeros(num_bend,2)];  %combine orientation and position data of bending joint EE from motor resolution %Data that should be interpolated 

bf_Bend = [Mat_Bend;Mat_res_Bend];  %data combine 

sort_bf_Bend = sortrows(bf_Bend);  % Organize data in ascending order of bending angle 

sort_bf_Bend(1,:) = []; % delete [0,0,0]


%for yawing
Mat_yaw = [thetaSum_yaw,posEE_yaw];
Mat_res_yaw = [yawing_theta_per_tendon_disp_res,zeros(num_yaw,2)];

bf_yaw=[Mat_yaw;Mat_res_yaw];

sort_bf_yaw=sortrows(bf_yaw);

sort_bf_yaw(1,:) = [];


% before interpolaion
bf_ITPbend_x = [sort_bf_Bend(:,1),sort_bf_Bend(:,2)];  % for bending ) [orientation , x position] before interpolation % Given data + Data to be interpolated
bf_ITPbend_y = [sort_bf_Bend(:,1),sort_bf_Bend(:,3)];  % for bending ) [orientation , y position] before interpolation % Given data + Data to be interpolated

bf_ITPyaw_x = [sort_bf_yaw(:,1),sort_bf_yaw(:,2)];  % for yawing ) [orientation , x position] before interpolation  % Given data + Data to be interpolated
bf_ITPyaw_y = [sort_bf_yaw(:,1),sort_bf_yaw(:,3)];  % for yawing ) [orientation , y position] before interpolation % Given data + Data to be interpolated


% before interpolaion 
ITPbend_x = [sort_bf_Bend(:,1),sort_bf_Bend(:,2)];   % for bending ) [orientation , x position] before interpolation
ITPbend_y = [sort_bf_Bend(:,1),sort_bf_Bend(:,3)];  % for bending ) [orientation , y position] before interpolation


ITPyaw_x = [sort_bf_yaw(:,1),sort_bf_yaw(:,2)];  % for yawing ) [orientation , x position] before interpolation 
ITPyaw_y = [sort_bf_yaw(:,1),sort_bf_yaw(:,3)];  % for yawing ) [orientation , y position] before interpolation
  

% Given data + Data to be interpolated [orientation, x-position, y-position]
p_bend_motor_res = zeros(num_bend,3);  
p_yaw_motor_res = zeros(num_yaw,3);


p_bend_motor_res(1,:) = [0,0,243];
p_yaw_motor_res(1,:) = [0,0,163];

% position(x,y) interpolation 

% for bend

j = 1;
for i=2:1:(length(bf_ITPbend_x)-1)

        if bf_ITPbend_x(i,2) == 0
            j=j+1;

            conti_x_bend = linspace(bf_ITPbend_x(i-1,2),bf_ITPbend_x(i+1,2),100);
            conti_theta_bend = linspace(bf_ITPbend_x(i-1,1),bf_ITPbend_x(i+1,1),100);

            ITPbend_x(i,2) = interp1(conti_theta_bend,conti_x_bend,bf_ITPbend_x(i,1));


            p_bend_motor_res(j,2)=ITPbend_x(i,2);
        end

end


j = 1;
for i=2:1:(length(bf_ITPbend_y)-1)

    if bf_ITPbend_y(i,2) == 0
        j=j+1;

        conti_y_bend = linspace(bf_ITPbend_y(i-1,2),bf_ITPbend_y(i+1,2),100);
        conti_theta_bend = linspace(bf_ITPbend_y(i-1,1),bf_ITPbend_y(i+1,1),100);


        ITPbend_y(i,2) = interp1(conti_theta_bend,conti_y_bend,bf_ITPbend_y(i,1));
        p_bend_motor_res(j,3)=ITPbend_y(i,2);

    end

end

p_bend_motor_res(:,1)=bending_theta_per_tendon_disp_res;

ITPbend_y(:,1) = [];

p_bend=[ITPbend_x,ITPbend_y];


% for yaw
j = 1;
for i=2:1:(length(bf_ITPyaw_x)-1)

    if bf_ITPyaw_x(i,2) == 0
        j = j+1;

        dis_ITPyaw_x = [bf_ITPyaw_x(i-1,:);bf_ITPyaw_x(i,:);bf_ITPyaw_x(i+1,:)];

        conti_x_yaw = linspace(bf_ITPyaw_x(i-1,2),bf_ITPyaw_x(i+1,2),100);
        conti_theta_yaw = linspace(bf_ITPyaw_x(i-1,1),bf_ITPyaw_x(i+1,1),100);


        ITPyaw_x(i,2) = interp1(conti_theta_yaw,conti_x_yaw,bf_ITPyaw_x(i,1));
        p_yaw_motor_res(j,2)=ITPyaw_x(i,2);

    end
end

j = 1;
    for i=2:1:(length(bf_ITPyaw_y)-1)
        if bf_ITPyaw_y(i,2) == 0
            j = j+1;

            dis_ITPyaw_y = [bf_ITPyaw_y(i-1,:);bf_ITPyaw_y(i,:);bf_ITPyaw_y(i+1,:)];

            conti_y_yaw = linspace(bf_ITPyaw_y(i-1,2),bf_ITPyaw_y(i+1,2),100);
            conti_theta_yaw = linspace(bf_ITPyaw_y(i-1,1),bf_ITPyaw_y(i+1,1),100);


            ITPyaw_y(i,2) = interp1(conti_theta_yaw,conti_y_yaw,bf_ITPyaw_y(i,1));
            p_yaw_motor_res(j,3)=ITPyaw_y(i,2);

        end

    end


p_yaw_motor_res(:,1)=yawing_theta_per_tendon_disp_res;

ITPyaw_y(:,1) = [];

p_yaw=[ITPyaw_x,ITPyaw_y];


% save as csv : writematrix(p_bend_motor_res,'bend_data.csv') , writematrix(p_yaw_motor_res,'yaw_data.csv')


%% 
% First, find the angle of the bending base through interpolation.


thetaSum_bendbase=[sort_bf_Bend(:,1),sort_bf_Bend(:,2)];

j=1;
for i=1:1:length(thetaSum_bendbase)

    if thetaSum_bendbase(i,2) ~= 0
        thetaSum_bendbase(i,2) = oriEE_b(j+1);
        j=j+1;
    end

end


%bending base theta interpolation
ITP_bendbase_theta=zeros(length(p_bend_motor_res),1);
bf_ITP_bendbase_theta=thetaSum_bendbase;


j = 1;
for i=2:1:(length(thetaSum_bendbase)-1)

    if thetaSum_bendbase(i,2) == 0

        j=j+1;

        conti_theta_bendbase = linspace(thetaSum_bendbase(i-1,2),thetaSum_bendbase(i+1,2),100);
        conti_theta_bend_for_base = linspace(thetaSum_bendbase(i-1,1),thetaSum_bendbase(i+1,1),100);

        bf_ITP_bendbase_theta(i,2) = interp1(conti_theta_bend_for_base,conti_theta_bendbase,thetaSum_bendbase(i,1));
        ITP_bendbase_theta(j) = bf_ITP_bendbase_theta(i,2);

    end

end


%  Base position interpolation based on interpolated bend base theta

%for bend base
Mat_Bendbase = bendbaseEE_data;
Mat_res_Bendbase = [ITP_bendbase_theta,zeros(num_bend,2)];  %combine orientation and position data of bending joint EE from motor resolution %Data that should be interpolated 

bf_Bendbase = [Mat_Bendbase;Mat_res_Bendbase];  %data combine 

sort_bf_Bendbase = sortrows(bf_Bendbase);  % Organize data in ascending order of bending angle 

sort_bf_Bendbase(1,:) = []; % delete [0,0,0]


bf_ITPbendbase_x = [sort_bf_Bendbase(:,1),sort_bf_Bendbase(:,2)];  % for bending ) [orientation , x position] before interpolation 
bf_ITPbendbase_y = [sort_bf_Bendbase(:,1),sort_bf_Bendbase(:,3)];  % for bending ) [orientation , y position] before interpolation


% before interpolaion 
ITPbendbase_x = [sort_bf_Bendbase(:,1),sort_bf_Bendbase(:,2)];   % for bending ) [orientation , x position] before interpolation
ITPbendbase_y = [sort_bf_Bendbase(:,1),sort_bf_Bendbase(:,3)];  % for bending ) [orientation , y position] before interpolation

p_bendbase_motor_res = zeros(num_bend,3);  

p_bendbase_motor_res(1,:) = [0,0,243];


j = 1;
for i=2:1:(length(bf_ITPbendbase_x)-1)

        if bf_ITPbendbase_x(i,2) == 0
            j=j+1;

            conti_x_bendbase = linspace(bf_ITPbendbase_x(i-1,2),bf_ITPbendbase_x(i+1,2),100);
            conti_theta_bendbase = linspace(bf_ITPbendbase_x(i-1,1),bf_ITPbendbase_x(i+1,1),100);

            ITPbendbase_x(i,2) = interp1(conti_theta_bendbase,conti_x_bendbase,bf_ITPbendbase_x(i,1));


            p_bendbase_motor_res(j,2)=ITPbendbase_x(i,2);
        end

end


j = 1;
for i=2:1:(length(bf_ITPbendbase_y)-1)

    if bf_ITPbendbase_y(i,2) == 0
        j=j+1;


        conti_y_bendbase = linspace(bf_ITPbendbase_y(i-1,2),bf_ITPbendbase_y(i+1,2),100);
        conti_theta_bendbase = linspace(bf_ITPbendbase_y(i-1,1),bf_ITPbendbase_y(i+1,1),100);


        ITPbendbase_y(i,2) = interp1(conti_theta_bendbase,conti_y_bendbase,bf_ITPbendbase_y(i,1));
        p_bendbase_motor_res(j,3)=ITPbendbase_y(i,2);

    end

end

p_bendbase_motor_res(:,1)=ITP_bendbase_theta;

ITPbendbase_y(:,1) = [];

p_bendbase=[ITPbendbase_x,ITPbendbase_y];



for i = 1:size(p_bend_motor_res,1)
    
    p_bend_motor_res(i,2) = p_bend_motor_res(i,2)-6*cos((pi/2)-p_bend_motor_res(i,1));
    p_bend_motor_res(i,3) = p_bend_motor_res(i,3)+6*sin((pi/2)-p_bend_motor_res(i,1));
    
end
