%%%%% HEX-ROBOT simulation using MATLAB ROBOTICS TOOLBOX %%%%%
%%%%%%%%%%%%%%% authored By hsh?cz. Sustech 2021%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%

clear
clc;
close('all');
deg = pi/180;
m1=zeros(60,3);
m2=zeros(40,3);

%%
%%%%%%%%%%%%%%% create 6 legs & 1 arm & 1 cube for robot %%%%%%%%%%%%%%%
%%%%%%%%% z axis towards outside the screen%%%%%%%%%%%%

%%%%%%%%%% NEED TO MODIFY LINK LENGTH AND JOINT LIMITS%%%%

leglink1_len = 10;
leglink2_len = 10;
leglink3_len = 10;

griplink1_len = 12;
griplink2_len = 10;
griplink3_len = 8;



%%%%%%%%%%%%%%%%%%%%%%%%   LEG1   %%%%%%%%%%%%%%
LegN1(1)=Link('revolute','d',0,'a',leglink1_len,'alpha',pi/2,'offset',0,'standard'); 
LegN1(2)=Link('revolute','d',0,'a',leglink2_len,'alpha',0,'offset',-pi/2,'standard');
LegN1(3)=Link('revolute','d',0,'a',leglink3_len,'alpha',0,'offset',0,'standard');
LegN1(1).qlim = [-20,20]*deg;
LegN1(2).qlim = [-100,100]*deg;
LegN1(3).qlim = [-100,100]*deg;
Leg1 = SerialLink(LegN1,'name','leg1','base',transl(6.5, 0, 0));

%%%%%%%%%%%%%%%%%%%%  LEG2  %%%%%%%%%%%%
LegN2(1)=Link('revolute','d',0,'a',leglink1_len,'alpha',pi/2,'offset',pi/3,'standard'); 
LegN2(2)=Link('revolute','d',0,'a',leglink2_len,'alpha',0,'offset',-pi/2,'standard');
LegN2(3)=Link('revolute','d',0,'a',leglink3_len,'alpha',0,'offset',0,'standard');
LegN2(1).qlim =[-20,20]*deg;
LegN2(2).qlim = [-100,100]*deg;
LegN2(3).qlim = [-100,100]*deg;

Leg2 = SerialLink(LegN2,'name','leg2','base',transl(3, 6.5, 0));

%%%%%%%%%%%%%%%%%%%%  LEG3  %%%%%%%%%%%%
LegN3(1)=Link('revolute','d',0,'a',leglink1_len,'alpha',pi/2,'offset',2*pi/3,'standard'); 
LegN3(2)=Link('revolute','d',0,'a',leglink2_len,'alpha',0,'offset',-pi/2,'standard');
LegN3(3)=Link('revolute','d',0,'a',leglink3_len,'alpha',0,'offset',0,'standard');
LegN3(1).qlim = [-20,20]*deg;
LegN3(2).qlim = [-100,100]*deg;
LegN3(3).qlim = [-100,100]*deg;

Leg3 = SerialLink(LegN3,'name','leg3','base',transl(-3, 6.5, 0));

%%%%%%%%%%%%%%%%%%%%  LEG4  %%%%%%%%%%%%
LegN4(1)=Link('revolute','d',0,'a',leglink1_len,'alpha',pi/2,'offset',-1*pi/3,'standard'); 
LegN4(2)=Link('revolute','d',0,'a',leglink2_len,'alpha',0,'offset',-pi/2,'standard');
LegN4(3)=Link('revolute','d',0,'a',leglink3_len,'alpha',0,'offset',0,'standard');
LegN4(1).qlim = [-20,20]*deg;
LegN4(2).qlim = [-100,100]*deg;
LegN4(3).qlim = [-100,100]*deg;

Leg4 = SerialLink(LegN4,'name','leg4','base',transl(3, -6.5, 0));


%%%%%%%%%%%%%%%%%%%%  LEG5  %%%%%%%%%%%%
LegN5(1)=Link('revolute','d',0,'a',leglink1_len,'alpha',pi/2,'offset',-2*pi/3,'standard'); 
LegN5(2)=Link('revolute','d',0,'a',leglink2_len,'alpha',0,'offset',-pi/2,'standard');
LegN5(3)=Link('revolute','d',0,'a',leglink3_len,'alpha',0,'offset',0,'standard');
LegN5(1).qlim = [-20,20]*deg;
LegN5(2).qlim = [-100,100]*deg;
LegN5(3).qlim = [-100,100]*deg;

Leg5 = SerialLink(LegN5,'name','leg5','base',transl(-3, -6.5, 0));

%%%%%%%%%%%%%%%%%%%%  LEG6  %%%%%%%%%%%%
LegN6(1)=Link('revolute','d',0,'a',leglink1_len,'alpha',pi/2,'offset',pi,'standard'); 
LegN6(2)=Link('revolute','d',0,'a',leglink2_len,'alpha',0,'offset',-pi/2,'standard');
LegN6(3)=Link('revolute','d',0,'a',leglink3_len,'alpha',0,'offset',0,'standard');
LegN6(1).qlim = [-20,20]*deg;
LegN6(2).qlim = [-100,100]*deg;
LegN6(3).qlim = [-100,100]*deg;

Leg6 = SerialLink(LegN6,'name','leg6','base',transl(-6.5, 0, 0));

%%%%%%%%%%%%%%%%%%%%  ARM  %%%%%%%%%%%%
Grip(1)=Link('revolute','d',0,'a',griplink1_len,'alpha',0,'offset',0,'standard'); 
Grip(2)=Link('revolute','d',0,'a',griplink2_len,'alpha',0,'offset',0,'standard');
Grip(3)=Link('revolute','d',0,'a',griplink3_len,'alpha',0,'offset',0,'standard');
Grip(1).qlim = [-20,20]*deg;
Grip(2).qlim = [-80,80]*deg;
Grip(3).qlim = [-80,80]*deg;

RobotArm = SerialLink(Grip,'name','Grip','base',transl(0, 0, 4)*troty(-90));
Robot = [Leg1,Leg2,Leg3,Leg4,Leg5,Leg6,RobotArm];

%%%% draw all %%%%%%%
figure(1)
draw_Robot(Robot,[0 0 0],[0 0 0],[0 0 0],[0 0 0],[0 0 0],[0 0 0],[0 0 0]);

%%         %%%%%%%%%%%  traj test for Leg1 and leg2  %%%%%%%%

%%%%%%%%%%% multi_P1 and multi_P2 are points on the path for Leg1 and Leg2 
P1 = [16.4,0,-19.9];
P2 = [26.40,0,-17.22];
P3 = [25.19,6.74,-17.22];
P4 = [15.79,3.32,-19.9];
multi_P1 = [P1;P2;P3;P4;P1];

P5 = [7.9,15.0,-19.9];
P6 = [12.9,23.72,-17.22];
P7 = [6.37,26.09,-17.22];
P8 = [4.763,16.24,-19.9];
multi_P2 = [P5;P6;P7;P8;P5];

P9 = [-7.9,15.06,-19.9];
P10 = [-12.9,23.72,-17.22];
P11 = [-6.37,26.09,-17.22];
P12 = [-4.63,16.24,-19.9];
multi_P3 = [P9;P10;P11;P12;P9];

P13 = [7.9,-15.0,-19.9];
P14 = [12.9,-23.72,-17.22];
P15 = [18.22,-19.25,-17.22];
P16 = [10.4,-12.7,-19.9];
multi_P4 = [P13;P14;P15;P16;P13];

P17 = [-7.9,-15.06,-19.9];
P18 = [-12.9,-23.72,-17.22];
P19 = [-18.22,-19.25,-17.22];
P20 = [-10.56,-12.82,-19.9];
multi_P5 = [P17;P18;P19;P20;P17];

P21 = [-16.4,0,-19.9];
P22 = [-26.4,0,-17.22];
P23 = [-25.19,6.74,-17.22];
P24 = [-15.79,3.32,-19.9];
multi_P6 = [P21;P22;P23;P24;P21];

P25 = [0,0,33.9];
P26 = [0,28.87,11.66];
P27 = [0,19.55,-10.63];
P28 = [0,14.55,-12.0];

P29 = [0,0.1,33.9];
P290 = [0,4.28,30.4];
P291 = [0,24.96,10.2];
P292 = [0,18.9,19.8];
multi_P7 = [P29;P290;P291;P292;P29];
%%%%%%%%%%%%%% using GetTraj_plot and GetTraj to plot and draw
figure(2)
suptitle('Position/Velocity/Acceleration for Leg1(one step move)')
GetTraj_plot([],[],[],multi_P1,0,10,0.2);

figure(3)
suptitle('Position/Velocity/Acceleration for Leg2(one step move)')
GetTraj_plot([],[],[],multi_P2,0,10,0.2);

figure(4)
suptitle('Position/Velocity/Acceleration for Leg3(one step move)')
GetTraj_plot([],[],[],multi_P3,0,10,0.2);

figure(5)
suptitle('Position/Velocity/Acceleration for Leg4(one step move)')
GetTraj_plot([],[],[],multi_P4,0,10,0.2);

figure(6)
suptitle('Position/Velocity/Acceleration for Leg5(one step move)')
GetTraj_plot([],[],[],multi_P5,0,10,0.2);

figure(7)
suptitle('Position/Velocity/Acceleration for Leg6(one step move)')
GetTraj_plot([],[],[],multi_P6,0,10,0.2);

figure(8)
suptitle('Position/Velocity/Acceleration for Arm(one move)')
GetTraj_plot([],[],[],multi_P7,0,10,0.2);

t = 0:0.2:10-0.2;  %% insert value between two points
Q1 = GetTraj(multi_P1,t,Leg1,[]);
Q1 = [Q1;m2;Q1;m2;Q1;m2;Q1;m2;Q1;m2;Q1;m2;Q1;m2;Q1;m2;Q1;m2;Q1;m2;Q1;m2;Q1;m2];
Q2 = GetTraj(multi_P2,t,Leg2,[]);
Q2 = [m1;Q2;m2;Q2;m2;Q2;m2;Q2;m2;Q2;m2;Q2;m2;Q2;m2;Q2;m2;Q2;m2;Q2;m2;Q2;m2;Q2];
Q3 = GetTraj(multi_P3,t,Leg3,[]);
Q3 = [Q3;m2;Q3;m2;Q3;m2;Q3;m2;Q3;m2;Q3;m2;Q3;m2;Q3;m2;Q3;m2;Q3;m2;Q3;m2;Q3;m2];
Q4 = GetTraj(multi_P4,t,Leg4,[]);
Q4 = [m1;Q4;m2;Q4;m2;Q4;m2;Q4;m2;Q4;m2;Q4;m2;Q4;m2;Q4;m2;Q4;m2;Q4;m2;Q4;m2;Q4];
Q5 = GetTraj(multi_P5,t,Leg5,[]);
Q5 = [Q5;m2;Q5;m2;Q5;m2;Q5;m2;Q5;m2;Q5;m2;Q5;m2;Q5;m2;Q5;m2;Q5;m2;Q5;m2;Q5;m2];
Q6 = GetTraj(multi_P6,t,Leg6,[]);
Q6 = [m1;Q6;m2;Q6;m2;Q6;m2;Q6;m2;Q6;m2;Q6;m2;Q6;m2;Q6;m2;Q6;m2;Q6;m2;Q6;m2;Q6];
Q7 = GetTraj(multi_P7,t,RobotArm,[]);
Q7 = [Q7;m2;Q7;m2;Q7;m2;Q7;m2;Q7;m2;Q7;m2;Q7;m2;Q7;m2;Q7;m2;Q7;m2;Q7;m2;Q7;m2];

%%%%%%%%%%%%  animate  %%%%%%%%%
%%%%%%%%%%% must claim figure() and plot() when using animate %%%%%%%%%
%%
figure(9)


Leg1.plot(Q1(1,:),'trail','b-','workspace',[-35 35 -35 35 -35 35],'nobase')
hold on
Leg2.plot(Q2(1,:),'trail','r-','workspace',[-35 35 -35 35 -35 35],'nobase')
hold on
Leg3.plot(Q3(1,:),'trail','r-','workspace',[-35 35 -35 35 -35 35],'nobase')
hold on
Leg4.plot(Q4(1,:),'trail','r-','workspace',[-35 35 -35 35 -35 35],'nobase')
hold on
Leg5.plot(Q5(1,:),'trail','r-','workspace',[-35 35 -35 35 -35 35],'nobase')
hold on
Leg6.plot(Q6(1,:),'trail','r-','workspace',[-35 35 -35 35 -35 35],'nobase')
hold on
RobotArm.plot(Q7(1,:),'trail','r-','workspace',[-35 35 -35 35 -35 35],'nobase')
hold on
plotcube([10 10 6],[ -5  -5  -3],1,[1 0 0]);
hold on


opt.niterations=length(Q1);
opt.movie = [];
A = Animate(opt.movie);
for i=1:opt.niterations
    Leg1.animate(Q1(i,:));
    Leg2.animate(Q2(i,:));
    Leg3.animate(Q3(i,:));
    Leg4.animate(Q4(i,:));
    Leg5.animate(Q5(i,:));
    Leg6.animate(Q6(i,:));
    RobotArm.animate(Q7(i,:));
    drawnow
    A.add();
end



%%        
figure(11)
draw_Robot(Robot,[0 0 0],[0 0 0],[0 0 0],[0 0 0],[0 0 0],[0 0 0],[0 0 0]);
Get_working_space(5000,LegN1,Leg1); 
Get_working_space(5000,LegN2,Leg2); 
Get_working_space(5000,LegN3,Leg3); 
Get_working_space(5000,LegN4,Leg4); 
Get_working_space(5000,LegN5,Leg5); 
Get_working_space(5000,LegN6,Leg6); 
Get_working_space(5000,Grip,RobotArm); 
%RobotArm.teach



%%
%%%%%%%%%%%%%%%%% just a test block , can be used to test kinetic %%%%%%%%%%%%%%%         

Leg1.teach


q1 = 50.4*pi/180;
alp = 1.5708;
alp2 = 0;
alp3 = 0;
q2 = 39.1*pi/180;
q3 = 57.8*pi/180;
T01 = [cos(q1) -sin(q1)*cos(alp) sin(q1)*sin(alp) 10*cos(q1);
    sin(q1) cos(alp)*cos(q1) -cos(q1)*sin(alp) 10*sin(q1);
    0 sin(alp) cos(alp) 0;
    0 0 0 1];
T12 = [cos(q2) -sin(q2)*cos(alp2) sin(q2)*sin(alp2) 10*cos(q2);
    sin(q2) cos(alp2)*cos(q2) -cos(q2)*sin(alp2) 10*sin(q2);
    0 sin(alp2) cos(alp2) 0;
    0 0 0 1]; 
T23 = [cos(q3) -sin(q3)*cos(alp3) sin(q3)*sin(alp3) 10*cos(q3);
    sin(q3) cos(alp3)*cos(q3) -cos(q3)*sin(alp3) 10*sin(q3);
    0 sin(alp3) cos(alp3) 0;
    0 0 0 1]; 

T10 = T01*T12*T23;






%%                   Functions block

%%%%%%%%%%%%%%%%%%%%%%  Contain All Functions We Need %%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%% draw hex-Robot model %%%%%%%%%%%%%%%%
%%% description:
%%%%%%%% RobotSerialLinks:all RobotSerialLink in Robot,for instance:[Leg1,Leg2,....]
%%%%%%%% q1,q2,q3,q4,q5,q6,q7?initial joint variables for each leg and arm,
%%%%%%%% the initial value is all zero, q1-7 = [0,0,0]
function draw_Robot(RobotSerialLinks,q1,q2,q3,q4,q5,q6,q7)
grid on
RobotSerialLinks(1).plot(q1,'nobase')  %%%%% 
hold on
RobotSerialLinks(2).plot(q2,'nobase')
hold on
RobotSerialLinks(3).plot(q3,'nobase')
hold on
RobotSerialLinks(4).plot(q4,'nobase')
hold on
RobotSerialLinks(5).plot(q5,'nobase')
hold on
RobotSerialLinks(6).plot(q6,'nobase')
hold on
RobotSerialLinks(7).plot(q7,'nobase')
hold on
plotcube([10 10 6],[ -5  -5  -3],1,[1 0 0]);
end








%%%%%%%%%%%%%%% plot Traj graph(tool coordinates,velocity and acceleration on path ) %%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%% noted that inital_time and final_time is time between two points
%%%%%%%%%% in multi_P

%%% description:
%%%%%%% multi_P: points(x,y,z) on your robot's path include starting points and ternimal points (you can set more than 2 points)
%%%%%%% inital_time: time when robot starts moving 
%%%%%%% final_time: time when robot reaches next point in multi_P
%%%%%%% interpt: inset value
%%%%%%% P: many points coordinate(x,y,z) according to your path
%%%%%%% dP: many points velocity according to your path
%%%%%%% ddP: many points acceleration according to your path

%%% method:
%%%%%%% 1. using insert -value method to get many points
%%%%%%% coordinate,velocity and acceleration between 2 points in multi_P 
%%%%%%% 2. plot P,dP,ddP in subplot
function GetTraj_plot(P,dP,ddP,multi_P,inital_time,final_time,interpt)
t = inital_time:interpt:final_time;
for i = 1:size(multi_P,1)
    if i<=size(multi_P,1)-1
    [p,dp,ddp] = mtraj(@tpoly,multi_P(i,:),multi_P(i+1,:),t);
    %[p,dp,ddp] = tpoly(multi_P(i,:),multi_P(i+1,:),t);
    P = [P;p];
    dP = [dP;dp];
    ddP = [ddP;ddp];  
    end
end
t = inital_time:interpt:(final_time+interpt)*(size(multi_P,1)-1)-interpt;
grid on
subplot(1,3,1)
plot(t,P)
xlabel('time');
ylabel('position');
%title('position change');
xlim([0 41])
hold on
legend('x','y','z') 
subplot(1,3,2)
plot(t,dP)
xlabel('time');
ylabel('velocity');
%title('velocity change');
xlim([0 41])
hold on
legend('dx','dy','dz') 
subplot(1,3,3)
plot(t,ddP)
xlabel('time');
ylabel('acceleration');
%title('acceleration change');
xlim([0 41])
hold on
legend('ddx','ddy','ddz') 
end




%%%%%%%%%%%%%%% get robot tips' moving traj(plus RPY)%%%%%%%%%%%%%%

%%%%%%%%%%%  noted that points coordinates(x,y,z) must smaller than working space

%%% description:
%%%%%%% multi_P: points(x,y,z) on your robot's path include starting points and ternimal points (you can set more than 2 points)
%%%%%%% t: number of insert value,for example:0:0.01:1 = (1-0)/0.01 = 100 
%%%%%%% RobotSerialLink: RobotSerialLink,for example:Leg1
%%%%%%% P: matrix contains points_num points(x,y,z)
%%%%%%% Q: this method return many joint variables according to your path
%%%%%%% multi_RPY: RPY on your path

%%% method:
%%%%%%% 1. transform points(x,y,z) in multi_P to homogeous matrix T
%%%%%%% 2. using inverse kinetic to get joint variable[q1,q2,q3] according
%%%%%%% to your points in multi_P.
%%%%%%% 3. insert t joint variables[q1,q2,q3] between two joint variables
%%%%%%% and save them in Q
%%%%%%% 4. return Q
function Q = GetTrajPlusRPY(multi_P,multi_RPY,t,RobotSerialLink,Q)
T = zeros(4,4,size(multi_P,1));
q = zeros(size(multi_P,1),3);
for i=1:size(multi_P,1)
    T(:,:,i) = transl(multi_P(i,:))*trotx(multi_RPY(3))*troty(multi_RPY(2))*trotz(multi_RPY(1));
    q(i,:) = RobotSerialLink.ikine(T(:,:,i),'mask',[1 1 1 0 0 0]);
end

for i=1:size(multi_P,1)
    if i<=size(multi_P,1)-1
    Qtraj = jtraj(q(i,:),q(i+1,:),t);
    Q = [Q;Qtraj];
    end
end
end


%%%%%%%%%%%%%%% get robot tips' moving traj(not RPY)%%%%%%%%%%%%%%

%%%%%%%%%%%  noted that points coordinates(x,y,z) must smaller than working space

%%% description:
%%%%%%% multi_P: points(x,y,z) on your robot's path include starting points and ternimal points (you can set more than 2 points)
%%%%%%% t: number of insert value,for example:0:0.01:1 = (1-0)/0.01 = 100 
%%%%%%% RobotSerialLink: RobotSerialLink,for example:Leg1
%%%%%%% P: matrix contains points_num points(x,y,z)
%%%%%%% Q: this method return many joint variables according to your path
%%%%%%% multi_RPY: RPY on your path

%%% method:
%%%%%%% 1. transform points(x,y,z) in multi_P to homogeous matrix T
%%%%%%% 2. using inverse kinetic to get joint variable[q1,q2,q3] according
%%%%%%% to your points in multi_P.
%%%%%%% 3. insert t joint variables[q1,q2,q3] between two joint variables
%%%%%%% and save them in Q
%%%%%%% 4. return Q
function Q = GetTraj(multi_P,t,RobotSerialLink,Q)
T = zeros(4,4,size(multi_P,1));
q = zeros(size(multi_P,1),3);
for i=1:size(multi_P,1)
    T(:,:,i) = transl(multi_P(i,:));
    q(i,:) = RobotSerialLink.ikine(T(:,:,i),'mask',[1 1 1 0 0 0]);
end

for i=1:size(multi_P,1)
    if i<=size(multi_P,1)-1
    Qtraj = jtraj(q(i,:),q(i+1,:),t);
    Q = [Q;Qtraj];
    end
end
end




%%%%%%%%%%%%%%%% get working space for 1 leg or 1 arm %%%%%%%%%

%%% description:
%%%%%%% points_num: number of points create randomly to draw working space
%%%%%%% Robot: Link, for example:LegN1
%%%%%%% RobotSerialLink: for example:[LegN1,LegN2,LegN3]

%%% method:
%%%%% 1.create points_num joint variables[q1 q2 q3] randomly
%%%%% 2.using forward kinetic to get points_num homogeouns matrixs T
%%%%% 3.get every T's x,y,z value nad save them in P
%%%%% 4.draw points_num points(x,y,z) to create working space
function Get_working_space(points_num,Robot,RobotSerialLink)
P = zeros(points_num,3);

for i = 1:points_num
    q1 = Robot(1).qlim(1)+rand * (Robot(1).qlim(2)-Robot(1).qlim(1));
    q2 = Robot(2).qlim(1)+rand * (Robot(2).qlim(2)-Robot(2).qlim(1));
    q3 = Robot(3).qlim(1)+rand * (Robot(3).qlim(2)-Robot(3).qlim(1));
    q = [q1 q2 q3];
    T = RobotSerialLink.fkine(q);
    P(i,:)= transl(T);
end
RobotSerialLink.plot([0,0,0],'nobase','noshadow')
hold on
plot3(P(:,1),P(:,2),P(:,3),'b.','markersize',1)
axis([-40 40 -40 40 -40 40]);
hold on
end







%%%%%%%%%%%  function to plot cube %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% cube is main body of Hex-Robot %%%%%%%%%%%%%%%%%
function plotcube(varargin)
% PLOTCUBE - Display a 3D-cube in the current axes
%
%   PLOTCUBE(EDGES,ORIGIN,ALPHA,COLOR) displays a 3D-cube in the current axes
%   with the following properties:
%   * EDGES : 3-elements vector that defines the length of cube edges
%   * ORIGIN: 3-elements vector that defines the start point of the cube
%   * ALPHA : scalar that defines the transparency of the cube faces (from 0
%             to 1)
%   * COLOR : 3-elements vector that defines the faces color of the cube
%
% Example:
%   >> plotcube([5 5 5],[ 2  2  2],.8,[1 0 0]);
%   >> plotcube([5 5 5],[10 10 10],.8,[0 1 0]);
%   >> plotcube([5 5 5],[20 20 20],.8,[0 0 1]);

% Default input arguments
inArgs = { ...
  [0.1 0.1 0.1] , ... % Default edge sizes (x,y and z)
  [0.1 0.1 0.1] , ... % Default coordinates of the origin point of the cube
  0.7          , ... % Default alpha value for the cube's faces
  [1 1 0]       ... % Default Color for the cube
  };
% Replace default input arguments by input values
inArgs(1:nargin) = varargin;
% Create all variables
[edges,origin,alpha,clr] = deal(inArgs{:});
XYZ = { ...
  [0 0 0 0]  [0 0 1 1]  [0 1 1 0] ; ...
  [1 1 1 1]  [0 0 1 1]  [0 1 1 0] ; ...
  [0 1 1 0]  [0 0 0 0]  [0 0 1 1] ; ...
  [0 1 1 0]  [1 1 1 1]  [0 0 1 1] ; ...
  [0 1 1 0]  [0 0 1 1]  [0 0 0 0] ; ...
  [0 1 1 0]  [0 0 1 1]  [1 1 1 1]   ...
  };
XYZ = mat2cell(...
  cellfun( @(x,y,z) x*y+z , ...
    XYZ , ...
    repmat(mat2cell(edges,1,[1 1 1]),6,1) , ...
    repmat(mat2cell(origin,1,[1 1 1]),6,1) , ...
    'UniformOutput',false), ...
  6,[1 1 1]);
cellfun(@patch,XYZ{1},XYZ{2},XYZ{3},...
  repmat({clr},6,1),...
  repmat({'FaceAlpha'},6,1),...
  repmat({alpha},6,1)...
  );
view(3); 
end












