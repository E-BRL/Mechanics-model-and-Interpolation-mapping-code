
function [thetaSum,posEE,bodyLine_force] = mechanics_model_for_yaw_and_bend_joint(CompValue_yaw)
% If you want to bend data, use CompValue_bend(not CompValue_yaw) and modify the code.
%% 1. condition setting (structure)

% niti size, mm
outD = 1.95;
outR = outD/2; %2.8~2.7
inD = 1.65;
inR = inD/2;

% multi-lumen tube size, mm
outD_mt = 2.7;
outR_mt = outD_mt/2;

% bending and yawing, length of each slit, mm
ben_len = 75;
yaw_len = 25;
ben_rigid = 138+30;
yaw_rigid = 138;

% unit, mm
heiS1 = 1.0;

numS1 = ben_len/heiS1;
numS1_r = ben_rigid/heiS1;
numS2 = yaw_len/heiS1;
numS2_r = yaw_rigid/heiS1;

% niti, bending rigidity
nitiE = 7*10^4; % N/mm^2 % 70 GPa = 70000 MPa

% second moment of area
nitiTube = (pi/64)*((outD^4)-(inD^4)); % mm^4

% These are bending rigidity.
%bendSb = flip(CompBend); %If you want to get bend data, you should use it.
bendSy = flip(CompYaw); 

% multi-lumen tube, bending rigidity
multiTubeE = 170; % 170 MPa
multiTubeI = (pi/64)*((2.9^4)-(2.0^4)); % mm^4

bendRigMT = multiTubeE*multiTubeI;

fric = 0.2; %friction coefficient

%% 2. Variables setting

% theta variable
syms theta1seg

rotMat = [1 0 0; 0 cos(theta1seg) -sin(theta1seg); 0 sin(theta1seg) cos(theta1seg)];
posForceMat = [0 (heiS1/theta1seg)*(cos(theta1seg)-1) (heiS1/theta1seg)*sin(theta1seg)]';
posDisMat = [0 (heiS1/theta1seg)*(cos(theta1seg/2)-1) (heiS1/theta1seg)*sin(theta1seg/2)]';


%% 3. calculate curvature

clearvars bodyLine thetaSum posEE

for f = 1:81

    tenForce = ((f-1)/10)*3; %0N to 24N(bending joint can bend about 90 deg)
    % actually, about 94.xx N

    piSeg = 0.2*5;  %temporary estimate value of angle sum. it should be updated after iterative process
    error = 1;
    ex2Err = 0;
    ex1Err = 0;

    %이 추정값은 전에 sumTheta error로 조정하면서 반복했던 그 코드에서 따옴.

    while (abs(error)>0.002)

        clearvars momentUnit
        
        % Calculate sum force of each segment, (left: +, right: -)
        sumForce = tenForce/exp(fric*piSeg);
        thetaMat = zeros(numS2+numS2_r,1); % *for bending data, use numS1+numS1_r

        for i = 1:numS2+numS2_r % *for bending data, use numS1+numS1_r

            if i <= numS2 % *bending data, use numS1
                bendRig = bendSy(i) + bendRigMT; % * for bending data, bendSb(i)
            else
                bendRig = nitiE*nitiTube + bendRigMT;
            end


            sumTheta_i = sum(thetaMat);  % 이전 unit까지의 각도 합 (known)

            tendon_force = rotMat*([0 0 -sumForce]'*exp(fric*(sumTheta_i))); 

            distributed_force= (sumForce*exp(fric*(sumTheta_i))*[0 -sin(theta1seg) cos(theta1seg)-1]');

            fixMoment = outR_mt*sumForce*exp(fric*(sumTheta_i)); % 마찰로 감쇠된 fix moment 그냥 구하기
            tenMoment = cross(posForceMat,tendon_force); % 마찰로 감쇠된 force 힘
            disMoment = cross(posDisMat,distributed_force);

            momentSum = fixMoment+tenMoment(1)+disMoment(1);

            eqn = bendRig*(theta1seg/heiS1) == momentSum;
            theta_eval = eval(vpasolve(eqn,theta1seg));
            thetaMat(i) = theta_eval;

            momentUnit(i) = momentSum;


        end

        error = piSeg-sum(thetaMat);

         if abs(error)>0.05
            delE = 0.03;
        elseif abs(error)>0.03
            delE = 0.01;
        elseif abs(error)>0.005
            delE = 0.001;
        end

        if error>0
            piSeg = piSeg-delE;
        elseif error<0
            piSeg = piSeg+delE;
        end

        %fprintf('current error = %.4f\n',error);

        if ex2Err == error
            break;
        end

        ex2Err = ex1Err;
        ex1Err = error;

        error = abs(error);
        %fprintf('current piSeg = %.4f\n',piSeg);

    end
    
    thetaSum(f) = sum(thetaMat); % Orientation of end-effector.
    
    % shape calculation
    
    clearvars xp1 yp1 zp1 

    th = thetaMat(numS2+numS2_r); % * for bending data, use numS1+numS1_r

    tranMat01 = [1 0 0 0; 0 cos(th) -sin(th) (heiS1/th)*(cos(th)-1); 0 sin(th) cos(th) (heiS1/th)*sin(th); 0 0 0 1];
    xp1(1) = 0;
    yp1(1) = (heiS1/th)*(cos(th)-1);
    zp1(1) = (heiS1/th)*sin(th);

    for j = 2:numS2+numS2_r % * for bending data, use numS1+numS1_r

        tr0 = tranMat01;
        th = thetaMat(numS2+numS2_r-j+1); % * for bending data, use numS1+numS1_r

        tranMat = [1 0 0 0; 0 cos(th) -sin(th) (heiS1/th)*(cos(th)-1); 0 sin(th) cos(th) (heiS1/th)*sin(th); 0 0 0 1];

        tranMat01 = tr0*tranMat;

        xp1(j) = tranMat01(1,4);
        yp1(j) = tranMat01(2,4);
        zp1(j) = tranMat01(3,4);

        %posMat = [tranMat(1,4) tranMat(2,4) tranMat(3,4)];
        %fprintf('norm : %.4f\n',norm(posMat));

    end

    % seg == 1 -> bodyLine 저장

    bodyLine(:,1)=yp1;
    bodyLine(:,2)=zp1;
    
    posEE(f,:) = bodyLine(numS2+numS2_r, :); % * for bending data, use numS1+numS1_r % position of end-effector
    bodyLine_force(f,:,:) = bodyLine;

end 
