% trajectory 행렬 생성
time = 1000;
True_Target = zeros(time, 6);
True_UUV1 = zeros(time, 6);
True_UUV2 = zeros(time, 6);
True_UUV3 = zeros(time, 6);
DOP = zeros(time, 1);
SimulationSetting=zeros(time, 31);

[True_Target, True_UUV1, True_UUV2, True_UUV3] = Real_trajectory( ...
     89, -17,  89, -20, 89 ,-19, ...
     68, -24,  67, -21, 68 ,-23, ...
   time,True_Target,True_UUV1,True_UUV2,True_UUV3);
%Target 각도
% 90 -20 40 -25                                                        

% Dop bad (무조건 80 이상)
% 89, -17,  89, -20, 89 ,-19, ...
%     68, -24,  67, -21, 68 ,-23, ...
% UUV1 = [100; 1000; -500; UUV1_vel*cos(UUV1_Eheading1)*cos(UUV1_Aheading1); UUV1_vel*sin(UUV1_Aheading1); UUV1_vel*sin(UUV1_Eheading1)];
%     UUV2 = [-100; 1000; -500; UUV2_vel*cos(UUV2_Eheading1)*cos(UUV2_Aheading1); UUV2_vel*sin(UUV2_Aheading1); UUV2_vel*sin(UUV2_Eheading1)];
%     UUV3 = [0; 100; -100; UUV3_vel*cos(UUV3_Eheading1)*cos(UUV3_Aheading1); UUV3_vel*sin(UUV3_Aheading1); UUV3_vel*sin(UUV3_Eheading1)];
%     Target = [0; 10000; -2000; Target_vel*cos(Target_Eheading1)*cos(Target_Aheading1); Target_vel*sin(Target_Aheading1); Target_vel*sin(Target_Eheading1)];

%DOP good(2~3사이)
% 115, -17,  80, -20, 89 ,-19, ...
%     95, -24,  60, -21, 80 ,-15, ...
% UUV1 = [10000; 1000; -100; UUV1_vel*cos(UUV1_Eheading1)*cos(UUV1_Aheading1); UUV1_vel*sin(UUV1_Aheading1); UUV1_vel*sin(UUV1_Eheading1)];
%     UUV2 = [-10000; 1000; -100; UUV2_vel*cos(UUV2_Eheading1)*cos(UUV2_Aheading1); UUV2_vel*sin(UUV2_Aheading1); UUV2_vel*sin(UUV2_Eheading1)];
%     UUV3 = [0; 100; -4000; UUV3_vel*cos(UUV3_Eheading1)*cos(UUV3_Aheading1); UUV3_vel*sin(UUV3_Aheading1); UUV3_vel*sin(UUV3_Eheading1)];
%     Target = [0; 10000; -2000; Target_vel*cos(Target_Eheading1)*cos(Target_Aheading1); Target_vel*sin(Target_Aheading1); Target_vel*sin(Target_Eheading1)];

[Esti_Target_pos] = Esti_EKF(True_Target,True_UUV1,True_UUV2,True_UUV3,CDN,Esti_Target_pos,time);%, Esti_Target_pro

plot4line(True_Target,True_UUV1,True_UUV2,True_UUV3)
plot5line(True_Target,True_UUV1,True_UUV2,True_UUV3,Esti_Target_pos)



 % for i = 1 : time
 %     PDOP = calculatedop(True_Target(i,1:3),True_UUV1(i,1:3), True_UUV2(i,1:3), True_UUV3(i,1:3));
 %     DOP(i,1) = PDOP;
 % end



DOPgoodtrajectories1 = struct();

% Adding fields (replace 'Field1', 'Field2' with actual field names)
DOPgoodtrajectories1.target = True_Target(:,1:3);
DOPgoodtrajectories1.mship = True_UUV3(:,1:3);
DOPgoodtrajectories1.torpedo1 = True_UUV1(:,1:3); 
DOPgoodtrajectories1.torpedo2 = True_UUV2(:,1:3); 



clc; clear; close all;
CDN = zeros(1000, 1);
%% Sensor / Target initialize

% 패시브 소나와 EKF를 이용한 방위각 기반 타겟 위치 추정 알고리즘
% 두개의 어뢰와 모선에 있는 소나 센서를 이용
% State=[x y z vx vy vz]'
% measurement=[b e]'각각 방위각, 양각 
% 필터 :  EKF
load("Btraj.mat")% trajectory 파일들은 따로 생성 후 관리. 이 파일 안에 trajectories.target 형태로 저장.
% 새로 만들거나 수정할 때는 그냥 matlab에서 trajectories.target=[0 0 0; 1 1 1; ...] 이렇게 추가하면됨
target=initializeTarget([90 -20],15,DOPgoodtrajectories1.target); % [azimuth elevation](deg) velocity trajectory

%% Mothership and Torpedoes initialization
mship=initializeSensor(20,100,1,target,DOPgoodtrajectories1.mship);%v,maxD,noise(deg),target,trajectory
torpedo1=initializeSensor(20,100,1,target,DOPgoodtrajectories1.torpedo1);%v,maxD,noise(deg),target,trajectory
torpedo2=initializeSensor(20,100,1,target,DOPgoodtrajectories1.torpedo2);%v,maxD,noise(deg),target,trajectory

%% Simulation

for i=1:1:length(target.traj)
    updateSimulation(target);
    updateSimulation(mship);
    updateSimulation(torpedo1);
    updateSimulation(torpedo2);
    [uncertainArea, eigenVect, eigenVal, condNum] = findUncertaintyArea(mship.sonar, torpedo1.sonar, torpedo2.sonar, 0.1);
    CDN(i,1) = condNum;
end


%% Functions

% Update Sensor Function
function sonar = updateSensor(maxDistance, noise, position, heading)
    sonar.maxD = maxDistance; % 최대 측정 거리
    sonar.noise = deg2rad(noise); % 각 센서의 measurement Noise (angle)
    sonar.maxR = tan(sonar.noise) * sonar.maxD; % noise 기반 원뿔 반지름 생성
    sonar.numPoints = 10000; % 포인트 수

    sonar.P = position'; % 센서 위치
    sonar.D = [cos(heading(2)) * cos(heading(1)), ...
               cos(heading(2)) * sin(heading(1)), ...
               sin(heading(2))]; % 헤딩 방향은 방위각과 양각 기반으로 생성

    % 랜덤 포인트 생성
    theta1 = 2 * pi * rand(sonar.numPoints, 1);
    z1 = sonar.maxD * rand(sonar.numPoints, 1);
    r1 = sqrt(rand(sonar.numPoints, 1)) .* (z1 / sonar.maxD) * sonar.maxR;
    x1 = r1 .* cos(theta1);
    y1 = r1 .* sin(theta1);

    % 좌표 변환 (회전 및 병진 변환 적용)
    sonar.points = [x1, y1, z1];
    zAxis = [0, 0, 1];
    v = cross(zAxis, sonar.D);
    s = norm(v);
    c = dot(zAxis, sonar.D);
    vx = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
    R = eye(3) + vx + vx^2 * ((1 - c) / s^2);
    sonar.points = (R * sonar.points')' + sonar.P;
end

% Find Uncertainty Area Function
function [uncertainty, condNum] = findUncertaintyArea(sensor1, sensor2, sensor3, th)
    % sensor1, sensor2, sensor3: 구조체, 각각 points (Point Cloud), P (위치), D (방향) 등 포함
    % th: 거리 임계값 (겹치는 영역 기준)
    % uncertainty: 세 UUV의 겹치는 영역 (공통 포인트)

    % Step 1: Sensor 1과 Sensor 2 간의 겹치는 영역 계산
    distances12 = pdist2(sensor1.points, sensor2.points);
    [UUV1_matchIdx_12, ~] = find(distances12 < th);
    overlap12 = sensor1.points(UUV1_matchIdx_12, :);

    % Step 2: Sensor 3과 overlap12의 겹치는 영역 계산
    distances123 = pdist2(overlap12, sensor3.points);
    [overlap12_matchIdx, ~] = find(distances123 < th);
    uncertainty = overlap12(overlap12_matchIdx, :);

    % 공분산 행렬 계산
    covMat = cov(uncertainty);

    % 고유 값 및 고유 벡터 계산
    %[eigenVect, eigenVal] = eig(covMat);

    % 컨디션 넘버 계산
    condNum = cond(covMat);
end

%모델 초기화
function target=initializeTarget(heading,v,trajectory)
    target.vel = v; % 속력
    target.heading = [deg2rad(heading(1)) deg2rad(heading(2))]; % 헤딩각
    target.Pos = trajectory(1,:);
    target.PosIdx=1;
    target.State = [target.Pos, ...
    target.vel * cos(target.heading(2)) * cos(target.heading(1)), ...
    target.vel * cos(target.heading(2)) * sin(target.heading(1)), ...
    target.vel * sin(target.heading(2))]'; % 상태 변수
    target.traj=trajectory;
end

function tracker=initializeSensor(v,maxD,noise,target,trajectory)
    tracker.vel = v;
    tracker.Pos = trajectory(1,:);
    tracker.PosIdx=1;
    tracker.heading = [atan2(target.Pos(2) - tracker.Pos(2), target.Pos(1) - tracker.Pos(1)), ...
        atan2(target.Pos(3) - tracker.Pos(3), norm(target.Pos(1:2) - tracker.Pos(1:2)))];
    tracker.State = [tracker.Pos, ...
        tracker.vel * cos(tracker.heading(2)) * cos(tracker.heading(1)), ...
        tracker.vel * cos(tracker.heading(2)) * sin(tracker.heading(1)), ...
        tracker.vel * sin(tracker.heading(2))]';
    % 센서 업데이트 (원뿔 생성)
    tracker.sonar = updateSensor(maxD, noise, tracker.State(1:3), tracker.heading);
    tracker.traj=trajectory;
end
function updateSimulation(model)
    model.PosIdx=model.PosIdx+1;
    model.Pos=model.traj(model.PosIdx,:);
end

궤적 생성 함수
function [True_Target, True_UUV1, True_UUV2, True_UUV3] = Real_trajectory( ...
    UUV1Azimuth1, UUV1Elev1, UUV2Azimuth1, UUV2Elev1,UUV3Azimuth1, UUV3Elev1, ...
    UUV1Azimuth2, UUV1Elev2, UUV2Azimuth2, UUV2Elev2,UUV3Azimuth2, UUV3Elev2, ...
    time, True_Target, True_UUV1, True_UUV2, True_UUV3)

    % 속도와 각도 세팅
    Target_vel = 15;
    UUV1_vel = 20;
    UUV2_vel = 20;
    UUV3_vel = 20;

    % UUV1
    UUV1_Aheading1 = deg2rad(UUV1Azimuth1);
    UUV1_Eheading1 = deg2rad(UUV1Elev1);
    UUV1_Aheading2 = deg2rad(UUV1Azimuth2);
    UUV1_Eheading2 = deg2rad(UUV1Elev2);

    % UUV2
    UUV2_Aheading1 = deg2rad(UUV2Azimuth1);
    UUV2_Eheading1 = deg2rad(UUV2Elev1);
    UUV2_Aheading2 = deg2rad(UUV2Azimuth2);
    UUV2_Eheading2 = deg2rad(UUV2Elev2);

    % UUV3
    UUV3_Aheading1 = deg2rad(UUV3Azimuth1);
    UUV3_Eheading1 = deg2rad(UUV3Elev1);
    UUV3_Aheading2 = deg2rad(UUV3Azimuth2);
    UUV3_Eheading2 = deg2rad(UUV3Elev2);

    % Target
    Target_Aheading1 = deg2rad(90);
    Target_Eheading1 = deg2rad(-20);
    Target_Aheading2 = deg2rad(40);
    Target_Eheading2 = deg2rad(-25);

    % 초기 상태 설정
    UUV1 = [100; 1000; -500; UUV1_vel*cos(UUV1_Eheading1)*cos(UUV1_Aheading1); UUV1_vel*sin(UUV1_Aheading1); UUV1_vel*sin(UUV1_Eheading1)];
     UUV2 = [-100; 1000; -500; UUV2_vel*cos(UUV2_Eheading1)*cos(UUV2_Aheading1); UUV2_vel*sin(UUV2_Aheading1); UUV2_vel*sin(UUV2_Eheading1)];
     UUV3 = [0; 100; -100; UUV3_vel*cos(UUV3_Eheading1)*cos(UUV3_Aheading1); UUV3_vel*sin(UUV3_Aheading1); UUV3_vel*sin(UUV3_Eheading1)];     Target = [0; 10000; -2000; Target_vel*cos(Target_Eheading1)*cos(Target_Aheading1); Target_vel*sin(Target_Aheading1); Target_vel*sin(Target_Eheading1)];
    % 이동 모델
    motion_model = [1 0 0 1 0 0;
                    0 1 0 0 1 0;
                    0 0 1 0 0 1;
                    0 0 0 1 0 0;
                    0 0 0 0 1 0;
                    0 0 0 0 0 1];

    % 시뮬레이션 시작(궤적생성)
    for i = 1 : time
        if i == time/2 % 회전각도 변화
            Target = rotate_model(Target,Target_vel,Target_Aheading2,Target_Eheading2);
            UUV1 = rotate_model(UUV1,UUV1_vel,UUV1_Aheading2,UUV1_Eheading2);
            UUV2 = rotate_model(UUV2,UUV2_vel,UUV2_Aheading2,UUV2_Eheading2);
            UUV3 = rotate_model(UUV3,UUV3_vel,UUV3_Aheading2,UUV3_Eheading2);
        end
    Target = nextmotion_model(Target,motion_model);
    UUV1 = nextmotion_model(UUV1,motion_model);
    UUV2 = nextmotion_model(UUV2,motion_model);
    UUV3 = nextmotion_model(UUV3,motion_model);

    True_Target(i, :) = Target;
    True_UUV1(i, :) = UUV1;
    True_UUV2(i, :) = UUV2;
    True_UUV3(i, :) = UUV3;

    end
end

%다음 dt로의 이동
function x = nextmotion_model(x, f)
    x = f * x;
end
%다음 헤딩각도 변경
function x = rotate_model(x, vel, update_heading, update_elevation)
    heading = update_heading;
    elevation = update_elevation;
    x(4) = vel * cos(heading) * cos(elevation); % X 방향 속도
    x(5) = vel * sin(heading) * cos(elevation); % Y 방향 속도
    x(6) = vel * sin(elevation); % Z 방향 속도
end


EKF
%MAIN EKF 
function [Esti_Target_pos] = Esti_EKF(Target,UUV1,UUV2,UUV3,CDN,Esti_Target_pos,time)%, Esti_Target_pro
    %% EKF Setting
    Esti_P = eye(6); % 오차 공분산 행렬
    Q = eye(6)*0.0001; % 시스템 잡음 공분산 행렬
    % sigma = deg2rad(2);
    % R = (sigma^2) * eye(6);
    Esti_X = [20000; 1000; 0; 15*cos(-20)*cos(70); 15*sin(70); 15*sin(-20)];
    for i = 1 : time
        R = eye(6)*CDN(i,1);
        [Esti_X,Esti_P] = EKF(Esti_X,Esti_P,Q,R,Target(i,:)',UUV1(i,:)',UUV2(i,:)',UUV3(i,:)');
        Esti_Target_pos(i,:) = Esti_X;
        %Esti_Target_pro(i,:) = Esti_P;
    end

end

%EKF알고리즘
function [X,P] = EKF(X,P,Q,R,Target,UUV1,UUV2,UUV3)
    motion_model = [1 0 0 1 0 0; % model
                    0 1 0 0 1 0;
                    0 0 1 0 0 1;
                    0 0 0 1 0 0;
                    0 0 0 0 1 0;
                    0 0 0 0 0 1];
    % EKF돌리기
    %X = nextmotion_model(X,motion_model);
    X = motion_model * X;
    
    % 야코비안 계산4 * 6
    H = measurement_model(X, UUV1, UUV2,UUV3);
    
    % 오차 공분산 예측 6 *6
    P = prediction_step(P, Q);

    % 칼만 이득 계산
    K = P * H' * inv(H * P * H' + R);

    % 실제 경로의 상대 방위각 및 양각 정보(참값)
    z = h(Target, UUV1, UUV2, UUV3);

    % 참값에 노이즈 추가 -> 센서값
    z = z + [deg2rad(randn()); deg2rad(randn()); deg2rad(randn()); deg2rad(randn());deg2rad(randn()); deg2rad(randn())];
    
    % 비선형 측정 방정식 h(X)
    X = X + K * (z - h(X, UUV1, UUV2, UUV3));
    P = (eye(6) - K * H) * P;
end

% measurement_model
function H = measurement_model(target_state, torpedo1_state, torpedo2_state,mship_state)
    dx1 = target_state(1) - torpedo1_state(1);
    dy1 = target_state(2) - torpedo1_state(2);
    dz1 = target_state(3) - torpedo1_state(3);
    dx2 = target_state(1) - torpedo2_state(1);
    dy2 = target_state(2) - torpedo2_state(2);
    dz2 = target_state(3) - torpedo2_state(3);
    dx3 = target_state(1) - mship_state(1);
    dy3 = target_state(2) - mship_state(2);
    dz3 = target_state(3) - mship_state(3);

    % q1, q2는 거리 제곱합
    q1 = dx1^2 + dy1^2 + dz1^2;
    q2 = dx2^2 + dy2^2 + dz2^2;
    q3 = dx3^2 + dy3^2 + dz3^2;

    % 방위각 및 양각에 대한 편미분 계산
    H_bearing1 = [-dy1/q1, dx1/q1, 0, 0, 0, 0];
    H_elevation1 = [-dx1*dz1/(q1*sqrt(dx1^2 + dy1^2)), -dy1*dz1/(q1*sqrt(dx1^2 + dy1^2)), sqrt(dx1^2 + dy1^2)/q1, 0, 0, 0];
    
    H_bearing2 = [-dy2/q2, dx2/q2, 0, 0, 0, 0];
    H_elevation2 = [-dx2*dz2/(q2*sqrt(dx2^2 + dy2^2)), -dy2*dz2/(q2*sqrt(dx2^2 + dy2^2)), sqrt(dx2^2 + dy2^2)/q2, 0, 0, 0];
    
    H_bearing3 = [-dy3/q3, dx3/q3, 0, 0, 0, 0];
    H_elevation3 = [-dx3*dz3/(q3*sqrt(dx3^2 + dy3^2)), -dy3*dz2/(q3*sqrt(dx3^2 + dy3^2)), sqrt(dx3^2 + dy3^2)/q3, 0, 0, 0];

    % 최종 H 행렬
    H = [H_bearing1;
         H_elevation1;
         H_bearing2;
         H_elevation2;
         H_bearing3;
         H_elevation3;
         ];
end

% 타겟의 XYZ 위치 정보를 베어링 및 양각 정보로 변환(센서 데이터화)
function z = h(target_state, torpedo1_state, torpedo2_state,mship_state)
    dx1 = target_state(1) - torpedo1_state(1);
    dy1 = target_state(2) - torpedo1_state(2);
    dz1 = target_state(3) - torpedo1_state(3);
    dx2 = target_state(1) - torpedo2_state(1);
    dy2 = target_state(2) - torpedo2_state(2);
    dz2 = target_state(3) - torpedo2_state(3);
    dx3 = target_state(1) - mship_state(1);
    dy3 = target_state(2) - mship_state(2);
    dz3 = target_state(3) - mship_state(3);

    % 방위각 및 양각 계산
    bearing1 = atan2(dy1, dx1);
    elevation1 = atan(dz1 / sqrt(dx1^2 + dy1^2));
    
    bearing2 = atan2(dy2, dx2);
    elevation2 = atan(dz2 / sqrt(dx2^2 + dy2^2));

    bearing3 = atan2(dy3, dx3);
    elevation3 = atan(dz3 / sqrt(dx3^2 + dy3^2));

    % z는 방위각과 양각을 포함
    z = [bearing1; elevation1; bearing2; elevation2; bearing3; elevation3];
end

% P_prediction
function P = prediction_step(P, Q)
    H = [1 0 0 1 0 0;
         0 1 0 0 1 0;
         0 0 1 0 0 1;
         0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1];
    P = H * P * H' + Q;
end



DOP 계산
function [DOP] = calculatedop(Target, UUV1, UUV2, UUV3)
    % PDOP를 계산하는 함수

    % UUV의 좌표를 설정합니다.
    x1 = UUV1(1); y1 = UUV1(2); z1 = UUV1(3);
    x2 = UUV2(1); y2 = UUV2(2); z2 = UUV2(3);
    x3 = UUV3(1); y3 = UUV3(2); z3 = UUV3(3);
    
    % 목표(Target)의 좌표를 설정합니다.
    xt = Target(1); yt = Target(2); zt = Target(3);
    
    % UUV와 Target 사이의 거리 계산
    d1 = sqrt((x1 - xt)^2 + (y1 - yt)^2 + (z1 - zt)^2);
    d2 = sqrt((x2 - xt)^2 + (y2 - yt)^2 + (z2 - zt)^2);
    d3 = sqrt((x3 - xt)^2 + (y3 - yt)^2 + (z3 - zt)^2);
    
    % 기울기 행렬(G)을 계산합니다.
    G = [ (x1 - xt) / d1, (y1 - yt) / d1, (z1 - zt) / d1;
          (x2 - xt) / d2, (y2 - yt) / d2, (z2 - zt) / d2;
          (x3 - xt) / d3, (y3 - yt) / d3, (z3 - zt) / d3];
    
    % (G' * G)의 역행렬을 계산하여 Q 행렬을 구합니다.
    Q = inv(G' * G);
    
    % PDOP는 Q 행렬의 대각 원소를 이용하여 계산합니다.
    PDOP = sqrt(Q(1,1) + Q(2,2) + Q(3,3));
    
    % 최종 PDOP 값을 반환합니다.
    DOP = PDOP;
end


궤적및 그래프 그리는 함수
%Target UUV1 UUV2 실제 궤적 그리기
function UUV5Target = plot5line(Target,UUV1,UUV2,UUV3,Esti_Tar)
    figure;
    plot3(Target(:, 1), Target(:, 2),Target(:,3), 'k.', 'LineWidth', 2); hold on;
    plot3(UUV1(:, 1), UUV1(:, 2),UUV1(:, 3), 'g-', 'LineWidth', 2);
    plot3(UUV2(:, 1), UUV2(:, 2),UUV2(:, 3), 'b-', 'LineWidth', 2);
    plot3(UUV3(:, 1), UUV3(:, 2),UUV3(:, 3), '-y', 'LineWidth', 2);
    plot3(Esti_Tar(:, 1), Esti_Tar(:, 2),Esti_Tar(:, 3), 'r-', 'LineWidth', 1);

    xlabel('X 위치(m)');
    ylabel('Y 위치(m)');
    zlabel('Z 위치(m)');
    title('실제 궤적(m)'); 
    legend('Target', 'UUV1', 'UUV2','UUV3','추정궤적');
end

%P파라미터 그래프 그리기
% function P_para = plotP(Esti_P)
%     figure;
%     plot(Esti_P);
%     xlabel('time');
%     ylabel('P(파라미터)');
%     title('EKF P 파라미터 추이');
% end

%Esti_Target Target UUV1 UUV2 실제 궤적 그리기
function RealTrajec = plot4line(Target,UUV1,UUV2,UUV3)
    figure;
    plot3(Target(:, 1), Target(:, 2),Target(:,3), 'k.', 'LineWidth', 2); hold on;
    plot3(UUV1(:, 1), UUV1(:, 2),UUV1(:, 3), 'g.', 'LineWidth', 2);
    plot3(UUV2(:, 1), UUV2(:, 2),UUV2(:, 3), 'b.', 'LineWidth', 2);
    plot3(UUV3(:, 1), UUV3(:, 2),UUV3(:,3), 'r.', 'LineWidth', 2);

    xlabel('X 위치');
    ylabel('Y 위치');
    zlabel('Z 위치')
    title('실제 궤적(m)'); 
    legend('Target', 'UUV1', 'UUV2','UUV3');
end


