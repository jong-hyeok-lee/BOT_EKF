%% EKF알고리즘
function [X,P] = EKF(X,P,Q,R,Target,UUV1,UUV2,UUV3,dt)
    motion_model = [1 0 0 dt 0 0; % model
                    0 1 0 0 dt 0;
                    0 0 1 0 0 dt;
                    0 0 0 1 0 0;
                    0 0 0 0 1 0;
                    0 0 0 0 0 1];
    % EKF돌리기
    %X = nextmotion_model(X,motion_model);
    X = motion_model * X;
    
    % 야코비안 계산4 * 6
    H = measurement_model(X, UUV1, UUV2,UUV3);
    
    % 오차 공분산 예측 6 *6
    P = prediction_step(P, Q,dt);

    % 칼만 이득 계산
    K = P * H' * inv(H * P * H' + R);

    % 실제 경로의 상대 방위각 및 양각 정보(참값)
    z = h(Target, UUV1, UUV2, UUV3);
    
    % 참값에 노이즈 추가 -> 센서값
    p=1;%sensor noise scale
    z = z + [deg2rad(randp(p)); deg2rad(randp(p)); deg2rad(randp(p)); deg2rad(randp(p));deg2rad(randp(p)); deg2rad(randp(p))];
    
    % 비선형 측정 방정식 h(X)
    X = X + K * (z - h(X, UUV1, UUV2, UUV3));
    P = (eye(6) - K * H) * P;
end