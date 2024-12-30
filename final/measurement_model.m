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
    H_elevation3 = [-dx3*dz3/(q3*sqrt(dx3^2 + dy3^2)), -dy3*dz3/(q3*sqrt(dx3^2 + dy3^2)), sqrt(dx3^2 + dy3^2)/q3, 0, 0, 0];

    % 최종 H 행렬
    H = [H_bearing1;
         H_elevation1;
         H_bearing2;
         H_elevation2;
         H_bearing3;
         H_elevation3;
         ];
end