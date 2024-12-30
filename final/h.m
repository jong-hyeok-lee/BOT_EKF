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