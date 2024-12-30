function uuv = updateState(uuv, velocity_vector,dt)
    % updateState: 현재 위치와 속도를 기반으로 다음 step의 x, y, z 좌표를 반환
    % 1. 현재 위치에 속도 변화량을 더하여 새로운 위치 계산
    uuv.Pose = uuv.Pose + velocity_vector*dt; % 위치 업데이트
    uuv.Sonar = updateSensor(30000, 2, uuv.Pose, uuv.Heading);

end