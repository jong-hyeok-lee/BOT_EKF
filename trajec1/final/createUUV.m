function uuv = createUUV(initialstate, heading)
    % createUUV : 각 수중 운동체의 초기 상태를 정의후 수중 운동체 객체 생성
    uuv.InitialState = initialstate; % [x y z vx vy vz]
    uuv.Pose = uuv.InitialState(1:3); % [x, y, z]
    uuv.Velocity = initialstate(4:6); % [vx, vy, vz] 초기값
    uuv.Heading = heading; % [bearing, elevation]
    uuv.Sonar = updateSensor(15000, 2, uuv.Pose, uuv.Heading);
end