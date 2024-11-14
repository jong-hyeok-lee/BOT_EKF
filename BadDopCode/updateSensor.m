function sonar = updateSensor(maxDistance, noise, position, heading)
    sonar.maxD = maxDistance; % 최대 측정 거리
    sonar.noise = deg2rad(noise); % 각 센서의 measurement Noise (angle)
    sonar.maxR = tan(sonar.noise) * sonar.maxD; % noise 기반 원뿔 반지름 생성
    sonar.numPoints = 30000; % 포인트 수

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