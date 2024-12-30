%% Function
function Tracker = guidedMotion1(Target, Tracker, dt, maxTurnAngle)
    % guidedMotion: Target을 Tracker가 유도하여 다음 step의 x, y 좌표를 계산
    % 입력:
    %   - Target: [x, y] 형태의 타겟 위치
    %   - Tracker: 구조체로, 현재 Pose와 Velocity를 포함
    %   - dt: 시간 간격
    %   - maxTurnAngle: 최대 선회각 (degree 단위)
    % 출력:
    %   - Tracker: 업데이트된 Tracker 구조체
    
    % 1. Target과 Tracker의 상대 위치 계산
    dx = Target(1) - Tracker.Pose(1);
    dy = Target(2) - Tracker.Pose(2);

    % 2. 헤딩각(bearing) 계산
    targetBearing = atan2d(dy, dx) + deg2rad(randp(1)); % 타겟 방향의 방위각
    currentBearing = atan2d(Tracker.Velocity(2), Tracker.Velocity(1)); % 현재 진행 방향의 방위각
    
    % 3. 두 벡터 간 각도 차 계산
    angleDifference = targetBearing - currentBearing;
    
    % 4. 선회각 제한
    % 각도 차를 -180 ~ 180 범위로 조정
    if angleDifference > 180
        angleDifference = angleDifference - 360;
    elseif angleDifference < -180
        angleDifference = angleDifference + 360;
    end
    
    if angleDifference > maxTurnAngle
        angleDifference = maxTurnAngle;
    elseif angleDifference < -maxTurnAngle
        angleDifference = -maxTurnAngle;
    end

    
    % 5. 새로운 방향 벡터 계산
    newBearing = currentBearing + angleDifference;
    speed = norm(Tracker.Velocity); % 속력 유지
    vx = speed * cosd(newBearing);
    vy = speed * sind(newBearing);
    Tracker.Velocity = [vx, vy];
    
    % 6. 위치 업데이트
    Tracker.Pose(1) = Tracker.Pose(1) + Tracker.Velocity(1) * dt;
    Tracker.Pose(2) = Tracker.Pose(2) + Tracker.Velocity(2) * dt;
end