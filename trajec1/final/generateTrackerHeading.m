function Tracker = generateTrackerHeading(Target, Tracker)
    % generateTrackerHeading: 타겟과 트래커의 위치를 기반으로 헤딩 계산
     dx = Target.Pose(1) - Tracker.Pose(1);
     dy = Target.Pose(2) - Tracker.Pose(2);
     dz = Target.Pose(3) - Tracker.Pose(3);
    % dx = 0 - Tracker.Pose(1);
    % dy = 0 - Tracker.Pose(2);
    % dz = 0 - Tracker.Pose(3);
    % 방위각과 양각 계산
    bearing = atan2(dy, dx); % 방위각
    elevation = atan(dz / sqrt(dx^2 + dy^2)); % 양각
    Tracker.Heading = [bearing, elevation];
end
