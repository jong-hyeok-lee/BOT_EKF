% 경보 범위 밖에서의 회피 기동
function Target = calculate_new_heading(Target, direction_to_closest)
    % 방향 벡터 정규화
    direction_to_closest = direction_to_closest / norm(direction_to_closest);
    
    % 2차원으로 변환 (x, y 요소만 사용)
    direction_to_closest = direction_to_closest(1:2);
    
    % 수직 벡터 계산
    perp1 = [-direction_to_closest(2), direction_to_closest(1)];
    perp2 = [direction_to_closest(2), -direction_to_closest(1)];
    
    % 최근접한 물체와 반대 방향 선택
    dot1 = dot(-direction_to_closest, perp1);
    dot2 = dot(-direction_to_closest, perp2);

    speed = norm(Target.Velocity);

    if dot1 < dot2
        Target.Velocity = [speed * perp1 / norm(perp1), 0];
    else
        Target.Velocity = [speed * perp2 / norm(perp2), 0];
    end
end