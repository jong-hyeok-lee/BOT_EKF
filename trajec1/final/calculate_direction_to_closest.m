% 물체 4와 가장 가까운 물체의 방향을 계산하는 함수
function [direction_to_closest, closest_idx] = calculate_direction_to_closest(pos4, pos1, pos2, pos3)
    % 거리 계산
    distance1 = norm(pos1 - pos4);
    distance2 = norm(pos2 - pos4);
    distance3 = norm(pos3 - pos4);

    % 가장 가까운 물체 결정
    [~, closest_idx] = min([distance1, distance2, distance3]);

    % 방향 계산
    if closest_idx == 1
        direction_to_closest = pos1 - pos4;
    elseif closest_idx == 2
        direction_to_closest = pos2 - pos4;
    else
        direction_to_closest = pos3 - pos4;
    end

    
end