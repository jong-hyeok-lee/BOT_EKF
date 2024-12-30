% Find Uncertainty Area Function
function [uncertainty, condNum] = findUncertaintyArea(UUV1, UUV2, UUV3, th)
    % sensor1, sensor2, sensor3: 구조체, 각각 points (Point Cloud), P (위치), D (방향) 등 포함
    % th: 거리 임계값 (겹치는 영역 기준)
    % uncertainty: 세 UUV의 겹치는 영역 (공통 포인트)

    % Step 1: Sensor 1과 Sensor 2 간의 겹치는 영역 계산
    distances12 = pdist2(UUV1.Sonar.points, UUV2.Sonar.points);
    [UUV1_matchIdx_12, ~] = find(distances12 < th);
    overlap12 = UUV1.Sonar.points(UUV1_matchIdx_12, :);

    % Step 2: Sensor 3과 overlap12의 겹치는 영역 계산
    distances123 = pdist2(overlap12, UUV3.Sonar.points);
    [overlap12_matchIdx, ~] = find(distances123 < th);
    uncertainty = overlap12(overlap12_matchIdx, :);

    % 공분산 행렬 계산
    covMat = cov(uncertainty);

    % 컨디션 넘버 계산
    condNum = cond(covMat);
    if isnan(condNum)
        % Step 1: Sensor 1과 Sensor 3 간의 겹치는 영역 계산
        distances13 = pdist2(UUV1.Sonar.points, UUV3.Sonar.points);
        [UUV1_matchIdx_13, ~] = find(distances13 < th);
        uncertainty = UUV1.Sonar.points(UUV1_matchIdx_13, :);
        
        % 공분산 행렬 계산
        condNum = cond(cov(uncertainty));

        if isnan(condNum)
        % Step 1: Sensor 2과 Sensor 3 간의 겹치는 영역 계산
        distances23 = pdist2(UUV2.Sonar.points, UUV3.Sonar.points);
        [UUV2_matchIdx_23, ~] = find(distances23 < th);
        uncertainty = UUV2.Sonar.points(UUV2_matchIdx_23, :);
        
        % 공분산 행렬 계산
        condNum = cond(cov(uncertainty));

            if isnan(condNum)
                disp("CDN is NAN");
                condNum=10000;
            elseif isinf(condNum)
                disp("CDN is INF");
                condNum=10000;
            end
        end
    end
    % condNum
end