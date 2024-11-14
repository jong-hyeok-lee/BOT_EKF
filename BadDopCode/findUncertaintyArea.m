% Find Uncertainty Area Function
function [uncertainty, eigenVect, eigenVal, condNum] = findUncertaintyArea(sensor1, sensor2, sensor3, th)
    % sensor1, sensor2, sensor3: 구조체, 각각 points (Point Cloud), P (위치), D (방향) 등 포함
    % th: 거리 임계값 (겹치는 영역 기준)
    % uncertainty: 세 UUV의 겹치는 영역 (공통 포인트)
    checkpoint_a=length(sensor1.points)/2;
    checkpoint_b=checkpoint_a+1;
    
    th = mean([norm(sensor1.points(checkpoint_a)-sensor1.points(checkpoint_b))...
            ,norm(sensor2.points(checkpoint_a)-sensor2.points(checkpoint_b)) ...
            ,norm(sensor3.points(checkpoint_a)-sensor3.points(checkpoint_b))]);
    disp(th)
    % Step 1: Sensor 1과 Sensor 2 간의 겹치는 영역 계산
    distances12 = pdist2(sensor1.points, sensor2.points);
    [UUV1_matchIdx_12, ~] = find(distances12 < th);
    overlap12 = sensor1.points(UUV1_matchIdx_12, :);

    % Step 2: Sensor 3과 overlap12의 겹치는 영역 계산
    distances123 = pdist2(overlap12, sensor3.points);
    [overlap12_matchIdx, ~] = find(distances123 < th);
    uncertainty = overlap12(overlap12_matchIdx, :);

    % 공분산 행렬 계산
    covMat = cov(uncertainty);

    % 고유 값 및 고유 벡터 계산
    %[eigenVect, eigenVal] = eig(covMat);

    % 컨디션 넘버 계산
    condNum = cond(covMat);
end
