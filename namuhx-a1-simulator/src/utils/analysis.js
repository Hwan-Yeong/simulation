import * as THREE from 'three';

/**
 * 센서의 바닥 교차점 및 범위를 분석합니다.
 * @param {object} sensor - SensorVisualizer 인스턴스
 * @param {object} params - GUI 파라미터 객체
 * @param {string} paramsKey - 센서 파라미터의 접두사 (e.g., 'tofA')
 * @returns {string} 분석 결과를 담은 텍스트
 */
export function analyzeFloorIntersection(sensor, params, paramsKey) {
    const yawRad = THREE.MathUtils.degToRad(params[`${paramsKey}_yaw`]);
    const pitchRad = THREE.MathUtils.degToRad(params[`${paramsKey}_pitch`]);
    const fovRad = THREE.MathUtils.degToRad(params[`${paramsKey}_fov`]);

    // 센서 절대 위치 (로봇 그룹 회전이 없다고 가정)
    const sensorPos = new THREE.Vector3();
    sensor.pivot.getWorldPosition(sensorPos);
    const sensorHeight = sensorPos.y;

    // 센서가 향하는 방향 벡터
    const direction = new THREE.Vector3(0, 0, 1);
    direction.applyEuler(sensor.pivot.rotation);

    const pitchAngle = Math.abs(pitchRad);

    let analysisText = `--- ${sensor.name} ---\n`;

    const dirY = direction.y;

    if (dirY >= -0.001) {
        analysisText += `  ❌ 바닥과 교차하지 않음 (수평 또는 위쪽 지향)\n`;
        sensor.floorIndicator.visible = false;
        return analysisText;
    }

    // 1. 센서 중심선이 바닥과 만나는 점
    const distanceToFloor = sensorHeight / Math.abs(dirY);

    const intersectionPoint = new THREE.Vector3(
        sensorPos.x + direction.x * distanceToFloor,
        0, // Y=0
        sensorPos.z + direction.z * distanceToFloor
    );

    const planarDistanceToFloor = Math.sqrt(
        (intersectionPoint.x - sensorPos.x) ** 2 + (intersectionPoint.z - sensorPos.z) ** 2
    );

    analysisText += `  * [직선 거리] 바닥까지의 거리: ${distanceToFloor.toFixed(3)} m\n`;

    // 2. FoV 경계가 바닥과 만나는 범위
    const halfFoV = fovRad / 2;

    if (pitchAngle < halfFoV) {
        analysisText += `  ⚠️ FoV가 바닥을 완전히 덮지 않음 (Min Range 계산 불가)\n`;
        sensor.floorIndicator.visible = false;
        return analysisText;
    }

    const angleMin = pitchAngle - halfFoV;
    const distanceMin = sensorHeight / Math.sin(angleMin);
    const planarMin = sensorHeight / Math.tan(angleMin);

    const angleMax = pitchAngle + halfFoV;
    const distanceMax = sensorHeight / Math.sin(angleMax);
    const planarMax = sensorHeight / Math.tan(angleMax);

    analysisText += `  * [직선 거리] 범위 최소 거리 (min) : ${distanceMin.toFixed(3)} m\n`;
    analysisText += `  * [직선 거리] 범위 최대 거리 (MAX) : ${distanceMax.toFixed(3)} m\n`;
    analysisText += '\n';
    analysisText += `  * [수평 거리] 바닥까지의 거리: ${planarDistanceToFloor.toFixed(3)} m\n`;
    analysisText += `  * [수평 거리] 범위 최소 거리 (min) : ${planarMin.toFixed(3)} m\n`;
    analysisText += `  * [수평 거리] 범위 최대 거리 (MAX) : ${planarMax.toFixed(3)} m\n`;

    sensor.floorIndicator.visible = true;
    const maxRadius = planarMax;

    // 바닥 시각화 업데이트
    sensor.floorIndicator.position.x = intersectionPoint.x;
    sensor.floorIndicator.position.z = intersectionPoint.z;
    sensor.floorIndicator.scale.set(maxRadius, maxRadius, 1);

    return analysisText;
}