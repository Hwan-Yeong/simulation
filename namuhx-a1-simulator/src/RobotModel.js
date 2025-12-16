import * as THREE from 'three';

// 로봇 본체 파츠의 고정 상수 (m)
const H_BOTTOM = 0.10;
const H_TOP = 0.03;
const GAP_H = 0.03;
const ROBOT_GROUND_OFFSET = 0.02;

export class RobotModel extends THREE.Group {
    constructor(initialParams) {
        super();
        this.params = initialParams; // params 객체 참조
        this.bodyMat = new THREE.MeshStandardMaterial({ color: 0xffffff, transparent: true, opacity: 0.8 });
        this.robotBottom = null;
        this.robotMiddle = null;
        this.robotTop = null;
        this.rgbCamera = null;

        this.updateBody(); // 초기 모델링
    }

    /**
     * GUI 파라미터에 따라 로봇 본체 모델을 업데이트합니다.
     */
    updateBody() {
        // 기존 매쉬 제거
        this.clear(); // THREE.Group의 모든 자식 제거

        const ROBOT_MAX_H = this.params.heightRobot;
        const H_MIDDLE = ROBOT_MAX_H - (ROBOT_GROUND_OFFSET + H_BOTTOM + GAP_H + H_TOP);

        // H_MIDDLE이 0보다 작으면 모델링 실패. 최소 높이 보장 필요.
        if (H_MIDDLE < 0.01) {
             console.error("Warning: Robot height is too low for current part sizes.");
             return; // 모델링 중단
        }

        // 파트별 시작 Y 위치 (바닥으로부터의 중심점) 재계산
        const Y_BOTTOM = ROBOT_GROUND_OFFSET + H_BOTTOM / 2;
        const Y_MIDDLE = ROBOT_GROUND_OFFSET + H_BOTTOM + GAP_H + (H_MIDDLE / 2);
        const Y_TOP = ROBOT_GROUND_OFFSET + H_BOTTOM + GAP_H + H_MIDDLE + (H_TOP / 2);

        const rB = this.params.radiusBottom;
        const rM = this.params.radiusMiddle;
        const rT = this.params.radiusTop;

        // 1. 하단부
        const rM_B = rB + (rM - rB) * (Y_BOTTOM / (ROBOT_MAX_H / 2));
        const bottomGeo = new THREE.CylinderGeometry(rM_B, rB, H_BOTTOM, 32);
        this.robotBottom = new THREE.Mesh(bottomGeo, this.bodyMat);
        this.robotBottom.position.y = Y_BOTTOM;
        this.robotBottom.castShadow = true;
        this.add(this.robotBottom);

        // 2. 중간부
        const rM_T = rM + (rT - rM) * (H_MIDDLE / (H_MIDDLE + H_TOP));
        const middleGeo = new THREE.CylinderGeometry(rM_T, rM_B, H_MIDDLE, 32);
        this.robotMiddle = new THREE.Mesh(middleGeo, this.bodyMat);
        this.robotMiddle.position.y = Y_MIDDLE;
        this.robotMiddle.castShadow = true;
        this.add(this.robotMiddle);

        // 3. 상단부
        const topGeo = new THREE.CylinderGeometry(rT, rM_T, H_TOP, 32);
        this.robotTop = new THREE.Mesh(topGeo, this.bodyMat);
        this.robotTop.position.y = Y_TOP;
        this.robotTop.castShadow = true;
        this.add(this.robotTop);

        // 4. RGB 카메라 (센서가 아닌 본체 부속품)
        const rgbCameraGeo = new THREE.CylinderGeometry(0.01, 0.01, 0.03, 32);
        const rgbCameraMat = new THREE.MeshBasicMaterial({ color: 0x333333 });
        this.rgbCamera = new THREE.Mesh(rgbCameraGeo, rgbCameraMat);

        const rgbCamY = ROBOT_MAX_H - H_TOP / 2 - 0.05;

        this.rgbCamera.position.set(0, rgbCamY, rT + 0.001);
        this.rgbCamera.rotation.x = THREE.MathUtils.degToRad(180.0 - (90.0 - 25.0));
        this.add(this.rgbCamera);
    }
}