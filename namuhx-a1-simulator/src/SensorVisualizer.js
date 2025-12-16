import * as THREE from 'three';

const clippingPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0.001);

export class SensorVisualizer {
    /**
     * @param {string} name - 센서 이름 (e.g., "[Red] ToF Bottom")
     * @param {number} color - 센서 색상 (Hex)
     */
    constructor(name, color) {
        this.name = name;
        this.color = color;

        // 1. 센서 전체를 움직이는 Pivot 그룹
        this.pivot = new THREE.Group();

        // 2. 센서 박스 모델
        this.sensorBox = new THREE.Mesh(
            new THREE.BoxGeometry(0.04, 0.02, 0.02),
            new THREE.MeshBasicMaterial({ color: color })
        );
        this.pivot.add(this.sensorBox);

        // 3. FoV 원뿔 모델
        const coneGeo = new THREE.ConeGeometry(1, 1, 32, 1, true);
        coneGeo.rotateX(-Math.PI / 2);
        coneGeo.translate(0, 0, 0.5);

        const coneMat = new THREE.MeshBasicMaterial({
            color: color,
            transparent: true,
            opacity: 0.3,
            side: THREE.DoubleSide,
            depthWrite: false,
            clippingPlanes: [clippingPlane],
            clipShadows: true
        });
        this.fovMesh = new THREE.Mesh(coneGeo, coneMat);
        this.fovMesh.position.z = 0.02;
        this.pivot.add(this.fovMesh);

        // 4. 바닥 교차점 시각화 (RobotSimulator에서 관리하는 그룹에 추가됨)
        this.floorIndicator = new THREE.Mesh(
            new THREE.CircleGeometry(0.1, 32),
            new THREE.MeshBasicMaterial({ color: color, transparent: true, opacity: 0.5, side: THREE.DoubleSide })
        );
        this.floorIndicator.rotation.x = -Math.PI / 2;
        this.floorIndicator.position.y = 0.001;
    }

    /**
     * GUI 파라미터에 따라 센서의 위치, 각도, FoV 시각화를 업데이트합니다.
     * @param {object} params - GUI 파라미터 객체
     * @param {string} paramsKey - 센서 파라미터의 접두사 (e.g., 'tofA')
     */
    update(params, paramsKey) {
        const range = params[`${paramsKey}_range`];
        const fov = params[`${paramsKey}_fov`];
        const visible = params[`${paramsKey}_visible`];

        // 1. 위치 및 회전
        this.pivot.position.set(
            params[`${paramsKey}_x`],
            params[`${paramsKey}_y`],
            params[`${paramsKey}_z`]
        );
        this.pivot.rotation.y = THREE.MathUtils.degToRad(params[`${paramsKey}_yaw`]);
        this.pivot.rotation.x = THREE.MathUtils.degToRad(params[`${paramsKey}_pitch`]);

        // 2. FoV 시각화 크기
        const radius = range * Math.tan(THREE.MathUtils.degToRad(fov) / 2);
        this.fovMesh.scale.set(radius, radius, range);

        // 3. 가시성
        this.pivot.visible = visible;
        if (!visible) this.floorIndicator.visible = false;
    }
}