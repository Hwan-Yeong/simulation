import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GUI } from 'three/addons/libs/lil-gui.module.min.js';
import { RobotModel } from './RobotModel.js';
import { SensorVisualizer } from './SensorVisualizer.js';
import { analyzeFloorIntersection } from './utils/analysis.js';

export class RobotSimulator {
    constructor(container) {
        this.container = container;
        this.params = this._getInitialParams();
        this.sensors = [];
        this.raycaster = new THREE.Raycaster();
        this.pointer = new THREE.Vector2();
        this.plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
        this.measurementPoint = null;

        this._setupScene();
        this._setupLights();
        this._setupEnvironment();
        this._setupRobot();
        this._setupSensors();
        this._setupGUI();
        this._setupEvents();

        this.updateSensors(); // 초기 업데이트
        this.animate();
    }

    _getInitialParams() {
        return {
            heightRobot: 0.68,
            radiusBottom: 0.15,
            radiusMiddle: 0.25,
            radiusTop: 0.15,

            // --- ToF A (Bottom) ---
            tofA_x: -0.076, tofA_y: 0.08, tofA_z: 0.145, tofA_yaw: -15, tofA_pitch: -5,
            tofA_fov: 45, tofA_range: 1.5, tofA_visible: true,

            // --- Ultrasonic (하단 장애물) ---
            ultra_x: 0, ultra_y: 0.11, ultra_z: 0.16, ultra_yaw: 0, ultra_pitch: -22.5,
            ultra_fov: 45, ultra_range: 0.7902, ultra_visible: false,

            // --- ToF B (Top) ---
            tofB_x: 0.076, tofB_y: 0.08, tofB_z: 0.145, tofB_yaw: 15, tofB_pitch: -5,
            tofB_fov: 45, tofB_range: 1.5, tofB_visible: true,

            // TODO: New Sensors ...
        };
    }

    _setupScene() {
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x222222);
        this.scene.fog = new THREE.Fog(0x222222, 2, 5);

        this.camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.01, 100);
        this.camera.position.set(1.5, 1.0, 1.5);

        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.renderer.shadowMap.enabled = true;
        this.renderer.localClippingEnabled = true;
        this.container.appendChild(this.renderer.domElement);

        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.target.set(0, 0.34, 0);
        this.controls.update();
    }

    _setupLights() {
        const ambientLight = new THREE.AmbientLight(0x404040, 2);
        this.scene.add(ambientLight);
        const dirLight = new THREE.DirectionalLight(0xffffff, 1);
        dirLight.position.set(1, 2, 1);
        dirLight.castShadow = true;
        this.scene.add(dirLight);
    }

    _setupEnvironment() {
        const gridHelper = new THREE.GridHelper(5, 50, 0x444444, 0x333333);
        this.scene.add(gridHelper);
        const axesHelper = new THREE.AxesHelper(0.5);
        this.scene.add(axesHelper);

        // 바닥 교차점 시각화를 위한 그룹
        this.floorIntersectionGroup = new THREE.Group();
        this.scene.add(this.floorIntersectionGroup);

        // 측정 지점 가시화
        const pointGeo = new THREE.SphereGeometry(0.03, 16, 16);
        const pointMat = new THREE.MeshBasicMaterial({ color: 0xffff00 });
        this.measurementPoint = new THREE.Mesh(pointGeo, pointMat);
        this.measurementPoint.position.y = 0.005;
        this.measurementPoint.visible = false;
        this.scene.add(this.measurementPoint);
    }

    _setupRobot() {
        this.robotModel = new RobotModel(this.params);
        this.scene.add(this.robotModel);
    }

    _setupSensors() {
        // [새 센서 추가/제거는 이 부분을 수정]
        this.tofA = new SensorVisualizer("[Red] ToF Bottom", 0xff0000);
        this.ultra = new SensorVisualizer("[Green] Ultrasonic", 0x00ff00);
        this.tofB = new SensorVisualizer("[Blue] ToF Top", 0x0000ff);
        // TODO: New Sensors ...

        this.sensors.push(this.tofA, this.ultra, this.tofB);

        this.sensors.forEach(sensor => {
            this.robotModel.add(sensor.pivot); // 로봇 그룹에 센서 피벗 추가
            this.floorIntersectionGroup.add(sensor.floorIndicator); // 바닥 그룹에 인디케이터 추가
        });
    }

    // 센서 업데이트 및 분석
    updateSensors = () => {
        // 로봇 모델 업데이트 (높이/반지름 변경 반영)
        this.robotModel.updateBody();

        // 각 센서 업데이트
        this.tofA.update(this.params, 'tofA');
        this.ultra.update(this.params, 'ultra');
        this.tofB.update(this.params, 'tofB');

        // 분석 결과 업데이트
        let analysisOutput = ' <b>< "센서"로 부터의 거리로 표현 [m] ></b> \n<br>';
        analysisOutput += analyzeFloorIntersection(this.tofA, this.params, 'tofA');
        analysisOutput += '\n';
        analysisOutput += analyzeFloorIntersection(this.ultra, this.params, 'ultra');
        analysisOutput += '\n';
        analysisOutput += analyzeFloorIntersection(this.tofB, this.params, 'tofB');
        document.getElementById('analysis-output').innerHTML = analysisOutput;
    }

    _setupGUI() {
        this.gui = new GUI({ title: '센서 설정 (Controls)' });

        const f0 = this.gui.addFolder('0. 로봇 본체');
        f0.add(this.params, 'heightRobot', 0.4, 1.0).name('로봇 전체 높이(m)').onChange(this.updateSensors); 
        f0.add(this.params, 'radiusBottom', 0.1, 0.3).name('하단 반지름(m)').onChange(this.updateSensors);
        f0.add(this.params, 'radiusMiddle', 0.1, 0.3).name('중단 반지름(m)').onChange(this.updateSensors);
        f0.add(this.params, 'radiusTop', 0.1, 0.3).name('상단 반지름(m)').onChange(this.updateSensors);
        f0.open();

        const sensorMaxY = 1.0;

        // [새 센서 추가/제거 시 GUI 컨트롤도 수정]
        const f1 = this._addSensorGUI(this.gui.addFolder('1. 바닥 감지 ToF (Red)'), 'tofA', 'tofA', sensorMaxY);
        const f2 = this._addSensorGUI(this.gui.addFolder('2. 하단 초음파 (Green)'), 'ultra', 'ultra', sensorMaxY);
        const f3 = this._addSensorGUI(this.gui.addFolder('3. 상단 ToF (Blue)'), 'tofB', 'tofB', sensorMaxY);
        // TODO: New Sensors ...

        f1.open(); f2.open(); f3.open(); // TODO: New Sensors ...

        // 설정값 추출 기능
        const guiActions = {
            exportParams: () => {
                const jsonString = JSON.stringify(this.params, null, 4);
                // ... (이전 코드와 동일한 UI 출력 로직)
                const outputArea = document.createElement('textarea');
                outputArea.value = `// 복사 후 Object.assign(params, ...)에 붙여넣으세요.\n${jsonString}`;
                outputArea.style.position = 'fixed';
                outputArea.style.bottom = '10px';
                outputArea.style.left = '10px';
                outputArea.style.width = '400px';
                outputArea.style.height = '300px';
                outputArea.style.backgroundColor = '#1e1e1e';
                outputArea.style.color = '#00ffcc';
                outputArea.style.border = '1px solid #00ffcc';
                outputArea.style.zIndex = '1000';
                document.body.appendChild(outputArea);
                alert("현재 설정값이 화면 좌측 하단에 생성되었습니다. 내용을 복사하여 코드의 'Object.assign(params, ...)' 부분에 붙여넣으세요.");
            }
        };
        this.gui.add(guiActions, 'exportParams').name('현재 설정값 코드 생성');
    }

    _addSensorGUI(folder, key, name, maxY) {
        folder.add(this.params, `${key}_x`, -0.3, 0.3).name('X 위치(m)').onChange(this.updateSensors);
        folder.add(this.params, `${key}_y`, 0, maxY).name('Y 높이(m)').onChange(this.updateSensors);
        folder.add(this.params, `${key}_z`, -0.3, 0.3).name('Z 위치(m)').onChange(this.updateSensors);
        folder.add(this.params, `${key}_yaw`, -180, 180).name('YAW(deg)').onChange(this.updateSensors);
        folder.add(this.params, `${key}_pitch`, -180, 180).name('PITCH(deg)').onChange(this.updateSensors);
        folder.add(this.params, `${key}_fov`, 5, 90).name('FoV(deg)').onChange(this.updateSensors);
        folder.add(this.params, `${key}_range`, 0.05, 10.0).name('표시 거리(m)').onChange(this.updateSensors);
        folder.add(this.params, `${key}_visible`).name('시각화 활성화').onChange(this.updateSensors);
        return folder;
    }

    _setupEvents() {
        window.addEventListener('resize', this.onWindowResize);
        window.addEventListener('dblclick', this.onDoubleClick);
    }

    onWindowResize = () => {
        this.camera.aspect = window.innerWidth / window.innerHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(window.innerWidth, window.innerHeight);
    }

    onDoubleClick = (event) => {
        this.pointer.x = (event.clientX / window.innerWidth) * 2 - 1;
        this.pointer.y = - (event.clientY / window.innerHeight) * 2 + 1;

        this.raycaster.setFromCamera(this.pointer, this.camera);

        const intersection = new THREE.Vector3();
        if (this.raycaster.ray.intersectPlane(this.plane, intersection)) {

            this.measurementPoint.position.x = intersection.x;
            this.measurementPoint.position.z = intersection.z;
            this.measurementPoint.visible = true;

            let output = `--- 측정 결과 ---\n`;
            output += `측정 지점 (X, Z): (${intersection.x.toFixed(3)}, ${intersection.z.toFixed(3)}) m\n`;
            output += `---------------------\n`;

            this.sensors.forEach(sensor => {
                const sensorAbsPos = new THREE.Vector3();
                sensor.pivot.getWorldPosition(sensorAbsPos);

                const distance = sensorAbsPos.distanceTo(intersection);

                output += `${sensor.name}: ${distance.toFixed(3)} m\n`;
            });

            document.getElementById('measurement-output').textContent = output;

        } else {
            document.getElementById('measurement-output').textContent = "측정 실패: 바닥 평면을 벗어났습니다.";
        }
    }

    animate = () => {
        requestAnimationFrame(this.animate);
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
}