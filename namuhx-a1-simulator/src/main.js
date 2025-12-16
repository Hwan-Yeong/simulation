import { RobotSimulator } from './RobotSimulator.js';

// Three.js 라이브러리를 CDN 대신 NPM을 통해 로드한다고 가정합니다.
// 만약 CDN을 계속 사용하려면, index.html의 script type="module"을 수정해야 합니다.
// NPM 환경에서는 import map이 필요 없습니다.

document.addEventListener('DOMContentLoaded', () => {
    // RobotSimulator 클래스가 모든 것을 캡슐화하고 DOM에 렌더링합니다.
    new RobotSimulator(document.body);
});