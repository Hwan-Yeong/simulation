// vite.config.js
import { defineConfig } from 'vite';

export default defineConfig({
  // Vite의 기본 설정 객체입니다.
  // 이 프로젝트는 HTML 파일을 엔트리 포인트로 사용하므로, 특별한 설정 없이 비워둡니다.
  // Three.js 프로젝트에서 일반적으로 필요한 설정은 아래와 같습니다.

  // base: '/namuhx-a1-simulator/', // GitHub Pages 배포 시에 필요

  build: {
    // 빌드 출력 폴더 (npm run build 시)
    outDir: 'dist',
  },
});