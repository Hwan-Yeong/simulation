// src/SensorAnalysis.js 수정본
export const SensorAnalysis = {
    activeId: null,

    formatContent: function(rawData) {
        if (typeof rawData === 'string') {
            return rawData
                .replace(/---/g, '<hr style="border:0; border-top:1px solid #333; margin:10px 0;">')
                .replace(/\*/g, '<br>📍')
                .replace(/\[/g, '<b style="color:#fff;">[')
                .replace(/\]/g, ']</b>');
        }
        let html = "";
        for (const [key, value] of Object.entries(rawData)) {
            html += `<div style="margin-bottom:8px;"><span style="color:#888;">${key}:</span><br><span style="color:#fff;">${value}</span></div>`;
        }
        return html;
    },

    toggle: function (sensorId, rawData) {
        if (this.activeId === sensorId) {
            this.close();
            return;
        }
        this.close();
        this.create(sensorId, rawData);
    },

    create: function (sensorId, rawData) {
        const div = document.createElement('div');
        div.id = 'sensor-popup-window';
        div.style.cssText = `
            position: fixed; top: 100px; left: 20px;
            width: 320px; background: rgba(10, 10, 10, 0.9);
            color: #00ffcc; border: 1px solid #444;
            padding: 15px; border-radius: 12px; z-index: 10000;
            font-family: 'Consolas', 'Courier New', monospace;
            box-shadow: 0 10px 30px rgba(0,0,0,0.5);
            backdrop-filter: blur(5px);
        `;

        // 데이터 가공 로직
        // 1. 객체일 경우 텍스트로 변환, 2. "*" 기호를 기준으로 줄바꿈 및 아이콘 추가
        let formattedContent = "";

        if (typeof rawData === 'string') {
            formattedContent = rawData
                .replace(/---/g, '<hr style="border:0; border-top:1px solid #333; margin:10px 0;">')
                .replace(/\*/g, '<br>📍') // 별표를 위치 아이콘으로 변경하고 줄바꿈
                .replace(/\[/g, '<b style="color:#fff;">[')
                .replace(/\]/g, ']</b>');
        } else {
            // 객체로 들어올 경우 (기존 코드 대응)
            for (const [key, value] of Object.entries(rawData)) {
                formattedContent += `<div style="margin-bottom:8px;"><span style="color:#888;">${key}:</span><br><span style="color:#fff;">${value}</span></div>`;
            }
        }

        div.innerHTML = `
            <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:10px;">
                <strong style="color:#00ffcc; font-size:14px;">📡 SENSOR REPORT</strong>
                <span id="popup-close-btn" style="cursor:pointer; font-size:18px; color:#666;">&times;</span>
            </div>
            <div style="font-size:12px; line-height:1.6; color:#ddd;">
                ${formattedContent}
            </div>
        `;

        document.body.appendChild(div);
        this.activeId = sensorId;
        document.getElementById('popup-close-btn').onclick = () => this.close();
    },

    // 실시간 업데이트를 위한 함수
    update: function (sensorId, rawData) {
        if (this.activeId === sensorId) {
            const contentDiv = document.getElementById('popup-content');
            if (contentDiv) {
                contentDiv.innerHTML = this.formatContent(rawData);
            }
        }
    },

    close: function () {
        const popup = document.getElementById('sensor-popup-window');
        if (popup) popup.remove();
        this.activeId = null;
    }
};