document.addEventListener('DOMContentLoaded', function() {
    const socket = io({
        reconnection: true,
        reconnectionAttempts: 5,
        reconnectionDelay: 1000,
    });

    // --- DOM Elements ---
    const connectionStatus = document.getElementById('connection-status');
    const logContainer = document.getElementById('log-container');
    const resetPoseBtn = document.getElementById('reset-pose-btn');
    
    // Status
    const refreshStatusBtn = document.getElementById('refresh-status-btn');
    const rightArmList = document.getElementById('right-arm-list');
    const leftArmList = document.getElementById('left-arm-list');

    // Recording
    const recordFilenameInput = document.getElementById('record-filename');
    const recordArmsSelect = document.getElementById('record-arms');
    const startRecordBtn = document.getElementById('start-record-btn');
    const stopRecordBtn = document.getElementById('stop-record-btn');
    const recordingInfo = document.getElementById('recording-info');

    // Playback
    const playFilenameSelect = document.getElementById('play-filename-select');
    const playbackSpeedInput = document.getElementById('playback-speed');
    const startPlaybackBtn = document.getElementById('start-playback-btn');
    const stopPlaybackBtn = document.getElementById('stop-playback-btn');
    const playbackProgressBar = document.querySelector('#playback-progress .progress-bar');

    // Filtering
    const filterFileSelect = document.getElementById('filter-file-select');
    const filterFileBtn = document.getElementById('filter-file-btn');
    const smoothingFactorInput = document.getElementById('smoothing-factor');
    const deadzoneThresholdInput = document.getElementById('deadzone-threshold');

    // Sync
    const syncModeSelect = document.getElementById('sync-mode');
    const startSyncBtn = document.getElementById('start-sync-btn');
    const stopSyncBtn = document.getElementById('stop-sync-btn');

    // Power
    const powerTargetSelect = document.getElementById('power-target');
    const powerLoadBtn = document.getElementById('power-load-btn');
    const powerUnloadBtn = document.getElementById('power-unload-btn');

    // File List
    const actionFilesList = document.getElementById('action-files-list');
    const refreshFilesBtn = document.getElementById('refresh-files-btn');
    const contextMenu = document.getElementById('context-menu');
    let selectedFile = null;

    // Servo Fine-Tuning
    const leftArmSliders = document.getElementById('left-arm-sliders');
    const rightArmSliders = document.getElementById('right-arm-sliders');

    // --- Utility Functions ---
    function addLog(message, level = 'info') {
        const entry = document.createElement('div');
        entry.classList.add('log-entry', `level-${level}`);
        
        const timestamp = new Date().toLocaleTimeString();
        const timeEl = document.createElement('span');
        timeEl.className = 'timestamp';
        timeEl.textContent = `[${timestamp}]`;
        
        const msgEl = document.createElement('span');
        msgEl.className = 'message';
        msgEl.textContent = message;

        entry.appendChild(timeEl);
        entry.appendChild(msgEl);
        
        // 将新日志插入到顶部，而不是底部
        logContainer.insertBefore(entry, logContainer.firstChild);
        
        // 限制日志条数，避免积累太多（可选）
        const maxLogEntries = 100;
        while (logContainer.children.length > maxLogEntries) {
            logContainer.removeChild(logContainer.lastChild);
        }
    }

    function updateArmStatus(arm, statusData) {
        const list = arm === 'right' ? rightArmList : leftArmList;
        list.innerHTML = '';
        if (Object.keys(statusData).length === 0) {
            list.innerHTML = '<li>无数据或未连接</li>';
            return;
        }
        for (const [id, data] of Object.entries(statusData)) {
            const li = document.createElement('li');
            const pos = data.pos;
            const volt = data.volt;
            const temp = data.temp;
            const load = data.load; // 新增：获取负载

            const angle = pos !== null ? ((pos / 1000) * 240).toFixed(1) : 'N/A';
            const voltStr = volt !== null ? `${volt.toFixed(2)}V` : 'N/A';
            const tempStr = temp !== null ? `${temp}°C` : 'N/A';
            const loadStr = load !== null ? `${load}` : 'N/A'; // 新增：负载字符串

            // 新增：堵转检测逻辑
            if (load !== null && load > 800) { // 假设负载大于800为堵转
                li.style.backgroundColor = 'rgba(255, 59, 48, 0.2)';
                li.style.color = '#ff3b30';
                li.title = '检测到可能发生堵转！';
            }

            li.innerHTML = `
                <span class="servo-id">ID ${id}:</span>
                <span class="servo-pos">${pos !== null ? pos : 'N/A'} (${angle}°)</span>
                <span class="servo-details">${voltStr} / ${tempStr} / 负载:${loadStr}</span>
            `;
            list.appendChild(li);
        }
    }

    function updateActionFiles(files) {
        // Clear all lists
        actionFilesList.innerHTML = '';
        playFilenameSelect.innerHTML = '';
        filterFileSelect.innerHTML = '';

        if (files && files.length > 0) {
            files.forEach(file => {
                // Populate the main file list
                const li = document.createElement('li');
                li.textContent = file;
                li.dataset.filename = file;
                actionFilesList.appendChild(li);

                // Populate the playback dropdown
                const playOption = document.createElement('option');
                playOption.value = file;
                playOption.textContent = file;
                playFilenameSelect.appendChild(playOption);

                // Populate the filter dropdown
                const filterOption = document.createElement('option');
                filterOption.value = file;
                filterOption.textContent = file;
                filterFileSelect.appendChild(filterOption);
            });
        } else {
            actionFilesList.innerHTML = '<li>没有找到动作文件。</li>';
            const defaultOption = '<option>无文件</option>';
            playFilenameSelect.innerHTML = defaultOption;
            filterFileSelect.innerHTML = defaultOption;
        }
    }

    function createServoSliders(arm, statusData) {
        const container = arm === 'left' ? leftArmSliders : rightArmSliders;
        container.innerHTML = ''; // Clear existing sliders

        if (Object.keys(statusData).length === 0) {
            container.innerHTML = '<p>无数据</p>';
            return;
        }

        // Sort by servo ID
        const sortedIds = Object.keys(statusData).sort((a, b) => a - b);

        for (const id of sortedIds) {
            const data = statusData[id];
            const pos = data.pos !== null ? data.pos : 500; // Default to center if null

            const sliderGroup = document.createElement('div');
            sliderGroup.className = 'slider-group';

            const label = document.createElement('label');
            label.textContent = `ID ${id}`;
            label.htmlFor = `${arm}-servo-${id}`;

            const slider = document.createElement('input');
            slider.type = 'range';
            slider.id = `${arm}-servo-${id}`;
            slider.min = 0;
            slider.max = 1000;
            slider.value = pos;

            const valueDisplay = document.createElement('span');
            valueDisplay.className = 'slider-value';
            valueDisplay.textContent = pos;

            slider.addEventListener('input', () => {
                valueDisplay.textContent = slider.value;
            });

            slider.addEventListener('change', () => {
                const newPos = parseInt(slider.value, 10);
                addLog(`精调 ${arm} 臂舵机 #${id} 到位置 ${newPos}`);
                socket.emit('set_servo_position', {
                    arm: arm,
                    servo_id: parseInt(id, 10),
                    position: newPos
                });
            });

            sliderGroup.appendChild(label);
            sliderGroup.appendChild(slider);
            sliderGroup.appendChild(valueDisplay);
            container.appendChild(sliderGroup);
        }
    }

    // --- Socket.IO Event Handlers ---
    socket.on('connect', () => {
        // This event is now lightweight and non-blocking.
    });

    socket.on('connection_established', () => {
        connectionStatus.textContent = '已连接';
        connectionStatus.className = 'status-connected';
        addLog('✅ 成功连接到服务器。');
    });

    socket.on('disconnect', () => {
        connectionStatus.textContent = '已断开';
        connectionStatus.className = 'status-disconnected';
        addLog('🔌 与服务器断开连接。', 'error');
    });
    
    socket.on('connect_error', (err) => {
        addLog(`连接错误: ${err.message}`, 'error');
    });

    socket.on('init_complete', (data) => {
        if (data.success) {
            addLog('⚙️ 硬件初始化成功！', 'success');
            // 初始化成功后，自动获取一次状态和文件列表
            socket.emit('get_status');
            socket.emit('get_action_files');
        } else {
            addLog(`❌ 硬件初始化失败: ${data.message}`, 'error');
        }
    });

    socket.on('force_refresh', () => {
        addLog('正在强制刷新前端状态...');
        socket.emit('get_status');
        socket.emit('get_action_files');
    });

    socket.on('status_update', (data) => {
        addLog(data.msg, data.level || 'info');
    });

    socket.on('arm_status', (data) => {
        updateArmStatus('right', data.right);
        updateArmStatus('left', data.left);
        // Also update sliders when arm status is received
        createServoSliders('right', data.right);
        createServoSliders('left', data.left);
    });

    socket.on('action_files_list', (data) => {
        updateActionFiles(data.files);
    });

    socket.on('action_files_updated', (data) => {
        addLog('文件列表已更新。');
        updateActionFiles(data.files);
    });

    socket.on('rename_failed', (data) => {
        alert(`重命名失败: ${data.error}`);
    });

    socket.on('recording_failed', (data) => {
        alert(`开始录制失败: ${data.error}`);
    });

    socket.on('recording_started', (data) => {
        addLog(`开始录制到文件: ${data.filename}`, 'info');
        startRecordBtn.disabled = true;
        stopRecordBtn.disabled = false;
        recordingInfo.textContent = '录制中...';
    });

    socket.on('recording_progress', (data) => {
        recordingInfo.textContent = `录制中... ${data.time} / ${data.frames} 帧`;
    });

    socket.on('recording_stopped', () => {
        startRecordBtn.disabled = false;
        stopRecordBtn.disabled = true;
        recordingInfo.textContent = '';
        addLog('录制已停止。');
    });

    socket.on('playback_started', (data) => {
        addLog(`开始播放文件: ${data.filename}`, 'info');
        startPlaybackBtn.disabled = true;
        stopPlaybackBtn.disabled = false;
        playbackProgressBar.style.width = '0%';
    });

    socket.on('playback_progress', (data) => {
        const progress = (data.current / data.total) * 100;
        playbackProgressBar.style.width = `${progress}%`;
        playbackProgressBar.textContent = `${data.current}/${data.total}`;
    });

    socket.on('playback_stopped', () => {
        startPlaybackBtn.disabled = false;
        stopPlaybackBtn.disabled = true;
        playbackProgressBar.style.width = '0%';
        playbackProgressBar.textContent = '';
        addLog('播放已停止。');
    });

    socket.on('sync_started', () => {
        addLog('镜像同步已启动。', 'info');
        startSyncBtn.disabled = true;
        stopSyncBtn.disabled = false;
    });

    socket.on('sync_stopped', () => {
        addLog('镜像同步已停止。', 'info');
        startSyncBtn.disabled = false;
        stopSyncBtn.disabled = true;
    });

    // --- UI Event Listeners ---
    resetPoseBtn.addEventListener('click', () => {
        addLog('正在请求重置位姿...');
        socket.emit('reset_pose');
    });

    refreshStatusBtn.addEventListener('click', () => socket.emit('get_status'));
    refreshFilesBtn.addEventListener('click', () => socket.emit('get_action_files'));

    // Recording
    startRecordBtn.addEventListener('click', () => {
        const filename = recordFilenameInput.value.trim();
        const selectedArms = recordArmsSelect.value;
        let arms = [];
        if (selectedArms === 'both') {
            arms = ['left', 'right'];
        } else {
            arms = [selectedArms];
        }
        
        socket.emit('start_recording', { filename: filename, arms: arms });
    });
    stopRecordBtn.addEventListener('click', () => socket.emit('stop_recording'));

    // Playback
    startPlaybackBtn.addEventListener('click', () => {
        const filename = playFilenameSelect.value;
        if (!filename || filename === '无文件') {
            addLog('请先选择一个动作文件。', 'warning');
            return;
        }
        socket.emit('start_playback', {
            filename: filename,
            speed: parseFloat(playbackSpeedInput.value)
        });
    });
    stopPlaybackBtn.addEventListener('click', () => socket.emit('stop_playback'));

    // Filtering
    filterFileBtn.addEventListener('click', () => {
        const filename = filterFileSelect.value;
        if (!filename || filename === '无文件') {
            addLog('请先选择一个要滤波的文件。', 'warning');
            return;
        }
        socket.emit('filter_file', {
            filename: filename,
            smoothingFactor: parseFloat(smoothingFactorInput.value),
            deadzoneThreshold: parseInt(deadzoneThresholdInput.value)
        });
    });

    // Sync
    startSyncBtn.addEventListener('click', () => {
        socket.emit('start_sync', { mode: syncModeSelect.value });
    });
    stopSyncBtn.addEventListener('click', () => socket.emit('stop_sync'));

    // Power
    powerLoadBtn.addEventListener('click', () => {
        socket.emit('manage_power', { target: powerTargetSelect.value, action: 'load' });
    });
    powerUnloadBtn.addEventListener('click', () => {
        socket.emit('manage_power', { target: powerTargetSelect.value, action: 'unload' });
    });

    // File List Context Menu
    actionFilesList.addEventListener('contextmenu', (e) => {
        const li = e.target.closest('li');
        if (li && li.dataset.filename) {
            e.preventDefault();
            selectedFile = li.dataset.filename;

            // Highlight the item
            const items = actionFilesList.querySelectorAll('li');
            items.forEach(item => item.classList.remove('context-menu-active'));
            li.classList.add('context-menu-active');

            contextMenu.style.display = 'block';
            contextMenu.style.left = `${e.pageX}px`;
            contextMenu.style.top = `${e.pageY}px`;
        }
    });

    document.addEventListener('click', (e) => {
        // Hide context menu if clicked outside
        if (!contextMenu.contains(e.target)) {
            contextMenu.style.display = 'none';
            const items = actionFilesList.querySelectorAll('li');
            items.forEach(item => item.classList.remove('context-menu-active'));
        }
    });

    document.getElementById('menu-delete').addEventListener('click', () => {
        if (selectedFile && confirm(`确定要删除文件 "${selectedFile}"吗？`)) {
            socket.emit('delete_action_file', { filename: selectedFile });
        }
        contextMenu.style.display = 'none';
    });

    document.getElementById('menu-rename').addEventListener('click', () => {
        if (selectedFile) {
            const newName = prompt('请输入新的文件名:', selectedFile);
            if (newName && newName.trim() !== '' && newName !== selectedFile) {
                socket.emit('rename_action_file', {
                    old_filename: selectedFile,
                    new_filename: newName
                });
            }
        }
        contextMenu.style.display = 'none';
    });
});

