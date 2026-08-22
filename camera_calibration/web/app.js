"use strict";

const elements = Object.fromEntries([
  "session-input", "new-session", "task-list", "task-total", "task-kicker",
  "task-name", "task-description", "demo-badge", "ros-badge", "preview-grid",
  "preview-updated", "stream-metrics", "pose-count", "pose-board", "pose-title",
  "pose-instruction", "quality-gates", "pose-prev", "pose-done", "pose-next",
  "pose-progress", "pose-progress-copy", "process-state", "gate-summary",
  "check-list", "baseline-field", "baseline-input", "preflight-action",
  "capture-action", "solve-action", "stop-action", "output-path",
  "capture-count", "process-log", "log-count", "toast"
].map((id) => [id, document.getElementById(id)]));

const app = {
  taskId: localStorage.getItem("art-cali:last-task") || "stereo_center",
  poseIndex: 0,
  state: null,
  refreshTimer: null,
  frameTimer: null,
  toastTimer: null,
  loadingAction: false,
};

function timestampSession() {
  const now = new Date();
  const part = (value) => String(value).padStart(2, "0");
  return `${now.getFullYear()}${part(now.getMonth() + 1)}${part(now.getDate())}_` +
    `${part(now.getHours())}${part(now.getMinutes())}${part(now.getSeconds())}`;
}

function sessionName() {
  return elements["session-input"].value.trim();
}

function sessionValid() {
  return /^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$/.test(sessionName());
}

function progressKey() {
  return `art-cali:poses:${sessionName()}:${app.taskId}`;
}

function completedPoses() {
  try {
    const parsed = JSON.parse(localStorage.getItem(progressKey()) || "[]");
    return new Set(Array.isArray(parsed) ? parsed : []);
  } catch (_error) {
    return new Set();
  }
}

function saveCompleted(poses) {
  localStorage.setItem(progressKey(), JSON.stringify([...poses]));
}

function showToast(message, error = false) {
  clearTimeout(app.toastTimer);
  elements.toast.textContent = message;
  elements.toast.className = `toast show${error ? " error" : ""}`;
  app.toastTimer = setTimeout(() => {
    elements.toast.className = "toast";
  }, 4200);
}

function pill(element, label, status) {
  element.textContent = label;
  element.className = `status-pill ${status}`;
}

function formatRate(rate) {
  return Number.isFinite(rate) ? `${rate.toFixed(1)} Hz` : "—";
}

function formatSharpness(value) {
  return Number.isFinite(value) ? value.toFixed(0) : "—";
}

function isSizeGood(stream) {
  return stream.width === 2064 && stream.height === 1544;
}

function isRateGood(stream) {
  return Number.isFinite(stream.rate_hz) && stream.rate_hz >= 8 && stream.rate_hz <= 12;
}

function isSharp(stream) {
  return Number.isFinite(stream.sharpness) && stream.sharpness >= 30;
}

function createNode(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

function renderTaskList() {
  const state = app.state;
  if (!state) return;
  elements["task-list"].replaceChildren();
  let completed = 0;

  state.tasks.forEach((task) => {
    const button = createNode("button", `task-button${task.task_id === app.taskId ? " active" : ""}`);
    button.type = "button";
    button.dataset.task = task.task_id;
    const icon = createNode("span", "task-icon", task.kind === "stereo" ? "ST" : task.short_label.slice(0, 2).toUpperCase());
    const label = createNode("span", "task-label");
    label.append(createNode("strong", "", task.short_label));
    label.append(createNode("span", "", task.kind === "stereo" ? "Pinhole pair" : "Fisheye mono"));
    const done = localStorage.getItem(`art-cali:accepted:${sessionName()}:${task.task_id}`) === "true";
    if (done) completed += 1;
    const dot = createNode("span", `task-dot${done ? " complete" : ""}`);
    button.append(icon, label, dot);
    button.addEventListener("click", () => selectTask(task.task_id));
    elements["task-list"].append(button);
  });
  elements["task-total"].textContent = `${completed} / ${state.tasks.length}`;
}

function renderHeader() {
  const state = app.state;
  const task = state.selected_task;
  elements["task-kicker"].textContent = task.kind === "stereo"
    ? "JOINT STEREO · PINHOLE"
    : "MONOCULAR · FISHEYE / EQUIDISTANT";
  elements["task-name"].textContent = task.label;
  elements["task-description"].textContent = task.description;
  elements["demo-badge"].hidden = !state.demo;

  const activeStreams = task.stream_keys.map((key) => state.streams[key]);
  const online = activeStreams.every((stream) => stream.online);
  const transport = state.transport || { enabled: false, running: true };
  let label = online ? "ROS 图像在线" : "ROS 图像离线";
  if (!online && transport.enabled) {
    label = transport.running ? "等待车端图像" : "车端转发未运行";
  }
  pill(elements["ros-badge"], label, online ? "success" : "danger");
}

function refreshFrames() {
  const state = app.state;
  if (!state) return;
  const token = Date.now();
  state.selected_task.stream_keys.forEach((key) => {
    const image = document.querySelector(`img[data-stream="${key}"]`);
    if (image && state.streams[key].online) {
      image.src = `/api/frame/${encodeURIComponent(key)}.jpg?t=${token}`;
    }
  });
  elements["preview-updated"].textContent = new Date().toLocaleTimeString("zh-CN", { hour12: false });
}

function renderPreviews() {
  const state = app.state;
  const keys = state.selected_task.stream_keys;
  const grid = elements["preview-grid"];
  grid.className = `preview-grid${keys.length === 1 ? " single" : ""}`;
  grid.replaceChildren();

  keys.forEach((key, index) => {
    const stream = state.streams[key];
    const frame = createNode("div", "preview-frame");
    const label = createNode("span", "preview-label", keys.length === 2 ? (index === 0 ? "LEFT" : "RIGHT") : state.selected_task.short_label.toUpperCase());
    const signal = createNode("span", `preview-signal${stream.online ? "" : " offline"}`, stream.online ? "● LIVE" : "● OFFLINE");
    const image = document.createElement("img");
    image.dataset.stream = key;
    image.alt = `${stream.topic} 实时预览`;
    image.addEventListener("error", () => {
      image.alt = `${stream.topic} 暂无预览`;
    });
    frame.append(image, label, signal);
    grid.append(frame);
  });
  refreshFrames();

  const active = keys.map((key) => state.streams[key]);
  const metrics = [
    ["分辨率", active.every(isSizeGood) ? "2064 × 1544" : active.map((s) => `${s.width || "—"} × ${s.height || "—"}`).join(" / ")],
    ["帧率", active.map((stream) => formatRate(stream.rate_hz)).join(" / ")],
    ["清晰度", active.map((stream) => formatSharpness(stream.sharpness)).join(" / ")],
    [keys.length === 2 ? "同步差" : "角点", keys.length === 2
      ? (Number.isFinite(state.streams.stereo_sync_delta_ms) ? `${state.streams.stereo_sync_delta_ms.toFixed(2)} ms` : "—")
      : (active[0].board_detected ? "已识别" : "未识别")]
  ];
  elements["stream-metrics"].replaceChildren(...metrics.map(([label, value]) => {
    const metric = createNode("div", "metric");
    metric.append(createNode("span", "", label), createNode("strong", "", value));
    return metric;
  }));
}

function renderPose() {
  const state = app.state;
  if (!state || !state.poses.length) return;
  app.poseIndex = Math.max(0, Math.min(app.poseIndex, state.poses.length - 1));
  const pose = state.poses[app.poseIndex];
  const complete = completedPoses();
  const done = complete.has(pose.id);
  elements["pose-count"].textContent = `${app.poseIndex + 1} / ${state.poses.length}`;
  elements["pose-title"].textContent = pose.title;
  elements["pose-instruction"].textContent = pose.instruction;
  elements["pose-board"].style.setProperty("--pose-x", `${pose.x}%`);
  elements["pose-board"].style.setProperty("--pose-y", `${pose.y}%`);
  elements["pose-board"].style.setProperty("--pose-scale", pose.scale);
  elements["pose-board"].style.setProperty("--pose-rotate", `${pose.rotate}deg`);
  elements["pose-prev"].disabled = app.poseIndex === 0;
  elements["pose-next"].disabled = app.poseIndex === state.poses.length - 1;
  elements["pose-done"].textContent = done ? "✓ 已完成" : "标记完成";
  elements["pose-done"].className = `button ${done ? "secondary" : "primary"}`;
  elements["pose-progress"].style.width = `${(complete.size / state.poses.length) * 100}%`;
  elements["pose-progress-copy"].textContent = `已完成 ${complete.size} / ${state.poses.length} 个姿态`;

  const streams = state.selected_task.stream_keys.map((key) => state.streams[key]);
  const gates = [
    ["整板角点", streams.every((stream) => stream.board_detected)],
    ["图像清晰", streams.every(isSharp)],
    [state.selected_task.kind === "stereo" ? "双目同步" : "帧率正常",
      state.selected_task.kind === "stereo"
        ? Number.isFinite(state.streams.stereo_sync_delta_ms) && state.streams.stereo_sync_delta_ms <= 2
        : streams.every(isRateGood)]
  ];
  elements["quality-gates"].replaceChildren(...gates.map(([label, pass]) =>
    createNode("span", `quality-chip ${pass ? "pass" : "fail"}`, label)));
}

function checkRow(label, pass) {
  const row = createNode("div", `check-item${pass ? " pass" : ""}`);
  row.append(createNode("span", "check-icon", pass ? "✓" : "!"), createNode("span", "", label));
  return row;
}

function currentStage() {
  const state = app.state;
  const process = state.process;
  const session = state.session;
  const task = state.selected_task;
  const accepted = localStorage.getItem(`art-cali:accepted:${sessionName()}:${task.task_id}`) === "true";
  if (accepted) return "review";
  if (session.result || session.mono_archive) return "review";
  if (process.running && process.label && process.label.endsWith(":calibrate")) return "solve";
  if (session.capture_count > 0 || (process.running && process.label && process.label.endsWith(":capture"))) return "capture";
  if (session.preflight || state.stream_gate.pass) return "preflight";
  return "connect";
}

function renderStages() {
  const names = ["connect", "preflight", "capture", "solve", "review"];
  const selected = names.indexOf(currentStage());
  document.querySelectorAll(".stage").forEach((node, index) => {
    node.classList.toggle("active", index === selected);
    node.classList.toggle("complete", index < selected);
  });
}

function resultStatus() {
  const session = app.state.session;
  if (session.result) return session.result.overall || session.result.status || "RESULT";
  if (session.mono_archive) return session.mono_archive.status || "RESULT";
  return null;
}

function renderControls() {
  const state = app.state;
  const task = state.selected_task;
  const streams = task.stream_keys.map((key) => state.streams[key]);
  const checks = [
    [state.transport && state.transport.enabled ? "所选相机车端转发已运行" : "使用本机 ROS 图像流",
      !state.transport || !state.transport.enabled || state.transport.running],
    ["所需 ROS 图像流在线", streams.every((stream) => stream.online)],
    ["所有图像均为 2064 × 1544", streams.every(isSizeGood)],
    ["所有图像帧率在 8–12 Hz", streams.every(isRateGood)],
  ];
  if (task.kind === "stereo") {
    checks.push(["左右时间戳差不超过 2 ms",
      Number.isFinite(state.streams.stereo_sync_delta_ms) && state.streams.stereo_sync_delta_ms <= 2]);
  }
  checks.push(["使用 11 × 8 方格、70 mm 标定板", true]);
  elements["check-list"].replaceChildren(...checks.map(([label, pass]) => checkRow(label, pass)));
  pill(elements["gate-summary"], state.stream_gate.pass ? "可以开始" : "未通过", state.stream_gate.pass ? "success" : "danger");

  elements["baseline-field"].hidden = task.kind !== "stereo";
  elements["capture-action"].hidden = task.kind !== "stereo";
  elements["solve-action"].textContent = task.kind === "stereo" ? "离线求解" : "打开标定器";
  elements["solve-action"].disabled = app.loadingAction || state.process.running ||
    (task.kind === "stereo" && state.session.capture_count === 0) ||
    (task.kind === "mono" && !state.stream_gate.pass);
  elements["preflight-action"].disabled = app.loadingAction || state.process.running;
  elements["capture-action"].disabled = app.loadingAction || state.process.running || !state.stream_gate.pass || state.session.capture_count > 0;
  elements["stop-action"].disabled = !state.process.running;
  elements["output-path"].textContent = state.session.base;
  elements["output-path"].title = state.session.base;
  elements["capture-count"].textContent = task.kind === "stereo"
    ? `${state.session.capture_count} / ${task.target_captures} 对`
    : (state.session.mono_archive ? `结果 ${state.session.mono_archive.status}` : "等待保存");

  const process = state.process;
  if (process.running) {
    elements["process-state"].className = "process-state running";
    elements["process-state"].lastChild.textContent = `运行中 · ${process.label}`;
  } else if (process.return_code !== null && process.return_code !== 0) {
    elements["process-state"].className = "process-state failed";
    elements["process-state"].lastChild.textContent = `失败 · code ${process.return_code}`;
  } else {
    elements["process-state"].className = "process-state idle";
    elements["process-state"].lastChild.textContent = process.return_code === 0 ? "已完成" : "空闲";
  }

  const logs = process.logs || [];
  elements["process-log"].textContent = logs.length ? logs.join("\n") : "等待操作。";
  elements["log-count"].textContent = `${logs.length} lines`;
  elements["process-log"].scrollTop = elements["process-log"].scrollHeight;

  const status = resultStatus();
  if (status === "PASS") {
    localStorage.setItem(`art-cali:accepted:${sessionName()}:${task.task_id}`, "true");
  }
}

function render() {
  if (!app.state) return;
  renderTaskList();
  renderHeader();
  renderPreviews();
  renderPose();
  renderControls();
  renderStages();
}

async function fetchState({ quiet = false } = {}) {
  if (!sessionValid()) {
    if (!quiet) showToast("Session 名称只能包含字母、数字、点、下划线和连字符。", true);
    return;
  }
  const query = new URLSearchParams({ task: app.taskId, session: sessionName() });
  try {
    const response = await fetch(`/api/state?${query}`, { cache: "no-store" });
    const payload = await response.json();
    if (!response.ok) throw new Error(payload.error || `HTTP ${response.status}`);
    const firstRender = !app.state;
    app.state = payload;
    if (firstRender) render();
    else {
      renderTaskList();
      renderHeader();
      renderPose();
      renderControls();
      renderStages();
      updatePreviewStatus();
    }
  } catch (error) {
    pill(elements["ros-badge"], "前端后端断开", "danger");
    if (!quiet) showToast(error.message, true);
  }
}

function updatePreviewStatus() {
  if (!app.state) return;
  const task = app.state.selected_task;
  task.stream_keys.forEach((key) => {
    const frame = document.querySelector(`img[data-stream="${key}"]`)?.closest(".preview-frame");
    const signal = frame?.querySelector(".preview-signal");
    if (!signal) return;
    const online = app.state.streams[key].online;
    signal.textContent = online ? "● LIVE" : "● OFFLINE";
    signal.classList.toggle("offline", !online);
  });
  const metrics = elements["stream-metrics"].querySelectorAll("strong");
  const streams = task.stream_keys.map((key) => app.state.streams[key]);
  if (metrics.length === 4) {
    metrics[0].textContent = streams.every(isSizeGood) ? "2064 × 1544" : streams.map((s) => `${s.width || "—"} × ${s.height || "—"}`).join(" / ");
    metrics[1].textContent = streams.map((s) => formatRate(s.rate_hz)).join(" / ");
    metrics[2].textContent = streams.map((s) => formatSharpness(s.sharpness)).join(" / ");
    metrics[3].textContent = task.kind === "stereo"
      ? (Number.isFinite(app.state.streams.stereo_sync_delta_ms) ? `${app.state.streams.stereo_sync_delta_ms.toFixed(2)} ms` : "—")
      : (streams[0].board_detected ? "已识别" : "未识别");
  }
}

async function selectTask(taskId) {
  if (app.state?.process.running) {
    showToast("当前任务运行中，请先停止或等待完成。", true);
    return;
  }
  app.taskId = taskId;
  app.poseIndex = 0;
  app.state = null;
  localStorage.setItem("art-cali:last-task", taskId);
  await fetchState();
  if (app.state) render();
}

async function runAction(action) {
  if (!sessionValid()) {
    showToast("请先填写有效的 Session 名称。", true);
    return;
  }
  app.loadingAction = true;
  renderControls();
  const payload = { action, task_id: app.taskId, session: sessionName() };
  if (app.state.selected_task.kind === "stereo" && elements["baseline-input"].value) {
    payload.expected_baseline_m = elements["baseline-input"].value;
  }
  try {
    const response = await fetch("/api/action", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
    if (action === "preflight" && result.started === false) {
      showToast(result.preflight_pass ? "单目流预检通过。" : `预检失败：${result.failures.join("；")}`, !result.preflight_pass);
    } else if (action === "stop") {
      showToast(result.stopped ? "已发送停止信号。" : "当前没有运行中的任务。", false);
    } else {
      showToast("任务已启动，日志会在下方持续更新。", false);
    }
  } catch (error) {
    showToast(error.message, true);
  } finally {
    app.loadingAction = false;
    await fetchState({ quiet: true });
  }
}

function changePose(delta) {
  if (!app.state) return;
  app.poseIndex = Math.max(0, Math.min(app.poseIndex + delta, app.state.poses.length - 1));
  renderPose();
}

function togglePoseDone() {
  const pose = app.state?.poses[app.poseIndex];
  if (!pose) return;
  const complete = completedPoses();
  if (complete.has(pose.id)) complete.delete(pose.id);
  else complete.add(pose.id);
  saveCompleted(complete);
  if (complete.has(pose.id) && app.poseIndex < app.state.poses.length - 1) app.poseIndex += 1;
  renderPose();
}

function bindEvents() {
  elements["new-session"].addEventListener("click", async () => {
    if (app.state?.process.running) {
      showToast("任务运行中，不能切换 Session。", true);
      return;
    }
    elements["session-input"].value = timestampSession();
    localStorage.setItem("art-cali:last-session", sessionName());
    app.poseIndex = 0;
    app.state = null;
    await fetchState();
    if (app.state) render();
  });
  elements["session-input"].addEventListener("change", async () => {
    if (!sessionValid()) {
      showToast("Session 名称格式不正确。", true);
      return;
    }
    localStorage.setItem("art-cali:last-session", sessionName());
    app.poseIndex = 0;
    app.state = null;
    await fetchState();
    if (app.state) render();
  });
  elements["pose-prev"].addEventListener("click", () => changePose(-1));
  elements["pose-next"].addEventListener("click", () => changePose(1));
  elements["pose-done"].addEventListener("click", togglePoseDone);
  elements["preflight-action"].addEventListener("click", () => runAction("preflight"));
  elements["capture-action"].addEventListener("click", () => runAction("capture"));
  elements["solve-action"].addEventListener("click", () => runAction("calibrate"));
  elements["stop-action"].addEventListener("click", () => runAction("stop"));
  document.addEventListener("keydown", (event) => {
    if (event.target instanceof HTMLInputElement) return;
    if (event.key === "ArrowLeft") changePose(-1);
    if (event.key === "ArrowRight") changePose(1);
    if (event.key === " " || event.key === "Enter") {
      event.preventDefault();
      togglePoseDone();
    }
  });
}

async function boot() {
  elements["session-input"].value = localStorage.getItem("art-cali:last-session") || timestampSession();
  bindEvents();
  await fetchState();
  if (!app.state) return;
  render();
  app.refreshTimer = setInterval(() => fetchState({ quiet: true }), 1000);
  app.frameTimer = setInterval(refreshFrames, 700);
}

boot();
