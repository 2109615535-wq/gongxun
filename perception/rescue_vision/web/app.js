"use strict";
const names = {
  normal_material: "普通物资",
  core_material: "核心物资",
  casualty: "伤员",
  hazard: "危险目标",
  gripper: "夹爪",
  gripper_closed: "闭合夹爪",
  red_material_zone: "红方物资区",
  red_casualty_zone: "红方伤员区",
  blue_material_zone: "蓝方物资区",
  blue_casualty_zone: "蓝方伤员区",
};
const statuses = {
  DISABLED: "未开启",
  CONNECTING: "连接中",
  OK: "观测正常",
  STALE: "观测过期",
  CAMERA_FAULT: "相机异常",
  INFERENCE_FAULT: "推理异常",
  CALIBRATION_ERROR: "标定异常",
  RESOLUTION_MISMATCH: "分辨率不符",
  INVALID_DETECTIONS: "检测输入异常",
  REPLAY_ENDED: "回放结束",
  STOPPED: "已停止",
  UNCONFIGURED: "待配置区域",
  NOT_APPLICABLE: "无核验任务",
  WAITING_TASK: "等待核验任务",
  WAITING_TASK_FRAME: "等待任务观测",
  CONFIRMING: "连续帧确认中",
  MATCH: "目标相符",
  MISMATCH: "物体不符",
  EMPTY: "区域为空",
  HAZARD: "危险目标",
  AMBIGUOUS: "多目标待判断",
  UNCLASSIFIED_OBJECT: "未分类物体待确认",
  MATCH_UNVERIFIED: "相符 · 待实车验证",
  MISMATCH_UNVERIFIED: "不符 · 待实车验证",
  EMPTY_UNVERIFIED: "空区 · 待实车验证",
  CONNECTED: "已连接",
  DISCONNECTED: "未接入",
  ERROR: "异常",
  LOADING: "加载中",
  NOT_LOADED: "未加载",
  READY: "已就绪",
};
const qualities = {
  local_checked: "局部独立检查通过",
  fit_only: "拟合完成 · 待独立检查",
  validated: "标定验证通过",
  near_estimate: "近处外推估算",
  bbox_ground_estimate: "框底边地面估算",
  unavailable: "暂无地面坐标",
};
const byId = (id) => document.getElementById(id);
const label = (value) => statuses[value] || value || "—";
const mm = (value) => (Number.isFinite(value) ? `${value.toFixed(1)} mm` : "—");
let paused = false;
let lastState = null;
let pollRunning = false;
let failures = 0;
let taskFormDirty = false;
let taskSubmitting = false;

function stopStream(role) {
  const image = byId(`${role}-image`);
  image.removeAttribute("src");
  image.hidden = true;
  byId(`${role}-empty`).hidden = false;
}

function renderCamera(role, camera) {
  const badge = byId(`${role}-status`);
  badge.textContent = label(camera.status);
  badge.className = `badge ${camera.status === "OK" ? "good" : camera.status === "DISABLED" ? "" : "bad"}`;
  const image = byId(`${role}-image`);
  if (!paused && camera.stream_available) {
    if (!image.getAttribute("src"))
      image.src = `/stream/${role}.mjpg?t=${Date.now()}`;
    image.hidden = false;
    byId(`${role}-empty`).hidden = true;
  } else {
    stopStream(role);
    byId(`${role}-empty`).querySelector("strong").textContent = paused
      ? "画面已暂停"
      : `等待${role === "upper" ? "上视" : "下视"}摄像头`;
  }
  byId(`${role}-mode`).textContent =
    camera.mode === "video" || camera.info?.kind === "video"
      ? "VIDEO REPLAY"
      : "LIVE INPUT";
  const point = camera.selected?.position_mm;
  byId(`${role}-x`).textContent = mm(point?.x_right);
  byId(`${role}-y`).textContent = mm(point?.y_forward);
  byId(`${role}-age`).textContent = Number.isFinite(camera.age_ms)
    ? `${Math.round(camera.age_ms)} ms`
    : "—";
  const quality =
    camera.selected?.position_quality ||
    camera.calibration_quality ||
    camera.info?.quality;
  byId(`${role}-quality`).textContent = qualities[quality] || "等待加载标定";
  byId(`${role}-inference`).textContent = Number.isFinite(camera.inference_ms)
    ? `检测 ${Number.isFinite(camera.processing_hz) ? camera.processing_hz.toFixed(1) : "—"} Hz · 画面 ${Number.isFinite(camera.info?.preview_hz) ? camera.info.preview_hz.toFixed(1) : "—"} Hz`
    : "推理 —";
  byId(`${role}-inference`).title = Number.isFinite(camera.inference_ms)
    ? `最近处理 ${Math.round(camera.inference_ms)} ms · 采集 ${Number.isFinite(camera.info?.capture_hz) ? camera.info.capture_hz.toFixed(1) : "—"} Hz`
    : "";
  const detections = byId(`${role}-detections`);
  detections.replaceChildren();
  for (const item of camera.detections || []) {
    const chip = document.createElement("span");
    chip.className = `detection ${item.name === "hazard" ? "hazard" : ""}`;
    chip.textContent = `${names[item.name] || item.name} ${Math.round(item.confidence * 100)}%${item.inside_gripper_roi ? " · 夹爪区" : ""}`;
    detections.append(chip);
  }
  if (!detections.childElementCount) detections.textContent = "暂无识别结果";
  byId(`${role}-error`).textContent = camera.error || "";
  byId(`${role}-error`).hidden = !camera.error;
}

function render(state) {
  lastState = state;
  for (const role of ["upper", "lower"])
    renderCamera(role, state.cameras[role]);
  const nav = state.navigation;
  byId("distance-source").textContent =
    nav.source === "upper"
      ? "上视主测距"
      : nav.source === "lower"
        ? "下视辅助测距"
        : "等待观测";
  byId("task-name").textContent = names[state.task.expected] || "等待任务";
  byId("task-source").textContent =
    state.task.source === "controller"
      ? `主控任务 ${state.task.task_id}`
      : state.task.expected
        ? `网页联调 · ${state.task.state || "IDLE"}`
        : `等待目标 · ${state.task.state || "IDLE"}`;
  const gripStatus = state.gripper_config.roi_raw_xyxy
    ? state.gripper.status
    : "UNCONFIGURED";
  byId("grip-state").textContent = label(gripStatus);
  byId("grip-detail").textContent = state.gripper.observed
    ? `识别为${names[state.gripper.observed] || state.gripper.observed} · 连续 ${state.gripper.count} 帧`
    : "下视连续帧确认";
  byId("motion-state").textContent = nav.motion_valid
    ? "观测有效"
    : "控制条件未满足";
  byId("motion-state").className = nav.motion_valid ? "positive" : "warning";
  byId("hazard-state").textContent = nav.hazard_blocked
    ? "检测到危险或未知类别"
    : "测距与核验按验证状态输出";
  byId("hazard-state").className = nav.hazard_blocked ? "danger" : "";
  byId("controller-status").textContent = label(state.controller.status);
  byId("controller-state").textContent =
    state.controller.state ||
    (state.controller.status === "STALE" ? "主控状态已过期" : "等待主控状态");
  byId("controller-detail").textContent = state.controller.state
    ? `任务 ${state.controller.task_id} · 最近回传 ${Math.round(state.controller.age_ms)} ms`
    : "接入状态回传后，这里显示主控实际状态。";
  byId("debug-target").disabled =
    state.controller.status === "CONNECTED" ||
    Boolean(state.controller.session_id);
  byId("debug-state").disabled = byId("debug-target").disabled;
  byId("apply-task").disabled = byId("debug-target").disabled || taskSubmitting;
  if (
    state.task.source === "debug" &&
    !taskFormDirty &&
    document.activeElement !== byId("debug-target")
  ) {
    byId("debug-target").value = state.task.expected || "";
    byId("debug-state").value = state.task.state || "IDLE";
  }
  byId("runtime-tag").textContent =
    state.model.backend === "RKNN" ? "RK3588 NPU · 板端运行" : "电脑端视觉联调";
  byId("model-state").textContent = label(state.model.status);
  byId("model-state").title = state.model.error || "";
  byId("roi-state").textContent = !state.gripper_config.roi_raw_xyxy
    ? "待配置"
    : state.gripper_config.validated
      ? "已配置并验证"
      : "已配置 · 待实车验证";
  byId("serial-state").textContent = label(state.link.status);
  byId("serial-state").title = state.link.error || "";
  byId("last-update").textContent =
    `最近更新 ${new Date().toLocaleTimeString("zh-CN")}`;
}

function disconnected() {
  byId("connection-alert").hidden = false;
  byId("service-status").textContent = "服务连接中断";
  byId("service-dot").className = "dot";
  for (const role of ["upper", "lower"]) {
    stopStream(role);
    renderCamera(role, {
      status: "STALE",
      detections: [],
      stream_available: false,
    });
  }
  byId("motion-state").textContent = "观测已失效";
  byId("motion-state").className = "warning";
  byId("grip-state").textContent = "等待新鲜观测";
  byId("distance-source").textContent = "等待观测";
  byId("task-name").textContent = "任务状态未知";
  byId("controller-state").textContent = "主控状态未知";
  byId("controller-status").textContent = "连接中断";
}

async function poll() {
  if (pollRunning) return;
  pollRunning = true;
  try {
    const response = await fetch("/api/status", {
      cache: "no-store",
      signal: AbortSignal.timeout(1500),
    });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    render(await response.json());
    failures = 0;
    byId("connection-alert").hidden = true;
    byId("service-status").textContent = "视觉服务在线";
    byId("service-dot").className = "dot on";
  } catch (_error) {
    failures += 1;
    disconnected();
  } finally {
    pollRunning = false;
    setTimeout(poll, failures ? 1000 : 250);
  }
}

byId("pause").addEventListener("click", () => {
  paused = !paused;
  byId("pause").textContent = paused ? "恢复画面" : "暂停画面";
  if (lastState && !failures) render(lastState);
});
async function applyTask() {
  if (taskSubmitting) return;
  taskFormDirty = true;
  taskSubmitting = true;
  byId("apply-task").disabled = true;
  try {
    const response = await fetch("/api/debug-task", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        expected: byId("debug-target").value || null,
        state: byId("debug-state").value.trim(),
      }),
      signal: AbortSignal.timeout(2000),
    });
    if (!response.ok) throw new Error(await response.text());
    taskFormDirty = false;
    byId("debug-feedback").textContent = "任务状态与目标已更新，等待连续帧观测";
  } catch (error) {
    byId("debug-feedback").textContent = `设置失败 · ${error.message}`;
  } finally {
    taskSubmitting = false;
    byId("apply-task").disabled = false;
  }
}
byId("debug-target").addEventListener("change", applyTask);
byId("apply-task").addEventListener("click", applyTask);
byId("debug-state").addEventListener("input", () => {
  taskFormDirty = true;
});
byId("debug-state").addEventListener("keydown", (event) => {
  if (event.key === "Enter") applyTask();
});

const roiCanvas = byId("roi-canvas");
const roiContext = roiCanvas.getContext("2d");
let roiImage = null;
let pendingRoi = null;
let roiStart = null;

function drawRoi() {
  roiContext.clearRect(0, 0, roiCanvas.width, roiCanvas.height);
  if (roiImage) roiContext.drawImage(roiImage, 0, 0);
  if (pendingRoi) {
    const [left, top, right, bottom] = pendingRoi;
    roiContext.strokeStyle = "#4cd6b0";
    roiContext.lineWidth = 2;
    roiContext.strokeRect(left, top, right - left, bottom - top);
    byId("roi-coordinates").textContent =
      `[${pendingRoi.join(", ")}] · 原图像素`;
  } else {
    byId("roi-coordinates").textContent = "拖动框选区域";
  }
  byId("save-roi").disabled =
    !pendingRoi ||
    pendingRoi[2] <= pendingRoi[0] ||
    pendingRoi[3] <= pendingRoi[1];
}

async function refreshRoi() {
  byId("save-roi").disabled = true;
  roiStart = null;
  try {
    const response = await fetch("/snapshot/lower-raw.jpg", {
      cache: "no-store",
      signal: AbortSignal.timeout(3000),
    });
    if (!response.ok) throw new Error("下视当前没有新鲜画面，请检查视频源");
    const nextImage = await createImageBitmap(await response.blob());
    if (roiImage) roiImage.close();
    roiImage = nextImage;
    roiCanvas.width = roiImage.width;
    roiCanvas.height = roiImage.height;
    pendingRoi = lastState?.gripper_config.roi_raw_xyxy?.slice() || null;
    drawRoi();
  } catch (error) {
    if (roiImage) roiImage.close();
    roiImage = null;
    pendingRoi = null;
    drawRoi();
    byId("roi-coordinates").textContent = error.message;
  }
}

function roiPoint(event) {
  const bounds = roiCanvas.getBoundingClientRect();
  return [
    Math.round(
      Math.max(
        0,
        Math.min(
          roiCanvas.width,
          ((event.clientX - bounds.left) * roiCanvas.width) / bounds.width,
        ),
      ),
    ),
    Math.round(
      Math.max(
        0,
        Math.min(
          roiCanvas.height,
          ((event.clientY - bounds.top) * roiCanvas.height) / bounds.height,
        ),
      ),
    ),
  ];
}

roiCanvas.addEventListener("pointerdown", (event) => {
  if (!roiImage) return;
  roiStart = roiPoint(event);
  roiCanvas.setPointerCapture(event.pointerId);
  pendingRoi = [...roiStart, ...roiStart];
  drawRoi();
});
roiCanvas.addEventListener("pointermove", (event) => {
  if (!roiStart) return;
  const point = roiPoint(event);
  pendingRoi = [
    Math.min(roiStart[0], point[0]),
    Math.min(roiStart[1], point[1]),
    Math.max(roiStart[0], point[0]),
    Math.max(roiStart[1], point[1]),
  ];
  drawRoi();
});
roiCanvas.addEventListener("pointerup", () => {
  roiStart = null;
});
roiCanvas.addEventListener("pointercancel", () => {
  roiStart = null;
});

async function saveRoi(roi) {
  try {
    const response = await fetch("/api/gripper-roi", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ roi_raw_xyxy: roi }),
      signal: AbortSignal.timeout(3000),
    });
    if (!response.ok) throw new Error(await response.text());
    byId("roi-feedback").textContent = roi
      ? "区域已保存并生效，请用夹爪内实物核验"
      : "区域已清除";
    byId("roi-dialog").close();
  } catch (error) {
    byId("roi-coordinates").textContent = `保存失败 · ${error.message}`;
  }
}

byId("edit-roi").addEventListener("click", () => {
  byId("roi-dialog").showModal();
  refreshRoi();
});
byId("close-roi").addEventListener("click", () => byId("roi-dialog").close());
byId("refresh-roi").addEventListener("click", refreshRoi);
byId("save-roi").addEventListener("click", () => saveRoi(pendingRoi));
byId("clear-roi").addEventListener("click", () => saveRoi(null));
for (const role of ["upper", "lower"]) {
  byId(`${role}-image`).addEventListener("error", () => stopStream(role));
}
setInterval(() => {
  byId("clock").textContent = new Date().toLocaleTimeString("zh-CN", {
    hour12: false,
  });
}, 1000);
poll();
