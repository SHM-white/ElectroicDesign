(() => {
  const state = {
    mode: "BOOT_LOCKED",
    task: null,
    selectionId: null,
    epoch: null,
    reason: "WAITING_FOR_CURRENT_PRESTART_EPOCH",
    route: "IDLE",
    links: { ap: null, car: null, ros: null, vision: null },
    ages: { ap: null, car: null, ros: null, vision: null },
    pendingAck: false,
    carStarted: false
  };
  const $ = (id) => document.getElementById(id);
  const taskButtons = [...document.querySelectorAll("[data-task]")];
  const nowAge = (started) => started === null ? null : Math.max(0, Math.floor((performance.now() - started) / 1000));
  const setLink = (name, fresh, age) => {
    const status = $(`${name}-status`), ageNode = $(`${name}-age`);
    status.textContent = fresh === null ? "UNKNOWN" : fresh ? "OK" : "STALE";
    status.className = fresh === true ? "ok" : fresh === false ? "warn" : "";
    ageNode.textContent = age === null ? "AGE --" : `AGE ${age} s`;
  };
  const render = () => {
    $("state-chip").textContent = state.mode;
    $("authority-state").textContent = state.mode;
    $("selection-id").textContent = state.selectionId ?? "--";
    $("epoch").textContent = state.epoch ?? "--";
    $("reason").textContent = state.reason;
    $("lock-copy").textContent = state.carStarted ? "READ ONLY / 运行中" : state.mode === "PRESTART" ? "READY / 可选择" : "LOCKED / 等待当前预启动周期";
    $("route").textContent = state.route;
    const selectable = state.mode === "PRESTART" && !state.carStarted;
    taskButtons.forEach((button) => {
      button.disabled = !selectable;
      button.classList.toggle("selected", state.task === button.dataset.task);
    });
    $("confirm").disabled = !selectable || state.task === null;
    $("ack").disabled = !state.pendingAck;
    $("start").disabled = state.mode !== "ARMED_READY" || state.carStarted;
    setLink("ap", state.links.ap, state.ages.ap);
    setLink("car", state.links.car, state.ages.car);
    setLink("ros", state.links.ros, state.ages.ros);
    setLink("vision", state.links.vision, state.ages.vision);
  };
  $("authority").addEventListener("click", () => {
    state.mode = "PRESTART"; state.epoch = "01020304"; state.reason = ""; state.links.ros = true; state.ages.ros = 0; render();
  });
  taskButtons.forEach((button) => button.addEventListener("click", () => { state.task = button.dataset.task; render(); }));
  $("confirm").addEventListener("click", () => {
    state.mode = "SELECT_PENDING"; state.selectionId = (state.selectionId ?? 0) + 1; state.pendingAck = true; state.reason = "WAITING_FOR_AUTHORITATIVE_ACK"; render();
  });
  $("ack").addEventListener("click", () => {
    state.mode = "ARMED_READY"; state.pendingAck = false; state.reason = "ACK_COMMITTED_SELECTION"; state.links.ros = true; render();
  });
  $("start").addEventListener("click", () => {
    state.mode = "CAR_RUNNING"; state.carStarted = true; state.route = "START"; state.reason = "READ_ONLY_AFTER_START"; state.links.car = true; render();
  });
  $("reboot").addEventListener("click", () => {
    state.mode = "BOOT_LOCKED"; state.task = null; state.selectionId = null; state.epoch = null; state.pendingAck = false; state.carStarted = false; state.route = "IDLE"; state.reason = "WAITING_FOR_CURRENT_PRESTART_EPOCH"; render();
  });
  window.__hmi = { state, render };
  setInterval(() => {
    $("clock").textContent = new Date(0, 0, 0, 0, 0, Math.floor(performance.now() / 1000)).toTimeString().slice(0, 8);
  }, 1000);
  render();
})();
