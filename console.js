/* ===========================================================================
   SONAIR Inspection Console.

   One socket to the host agent, one place that renders state. The rule
   throughout: nothing raw reaches the screen. Every value is formatted with
   its unit, every fault is a sentence, and diagnostics that only an engineer
   can act on stay in the agent's log window.
   =========================================================================== */
(function () {
  "use strict";
  var $ = function (id) { return document.getElementById(id); };

  var ws = null;
  var state = {
    ur: null, urAge: 0, camAge: 0, imuAge: 0,
    camProbe: null, camNoCamera: false, camError: "",
    located: null, plan: null, detect: null,
    frames: { color: null, depth: null },
    view: "color", jogSpeed: 40
  };

  /* -------------------------------------------------- small helpers ------ */
  function fmt(v, dp, dflt) {
    if (v === null || v === undefined || (typeof v === "number" && !isFinite(v)))
      return dflt === undefined ? "—" : dflt;
    return Number(v).toFixed(dp === undefined ? 1 : dp);
  }
  function esc(t) {
    return String(t === null || t === undefined ? "" : t)
      .replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
  }
  function say(id, text, kind) {
    var el = $(id); if (!el) return;
    el.textContent = text;
    el.className = "note " + (kind || "info");
  }
  function lamp(id, vid, cls, text) {
    var el = $(id); if (el) el.className = "lamp" + (cls ? " " + cls : "");
    var v = $(vid); if (v) v.textContent = text;
  }
  function send(msg) {
    if (!ws || ws.readyState !== WebSocket.OPEN) return false;
    ws.send(JSON.stringify(msg));
    return true;
  }
  function requireLink(noteId) {
    if (ws && ws.readyState === WebSocket.OPEN) return true;
    if (noteId) say(noteId, "Not connected to the host agent. Go to step 1.", "bad");
    return false;
  }

  /* -------------------------------------------------- navigation --------- */
  var PAGES = ["connect", "robot", "inspect", "record"];
  document.querySelectorAll("nav.rail .step").forEach(function (b) {
    b.addEventListener("click", function () {
      document.querySelectorAll("nav.rail .step").forEach(function (o) {
        o.setAttribute("aria-current", o === b ? "true" : "false");
      });
      PAGES.forEach(function (p) {
        var el = $("page-" + p); if (el) el.hidden = (p !== b.dataset.page);
      });
      if (b.dataset.page === "robot") resize3D();
      if (b.dataset.page === "inspect") drawInspect();
    });
  });

  /* -------------------------------------------------- connection --------- */
  $("btnConnect").addEventListener("click", function () {
    var url = ($("wsUrl").value || "").trim();
    if (ws) { try { ws.close(); } catch (e) {} ws = null; }
    say("connMsg", "Connecting…", "info");
    try { ws = new WebSocket(url); }
    catch (e) { say("connMsg", "That address is not valid: " + e.message, "bad"); return; }

    ws.onopen = function () {
      say("connMsg", "Connected to the host agent.", "ok");
      $("btnConnect").textContent = "Reconnect";
      send({ type: "auth", role: "host", site: "UoN" });
      send({ type: "ur_service_status" });
      send({ type: "camera_probe" });
      send({ type: "inspect_status" });
      send({ type: "bench_status" });
    };
    ws.onclose = function () {
      say("connMsg", "Disconnected from the host agent.", "bad");
      lamp("lampRobot", "lampRobotV", "", "Not connected");
      lamp("lampCam", "lampCamV", "", "Not connected");
      lamp("lampImu", "lampImuV", "", "Not connected");
      ws = null;
    };
    ws.onerror = function () {
      say("connMsg", "Could not reach the host agent. Is multimodal_bridge.py "
        + "running on this PC?", "bad");
    };
    ws.onmessage = function (ev) {
      var d; try { d = JSON.parse(ev.data); } catch (e) { return; }
      handle(d);
    };
  });

  /* -------------------------------------------------- inbound ------------ */
  function handle(d) {
    switch (d.type) {
      case "ur_state": state.ur = d.s; state.urAge = performance.now(); renderRobot(d.s); break;
      case "state": state.urAge = performance.now(); break;
      case "tcp_pose": state.urAge = performance.now(); break;
      case "camera_frame": onFrame(d); break;
      case "imu": state.imuAge = performance.now(); onImu(d); break;

      case "ur_service_status": renderUrService(d); break;
      case "ur_cmd_res":
        if (!d.ok) say("jogMsg", plainCmdError(d), "bad");
        break;

      case "camera_probe_res": renderProbe(d); break;
      case "camera_stats_res": renderQuality(d); break;
      case "camera_config_ack": onCamAck(d); break;
      case "camera_option_res": break;

      case "inspect_status_res": renderInspectStatus(d); break;
      case "inspect_locate_res": onLocate(d); break;
      case "inspect_plan_res": onPlan(d); break;
      case "inspect_detect_res": onDetect(d); break;

      case "bench_status": onBenchStatus(d); break;
      case "bench_start_res":
        say("rcMsg", d.ok ? ("Recording " + d.run_id) : ("Could not start: " + d.error),
          d.ok ? "ok" : "bad");
        break;
      case "bench_stop_res":
        say("rcMsg", d.ok ? ("Saved " + d.n + " samples to " + d.run_id)
          : ("Stop failed: " + d.error), d.ok ? "ok" : "bad");
        break;
      case "bench_tap_res":
        if (d.spread_ms !== undefined) {
          $("rcTap").innerHTML = fmt(d.spread_ms, 2) + '<span class="u">ms</span>';
          say("rcMsg", d.spread_ms < 5
            ? "Sensors agree to within " + fmt(d.spread_ms, 2) + " ms."
            : "Sensors are " + fmt(d.spread_ms, 1) + " ms apart. That will show up "
              + "later as a position error. Measure the offset before recording.",
            d.spread_ms < 5 ? "ok" : "warn");
        } else { say("rcMsg", d.error || "No tap detected.", "warn"); }
        break;
    }
  }

  function plainCmdError(d) {
    var m = String(d.msg || "");
    if (/timed out/i.test(m) && /30002/.test(m))
      return "The robot did not accept the command. Check the teach pendant is "
        + "in Remote Control, and that the address is right.";
    if (/envelope/i.test(m))
      return "That position is outside the allowed working area, so it was not sent.";
    if (/protective/i.test(m))
      return "The robot is in protective stop. Clear it on the pendant first.";
    return m;
  }

  /* -------------------------------------------------- robot -------------- */
  function renderUrService(d) {
    var h = d.health || {};
    if (!d.enabled) {
      $("devRobot").className = "dev bad";
      $("devRobotD").textContent = "The agent has not started the robot link.";
      return;
    }
    if (h.connected) {
      $("devRobot").className = "dev ok";
      $("devRobotD").textContent = "Connected, reading " + Math.round(h.rate_hz)
        + " updates a second" + (h.source === "primary-30003"
          ? ". Running on the backup channel, so input/output states and program "
            + "status are not available." : ".");
    } else {
      $("devRobot").className = "dev warn";
      $("devRobotD").textContent = "Trying to reach the robot…";
    }
  }

  var JOINT_NAMES = ["Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"];

  function renderRobot(s) {
    if (!s) return;
    var p = s.actual_TCP_pose || [];
    $("tX").textContent = fmt(p[0] * 1000, 1);
    $("tY").textContent = fmt(p[1] * 1000, 1);
    $("tZ").textContent = fmt(p[2] * 1000, 1);

    $("tForce").innerHTML = fmt(s.tcp_force_magnitude, 1) + '<span class="u">N</span>';
    $("tTorque").innerHTML = fmt(s.tcp_torque_magnitude, 2) + '<span class="u">Nm</span>';
    var ft = $("tileForce");
    ft.className = "tile" + (s.tcp_force_magnitude > 60 ? " bad"
      : s.tcp_force_magnitude > 25 ? " warn" : "");

    $("tMode").textContent = friendlyMode(s.robot_mode_text);
    $("tSafety").textContent = friendlySafety(s.safety_mode_text);
    var temps = s.joint_temperatures || [];
    if (temps.length) $("tTemp").innerHTML = fmt(Math.max.apply(null, temps), 0) + '<span class="u">&deg;C</span>';
    if (s.speed_scaling !== undefined)
      $("tSpeed").innerHTML = fmt(s.speed_scaling * 100, 0) + '<span class="u">%</span>';

    var q = s.actual_q || [], qd = s.actual_qd || [], cur = s.actual_current || [],
        tq = s.target_moment || [];
    $("jointTable").innerHTML = JOINT_NAMES.map(function (n, i) {
      return "<tr><td>" + n + "</td>"
        + '<td class="n">' + fmt(q[i] * 180 / Math.PI, 1) + "&deg;</td>"
        + '<td class="n">' + fmt(qd[i], 3) + " rad/s</td>"
        + '<td class="n">' + fmt(cur[i], 2) + " A</td>"
        + '<td class="n">' + fmt(tq[i], 2) + " Nm</td>"
        + '<td class="n">' + fmt(temps[i], 0) + " &deg;C</td></tr>";
    }).join("");

    var tiles = [];
    if (s.actual_robot_voltage !== undefined)
      tiles.push(["Supply", fmt(s.actual_robot_voltage, 1), "V"]);
    if (s.actual_robot_current !== undefined)
      tiles.push(["Draw", fmt(s.actual_robot_current, 2), "A"]);
    if (s.runtime_state_text) tiles.push(["Program", friendlyProgram(s.runtime_state_text), ""]);
    if (s.actual_tool_accelerometer)
      tiles.push(["Tool tilt sensor", s.actual_tool_accelerometer.map(function (v) {
        return fmt(v, 1); }).join("  "), "m/s&sup2;"]);
    $("elecTiles").innerHTML = tiles.map(function (t) {
      return '<div class="tile"><div class="k">' + t[0] + '</div><div class="v" style="font-size:16px">'
        + t[1] + (t[2] ? '<span class="u">' + t[2] + "</span>" : "") + "</div></div>";
    }).join("");

    renderIo("ioIn", s.digital_inputs, false);
    renderIo("ioOut", s.digital_outputs, true);
    if (window.__setJoints && q.length === 6) window.__setJoints(q);
  }

  function friendlyMode(m) {
    return ({ RUNNING: "Ready", IDLE: "Idle", POWER_OFF: "Powered off",
      POWER_ON: "Powered, brakes on", BOOTING: "Starting up",
      BACKDRIVE: "Hand guiding", CONFIRM_SAFETY: "Confirm safety on pendant"
    })[m] || (m || "—");
  }
  function friendlySafety(m) {
    return ({ NORMAL: "Normal", REDUCED: "Reduced speed",
      PROTECTIVE_STOP: "Protective stop", SAFEGUARD_STOP: "Guard open",
      ROBOT_EMERGENCY_STOP: "E-stop pressed", SYSTEM_EMERGENCY_STOP: "E-stop pressed",
      FAULT: "Fault", VIOLATION: "Safety violation", RECOVERY: "Recovery mode"
    })[m] || (m || "—");
  }
  function friendlyProgram(m) {
    return ({ PLAYING: "Running", PAUSED: "Paused", STOPPED: "Stopped",
      STOPPING: "Stopping", RESUMING: "Resuming" })[m] || m;
  }

  function renderIo(id, bits, clickable) {
    var host = $(id); if (!host || !bits) return;
    host.innerHTML = bits.slice(0, 8).map(function (b, i) {
      return '<button class="btn" data-pin="' + i + '" style="min-width:52px;padding:7px 10px;'
        + (b ? "background:var(--ok);border-color:var(--ok);color:#06210d;font-weight:700" : "")
        + '">' + i + "</button>";
    }).join("");
    if (clickable && !host.dataset.bound) {
      host.dataset.bound = "1";
      host.addEventListener("click", function (ev) {
        var t = ev.target.closest("button[data-pin]"); if (!t) return;
        var on = t.style.background !== "";
        send({ type: "ur_set_dout", pin: Number(t.dataset.pin), value: !on });
      });
    }
  }

  $("btnEstop").addEventListener("click", function () {
    if (!requireLink()) { alert("Not connected to the host agent."); return; }
    send({ type: "ur_estop" });
  });

  /* -------------------------------------------------- jog ---------------- */
  $("jogSpeed").addEventListener("input", function () {
    state.jogSpeed = Number(this.value);
    $("jogSpeedV").textContent = state.jogSpeed + " mm/s";
  });

  function bindPad(id, axes) {
    var disc = $(id); if (!disc) return;
    var knob = disc.querySelector(".knob");
    var active = false, timer = null, vec = [0, 0];

    function at(ev) {
      var r = disc.getBoundingClientRect();
      var cx = r.left + r.width / 2, cy = r.top + r.height / 2;
      var t = ev.touches ? ev.touches[0] : ev;
      var dx = (t.clientX - cx) / (r.width / 2);
      var dy = (t.clientY - cy) / (r.height / 2);
      var m = Math.hypot(dx, dy);
      if (m > 1) { dx /= m; dy /= m; }
      vec = [dx, dy];
      knob.style.left = (50 + dx * 33) + "%";
      knob.style.top = (50 + dy * 33) + "%";
    }
    function stop() {
      active = false;
      if (timer) { clearInterval(timer); timer = null; }
      knob.style.left = "50%"; knob.style.top = "50%";
      vec = [0, 0];
      send({ type: "ur_speedl", xd: [0, 0, 0, 0, 0, 0], a: 1.2, t: 0.2 });
    }
    function start(ev) {
      ev.preventDefault();
      if (!requireLink("jogMsg")) return;
      active = true; at(ev);
      if (timer) clearInterval(timer);
      // Re-send while held. A speedl decays after `t`, so a held joystick with
      // no repeat produces one twitch and then stops, which reads as a fault.
      timer = setInterval(function () {
        if (!active) return;
        var v = state.jogSpeed / 1000;
        var xd = [0, 0, 0, 0, 0, 0];
        xd[axes[0]] += vec[0] * v * (axes[2] || 1);
        xd[axes[1]] += -vec[1] * v * (axes[3] || 1);
        send({ type: "ur_speedl", xd: xd, a: 1.0, t: 0.25 });
      }, 100);
    }
    disc.addEventListener("mousedown", start);
    disc.addEventListener("touchstart", start, { passive: false });
    disc.addEventListener("mousemove", function (e) { if (active) at(e); });
    disc.addEventListener("touchmove", function (e) { if (active) { e.preventDefault(); at(e); } },
      { passive: false });
    ["mouseup", "mouseleave", "touchend", "touchcancel"].forEach(function (e) {
      disc.addEventListener(e, stop);
    });
    window.addEventListener("mouseup", function () { if (active) stop(); });
  }
  bindPad("padXY", [0, 1]);
  bindPad("padZR", [2, 5, 1, 12]);   // Z in m/s; rotation scaled to rad/s

  $("jointCards").innerHTML = JOINT_NAMES.map(function (n, i) {
    return '<div class="jcard"><span class="jn">J' + (i + 1) + '</span>'
      + '<button data-j="' + i + '" data-d="-1">&minus;</button>'
      + '<button data-j="' + i + '" data-d="1">+</button>'
      + '<span class="jval" id="jv' + i + '">' + n + "</span></div>";
  }).join("");

  (function bindJoints() {
    var host = $("jointCards"), timer = null;
    function go(j, dir) {
      if (!requireLink("jogMsg")) return;
      var qd = [0, 0, 0, 0, 0, 0];
      qd[j] = dir * (state.jogSpeed / 400);
      send({ type: "ur_speedj", qd: qd, a: 1.2, t: 0.25 });
    }
    host.addEventListener("mousedown", function (ev) {
      var b = ev.target.closest("button[data-j]"); if (!b) return;
      var j = Number(b.dataset.j), d = Number(b.dataset.d);
      go(j, d);
      timer = setInterval(function () { go(j, d); }, 120);
    });
    ["mouseup", "mouseleave"].forEach(function (e) {
      host.addEventListener(e, function () {
        if (timer) { clearInterval(timer); timer = null; }
        send({ type: "ur_speedj", qd: [0, 0, 0, 0, 0, 0], a: 1.5, t: 0.2 });
      });
    });
    window.addEventListener("mouseup", function () {
      if (timer) { clearInterval(timer); timer = null; }
    });
  })();

  /* -------------------------------------------------- camera ------------- */
  function onFrame(d) {
    state.camAge = performance.now();
    if (d.rgb) loadFrame("color", d.rgb);
    if (d.depth) loadFrame("depth", d.depth);
    var e = $("inspEmpty"); if (e) e.style.display = "none";
  }
  function loadFrame(kind, b64) {
    var img = new Image();
    img.onload = function () {
      state.frames[kind] = img;
      if (kind === state.view) drawInspect();
    };
    img.src = "data:image/jpeg;base64," + b64;
  }

  function onCamAck(d) {
    if (d.vision_available === false) {
      state.camNoCamera = true;
      state.camError = d.vision_error || "";
      renderCamDevice();
    } else { state.camNoCamera = false; }
  }

  function renderProbe(d) {
    state.camProbe = d;
    if (d.vision_available === false) {
      state.camNoCamera = true; state.camError = d.vision_error || "";
    }
    renderCamDevice();
    if (!d.available || !d.profiles) return;

    fillModes("camDepthRes", "camDepthFps", d.profiles.depth, "848x480");
    fillModes("camColorRes", "camColorFps", d.profiles.color, "640x480");
    $("camModeTag").textContent = (d.device && d.device.usb)
      ? ("USB " + d.device.usb) : "—";

    if (d.intrinsics && d.intrinsics.fx) {
      say("camMsg", "Camera measured itself: view angle "
        + fmt(d.intrinsics.hfov_deg, 0) + "° across, "
        + fmt(d.intrinsics.vfov_deg, 0) + "° down. Depth measurements are ready.",
        "ok");
    }
  }

  function fillModes(resId, fpsId, byRes, prefer) {
    var rs = $(resId), fs = $(fpsId);
    if (!rs || !byRes) return;
    var keys = Object.keys(byRes);
    if (!keys.length) { rs.innerHTML = "<option>none offered</option>"; return; }
    rs.disabled = false; fs.disabled = false;
    rs.innerHTML = keys.map(function (k) {
      return '<option value="' + k + '"' + (k === prefer ? " selected" : "") + ">" + k + "</option>";
    }).join("");
    function fps() {
      var list = byRes[rs.value] || [];
      fs.innerHTML = list.map(function (f) { return "<option>" + f + "</option>"; }).join("");
    }
    rs.onchange = fps; fps();
  }

  function renderCamDevice() {
    var d = state.camProbe || {};
    if (state.camNoCamera) {
      $("devCam").className = "dev bad";
      $("devCamD").textContent = "The host agent has no camera support installed, "
        + "so no picture can arrive. An engineer needs to install the camera "
        + "packages and restart the agent.";
      lamp("lampCam", "lampCamV", "bad", "Not available");
      return;
    }
    if (!d.available) {
      $("devCam").className = "dev warn";
      $("devCamD").textContent = d.error
        ? plainCamError(d.error)
        : "Looking for the camera…";
      lamp("lampCam", "lampCamV", "warn", "Not found");
      return;
    }
    var dev = d.device || {};
    $("devCam").className = "dev " + (dev.usb_ok ? "ok" : "warn");
    $("devCamD").innerHTML = esc(dev.name) + ", serial " + esc(dev.serial) + "<br>"
      + (dev.usb_ok ? "Connected on USB " + esc(dev.usb) + "."
        : "<b>On USB " + esc(dev.usb) + " &mdash; use a blue USB 3 port.</b> "
          + "On USB 2 the camera offers far fewer modes.")
      + (dev.has_imu ? " Built-in motion sensor present." : "");
  }

  function plainCamError(e) {
    if (/no RealSense device/i.test(e))
      return "No camera found. Check the cable is in a blue USB 3 port, and that "
        + "the camera appears in RealSense Viewer.";
    if (/resolve/i.test(e))
      return "The camera refused that combination of resolution and frame rate. "
        + "Pick a different one above.";
    return e;
  }

  $("btnCamApply").addEventListener("click", function () {
    if (!requireLink("camMsg")) return;
    send({ type: "camera_config", config: {
      stereo_res: $("camDepthRes").value, stereo_fps: Number($("camDepthFps").value),
      rgb_res: $("camColorRes").value, rgb_fps: Number($("camColorFps").value),
      depth_en: true, rgb_en: true, ir1_en: false, ir2_en: false,
      emitter: $("camLaser").checked ? "laser" : "none"
    }});
    say("camMsg", "Applying… the picture will pause for a second.", "info");
  });

  $("btnCamQuality").addEventListener("click", function () {
    if (!requireLink("camMsg")) return;
    send({ type: "camera_stats" });
  });

  function renderQuality(d) {
    if (!d.available) { say("camMsg", d.error || "Could not measure.", "warn"); return; }
    var pc = function (f) { return Math.round(f * 100) + "%"; };
    $("qFill").textContent = pc(d.fill_all);
    $("qFillRoi").textContent = pc(d.fill_roi);
    $("qFillRoi").parentElement.className = "tile " +
      (d.fill_roi > 0.7 ? "ok" : d.fill_roi > 0.4 ? "warn" : "bad");
    $("qDist").innerHTML = d.roi_median_m !== undefined
      ? fmt(d.roi_median_m * 1000, 0) + '<span class="u">mm</span>' : "—";
    $("qNoise").innerHTML = d.roi_std_mm !== undefined
      ? fmt(d.roi_std_mm, 2) + '<span class="u">mm</span>' : "—";
    say("camMsg", d.fill_roi > 0.7
      ? "Depth looks good where the part sits."
      : "Only " + pc(d.fill_roi) + " of the centre returned a depth reading. "
        + "Machined metal reflects the laser away; try turning the part, "
        + "moving the camera closer, or reducing the light on it.",
      d.fill_roi > 0.7 ? "ok" : "warn");
  }

  function onImu(d) {
    var units = Object.keys(d.units || {});
    if (units.length) {
      lamp("lampImu", "lampImuV", "ok", units.length + " sensor" + (units.length > 1 ? "s" : ""));
      $("devImu").className = "dev ok";
      $("devImuD").textContent = "Reading from: " + units.join(", ") + ".";
    }
  }

  /* -------------------------------------------------- inspection --------- */
  function renderInspectStatus(d) {
    if (!d.available) {
      say("locMsg", "The host agent cannot run inspection. An engineer needs to "
        + "install the vision packages.", "bad");
    } else if (!d.has_handeye) {
      say("locMsg", "The camera position on the arm has not been measured yet, so "
        + "the part can be found in the picture but not in robot coordinates. "
        + "Finding still works; planning a robot path does not.", "warn");
    }
  }

  $("btnLocate").addEventListener("click", function () {
    if (!requireLink("locMsg")) return;
    say("locMsg", "Looking for the part…", "info");
    send({ type: "inspect_locate",
      min_height_mm: Number($("ipHeight").value) || 5,
      max_range_m: Number($("ipRange").value) || 1.2 });
  });

  function onLocate(d) {
    if (!d.ok) {
      state.located = null; setFlow(1);
      say("locMsg", d.error || "Could not find the part.", "warn");
      drawInspect(); return;
    }
    state.located = d;
    $("ipSize").textContent = fmt(d.size_mm[0], 0) + " × " + fmt(d.size_mm[1], 0);
    $("ipHgt").textContent = fmt(d.height_mm, 1);
    $("ipArea").textContent = fmt(d.area_mm2, 0);
    var msg = "Found a part " + fmt(d.size_mm[0], 0) + " by " + fmt(d.size_mm[1], 0)
      + " mm, standing " + fmt(d.height_mm, 1) + " mm above the fixture.";
    if (d.warning) msg += " " + d.warning;
    say("locMsg", msg, d.warning ? "warn" : "ok");
    setFlow(2);
    drawInspect();
  }

  $("btnPlan").addEventListener("click", function () {
    if (!requireLink("planMsg")) return;
    send({ type: "inspect_plan", mode: $("ipMode").value,
      standoff_mm: Number($("ipStandoff").value) || 100,
      spacing_mm: Number($("ipSpacing").value) || 5,
      step_mm: Number($("ipSpacing").value) || 5,
      margin_mm: Number($("ipMargin").value) || 5 });
  });

  function onPlan(d) {
    if (!d.ok) {
      state.plan = null; $("btnRunScan").disabled = true;
      say("planMsg", d.error || "Could not plan a path.", "warn");
      drawInspect(); return;
    }
    state.plan = d;
    var len = 0, w = d.waypoints;
    for (var i = 1; i < w.length; i++) {
      var a = w[i - 1].coords, b = w[i].coords;
      len += Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    }
    $("ipPts").textContent = d.n_waypoints;
    $("ipLen").textContent = fmt(len * 1000, 0);
    var secs = len / Math.max(state.jogSpeed / 1000, 0.005);
    $("ipTime").textContent = secs > 90
      ? fmt(secs / 60, 1) + " min" : fmt(secs, 0) + " s";
    say("planMsg", d.n_waypoints + " points, " + fmt(len * 1000, 0)
      + " mm of travel, sensor held " + fmt(d.standoff_mm, 0)
      + " mm above the part.", "ok");
    $("btnRunScan").disabled = false;
    say("runMsg", "Ready. Check the area is clear before running.", "info");
    setFlow(3);
    drawInspect();
  }

  $("btnDetect").addEventListener("click", function () {
    if (!requireLink("detMsg")) return;
    send({ type: "inspect_detect",
      depth_thresh_mm: Number($("ipDepthTh").value) || 1.5,
      visual_thresh: Number($("ipVisTh").value) || 22,
      min_area_mm2: Number($("ipMinArea").value) || 0.5 });
  });

  function onDetect(d) {
    if (!d.ok) { say("detMsg", d.error || "Could not run the check.", "warn"); return; }
    state.detect = d;
    $("ipNCand").textContent = d.n_total;
    $("ipNSurf").textContent = d.n_surface;
    $("ipNVis").textContent = d.n_visual;
    var rows = d.candidates.slice(0, 25).map(function (c, i) {
      var what = c.channel === "surface"
        ? (c.kind === "depression" ? "Dip in the surface" : "Raised spot")
        : "Dark mark";
      var amount = c.channel === "surface"
        ? (c.deviation_mm > 0 ? "+" : "") + fmt(c.deviation_mm, 1) + " mm"
        : fmt(c.contrast, 0) + " darker";
      var pri = c.both ? "Check first" : c.score > .6 ? "High" : c.score > .3 ? "Medium" : "Low";
      return "<tr><td>" + (i + 1) + "</td><td>" + what + "</td>"
        + '<td class="n">' + fmt(c.area_mm2, 1) + " mm&sup2;</td>"
        + '<td class="n">' + amount + "</td>"
        + '<td class="n">' + pri + "</td></tr>";
    }).join("");
    $("candTable").innerHTML = rows ||
      '<tr><td colspan="5" style="color:var(--text-3)">Nothing stood out at these settings.</td></tr>';
    say("detMsg", d.n_total === 0
      ? "Nothing stood out. That is not a pass — this check cannot see below "
        + "the surface, and anything smaller than about 1 mm is below what the "
        + "camera can resolve."
      : d.n_total + " place" + (d.n_total > 1 ? "s" : "") + " worth checking"
        + (d.n_both ? ", " + d.n_both + " flagged by both methods" : "")
        + ". These are candidates, not confirmed defects.",
      "warn");
    setFlow(4);
    drawInspect();
  }

  $("btnRunScan").addEventListener("click", function () {
    if (!state.plan || !requireLink("runMsg")) return;
    if (!confirm("The arm will move through " + state.plan.n_waypoints
      + " points. Is the area clear?")) return;
    var w = state.plan.waypoints, sent = 0;
    var v = Math.max(0.005, state.jogSpeed / 1000);
    (function next(i) {
      if (i >= w.length) {
        say("runMsg", "Scan finished, " + sent + " points.", "ok"); return;
      }
      var c = w[i].coords;
      if (send({ type: "ur_movel", pose: c, a: 0.3, v: v, r: 0.002 })) sent++;
      say("runMsg", "Running… point " + (i + 1) + " of " + w.length, "info");
      setTimeout(function () { next(i + 1); }, 220);
    })(0);
  });

  function setFlow(n) {
    document.querySelectorAll("#inspFlow .fs").forEach(function (f) {
      var k = Number(f.dataset.fs);
      f.className = "fs" + (k < n ? " done" : k === n ? " active" : "");
    });
  }

  $("btnViewColor").addEventListener("click", function () { state.view = "color"; $("inspViewTag").textContent = "colour"; drawInspect(); });
  $("btnViewDepth").addEventListener("click", function () { state.view = "depth"; $("inspViewTag").textContent = "depth"; drawInspect(); });

  /* -------------------------------------------------- the overlay -------- */
  function drawInspect() {
    var cv = $("inspCanvas"); if (!cv) return;
    var img = state.frames[state.view] || state.frames.color;
    var ctx = cv.getContext("2d");

    if (img) { cv.width = img.naturalWidth; cv.height = img.naturalHeight; }
    ctx.clearRect(0, 0, cv.width, cv.height);
    if (img) ctx.drawImage(img, 0, 0, cv.width, cv.height);
    else { ctx.fillStyle = "#0a0e13"; ctx.fillRect(0, 0, cv.width, cv.height); }

    var L = state.located;
    if (L && L.contour_px && L.contour_px.length > 2) {
      // The outline is in the DEPTH image's pixel grid. When the colour frame
      // is a different size, scale rather than draw it in the wrong place —
      // an overlay that is silently offset is worse than none.
      var sx = L.image_size ? cv.width / L.image_size[0] : 1;
      var sy = L.image_size ? cv.height / L.image_size[1] : 1;
      ctx.save();
      ctx.strokeStyle = "#3fb950"; ctx.lineWidth = Math.max(2, cv.width / 320);
      ctx.beginPath();
      L.contour_px.forEach(function (p, i) {
        var x = p[0] * sx, y = p[1] * sy;
        i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      });
      ctx.closePath(); ctx.stroke();
      ctx.fillStyle = "rgba(63,185,80,.10)"; ctx.fill();

      if (L.centroid_px) {
        var cx = L.centroid_px[0] * sx, cy = L.centroid_px[1] * sy;
        ctx.strokeStyle = "#3fb950"; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(cx - 12, cy); ctx.lineTo(cx + 12, cy);
        ctx.moveTo(cx, cy - 12); ctx.lineTo(cx, cy + 12); ctx.stroke();
      }
      ctx.restore();
    }

    // The planned path lives in robot coordinates. Project it back through the
    // located outline's own bounding box so it lands on the part in the image.
    if (L && state.plan && state.plan.waypoints && L.contour_base_m
        && L.contour_base_m.length > 2 && L.bbox_px) {
      var pts = L.contour_base_m;
      var xs = pts.map(function (p) { return p[0]; });
      var ys = pts.map(function (p) { return p[1]; });
      var bx0 = Math.min.apply(null, xs), bx1 = Math.max.apply(null, xs);
      var by0 = Math.min.apply(null, ys), by1 = Math.max.apply(null, ys);
      var sxp = L.image_size ? cv.width / L.image_size[0] : 1;
      var syp = L.image_size ? cv.height / L.image_size[1] : 1;
      var bb = L.bbox_px;
      function toPx(c) {
        var u = (bx1 - bx0) ? (c[0] - bx0) / (bx1 - bx0) : .5;
        var v = (by1 - by0) ? (c[1] - by0) / (by1 - by0) : .5;
        return [(bb[0] + u * bb[2]) * sxp, (bb[1] + (1 - v) * bb[3]) * syp];
      }
      ctx.save();
      ctx.strokeStyle = "#2f81f7"; ctx.lineWidth = Math.max(1.5, cv.width / 500);
      ctx.globalAlpha = .9;
      ctx.beginPath();
      state.plan.waypoints.forEach(function (w, i) {
        var p = toPx(w.coords);
        i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]);
      });
      ctx.stroke();
      var first = toPx(state.plan.waypoints[0].coords);
      ctx.fillStyle = "#2f81f7"; ctx.globalAlpha = 1;
      ctx.beginPath(); ctx.arc(first[0], first[1], 5, 0, 7); ctx.fill();
      ctx.restore();
    }

    var D = state.detect;
    if (D && D.candidates && L) {
      var sx2 = L.image_size ? cv.width / L.image_size[0] : 1;
      var sy2 = L.image_size ? cv.height / L.image_size[1] : 1;
      D.candidates.forEach(function (c, i) {
        var x = c.px[0] * sx2, y = c.px[1] * sy2;
        var r = Math.max(7, Math.sqrt(c.area_mm2) * 2.2);
        ctx.save();
        ctx.strokeStyle = c.channel === "surface" ? "#f0883e" : "#f85149";
        ctx.lineWidth = c.both ? 3.5 : 2;
        ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.stroke();
        if (i < 12) {
          ctx.fillStyle = ctx.strokeStyle;
          ctx.font = "bold 13px ui-monospace,monospace";
          ctx.fillText(String(i + 1), x + r + 3, y - r - 2);
        }
        ctx.restore();
      });
    }
  }

  $("inspCanvas").addEventListener("click", function (ev) {
    if (!requireLink()) return;
    var r = this.getBoundingClientRect();
    var x = Math.round((ev.clientX - r.left) * this.width / r.width);
    var y = Math.round((ev.clientY - r.top) * this.height / r.height);
    send({ type: "camera_point", x: x, y: y, window: 5 });
  });

  /* -------------------------------------------------- recording ---------- */
  function onBenchStatus(d) {
    var rec = d.recorder || {};
    $("btnRecStart").disabled = !!rec.recording;
    $("btnRecStop").disabled = !rec.recording;
    var cur = rec.current || rec.last;
    if (cur) {
      $("rcName").textContent = cur.run_id || "—";
      $("rcN").textContent = cur.n || 0;
    }
  }

  $("btnRecStart").addEventListener("click", function () {
    if (!requireLink("rcMsg")) return;
    var vel = Number($("rcVel").value);
    var calib = ($("rcCalib").value || "").trim();
    if (!(vel > 0)) { say("rcMsg", "Elbow speed must be more than zero.", "bad"); return; }
    if (!calib) {
      say("rcMsg", "Enter the calibration version. Runs recorded either side of a "
        + "recalibration cannot be compared, and without this there is no way to "
        + "tell later which side a run came from.", "bad"); return;
    }
    var cfg = $("rcConfig").value, traj = $("rcTraj").value;
    var rep = parseInt($("rcRepeat").value, 10) || 0;
    var runId = ("v" + vel.toFixed(2) + "_" + cfg + "_" + traj + "_r"
      + String(rep).padStart(2, "0")).replace(/\./g, "p");
    send({ type: "bench_start", run_id: runId, joint_vel: vel, arm_config: cfg,
      traj_type: traj, repeat_idx: rep, calib_version: calib,
      rate_hz: Number($("rcRate").value) || 125 });
  });
  $("btnRecStop").addEventListener("click", function () { send({ type: "bench_stop" }); });
  $("btnTap").addEventListener("click", function () {
    if (!requireLink("rcMsg")) return;
    say("rcMsg", "Tap the sensor mount once, firmly.", "info");
    send({ type: "bench_tap", window_s: 5 });
  });

  /* -------------------------------------------------- 3D preview --------- */
  var three = null;
  function init3D() {
    if (typeof THREE === "undefined") {
      $("stage3dEmpty").textContent =
        "The 3D preview needs its libraries, which are in vendor/three next to "
        + "this page. Everything else works without it.";
      return;
    }
    var cv = $("stage3d");
    var w = cv.clientWidth || 600, h = 400;
    var scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0e13);
    var cam = new THREE.PerspectiveCamera(48, w / h, 0.01, 60);
    cam.position.set(1.5, 1.1, 1.7);
    var rend = new THREE.WebGLRenderer({ canvas: cv, antialias: true });
    rend.setSize(w, h, false);
    rend.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    scene.add(new THREE.HemisphereLight(0xdfefff, 0x101820, 1.1));
    var key = new THREE.DirectionalLight(0xffffff, 0.8); key.position.set(2, 3, 2);
    scene.add(key);
    scene.add(new THREE.GridHelper(2.4, 24, 0x2b3441, 0x1a222c));
    var ctrl = new THREE.OrbitControls(cam, rend.domElement);
    ctrl.enableDamping = true; ctrl.target.set(0, 0.3, 0);
    var pathLine = null, joints = [];

    three = { scene: scene, cam: cam, rend: rend, ctrl: ctrl,
      target: null,
      fit: function (obj) {
        // Frame the ROBOT, not the scene. Including the floor grid in the
        // bounding box pushes the camera back until the arm is a speck.
        var subject = obj || three.target || scene;
        var b = new THREE.Box3().setFromObject(subject);
        if (b.isEmpty()) return;
        var c = b.getCenter(new THREE.Vector3());
        var r = b.getSize(new THREE.Vector3()).length() / 2;
        var d = r / Math.tan(cam.fov * Math.PI / 360) * 1.05;
        cam.position.set(c.x + d * 0.62, c.y + d * 0.48, c.z + d * 0.62);
        cam.near = Math.max(d / 200, 0.01); cam.far = d * 12;
        cam.updateProjectionMatrix();
        ctrl.target.copy(c); ctrl.update();
      },
      setPath: function (pts) {
        if (pathLine) { scene.remove(pathLine); pathLine.geometry.dispose(); }
        if (!pts || !pts.length) { pathLine = null; return; }
        var g = new THREE.BufferGeometry().setFromPoints(pts.map(function (c) {
          return new THREE.Vector3(c[0], c[2], -c[1]);   // robot Z-up -> three Y-up
        }));
        pathLine = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0x2f81f7 }));
        scene.add(pathLine);
      } };

    if (typeof THREE.GLTFLoader === "function") {
      new THREE.GLTFLoader().load("ur5e.glb", function (g) {
        var root = g.scene;
        // Normalise whatever the export used. A UR5e reaches about 0.85 m, so
        // scale the model to that and re-centre it on the floor rather than
        // trusting the file's units — millimetre, centimetre and metre exports
        // all exist, and the wrong one puts the arm off-screen with no error.
        var box = new THREE.Box3().setFromObject(root);
        var size = box.getSize(new THREE.Vector3());
        var reach = Math.max(size.x, size.y, size.z);
        if (reach > 0) root.scale.setScalar(0.9 / reach);
        box = new THREE.Box3().setFromObject(root);
        var c = box.getCenter(new THREE.Vector3());
        root.position.sub(new THREE.Vector3(c.x, box.min.y, c.z));
        scene.add(root);
        $("stage3dEmpty").style.display = "none";
        three.target = root;
        three.fit(root);
        root.traverse(function (o) {
          if (/joint|link|shoulder|elbow|wrist|base/i.test(o.name || "")) joints.push(o);
        });
      }, undefined, function () {
        $("stage3dEmpty").textContent =
          "Could not load the robot model file (ur5e.glb). The preview is off; "
          + "nothing else is affected.";
      });
    }

    (function loop() {
      requestAnimationFrame(loop);
      ctrl.update(); rend.render(scene, cam);
    })();
  }
  function resize3D() {
    if (!three) return;
    var cv = $("stage3d");
    var w = cv.clientWidth || 600;
    three.cam.aspect = w / 400; three.cam.updateProjectionMatrix();
    three.rend.setSize(w, 400, false);
  }
  window.addEventListener("resize", resize3D);
  $("btn3dPath").addEventListener("click", function () {
    if (!three) return;
    if (!state.plan) { alert("Plan a scan first, on the Inspect page."); return; }
    three.setPath(state.plan.waypoints.map(function (w) { return w.coords; }));
  });
  $("btn3dFit").addEventListener("click", function () { if (three) three.fit(); });

  /* -------------------------------------------------- heartbeat ---------- */
  setInterval(function () {
    var now = performance.now();
    var live = ws && ws.readyState === WebSocket.OPEN;

    if (!live) { lamp("lampRobot", "lampRobotV", "", "Not connected"); }
    else if (now - state.urAge < 2000) {
      var s = state.ur || {};
      var bad = /STOP|FAULT|VIOLATION/.test(s.safety_mode_text || "");
      lamp("lampRobot", "lampRobotV", bad ? "bad" : "ok",
        bad ? friendlySafety(s.safety_mode_text) : friendlyMode(s.robot_mode_text));
    } else { lamp("lampRobot", "lampRobotV", "warn", "No data"); }

    if (!live) { lamp("lampCam", "lampCamV", "", "Not connected"); }
    else if (state.camNoCamera) { lamp("lampCam", "lampCamV", "bad", "Not available"); }
    else if (now - state.camAge < 2000) { lamp("lampCam", "lampCamV", "ok", "Live picture"); }
    else { lamp("lampCam", "lampCamV", "warn", "No picture"); }

    if (!live) { lamp("lampImu", "lampImuV", "", "Not connected"); }
    else if (now - state.imuAge < 2000) { /* set by onImu */ }
    else { lamp("lampImu", "lampImuV", "warn", "No data"); }

    if (live) { send({ type: "bench_status" }); send({ type: "ur_service_status" }); }
  }, 1000);

  if (document.readyState === "loading")
    document.addEventListener("DOMContentLoaded", init3D, { once: true });
  else init3D();
})();
