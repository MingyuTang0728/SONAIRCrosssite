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
  var PAGES = ["connect", "robot", "camera", "sensors", "calib", "inspect", "record"];
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
      if (API.onPage[b.dataset.page]) API.onPage[b.dataset.page]();
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
      send({ type: "handeye_status" });
      send({ type: "mv_status" });
      send({ type: "rs_enumerate" });
      send({ type: "imu_transports" });
      send({ type: "sensors_report" });
      API.fire("open", {});
    };
    ws.onclose = function () {
      say("connMsg", "Disconnected from the host agent.", "bad");
      lamp("lampRobot", "lampRobotV", "", "Not connected");
      lamp("lampCam", "lampCamV", "", "Not connected");
      lamp("lampImu", "lampImuV", "", "Not connected");
      ws = null;
      API.fire("close", {});
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

      case "jog_status_res": {
        var t = $("jogHealth"); if (!t) break;
        t.textContent = d.running
          ? (d.moving ? "moving · " + Math.round(d.rate_hz) + " Hz from the host"
                      : "ready · host keeps the timing")
          : "robot link not started";
        break;
      }
      case "jog_res":
        if (!d.ok && d.msg) say("jogMsg", d.msg, "warn");
        break;
      case "jog_step_res":
        if (!d.ok) say("jogMsg", plainCmdError({ msg: d.error || d.msg }), "bad");
        break;
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
      default:
        break;
    }
    // Everything also goes to the extension modules. Forwarding ALL messages
    // rather than only the unhandled ones means a module can react to the
    // camera frames and the robot state as well as to its own replies — and
    // it keeps one socket, one parse, one place where the wire format lives.
    API.fire(d.type, d);
  }

  /* -------------------------------------------------- module API ---------
     The console grew past one file. Rather than a second socket or a second
     copy of the formatting helpers, extension modules get this: the same
     send, the same helpers, and a message registry. One connection, one set
     of conventions, one place that knows the wire format.                  */
  var API = {
    handlers: {},
    onPage: {},
    send: send,
    require: requireLink,
    say: say,
    lamp: lamp,
    fmt: fmt,
    esc: esc,
    $: $,
    state: state,
    on: function (type, fn) {
      (this.handlers[type] || (this.handlers[type] = [])).push(fn);
      return this;
    },
    page: function (name, fn) { this.onPage[name] = fn; return this; },
    fire: function (type, d) {
      var list = this.handlers[type];
      if (!list) return;
      for (var i = 0; i < list.length; i++) {
        try { list[i](d); } catch (e) {
          // One module's exception must never stop the others from seeing the
          // message — a broken panel should not take the console down.
          if (window.console) console.error("[console] handler for " + type, e);
        }
      }
    },
    connected: function () { return !!(ws && ws.readyState === WebSocket.OPEN); }
  };
  window.SONAIR = API;

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
    // The cell view follows the real arm. Without this the model was a
    // picture of a robot, not a view of THIS robot.
    if (three && three.setJoints && s.actual_q) three.setJoints(s.actual_q);
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
    jog.keys = {}; jog.pad.xy = [0, 0]; jog.pad.zr = [0, 0];
    send({ type: "jog_halt" });
    send({ type: "ur_estop" });
  });

  /* -------------------------------------------------- jog ----------------
     The browser sends INTENT ONLY. It never sets the cadence of robot motion.

     Measured on this page under its real load — camera frames decoding and a
     3D view rendering — setInterval(100) fires at a median of 248 ms with
     nearly half the ticks past 250 ms. Any scheme where a late tick lets a
     speedl expire produces exactly the stutter this replaces. The host now
     re-issues at a steady 20 Hz from its own clock, ramps direction changes,
     and stops by watchdog if this page goes quiet, so a browser stall changes
     nothing at the arm.
     ---------------------------------------------------------------------- */
  var jog = {
    mode: "cont", frame: "base", step: 1,
    pad: { xy: [0, 0], zr: [0, 0] },
    keys: {}, keysOn: false, gamepad: null, slow: false,
    lastSent: [0, 0, 0, 0, 0, 0], sentAt: 0
  };

  $("jogSpeed").addEventListener("input", function () {
    state.jogSpeed = Number(this.value);
    $("jogSpeedV").textContent = state.jogSpeed + " mm/s";
  });

  function seg(id, attr, onPick) {
    var host = $(id); if (!host) return;
    host.addEventListener("click", function (ev) {
      var b = ev.target.closest(".segb"); if (!b) return;
      host.querySelectorAll(".segb").forEach(function (o) { o.classList.toggle("on", o === b); });
      onPick(b.dataset[attr]);
    });
  }
  seg("jogModeSeg", "mode", function (m) {
    jog.mode = m;
    $("jogCont").hidden = (m !== "cont");
    $("jogStep").hidden = (m !== "step");
    if (m !== "cont") sendVel([0, 0, 0, 0, 0, 0]);
  });
  seg("jogFrameSeg", "frame", function (f) { jog.frame = f; });
  seg("stepSeg", "step", function (v) { jog.step = Number(v); });

  /* ---- the single place a velocity leaves this page ---- */
  function sendVel(v) {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    var same = v.every(function (c, i) { return Math.abs(c - jog.lastSent[i]) < 1e-6; });
    var moving = v.some(function (c) { return Math.abs(c) > 1e-6; });
    var now = performance.now();
    // Repeat only WHILE MOVING. The refresh exists so the host's 400 ms
    // watchdog cannot trip mid-move; a stopped robot has nothing to keep
    // alive, so once the stop has been sent this goes quiet.
    //
    // Without the `moving` test this fired every 150 ms forever, on every
    // page, whether or not anyone was jogging — about seven messages a second
    // of pure noise sharing one socket with the camera frames, the telemetry
    // and every button press, and handled inline on the agent because jogs
    // must not queue. Buttons elsewhere felt sluggish for that reason alone.
    if (same && (!moving || now - jog.sentAt < 150)) return;
    jog.lastSent = v.slice(); jog.sentAt = now;
    ws.send(JSON.stringify({ type: "jog_vel", xd: v, ttl_ms: 400 }));
  }

  function currentVelocity() {
    if (jog.mode !== "cont") return [0, 0, 0, 0, 0, 0];
    var v = state.jogSpeed / 1000 * (jog.slow ? 0.25 : 1);
    var x = 0, y = 0, z = 0, rz = 0;

    x += jog.pad.xy[0]; y += -jog.pad.xy[1];
    z += -jog.pad.zr[1]; rz += jog.pad.zr[0];

    if (jog.keysOn) {
      if (jog.keys.ArrowRight) x += 1;
      if (jog.keys.ArrowLeft) x -= 1;
      if (jog.keys.ArrowUp) y += 1;
      if (jog.keys.ArrowDown) y -= 1;
      if (jog.keys.KeyE) z += 1;
      if (jog.keys.KeyQ) z -= 1;
      if (jog.keys.KeyD) rz += 1;
      if (jog.keys.KeyA) rz -= 1;
    }
    if (jog.gamepad) {
      var g = jog.gamepad;
      x += dz(g.axes[0]); y += -dz(g.axes[1]);
      z += -dz(g.axes[3]); rz += dz(g.axes[2]);
    }
    x = clamp1(x); y = clamp1(y); z = clamp1(z); rz = clamp1(rz);
    return [x * v, y * v, z * v, 0, 0, rz * (v * 12)];
  }
  function dz(a) { a = a || 0; return Math.abs(a) < 0.12 ? 0 : a; }
  function clamp1(a) { return Math.max(-1, Math.min(1, a)); }

  /* Driven by requestAnimationFrame, not setInterval: rAF is aligned to the
     compositor and is not throttled the way a timer is when the main thread is
     busy. Even so, nothing depends on it arriving on time. */
  function jogTick() {
    requestAnimationFrame(jogTick);
    pollGamepad();
    sendVel(currentVelocity());
  }
  requestAnimationFrame(jogTick);

  /* ---- on-screen pads: they set a vector, nothing more ---- */
  function bindPad(id, key) {
    var disc = $(id); if (!disc) return;
    var knob = disc.querySelector(".knob");
    var active = false;

    function at(ev) {
      var r = disc.getBoundingClientRect();
      var t = ev.touches ? ev.touches[0] : ev;
      var dx = (t.clientX - (r.left + r.width / 2)) / (r.width / 2);
      var dy = (t.clientY - (r.top + r.height / 2)) / (r.height / 2);
      var m = Math.hypot(dx, dy);
      if (m > 1) { dx /= m; dy /= m; }
      jog.pad[key] = [dx, dy];
      knob.style.left = (50 + dx * 33) + "%";
      knob.style.top = (50 + dy * 33) + "%";
    }
    function release() {
      active = false;
      jog.pad[key] = [0, 0];
      knob.style.left = "50%"; knob.style.top = "50%";
    }
    disc.addEventListener("pointerdown", function (ev) {
      if (!requireLink("jogMsg")) return;
      active = true; disc.setPointerCapture(ev.pointerId); at(ev);
    });
    disc.addEventListener("pointermove", function (ev) { if (active) at(ev); });
    ["pointerup", "pointercancel", "pointerleave"].forEach(function (e) {
      disc.addEventListener(e, release);
    });
    // A pointer released outside the disc must still stop the arm.
    window.addEventListener("blur", release);
  }
  bindPad("padXY", "xy");
  bindPad("padZR", "zr");

  /* ---- keyboard ---- */
  $("jogKeys").addEventListener("change", function () {
    jog.keysOn = this.checked;
    // Drop focus, so the arrow keys reach the window handler rather than
    // being spent scrolling the page from a focused control.
    this.blur();
    if (!this.checked) jog.keys = {};
    say("jogMsg", this.checked
      ? "Keyboard control is on. Click on the page first, then use the keys shown."
      : "Keyboard control is off.", "info");
  });
  var JOG_KEYS = ["ArrowLeft","ArrowRight","ArrowUp","ArrowDown","KeyQ","KeyE","KeyA","KeyD"];
  window.addEventListener("keydown", function (ev) {
    if (!jog.keysOn || jog.mode !== "cont") return;
    // Ignore keys only while TYPING. Treating every INPUT as a text field meant
    // the checkbox that turns this on kept focus and swallowed every arrow key
    // afterwards — the control appeared enabled and did nothing.
    var tag = (ev.target.tagName || "").toUpperCase();
    var kind = String(ev.target.type || "").toLowerCase();
    var typing = tag === "TEXTAREA" || tag === "SELECT"
      || (tag === "INPUT" && !/^(checkbox|radio|button|submit|range)$/.test(kind));
    if (typing) return;
    if (ev.code === "Space") { ev.preventDefault(); jog.keys = {}; sendVel([0,0,0,0,0,0]); return; }
    jog.slow = ev.shiftKey;
    if (JOG_KEYS.indexOf(ev.code) < 0) return;
    ev.preventDefault();
    jog.keys[ev.code] = true;
  });
  window.addEventListener("keyup", function (ev) {
    jog.slow = ev.shiftKey;
    if (jog.keys[ev.code]) delete jog.keys[ev.code];
  });
  // Releasing a key while the window is not focused never arrives, so a lost
  // focus has to clear every held key or the arm keeps moving.
  window.addEventListener("blur", function () { jog.keys = {}; jog.slow = false; });

  /* ---- gamepad ---- */
  function pollGamepad() {
    if (!navigator.getGamepads) return;
    var pads = navigator.getGamepads();
    var g = null;
    for (var i = 0; i < pads.length; i++) if (pads[i] && pads[i].connected) { g = pads[i]; break; }
    jog.gamepad = (g && jog.mode === "cont") ? g : null;
    var box = $("gpBox"); if (!box) return;
    if (g) {
      box.className = "gp on";
      box.textContent = "Gamepad: " + g.id.slice(0, 44)
        + " — left stick moves across the table, right stick lifts and turns.";
    } else if (box.className !== "gp") {
      box.className = "gp";
      box.textContent = "No gamepad. Plug one in and press a button.";
    }
  }

  /* ---- precise steps ---- */
  var STEP_AXES = [["x","X"],["y","Y"],["z","Z"],["rx","Rx"],["ry","Ry"],["rz","Rz"]];
  $("stepGrid").innerHTML = STEP_AXES.map(function (a) {
    return '<div class="stepax"><div class="sa">' + a[1] + '</div><div class="sbtns">'
      + '<button data-ax="' + a[0] + '" data-dir="-1">&minus;</button>'
      + '<button data-ax="' + a[0] + '" data-dir="1">+</button></div></div>';
  }).join("");
  $("stepGrid").addEventListener("click", function (ev) {
    var b = ev.target.closest("button[data-ax]"); if (!b) return;
    if (!requireLink("jogMsg")) return;
    var ax = b.dataset.ax, dir = Number(b.dataset.dir);
    // Rotation steps are in degrees and capped host-side; a rotation vector is
    // not three independent angles, so only small increments are meaningful.
    var dist = (ax[0] === "r" ? Math.min(jog.step, 5) : jog.step) * dir;
    send({ type: "jog_step", axis: ax, distance_mm: dist, frame: jog.frame,
      speed: state.jogSpeed / 1000 });
    say("jogMsg", "Moving " + ax.toUpperCase() + " by " + dist
      + (ax[0] === "r" ? "°" : " mm") + " in "
      + (jog.frame === "tool" ? "tool" : "table") + " axes.", "info");
  });

  /* -------------------------------------------------- camera ------------- */
  var _decoding = { color: false, depth: false };

  function onFrame(d) {
    state.camAge = performance.now();
    // Decode only what is on screen. Decoding both streams was costing the
    // main thread roughly twice what it needed to, and that thread is the one
    // the rest of the page's responsiveness comes out of.
    var want = (document.getElementById("page-inspect").hidden) ? null : state.view;
    if (d.rgb && (want === "color" || !state.frames.color)) loadFrame("color", d.rgb);
    if (d.depth && (want === "depth" || !state.frames.depth)) loadFrame("depth", d.depth);
    var e = $("inspEmpty"); if (e) e.style.display = "none";
  }

  function loadFrame(kind, b64) {
    // Drop a frame rather than queue it. At 30 fps a backlog only grows, and a
    // late frame is worth less than the main-thread time spent decoding it.
    if (_decoding[kind]) return;
    _decoding[kind] = true;

    var bin = atob(b64);
    var buf = new Uint8Array(bin.length);
    for (var i = 0; i < bin.length; i++) buf[i] = bin.charCodeAt(i);
    var blob = new Blob([buf], { type: "image/jpeg" });

    if (window.createImageBitmap) {
      // createImageBitmap decodes OFF the main thread. new Image() with a data
      // URL does not, and with two streams at 30 fps that decode was what
      // starved the page's timers — the root cause of the jerky jog.
      createImageBitmap(blob).then(function (bmp) {
        if (state.frames[kind] && state.frames[kind].close) state.frames[kind].close();
        state.frames[kind] = bmp;
        _decoding[kind] = false;
        if (kind === state.view) drawInspect();
      }).catch(function () { _decoding[kind] = false; });
      return;
    }

    var url = URL.createObjectURL(blob);
    var img = new Image();
    img.onload = function () {
      state.frames[kind] = img;
      _decoding[kind] = false;
      URL.revokeObjectURL(url);
      if (kind === state.view) drawInspect();
    };
    img.onerror = function () { _decoding[kind] = false; URL.revokeObjectURL(url); };
    img.src = url;
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

    if (img) {
      cv.width = img.naturalWidth || img.width;
      cv.height = img.naturalHeight || img.height;
    }
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

    // Extension modules draw last, on top of whatever this function drew, so
    // the 3D-scan overlays share one canvas and one frame rather than fighting
    // over two stacked ones.
    API.fire("inspect_draw", { ctx: ctx, canvas: cv, w: cv.width, h: cv.height });
  }
  API.drawInspect = drawInspect;

  $("inspCanvas").addEventListener("click", function (ev) {
    if (!requireLink()) return;
    if (API.swallowInspectClick) return;
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

    /* ---- the arm --------------------------------------------------------
       The model is a URDF export: six named revolute bones in a kinematic
       chain, in centimetres. Two things had to be measured rather than
       assumed, and both were, against the UR5e's real forward kinematics:

         the joints turn about their LOCAL Y axis, not Z;
         the export's frame is the robot's, rotated a half turn about
         vertical, so at 1/100 scale the flange lands exactly where forward
         kinematics says — 0.000 mm at the rest pose and at a test pose with
         every joint away from zero.

       That second part is why the view was wrong before: the model was fitted
       to the viewport by a scale-to-fill heuristic while the planned path was
       drawn in metres in a mirrored frame, so the arm and its path were
       neither the same size nor the same way round. Now there is one frame:
       robot base coordinates in metres, mapped to the view as (x, z, -y).  */

    var JOINT_BONES = ["shoulder_pan_joint", "shoulder_lift_joint",
                       "elbow_joint", "wrist_1_joint", "wrist_2_joint",
                       "wrist_3_joint"];
    var rig = null;       // { group, bones[], rest[], animated }
    var Y_AXIS = new THREE.Vector3(0, 1, 0);

    function unitScale(root) {
      // Millimetres, centimetres and metres all exist in the wild and none of
      // them say so in the file. The overall size does: a robot arm is about
      // a metre, so the magnitude names the unit.
      var size = new THREE.Box3().setFromObject(root).getSize(new THREE.Vector3());
      var reach = Math.max(size.x, size.y, size.z);
      if (reach > 200) return { s: 0.001, unit: "millimetres" };
      if (reach > 20) return { s: 0.01, unit: "centimetres" };
      return { s: 1, unit: "metres" };
    }

    function mountModel(root, label) {
      if (rig && rig.group) {
        scene.remove(rig.group);
        rig.group.traverse(function (o) {
          if (o.geometry) o.geometry.dispose();
          if (o.material) [].concat(o.material).forEach(function (m) { m.dispose(); });
        });
      }
      var bones = [], rest = [];
      root.traverse(function (o) {
        JOINT_BONES.forEach(function (n, i) {
          if ((o.name || "").indexOf(n) === 0) {
            bones[i] = o; rest[i] = o.quaternion.clone();
          }
        });
      });
      var animated = bones.filter(Boolean).length === 6;
      var group = new THREE.Group();
      var u = unitScale(root);

      if (animated) {
        // Known rig: put it in the robot's own frame so the arm, the path and
        // the tool position all agree.
        group.scale.setScalar(u.s);
        group.rotation.y = Math.PI;
        group.add(root);
      } else {
        // Unknown model: no joints to drive, so just make it visible —
        // centred on the floor at a believable size.
        var box = new THREE.Box3().setFromObject(root);
        var size = box.getSize(new THREE.Vector3());
        var reach = Math.max(size.x, size.y, size.z) || 1;
        root.scale.setScalar(0.9 / reach);
        box = new THREE.Box3().setFromObject(root);
        var c = box.getCenter(new THREE.Vector3());
        root.position.sub(new THREE.Vector3(c.x, box.min.y, c.z));
        group.add(root);
      }
      scene.add(group);
      rig = { group: group, bones: bones, rest: rest, animated: animated };
      three.target = group;
      three.fit(group);
      $("stage3dEmpty").hidden = true;
      say("cellMsg", animated
        ? (label + " loaded in " + u.unit + ". The six joints are named, so the "
           + "view follows the real arm.")
        : (label + " loaded, but it has no named joints "
           + "(shoulder_pan_joint, shoulder_lift_joint, elbow_joint, "
           + "wrist_1_joint … wrist_3_joint), so it is shown as a fixed shape "
           + "and will not follow the robot."),
        animated ? "ok" : "warn");
      if (state.ur && state.ur.actual_q) three.setJoints(state.ur.actual_q);
    }

    three.setJoints = function (q) {
      if (!rig || !rig.animated || !q || q.length < 6) return;
      for (var i = 0; i < 6; i++) {
        if (!rig.bones[i]) continue;
        rig.bones[i].quaternion.copy(rig.rest[i])
          .multiply(new THREE.Quaternion().setFromAxisAngle(Y_AXIS, q[i]));
      }
      // Frame it once, on the first real pose. The model loads at its rest
      // pose, which is not where the arm is standing, so a view fitted to the
      // rest pose can open with the real arm half out of frame. Refitting on
      // every update instead would swing the camera around continuously,
      // so it happens exactly once and the Fit button covers the rest.
      if (!rig.framed) {
        rig.framed = true;
        rig.group.updateMatrixWorld(true);
        three.fit(rig.group);
      }
    };
    three.hasRig = function () { return !!(rig && rig.animated); };

    /* Where the MODEL thinks the flange is, in robot base coordinates.
       Comparing that with the pose the robot reports checks the whole chain
       at once — the right model, the right joint mapping, the right units,
       the right frame. A view that silently disagrees with the robot is
       worse than no view, so this is what "Check the view" reports. */
    three.flangeInBase = function () {
      if (!rig || !rig.animated) return null;
      var node = null;
      rig.group.traverse(function (o) {
        if (!node && (o.name || "").indexOf("flange-tool0") === 0) node = o;
      });
      if (!node) {
        rig.group.traverse(function (o) {
          if ((o.name || "").indexOf("wrist_3_link") === 0) node = o;
        });
      }
      if (!node) return null;
      rig.group.updateMatrixWorld(true);
      var p = new THREE.Vector3();
      node.getWorldPosition(p);
      // the view's (x, y, z) is the robot's (x, -z, y)
      return [p.x, -p.z, p.y];
    };

    three.loadFrom = function (url, label, ext) {
      var done = function (root) { mountModel(root, label); };
      var fail = function (e) {
        say("cellMsg", "Could not read " + label + ". "
          + (e && e.message ? e.message : "The file may not be a model this "
             + "viewer understands."), "bad");
      };
      try {
        if (ext === "stl" && typeof THREE.STLLoader === "function") {
          new THREE.STLLoader().load(url, function (geo) {
            geo.computeVertexNormals();
            done(new THREE.Mesh(geo, new THREE.MeshStandardMaterial(
              { color: 0xb9c6d4, roughness: 0.65, metalness: 0.1 })));
          }, undefined, fail);
        } else if (ext === "ply" && typeof THREE.PLYLoader === "function") {
          new THREE.PLYLoader().load(url, function (geo) {
            geo.computeVertexNormals();
            done(new THREE.Points(geo, new THREE.PointsMaterial(
              { size: 0.004, color: 0x7fb4ef })));
          }, undefined, fail);
        } else {
          new THREE.GLTFLoader().load(url, function (g) { done(g.scene); },
                                     undefined, fail);
        }
      } catch (e) { fail(e); }
    };

    if (typeof THREE.GLTFLoader === "function") {
      three.loadFrom("ur5e.glb", "The built-in UR5e model", "glb");
    } else {
      $("stage3dEmpty").textContent =
        "The 3D preview needs its libraries, which are in vendor/three next to "
        + "this page. Everything else works without it.";
    }

    (function loop() {
      requestAnimationFrame(loop);
      // Rendering a hidden page spends GPU and main-thread time on pixels
      // nobody sees, and that time comes out of the same budget the jog needs.
      if (document.getElementById("page-robot").hidden) return;
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
    if (!three) {
      say("cellMsg", "The 3D view has not loaded.", "warn"); return;
    }
    // Not alert(): it blocks the page, it cannot be styled, and every other
    // message in this console appears in its own panel. One convention.
    var plan = state.plan;
    if (!plan || !plan.waypoints || !plan.waypoints.length) {
      say("cellMsg", "No scan path yet. Plan one on the Inspect page — either "
        + "route works — and it will appear here.", "warn");
      return;
    }
    three.setPath(plan.waypoints.map(function (w) { return w.coords; }));
    say("cellMsg", "Showing the planned path: " + plan.waypoints.length
      + " points. Blue is where the tool will travel."
      + (three.hasRig && three.hasRig() ? ""
         : " The loaded model has no named joints, so it is not placed in the "
           + "robot's frame — read the path against the grid, not against the "
           + "model."), "ok");
  });
  /* ---- load your own model ----------------------------------------------
     Read from the file the operator picks, not from a path typed somewhere:
     a blob URL needs no server, no copying into the project folder, and works
     the same whether the page is served locally or from anywhere else.

     A model with the six URDF joint names drives the arm; anything else is
     shown as a fixed shape and SAYS so, because a part that looks like it is
     tracking the robot and is not would be worse than no preview at all.  */
  $("btn3dLoad").addEventListener("click", function () { $("modelFile").click(); });

  $("modelFile").addEventListener("change", function () {
    var f = this.files && this.files[0];
    if (!f) return;
    var ext = (f.name.split(".").pop() || "").toLowerCase();
    if (["glb", "gltf", "stl", "ply"].indexOf(ext) < 0) {
      say("cellMsg", "That file type is not supported. Use GLB, glTF, STL or "
        + "PLY.", "bad");
      this.value = ""; return;
    }
    if (!three || !three.loadFrom) {
      say("cellMsg", "The 3D view has not loaded.", "warn");
      this.value = ""; return;
    }
    say("cellMsg", "Reading " + f.name + " (" + Math.round(f.size / 1048576 * 10) / 10
      + " MB)…", "info");
    var url = URL.createObjectURL(f);
    three.loadFrom(url, f.name, ext);
    // The loaders read the blob synchronously from the URL; release it on the
    // next turn so a big upload is not held in memory for the session.
    setTimeout(function () { URL.revokeObjectURL(url); }, 30000);
    this.value = "";
  });

  $("btn3dReset").addEventListener("click", function () {
    if (!three || !three.loadFrom) return;
    say("cellMsg", "Reloading the built-in model…", "info");
    three.loadFrom("ur5e.glb", "The built-in UR5e model", "glb");
  });

  /* Does the picture agree with the robot? */
  function cellCheck() {
    if (!three || !three.flangeInBase) return null;
    var model = three.flangeInBase();
    var s = state.ur || {};
    var tcp = s.actual_TCP_pose;
    if (!model || !tcp || tcp.length < 3) return null;
    // The reported pose is the TCP, which sits at the flange only when no
    // tool offset is set; a configured TCP legitimately moves it. So this is
    // reported as a distance, with that caveat, rather than as pass/fail.
    var d = Math.sqrt(Math.pow(model[0] - tcp[0], 2)
                    + Math.pow(model[1] - tcp[1], 2)
                    + Math.pow(model[2] - tcp[2], 2)) * 1000;
    return { model: model, tcp: tcp.slice(0, 3), mm: d };
  }
  API.cellCheck = cellCheck;

  $("btn3dCheck").addEventListener("click", function () {
    var r = cellCheck();
    if (!r) {
      say("cellMsg", "Nothing to compare yet — the view needs a model with "
        + "named joints and a connected robot.", "warn");
      return;
    }
    say("cellMsg", "The model puts the flange at "
      + r.model.map(function (v) { return fmt(v * 1000, 0); }).join(", ")
      + " mm; the robot reports its tool at "
      + r.tcp.map(function (v) { return fmt(v * 1000, 0); }).join(", ")
      + " mm — " + fmt(r.mm, 0) + " mm apart. That difference IS your tool "
      + "offset if one is set; with no tool offset it should be near zero, "
      + "and a large number means the model does not match this robot.",
      r.mm < 400 ? "ok" : "warn");
  });

  $("btn3dFit").addEventListener("click", function () {
    if (!three) { say("cellMsg", "The 3D view has not loaded.", "warn"); return; }
    three.fit();
    say("cellMsg", "View re-centred on the robot.", "info");
  });

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

    if (live) {
      send({ type: "bench_status" });
      send({ type: "ur_service_status" });
      send({ type: "jog_status" });
    }
  }, 1000);

  if (document.readyState === "loading")
    document.addEventListener("DOMContentLoaded", init3D, { once: true });
  else init3D();
})();
