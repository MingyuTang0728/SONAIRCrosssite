/* ===========================================================================
   SONAIR Inspection Console — camera, motion sensors, calibration, 3D scan.

   Runs on the API the main console exports: one socket, one set of helpers,
   one place that knows the wire format. Same rule as the main file — nothing
   raw reaches the screen. Every fault is a sentence in words the operator can
   act on, and the numbers carry their units.
   =========================================================================== */
(function () {
  "use strict";
  var S = window.SONAIR;
  if (!S) { console.error("[console_adv] main console did not load"); return; }
  var $ = S.$, send = S.send, say = S.say, fmt = S.fmt, esc = S.esc;

  /* Series colours for the live traces. Validated against this console's
     chart surface for lightness, chroma, colour-blind separation and
     contrast — and the axes are labelled X/Y/Z as well, so identity never
     rests on colour alone. */
  var SERIES = ["#3987e5", "#c96f3c", "#1f9b86"];
  var AXIS_NAMES = ["X", "Y", "Z"];

  function num(id, dflt) {
    var v = parseFloat(($(id) || {}).value);
    return isFinite(v) ? v : dflt;
  }
  function blankable(id) {
    var raw = (($(id) || {}).value || "").trim();
    if (!raw) return null;
    var v = parseFloat(raw);
    return isFinite(v) ? v : null;
  }
  function on(id, ev, fn) { var e = $(id); if (e) e.addEventListener(ev, fn); }
  function seg(id, attr, fn) {
    var host = $(id); if (!host) return;
    host.addEventListener("click", function (e) {
      var b = e.target.closest(".segb"); if (!b) return;
      host.querySelectorAll(".segb").forEach(function (o) {
        o.classList.toggle("on", o === b);
      });
      fn(b.dataset[attr]);
    });
  }

  /* =======================================================================
     1. CAMERA — four streams, the projector, the filter chain, quality
     ======================================================================= */
  var camBusy = {};

  function paint(canvasId, emptyId, b64) {
    var cv = $(canvasId); if (!cv) return;
    if (!b64) return;
    if (camBusy[canvasId]) return;         // drop, never queue: a backlog of
    camBusy[canvasId] = true;              // stale frames is worse than a gap
    var bin = atob(b64), arr = new Uint8Array(bin.length);
    for (var i = 0; i < bin.length; i++) arr[i] = bin.charCodeAt(i);
    createImageBitmap(new Blob([arr], { type: "image/jpeg" })).then(function (bmp) {
      cv.width = bmp.width; cv.height = bmp.height;
      cv.getContext("2d").drawImage(bmp, 0, 0);
      bmp.close && bmp.close();
      var e = $(emptyId); if (e) e.hidden = true;
      camBusy[canvasId] = false;
    }).catch(function () { camBusy[canvasId] = false; });
  }

  S.on("camera_frame", function (d) {
    if ($("page-camera") && !$("page-camera").hidden) {
      paint("sColor", "sColorE", d.rgb);
      paint("sDepth", "sDepthE", d.depth);
      paint("sIr1", "sIr1E", d.ir1);
      paint("sIr2", "sIr2E", d.ir2);
    }
  });

  on("cbIr", "change", function () {
    var want = this.checked;
    send({ type: "camera_config", config: {
      ir1_en: want, ir2_en: want, show_ir1: want, show_ir2: want } });
    say("camStreamMsg", want
      ? "Turning the infrared pair on. The picture restarts — give it a second."
      : "Infrared pair off.", "info");
  });

  ["sDepth", "sColor"].forEach(function (id) {
    on(id, "click", function (ev) {
      if (!S.require("camStreamMsg")) return;
      var r = this.getBoundingClientRect();
      send({ type: "camera_point",
             x: Math.round((ev.clientX - r.left) * this.width / r.width),
             y: Math.round((ev.clientY - r.top) * this.height / r.height),
             window: 5 });
    });
  });

  S.on("camera_point_res", function (d) {
    if (!d.ok) { say("camStreamMsg", d.error || "No depth at that point.", "warn"); return; }
    say("camStreamMsg", "That point is " + fmt(d.depth_mm, 0) + " mm away"
      + (d.point_m ? " — " + d.point_m.map(function (v) {
          return fmt(v * 1000, 0); }).join(", ") + " mm from the camera." : "."), "ok");
  });

  seg("emitSeg", "mode", function (mode) {
    if (!S.require("emitMsg")) return;
    send({ type: "rs_emitter", mode: mode, laser_power: num("laserPow", 150) });
  });
  on("laserPow", "input", function () { $("laserPowV").textContent = this.value; });
  on("laserPow", "change", function () {
    send({ type: "rs_emitter",
           mode: ($("emitSeg").querySelector(".segb.on") || {}).dataset.mode || "on",
           laser_power: num("laserPow", 150) });
  });

  S.on("rs_res", function (d) {
    if (d.cmd === "emitter") {
      say("emitMsg", d.ok ? "Projector set: " + (d.why || "") + "."
        : "Could not change the projector: " + (d.error || "unknown"), d.ok ? "ok" : "bad");
    } else if (d.cmd === "pointcloud") {
      say("camStreamMsg", d.ok
        ? "Saved " + d.n_points.toLocaleString() + " points to " + d.path
        : "Could not save: " + (d.error || ""), d.ok ? "ok" : "bad");
    } else if (d.cmd === "options") {
      var bad = (d.results || []).filter(function (r) { return !r.ok; });
      say("rsInfoMsg", bad.length ? bad.map(function (r) {
        return r.option + ": " + r.error; }).join("; ") : "Applied.",
        bad.length ? "warn" : "ok");
    }
  });

  on("btnCloud", "click", function () {
    if (!S.require("camStreamMsg")) return;
    say("camStreamMsg", "Saving…", "info");
    send({ type: "rs_pointcloud", stride: 2 });
  });
  on("btnMeta", "click", function () {
    if (!S.require("camStreamMsg")) return; send({ type: "rs_metadata" });
  });
  S.on("rs_metadata_res", function (d) {
    if (!d.available) { say("camStreamMsg", "No frame timing available yet.", "warn"); return; }
    var msg = "Frames are stamped on " + (d.hardware_clock
      ? "the camera's own clock — good, that is what the timing budget assumes."
      : "the PC's clock, not the camera's. That adds USB delay to every frame time.");
    if (d.actual_exposure) msg += " Exposure now: " + d.actual_exposure + " µs.";
    say("camStreamMsg", msg, d.hardware_clock ? "ok" : "warn");
  });

  on("btnIrWhy", "click", function () {
    if (!S.require("camStreamMsg")) return; send({ type: "rs_ir_diagnose" });
  });
  S.on("rs_ir_diagnose_res", function (d) {
    if (!d.ok) { say("camStreamMsg", d.error || "Could not read the infrared image.", "warn"); return; }
    var c = (d.causes || [])[0] || {};
    var lead = c.cause === "none found"
      ? "The infrared image looks fine. "
      : "Most likely cause: " + c.cause + ". ";
    say("camStreamMsg", lead + (c.detail || "") + ". What to do: " + (c.fix || "")
      + (d.depth_fill != null ? " Depth currently covers "
        + Math.round(d.depth_fill * 100) + "% of the centre." : ""),
      c.cause === "none found" ? "ok" : "warn");
  });

  on("btnQual2", "click", function () {
    if (!S.require("calMsg")) return; send({ type: "camera_stats", roi_frac: 0.25 });
  });
  S.on("camera_stats_res", function (d) {
    if (!$("q2Fill")) return;
    if (!d.available) { say("calMsg", d.error || "Could not measure depth.", "warn"); return; }
    var pc = function (f) { return f == null ? "—" : Math.round(f * 100) + "%"; };
    $("q2Fill").textContent = pc(d.fill_all);
    $("q2Roi").textContent = pc(d.fill_roi);
    $("q2Roi").parentElement.className = "tile "
      + (d.fill_roi > 0.7 ? "ok" : d.fill_roi > 0.4 ? "warn" : "bad");
    $("q2Dist").innerHTML = d.roi_median_m !== undefined
      ? fmt(d.roi_median_m * 1000, 0) + '<span class="u">mm</span>' : "—";
    $("q2Noise").innerHTML = d.roi_std_mm !== undefined
      ? fmt(d.roi_std_mm, 2) + '<span class="u">mm</span>' : "—";
  });

  on("btnSelfCal", "click", function () {
    if (!S.require("calMsg")) return;
    say("calMsg", "Re-calibrating. Hold the camera still for about ten seconds…", "info");
    send({ type: "rs_selfcal", mode: "calibrate", speed: 2 });
  });
  on("btnTare", "click", function () {
    if (!S.require("calMsg")) return;
    say("calMsg", "Taring against " + num("tareDist", 600) + " mm. This only helps "
      + "if that distance is measured, not estimated…", "info");
    send({ type: "rs_selfcal", mode: "tare", target_distance_mm: num("tareDist", 600) });
  });
  S.on("rs_selfcal_res", function (d) {
    say("calMsg", d.ok
      ? "Done — " + d.verdict + " (score " + fmt(d.health, 3) + "). " + (d.note || "")
      : "Could not re-calibrate: " + (d.error || "") + " " + (d.hint || ""),
      d.ok ? "ok" : "warn");
  });

  function filterConfig() {
    return {
      enabled: true,
      threshold_on: $("fThr").checked, spatial_on: $("fSpa").checked,
      temporal_on: $("fTmp").checked, decimation_on: $("fDec").checked,
      hole_filling_on: $("fHole").checked, disparity: $("fDisp").checked,
      threshold_min: num("fMin", 0.15), threshold_max: num("fMax", 1.5),
      spatial_alpha: num("fAlpha", 0.5)
    };
  }
  on("btnFilters", "click", function () {
    if (!S.require("filtMsg")) return;
    send({ type: "rs_filters", config: filterConfig() });
  });
  S.on("rs_filters_res", function (d) {
    var steps = (d.steps || []);
    say("filtMsg", steps.length
      ? "Depth now goes through: " + steps.join(", then ") + "."
      : "No processing — raw depth straight from the camera.",
      d.config && d.config.hole_filling_on ? "warn" : "ok");
  });

  on("btnRsEnum", "click", function () {
    if (!S.require("rsInfoMsg")) return; send({ type: "rs_enumerate" });
  });
  on("btnExtr", "click", function () {
    if (!S.require("rsInfoMsg")) return; send({ type: "rs_extrinsics" });
  });
  S.on("rs_extrinsics_res", function (d) {
    if (!d.available) { say("rsInfoMsg", d.error || "Not streaming.", "warn"); return; }
    var parts = Object.keys(d.to || {}).map(function (k) {
      return k + " is " + d.to[k].translation_mm.map(function (v) {
        return fmt(v, 1); }).join(", ") + " mm from the depth camera";
    });
    say("rsInfoMsg", parts.join("; ") + ".", "info");
  });

  S.on("rs_enumerate_res", function (d) {
    var host = $("rsInfoTiles"); if (!host) return;
    if (!d.available) {
      host.innerHTML = "";
      say("rsInfoMsg", plainCamFault(d.error), "bad");
      if ($("camDevTag")) $("camDevTag").textContent = "no camera";
      return;
    }
    var tiles = [
      ["Model", d.name || "—", ""],
      ["Serial", d.serial || "—", ""],
      ["Firmware", d.firmware || "—", d.recommended_firmware
        && d.recommended_firmware !== d.firmware
        ? "recommended " + d.recommended_firmware : ""],
      ["Connection", "USB " + (d.usb || "?"), d.usb3 ? "full speed" : "too slow"]
    ];
    host.innerHTML = tiles.map(function (t) {
      return '<div class="tile"><div class="k">' + esc(t[0]) + '</div><div class="v" '
        + 'style="font-size:14px">' + esc(String(t[1])) + '</div>'
        + (t[2] ? '<div class="d">' + esc(t[2]) + "</div>" : "") + "</div>";
    }).join("");
    if ($("camDevTag")) $("camDevTag").textContent = (d.name || "camera")
      + " · USB " + (d.usb || "?");
    var opts = (d.sensors || []).reduce(function (n, s) {
      return n + (s.options || []).length; }, 0);
    say("rsInfoMsg", d.usb_warning || ((d.sensors || []).length
      + " sensors, " + opts + " settings this camera actually supports. "
      + "Every range shown comes from the camera itself."),
      d.usb_warning ? "warn" : "info");
  });

  /* The operator sees sentences, not Python. Driver faults arrive as import
     errors and SDK messages; each of the ones that actually happens has a
     different fix, so they are named rather than pasted. */
  function plainCamFault(err) {
    var e = String(err || "");
    if (/pyrealsense2/i.test(e)) {
      return "The camera software is not installed on this PC. In the agent's "
        + "window run:  pip install pyrealsense2  then restart the agent. "
        + "Everything else — the robot, the motion sensors, recording — keeps "
        + "working without it.";
    }
    if (/no RealSense camera found|no camera/i.test(e)) {
      return "No camera found. Check the USB cable is in a blue USB 3 port, "
        + "and close RealSense Viewer if it is open — only one program can "
        + "hold the camera at a time.";
    }
    if (/busy|access|in use|Device or resource/i.test(e)) {
      return "The camera is being held by another program. Close RealSense "
        + "Viewer, or any other agent window, and press Refresh.";
    }
    if (/permission|denied/i.test(e)) {
      return "Windows refused access to the camera. Unplug it, plug it back "
        + "in, and press Refresh.";
    }
    return "The camera could not be read. " + (e ? "The agent reported: " + e
      + ". " : "") + "Unplug it, plug it back in, and press Refresh.";
  }

  /* =======================================================================
     2. MOTION SENSORS — link setup, discovery, attitude, traces
     ======================================================================= */
  var CFG_FIELDS = {
    "udp-listen": [["port", "Port on this PC", "number", 5005]],
    "tcp-client": [["host", "FusionHub address", "text", "127.0.0.1"],
                   ["port", "Port", "number", 5005]],
    "tcp-listen": [["port", "Port to listen on", "number", 5005]],
    "zmq-sub": [["endpoint", "Endpoint from FusionHub", "text", "tcp://*:8901"],
                ["topic", "Topic filter (blank = everything)", "text", ""]],
    "websocket-client": [["url", "WebSocket address",
                          "text", "ws://127.0.0.1:8080"]],
    "http-poll": [["url", "Web address", "text", "http://127.0.0.1:8080/api/imu"],
                  ["rate_hz", "Readings per second", "number", 50]],
    "serial": [["port", "COM port", "text", "COM3"],
               ["baud", "Speed", "number", 115200]],
    "file-tail": [["path", "File FusionHub is writing", "text", ""]]
  };

  function renderCfgFields() {
    var kind = ($("imuKind") || {}).value || "udp-listen";
    var host = $("imuCfgFields"); if (!host) return;
    host.innerHTML = (CFG_FIELDS[kind] || []).map(function (f) {
      return '<div class="field"><label>' + esc(f[1]) + "</label>"
        + '<input id="imucfg_' + f[0] + '" type="' + f[2] + '" value="'
        + esc(String(f[3])) + '"/></div>';
    }).join("");
  }
  on("imuKind", "change", renderCfgFields);
  renderCfgFields();

  function linkConfig() {
    var kind = ($("imuKind") || {}).value || "udp-listen";
    var cfg = {};
    (CFG_FIELDS[kind] || []).forEach(function (f) {
      var el = $("imucfg_" + f[0]);
      if (!el) return;
      cfg[f[0]] = f[2] === "number" ? Number(el.value) : el.value;
    });
    return { kind: kind, config: cfg };
  }

  on("btnImuStart", "click", function () {
    if (!S.require("imuMsg")) return;
    var lc = linkConfig();
    say("imuMsg", "Connecting…", "info");
    send({ type: "imu_link_start", unit: $("imuUnit").value, kind: lc.kind,
           config: lc.config, gyro_units: $("imuUnits").value });
  });
  on("btnImuStop", "click", function () {
    if (!S.require("imuMsg")) return;
    send({ type: "imu_link_stop", unit: $("imuUnit").value });
  });
  on("btnImuFind", "click", function () {
    if (!S.require("imuMsg")) return;
    say("imuMsg", "Listening on every likely port for six seconds. Make sure "
      + "FusionHub is streaming now…", "info");
    send({ type: "imu_discover", seconds: 6 });
  });
  on("btnImuRaw", "click", function () {
    if (!S.require("imuMsg")) return;
    send({ type: "imu_sniff", unit: $("imuUnit").value });
  });
  on("cbD435i", "change", function () {
    send({ type: "imu_d435i", on: this.checked });
  });
  on("btnImuZero", "click", function () {
    if (!S.require("attMsg")) return;
    send({ type: "imu_zero", unit: $("imuUnit").value });
    say("attMsg", "Re-levelling. Hold the sensor still for two seconds.", "info");
  });

  S.on("imu_link_res", function (d) {
    if (d.cmd === "stop") { say("imuMsg", "Disconnected.", "info"); return; }
    if (d.ok) {
      say("imuMsg", "Connected. Waiting for the first reading — if nothing "
        + "arrives within a few seconds, use “Show what is arriving”.", "ok");
    } else {
      say("imuMsg", "Could not connect: " + (d.error || "unknown") + ".", "bad");
    }
    if ($("imuKindTag")) $("imuKindTag").textContent = d.ok ? (d.kind || "") : "not connected";
  });

  S.on("imu_discover_res", function (d) {
    if (!d.ok) { say("imuMsg", d.error || "Could not search.", "bad"); return; }
    var usable = d.usable_ports || [];
    say("imuMsg", d.advice || "", usable.length ? "ok" : "warn");
    if (usable.length) {
      // Fill the form in rather than telling the operator a port number and
      // making them type it — the point of finding it is not having to.
      $("imuKind").value = "udp-listen";
      renderCfgFields();
      var pf = $("imucfg_port"); if (pf) pf.value = usable[0];
    }
    var found = d.found || {};
    var raw = Object.keys(found).map(function (p) {
      var sn = found[p].sniff || {};
      return "UDP " + p + " from " + found[p].from + " — " + found[p].packets
        + " packets, looks like " + (sn.format || "?")
        + (sn.fields && sn.fields.length ? ", carrying " + sn.fields.join(" + ") : "")
        + "\n  " + (sn.preview || "");
    }).join("\n");
    var box = $("imuRaw");
    if (box) { box.hidden = !raw; box.textContent = raw || ""; }
  });

  S.on("imu_sniff_res", function (d) {
    var box = $("imuRaw"); if (!box) return;
    box.hidden = false;
    if (!d.ok) { box.textContent = d.error || ""; return; }
    box.textContent = "Last packet (" + d.bytes + " bytes, looks like "
      + (d.format || "?") + "):\n" + (d.preview || "")
      + "\n\nRecognised: " + ((d.fields || []).join(", ") || "nothing")
      + (d.timestamp_found ? " · has its own timestamp" : " · no timestamp, arrival time used");
    if (d.advice) say("imuMsg", d.advice, "warn");
  });

  S.on("imu_transports_res", function (d) {
    var unit = ($("imuUnit") || {}).value || "ind0";
    var l = (d.links || {})[unit];
    if (!l || !l.running) return;
    if ($("imuKindTag")) {
      $("imuKindTag").textContent = l.kind + " · " + fmt(l.rate_hz, 0) + " Hz";
    }
    // The units the gyroscope reports in are DECIDED, not assumed, and the
    // operator is told which and on what basis — a silent factor of 57 is
    // the worst kind of wrong, because every number still looks plausible.
    // Held in state rather than appended to the chip row: that row is rebuilt
    // on every reading, so anything appended to it survives about 50 ms.
    att.units = (l.gyro_units && l.gyro_units !== "deciding")
      ? { units: l.gyro_units, basis: l.gyro_units_basis || "" } : null;
  });

  S.on("imu_zero_res", function (d) {
    say("attMsg", d.ok ? "Re-levelled. " + (d.note || "") : (d.note || "Not available."),
      d.ok ? "ok" : "warn");
  });

  /* ---- attitude cube -------------------------------------------------- */
  var att = { quat: [1, 0, 0, 0], euler: [0, 0, 0], src: "", have: false,
              units: null };

  function qmat(q) {
    var w = q[0], x = q[1], y = q[2], z = q[3];
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]];
  }

  function drawCube() {
    var cv = $("attCube"); if (!cv) return;
    var ctx = cv.getContext("2d"), W = cv.width, H = cv.height;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#1c232d"; ctx.fillRect(0, 0, W, H);
    if (!att.have) {
      ctx.fillStyle = "#6e7f90"; ctx.font = "13px system-ui"; ctx.textAlign = "center";
      ctx.fillText("No motion data yet", W / 2, H / 2);
      return;
    }
    var R = qmat(att.quat), s = Math.min(W, H) * 0.32, cx = W / 2, cy = H / 2 + 4;
    // A fixed three-quarter view, so a change on screen is always a change in
    // the sensor and never the camera drifting.
    function proj(p) {
      var v = [R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2],
               R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2],
               R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2]];
      var a = 0.62, b = 0.36;
      return [cx + (v[0] - v[1] * a) * s, cy - (v[2] - v[1] * b) * s, v[1]];
    }
    // ground shadow, so tilt reads even when the box is edge-on
    ctx.strokeStyle = "#2b3441"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.ellipse(cx, cy + s * 1.35, s * 1.5, s * 0.42, 0, 0, 6.3);
    ctx.stroke();

    var V = [[-1, -.66, -.28], [1, -.66, -.28], [1, .66, -.28], [-1, .66, -.28],
             [-1, -.66, .28], [1, -.66, .28], [1, .66, .28], [-1, .66, .28]];
    var P = V.map(proj);
    var F = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4],
             [2, 3, 7, 6], [1, 2, 6, 5], [0, 3, 7, 4]];
    F.map(function (f) {
      return { f: f, d: (P[f[0]][2] + P[f[1]][2] + P[f[2]][2] + P[f[3]][2]) / 4 };
    }).sort(function (a, b) { return b.d - a.d; }).forEach(function (o) {
      ctx.beginPath();
      o.f.forEach(function (i, k) {
        k ? ctx.lineTo(P[i][0], P[i][1]) : ctx.moveTo(P[i][0], P[i][1]);
      });
      ctx.closePath();
      var shade = Math.round(40 + 40 * (1 - o.d));
      ctx.fillStyle = "rgba(" + shade + "," + (shade + 18) + "," + (shade + 34) + ",.92)";
      ctx.fill();
      ctx.strokeStyle = "#48576a"; ctx.lineWidth = 1.2; ctx.stroke();
    });
    // axis triad, labelled — the colours match the traces below
    [[1.75, 0, 0, 0], [0, 1.35, 0, 1], [0, 0, 1.15, 2]].forEach(function (a) {
      var p = proj([a[0], a[1], a[2]]);
      ctx.strokeStyle = SERIES[a[3]]; ctx.lineWidth = 2.6;
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(p[0], p[1]); ctx.stroke();
      ctx.fillStyle = SERIES[a[3]];
      ctx.font = "bold 12px ui-monospace,monospace"; ctx.textAlign = "center";
      ctx.fillText(AXIS_NAMES[a[3]], p[0], p[1] - 5);
    });
    // gravity, for reference
    ctx.strokeStyle = "#6e7f90"; ctx.lineWidth = 1.6; ctx.setLineDash([4, 3]);
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx, cy + s * 1.3); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = "#6e7f90"; ctx.font = "10px system-ui"; ctx.textAlign = "center";
    ctx.fillText("down", cx, cy + s * 1.3 + 13);
  }

  /* ---- live traces ---------------------------------------------------- */
  var strip = { ch: "gyro", buf: [], max: 420 };
  seg("stripSeg", "ch", function (c) { strip.ch = c; strip.buf = []; drawStrip(); });

  function drawStrip() {
    var cv = $("imuStrip"); if (!cv) return;
    var ctx = cv.getContext("2d"), W = cv.width, H = cv.height;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#1c232d"; ctx.fillRect(0, 0, W, H);
    var pad = 30, n = strip.buf.length;
    if (n < 2) {
      ctx.fillStyle = "#6e7f90"; ctx.font = "12px system-ui"; ctx.textAlign = "center";
      ctx.fillText("Waiting for readings", W / 2, H / 2);
      return;
    }
    var lo = Infinity, hi = -Infinity;
    strip.buf.forEach(function (s) {
      for (var i = 0; i < 3; i++) { if (s[i] < lo) lo = s[i]; if (s[i] > hi) hi = s[i]; }
    });
    if (!(hi > lo)) { hi = lo + 1; }
    var span = hi - lo; lo -= span * 0.12; hi += span * 0.12;
    var y = function (v) { return H - pad / 2 - (v - lo) / (hi - lo) * (H - pad); };
    // zero line, when zero is in range — it is the reference every one of
    // these channels is read against
    if (lo < 0 && hi > 0) {
      ctx.strokeStyle = "#2b3441"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(0, y(0)); ctx.lineTo(W, y(0)); ctx.stroke();
    }
    for (var a = 0; a < 3; a++) {
      ctx.strokeStyle = SERIES[a]; ctx.lineWidth = 2; ctx.beginPath();
      for (var i = 0; i < n; i++) {
        var px = i / (strip.max - 1) * W, py = y(strip.buf[i][a]);
        i ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
      }
      ctx.stroke();
      // direct label at the live end: identity never rests on colour alone
      var last = strip.buf[n - 1][a];
      var lx = Math.min(W - 6, (n - 1) / (strip.max - 1) * W + 6);
      ctx.fillStyle = SERIES[a];
      ctx.font = "bold 11px ui-monospace,monospace"; ctx.textAlign = "left";
      ctx.fillText(AXIS_NAMES[a], lx, y(last) + 4);
    }
    ctx.fillStyle = "#6e7f90"; ctx.font = "10px ui-monospace,monospace";
    ctx.textAlign = "left";
    ctx.fillText(fmt(hi, 2), 5, 12);
    ctx.fillText(fmt(lo, 2), 5, H - 5);
    var unit = strip.ch === "gyro" ? "deg/s" : strip.ch === "accel" ? "m/s²" : "deg";
    ctx.textAlign = "right"; ctx.fillText(unit, W - 6, 12);
  }

  S.on("imu", function (d) {
    var units = d.units || {};
    var unit = ($("imuUnit") || {}).value || "ind0";
    var u = units[unit] || units[d.unit] || units[Object.keys(units)[0]];
    if (!u) return;
    if (u.quat) { att.quat = u.quat; att.have = true; }
    if (u.euler_deg) att.euler = u.euler_deg;
    att.src = u.quat_source || "";

    if ($("attRoll")) {
      $("attRoll").innerHTML = fmt(att.euler[0], 1) + '<span class="u">°</span>';
      $("attPitch").innerHTML = fmt(att.euler[1], 1) + '<span class="u">°</span>';
      $("attYaw").innerHTML = fmt(att.euler[2], 1) + '<span class="u">°</span>';
      $("attRate").innerHTML = fmt(u.gyro_norm_deg_s, 1) + '<span class="u">°/s</span>';
      $("attAcc").innerHTML = fmt(u.accel_norm, 2) + '<span class="u">m/s²</span>';
      $("attHz").innerHTML = fmt(u.rate_hz, 0) + '<span class="u">Hz</span>';
    }
    if ($("attSrcTag")) {
      $("attSrcTag").textContent = att.src === "device"
        ? "from the sensor" : att.src === "estimated" ? "worked out here" : "—";
    }

    var chips = [];
    if (u.quat_source === "estimated") {
      chips.push(["warn", "no compass — the yaw number drifts"]);
    }
    if (u.still) chips.push(["ok", "still"]);
    if (u.gyro_bias_deg_s) {
      var wb = Math.max.apply(null, u.gyro_bias_deg_s.map(Math.abs));
      chips.push([wb > 2 ? "warn" : "ok", "drift correction " + fmt(wb, 2) + " °/s"]);
    }
    if (u.device_vs_estimate_deg != null) {
      chips.push([u.device_vs_estimate_deg > 3 ? "warn" : "ok",
        "sensor vs our own estimate: " + fmt(u.device_vs_estimate_deg, 1) + "°"]);
    }
    if (u.filter_disagreement_deg != null && u.filter_disagreement_deg > 5) {
      chips.push(["warn", "the two estimators disagree — the data is noisy"]);
    }
    if (u._units_pending) {
      chips.push(["warn", "working out whether the turn rate is in degrees "
        + "or radians — hold on"]);
    } else if (att.units) {
      chips.push(["ok", "turn rate read as " + (att.units.units === "deg"
        ? "degrees/s" : "radians/s")]);
    }
    if ($("attChips")) {
      $("attChips").innerHTML = chips.map(function (c) {
        return '<span class="chip ' + c[0] + '">' + esc(c[1]) + "</span>";
      }).join("");
    }

    // Readings taken before the source settled what units its gyroscope uses
    // are not plotted, and the trace is cleared when the verdict lands. One
    // pre-verdict sample is a factor-of-57 spike that sets the whole y-axis,
    // which makes the several minutes of real data after it unreadable.
    if (u._units_pending) { strip.pending = true; }
    else if (strip.pending) { strip.pending = false; strip.buf = []; }
    if (!u._units_pending) {
      var v = strip.ch === "gyro"
        ? (u.gyro || [0, 0, 0]).map(function (r) { return r * 180 / Math.PI; })
        : strip.ch === "accel" ? (u.accel || [0, 0, 0]) : att.euler;
      strip.buf.push(v);
      while (strip.buf.length > strip.max) strip.buf.shift();
    }

    if ($("page-sensors") && !$("page-sensors").hidden) { drawCube(); drawStrip(); }
    if (!att.msgDone) {
      att.msgDone = true;
      say("attMsg", "Readings are arriving.", "ok");
      say("imuMsg", "Connected and receiving.", "ok");
      // Ask once the data is flowing: the units verdict only exists after
      // the link has seen enough of the stream to decide.
      setTimeout(function () { send({ type: "imu_transports" }); }, 2500);
    }
  });

  S.on("sensors_report_res", function (d) {
    var body = $("sensorTable"); if (!body) return;
    if (!d.ok) {
      body.innerHTML = '<tr><td colspan="6" style="color:var(--text-3)">'
        + esc(d.error || "Unavailable.") + "</td></tr>";
      return;
    }
    var STATE = { streaming: ["ok", "live"], present: ["ok", "connected"],
                  stale: ["warn", "no data"], failed: ["bad", "faulty"],
                  declared: ["", "not fitted yet"] };
    body.innerHTML = (d.channels || []).map(function (c) {
      var st = STATE[c.status] || ["", c.status];
      return "<tr><td>" + esc(c.label) + '<div style="color:var(--text-3);'
        + 'font-size:11px">' + esc(c.vendor || c.transport || "") + "</div></td>"
        + "<td>" + esc(String(c.modality).replace(/_/g, " ")) + '<div style="'
        + 'color:var(--text-3);font-size:11px">' + esc(c.units || "") + "</div></td>"
        + '<td class="n">' + (c.rate_hz ? fmt(c.rate_hz, 0) + " Hz" : "—") + "</td>"
        + "<td>" + esc(c.frame) + "</td>"
        + "<td>" + (c.scored
            ? '<span class="chip ok">scored</span>'
            : '<span class="chip">inspection</span>') + "</td>"
        + '<td><span class="chip ' + st[0] + '">' + esc(st[1]) + "</span></td></tr>";
    }).join("");
    say("sensorMsg", d.n_live + " of " + d.n_total + " channels are live; "
      + d.n_declared + " are wired up and waiting for hardware. "
      + (d.benchmark_channels || []).length + " count toward the benchmark."
      + ((d.conflicts || []).length ? " " + d.conflicts[0] : ""),
      (d.conflicts || []).length ? "warn" : "info");
  });

  S.page("sensors", function () {
    drawCube(); drawStrip();
    if (S.connected()) { send({ type: "sensors_report" }); send({ type: "imu_transports" }); }
  });

  /* =======================================================================
     3. HAND-EYE CALIBRATION
     ======================================================================= */
  var cal = { on: false, timer: null, n: 0, solved: null, corners: null, lastFrame: null };

  function calTarget() {
    return { kind: $("calKind").value, cols: num("calCols", 9),
             rows: num("calRows", 6), square_mm: num("calSq", 25) };
  }
  function calFlow(step) {
    var el = $("calFlow"); if (!el) return;
    el.querySelectorAll(".fs").forEach(function (f, i) {
      f.classList.toggle("done", i < step);
      f.classList.toggle("active", i === step);
    });
  }

  on("btnCalBegin", "click", function () {
    if (!S.require("calAdvice")) return;
    send({ type: "handeye_begin", target: calTarget() });
    cal.on = true;
    ["btnCalCap", "btnCalUndo", "btnCalClear"].forEach(function (b) { $(b).disabled = false; });
    calFlow(1);
  });
  on("btnCalCap", "click", function () {
    if (!S.require("calAdvice")) return; send({ type: "handeye_capture" });
  });
  on("btnCalUndo", "click", function () { send({ type: "handeye_undo" }); });
  on("btnCalClear", "click", function () { send({ type: "handeye_clear" }); });
  on("btnCalSolve", "click", function () {
    if (!S.require("calVerdict")) return;
    say("calVerdict", "Working it out…", "info");
    send({ type: "handeye_solve", method: "all", apply: true });
  });
  on("btnCalSave", "click", function () { send({ type: "handeye_save" }); });
  on("btnCalLoad", "click", function () { send({ type: "handeye_load" }); });

  S.on("handeye_status_res", function (d) {
    if (!d.available) {
      say("calLiveMsg", "Calibration needs OpenCV on this PC. In the agent's "
        + "window run:  pip install opencv-python", "bad");
      return;
    }
    if (d.saved && d.saved.calib_version) {
      say("calSaveMsg", "Saved calibration " + esc(d.saved.calib_version)
        + " — accurate to " + fmt(d.saved.target_spread_mm, 1) + " mm."
        + (d.applied ? " In use now." : " Not loaded — press “Load the saved one”."),
        d.applied ? "ok" : "info");
    }
    if (typeof d.n === "number") renderCalN(d);
  });

  function renderCalN(d) {
    cal.n = d.n || 0;
    if ($("calN")) $("calN").textContent = cal.n;
    if ($("calNTag")) $("calNTag").textContent = cal.n + " captured";
    var r = d.readiness || d;
    if ($("calRot")) $("calRot").innerHTML = r.max_rotation_deg != null
      ? fmt(r.max_rotation_deg, 0) + '<span class="u">°</span>' : "—";
    if ($("calRep")) $("calRep").innerHTML = r.mean_reprojection_px != null
      ? fmt(r.mean_reprojection_px, 2) + '<span class="u">px</span>' : "—";
    if ($("btnCalSolve")) $("btnCalSolve").disabled = !(cal.n >= 5);
    var adv = (r.advice || []);
    if (adv.length) {
      // "Ready" and "nothing to improve" are different things: a pose set can
      // be solvable and still have a bad frame in it. Green means nothing to
      // fix, not merely that the solve will run.
      var clean = r.ready && adv.length === 1 && /looks good/i.test(adv[0]);
      say("calAdvice", adv.join(" "), clean ? "ok" : "warn");
    }
    if ($("calList") && d.samples) {
      $("calList").innerHTML = d.samples.map(function (s, i) {
        return '<div class="sl"><div>Pose ' + (i + 1) + '</div><div class="sv">'
          + s.distance_mm + " mm · " + s.corners + " corners · "
          + fmt(s.reprojection_px, 2) + " px</div></div>";
      }).join("");
    }
    if (cal.n >= 5) calFlow(2);
  }

  S.on("handeye_capture_res", function (d) {
    if (!d.ok) { say("calAdvice", d.error || "Could not use that pose.", "bad"); return; }
    renderCalN(d);
  });

  S.on("handeye_preview_res", function (d) {
    cal.corners = d.ok ? d.corners : null;
    if (d.frame) {
      // The overlay and its image must come from the same frame.
      var bin = atob(d.frame), arr = new Uint8Array(bin.length);
      for (var i = 0; i < bin.length; i++) arr[i] = bin.charCodeAt(i);
      createImageBitmap(new Blob([arr], { type: "image/jpeg" }))
        .then(function (bmp) {
          if (cal.frame && cal.frame.close) cal.frame.close();
          cal.frame = bmp;
          drawCal();
        }).catch(function () {});
    }
    if ($("calSeenTag")) {
      $("calSeenTag").textContent = d.ok
        ? "board found · " + (d.distance_mm || "?") + " mm" : "no board";
    }
    if (!d.ok && d.error) say("calLiveMsg", d.error, "warn");
    else if (d.ok) {
      say("calLiveMsg", "Board found, " + d.n_corners + " corners"
        + (d.reprojection_px != null ? ", accurate to " + fmt(d.reprojection_px, 2)
          + " px" : "") + ". Move the arm and capture.", "ok");
    }
    drawCal();
  });

  S.on("handeye_solve_res", function (d) {
    if (!d.ok) { say("calVerdict", d.error || "Could not solve.", "bad"); return; }
    cal.solved = d;
    $("calT").textContent = d.translation_mm.map(function (v) { return fmt(v, 1); }).join(", ");
    $("calR").textContent = d.rotation_deg.map(function (v) { return fmt(v, 1); }).join(", ");
    $("calErr").innerHTML = fmt(d.target_spread_mm, 2) + '<span class="u">mm</span>';
    var tile = $("calTileErr");
    if (tile) {
      tile.className = "tile " + (d.target_spread_mm <= 2 ? "ok"
        : d.target_spread_mm <= 5 ? "warn" : "bad");
    }
    var good = d.target_spread_mm <= 5 && !/Do not use/.test(d.verdict || "");
    say("calVerdict", d.verdict + (d.warning ? " " + d.warning : ""),
      good ? "ok" : "bad");
    $("btnCalSave").disabled = !good;
    if (good) {
      calFlow(3);
      // Panels elsewhere warn that the camera position is unknown. It is now
      // known, so refresh them rather than leaving a warning that was true
      // thirty seconds ago.
      send({ type: "inspect_status" });
      send({ type: "mv_status" });
    }
  });

  S.on("handeye_res", function (d) {
    if (d.cmd === "save") {
      say("calSaveMsg", d.ok ? "Saved as " + esc(d.calib_version) + ". Use that "
        + "name on every run recorded from now on." : (d.error || ""), d.ok ? "ok" : "bad");
    } else if (d.cmd === "load") {
      say("calSaveMsg", d.ok ? "Loaded " + esc(d.calib_version || "") + " and in use."
        : (d.error || ""), d.ok ? "ok" : "warn");
      if (d.ok) calFlow(3);
    } else if (d.cmd === "begin") {
      say("calAdvice", d.note || "", "info");
    } else if (d.ok === false) {
      say("calAdvice", d.error || "", "bad");
    }
  });

  function drawCal() {
    var cv = $("calCanvas"); if (!cv) return;
    var img = cal.frame || S.state.frames.color;
    var ctx = cv.getContext("2d");
    if (img) { cv.width = img.naturalWidth || img.width; cv.height = img.naturalHeight || img.height; }
    ctx.clearRect(0, 0, cv.width, cv.height);
    if (img) { ctx.drawImage(img, 0, 0, cv.width, cv.height); $("calEmpty").hidden = true; }
    else { ctx.fillStyle = "#0a0e13"; ctx.fillRect(0, 0, cv.width, cv.height); }
    if (!cal.corners || !cal.corners.length) return;
    ctx.save();
    ctx.strokeStyle = "#e8c66a"; ctx.lineWidth = 2;
    ctx.beginPath();
    cal.corners.forEach(function (p, i) { i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]); });
    ctx.stroke();
    ctx.fillStyle = "#3ddc84";
    cal.corners.forEach(function (p) {
      ctx.beginPath(); ctx.arc(p[0], p[1], 3, 0, 6.3); ctx.fill();
    });
    ctx.restore();
  }

  S.page("calib", function () {
    drawCal();
    if (S.connected()) send({ type: "handeye_status" });
    if (cal.timer) clearInterval(cal.timer);
    // Poll the detector a few times a second while this page is open, and
    // stop the moment it is not: a board search on every frame costs the
    // agent more than it costs the operator to wait 300 ms.
    cal.timer = setInterval(function () {
      var p = $("page-calib");
      if (!p || p.hidden) { clearInterval(cal.timer); cal.timer = null; return; }
      if (S.connected()) send({ type: "handeye_preview", target: calTarget() });
    }, 350);
  });

  /* =======================================================================
     4. MULTI-VIEW 3D SCAN (on the Inspect page)
     ======================================================================= */
  var mv = { roi: null, drag: null, plan: null, model: null, view: "live",
             auto: false, autoIdx: 0, autoTimer: null };

  on("btnRoi", "click", function () {
    mv.drag = { arm: true };
    S.swallowInspectClick = true;
    $("roiHint").hidden = false;
    $("roiHint").textContent = "Drag a box around the part.";
  });

  (function bindRoi() {
    var cv = $("inspCanvas"); if (!cv) return;
    function at(ev) {
      var r = cv.getBoundingClientRect();
      return [Math.round((ev.clientX - r.left) * cv.width / r.width),
              Math.round((ev.clientY - r.top) * cv.height / r.height)];
    }
    cv.addEventListener("pointerdown", function (ev) {
      if (!mv.drag || !mv.drag.arm) return;
      var p = at(ev);
      mv.drag = { arm: true, live: true, x0: p[0], y0: p[1], x1: p[0], y1: p[1] };
      cv.setPointerCapture(ev.pointerId);
      ev.preventDefault();
    });
    cv.addEventListener("pointermove", function (ev) {
      if (!mv.drag || !mv.drag.live) return;
      var p = at(ev); mv.drag.x1 = p[0]; mv.drag.y1 = p[1];
      S.drawInspect();
    });
    ["pointerup", "pointercancel"].forEach(function (e) {
      cv.addEventListener(e, function () {
        if (!mv.drag || !mv.drag.live) return;
        var d = mv.drag;
        var x = Math.min(d.x0, d.x1), y = Math.min(d.y0, d.y1);
        var w = Math.abs(d.x1 - d.x0), h = Math.abs(d.y1 - d.y0);
        mv.drag = null;
        S.swallowInspectClick = false;
        if (w < 12 || h < 12) {
          $("roiHint").textContent = "That box is too small — try again.";
          return;
        }
        mv.roi = [x, y, w, h];
        $("roiHint").hidden = true;
        $("btnMvRegion").disabled = false;
        say("mvPlanMsg", "Box drawn. Press “Use the box I drew”.", "info");
        S.drawInspect();
      });
    });
  })();

  S.on("inspect_draw", function (e) {
    var ctx = e.ctx;
    if (mv.view === "model" && mv.model) { drawModel(ctx, e.w, e.h); return; }
    var box = null;
    if (mv.drag && mv.drag.live) {
      box = [Math.min(mv.drag.x0, mv.drag.x1), Math.min(mv.drag.y0, mv.drag.y1),
             Math.abs(mv.drag.x1 - mv.drag.x0), Math.abs(mv.drag.y1 - mv.drag.y0)];
    } else if (mv.roi) { box = mv.roi; }
    if (!box) return;
    ctx.save();
    ctx.strokeStyle = "#2f81f7"; ctx.lineWidth = Math.max(2, e.w / 360);
    ctx.setLineDash([9, 6]);
    ctx.strokeRect(box[0], box[1], box[2], box[3]);
    ctx.setLineDash([]);
    ctx.fillStyle = "rgba(47,129,247,.10)";
    ctx.fillRect(box[0], box[1], box[2], box[3]);
    ctx.restore();
  });

  function drawModel(ctx, W, H) {
    var pts = mv.model.points;
    ctx.fillStyle = "#0a0e13"; ctx.fillRect(0, 0, W, H);
    if (!pts || !pts.length) return;
    var xs = pts.map(function (p) { return p[0]; });
    var ys = pts.map(function (p) { return p[1]; });
    var zs = pts.map(function (p) { return p[2]; });
    var mnx = Math.min.apply(null, xs), mxx = Math.max.apply(null, xs);
    var mny = Math.min.apply(null, ys), mxy = Math.max.apply(null, ys);
    var mnz = Math.min.apply(null, zs), mxz = Math.max.apply(null, zs);
    var spanx = mxx - mnx || 1, spany = mxy - mny || 1, spanz = mxz - mnz || 1;
    var cx = (mnx + mxx) / 2, cy = (mny + mxy) / 2, cz = (mnz + mxz) / 2;
    var s = 0.62 * Math.min(W / (spanx + spany * 0.6),
                            H / (spanz + spany * 0.4));
    // A fixed three-quarter view with height mapped to colour, so the shape
    // of the surface is readable without the operator having to orbit it.
    pts.forEach(function (p) {
      var px = W / 2 + ((p[0] - cx) - (p[1] - cy) * 0.55) * s;
      var py = H / 2 + 14 - ((p[2] - cz) - (p[1] - cy) * 0.34) * s;
      var t = (p[2] - mnz) / spanz;
      ctx.fillStyle = "hsl(" + Math.round(215 - 175 * t) + ",72%,"
        + Math.round(34 + 34 * t) + "%)";
      ctx.fillRect(px, py, 2, 2);
    });
    ctx.fillStyle = "#9fb0c0"; ctx.font = "13px system-ui"; ctx.textAlign = "left";
    ctx.fillText(mv.model.n.toLocaleString() + " measured points · height shown "
      + "as colour, blue lowest", 14, H - 16);
  }

  on("btnViewModel", "click", function () {
    if (!mv.model) {
      if (!S.require("mvBuildMsg")) return;
      say("mvBuildMsg", "No model yet — take the views and build it first.", "warn");
      return;
    }
    mv.view = "model"; S.drawInspect();
  });
  ["btnViewColor", "btnViewDepth"].forEach(function (b) {
    on(b, "click", function () { mv.view = "live"; });
  });

  on("btnMvRegion", "click", function () {
    if (!S.require("mvPlanMsg") || !mv.roi) return;
    send({ type: "mv_region", roi: mv.roi });
  });
  S.on("mv_region_res", function (d) {
    if (!d.ok) { say("mvPlanMsg", d.error || "Could not measure that box.", "bad"); return; }
    $("mvSize").textContent = d.size_mm.slice(0, 3).map(function (v) {
      return fmt(v, 0); }).join(" × ");
    $("btnMvPlan").disabled = false;
    say("mvPlanMsg", (d.warning || d.note || "")
      + " The part is about " + fmt(d.distance_mm, 0) + " mm away and "
      + Math.round(d.fill * 100) + "% of the box returned depth."
      + (d.warning ? "" : " Now work out the angles."),
      d.warning ? "warn" : "ok");
  });

  on("btnMvPlan", "click", function () {
    if (!S.require("mvPlanMsg")) return;
    send({ type: "mv_plan", n_views: blankable("mvViews"),
           standoff_mm: blankable("mvStandoff"), tilt_deg: blankable("mvTilt"),
           overlap: 0.55 });
  });
  S.on("mv_plan_res", function (d) {
    if (!d.ok) {
      say("mvPlanMsg", d.error || "Could not plan the angles.", "bad");
      return;
    }
    mv.plan = d;
    fs3d("active");
    $("mvN").textContent = d.n;
    $("mvStand").innerHTML = fmt(d.standoff_mm, 0) + '<span class="u">mm</span>';
    $("btnMvBegin").disabled = false;
    var extra = d.n_rejected
      ? " " + d.n_rejected + " more were dropped because the arm cannot reach them."
      : "";
    say("mvPlanMsg", d.explain + " Covering " + fmt(d.azimuth_coverage_deg, 0)
      + "° around the part, " + fmt(d.travel_mm, 0) + " mm of travel." + extra,
      d.azimuth_coverage_deg < 200 ? "warn" : "ok");
    $("mvList").innerHTML = d.views.map(function (v) {
      return '<div class="sl"><div>View ' + (v.order + 1) + " — from "
        + fmt(v.azimuth_deg, 0) + "° around, " + fmt(v.tilt_deg, 0)
        + '° over</div><div class="sv">'
        + v.camera_xyz_mm.map(function (c) { return fmt(c, 0); }).join(", ")
        + " mm</div></div>";
    }).join("");
  });

  on("btnMvBegin", "click", function () {
    if (!S.require("mvRunMsg")) return;
    send({ type: "mv_begin", voxel_mm: num("mvVoxel", 1.5) });
  });
  on("btnMvCapture", "click", function () { send({ type: "mv_capture", stride: 2 }); });
  on("btnMvBuild", "click", function () {
    say("mvBuildMsg", "Merging the views…", "info");
    send({ type: "mv_build", min_hits: 2 });
  });
  on("btnMvPath", "click", function () {
    send({ type: "mv_plan_path", standoff_mm: num("ipStandoff", 100),
           spacing_mm: num("ipSpacing", 5), step_mm: num("ipSpacing", 5),
           margin_mm: num("ipMargin", 5) });
  });
  on("btnMvExport", "click", function () { send({ type: "mv_export" }); });

  S.on("mv_res", function (d) {
    if (d.cmd === "begin") {
      if (!d.ok) { say("mvRunMsg", d.error || "Could not start.", "bad"); return; }
      ["btnMvCapture", "btnMvAuto", "btnMvBuild"].forEach(function (b) {
        $(b).disabled = false; });
      $("mvTag").textContent = "scanning";
      say("mvRunMsg", "Scan started. Move to each planned view and press "
        + "“Take this view” — or let the arm do it.", "info");
    } else if (d.cmd === "export") {
      say("mvBuildMsg", d.ok ? "Saved " + d.n_points.toLocaleString()
        + " points to " + d.path : (d.error || ""), d.ok ? "ok" : "bad");
    } else if (d.ok === false) {
      say("mvRunMsg", d.error || "", "bad");
    }
  });

  S.on("mv_capture_res", function (d) {
    if (!d.ok) { say("mvRunMsg", d.error || "That view was not usable.", "warn"); return; }
    $("mvViewsDone").textContent = d.view + 1;
    say("mvRunMsg", "View " + (d.view + 1) + " merged — " + d.points_used
      .toLocaleString() + " points landed on the part.", "ok");
  });

  S.on("mv_build_res", function (d) {
    if (!d.ok) { say("mvBuildMsg", d.error || "Could not build the model.", "bad"); return; }
    $("mvPts").textContent = d.points.toLocaleString();
    if (d.size_mm) {
      $("mvMeasured").textContent = d.size_mm.map(function (v) {
        return fmt(v, 0); }).join(" × ");
    }
    ["btnMvPath", "btnMvExport"].forEach(function (b) { $(b).disabled = false; });
    $("mvTag").textContent = "model built";
    fs3d("done");
    say("mvBuildMsg", "Model built from " + d.views + " views. Press “3D model” "
      + "above the picture to look at it.", "ok");
    send({ type: "mv_preview", max_points: 9000 });
  });

  S.on("mv_preview_res", function (d) {
    if (!d.ok) return;
    mv.model = d;
    if (mv.view === "model") S.drawInspect();
  });

  S.on("mv_plan_path_res", function (d) {
    if (!d.ok) { say("mvBuildMsg", d.error || "Could not plan a path.", "bad"); return; }
    S.state.plan = d;
    say("mvBuildMsg", "Path planned on the measured model: "
      + (d.waypoints || []).length + " points"
      + (d.skipped ? ", skipping " + d.skipped + " places with no measurement"
        : "") + ". It follows the real surface, so the standoff is held even "
      + "where the top is not flat.", "ok");
    var run = $("btnRunScan"); if (run) run.disabled = false;
  });

  /* ---- automatic sweep: move, settle, capture, repeat ------------------ */
  on("btnMvAuto", "click", function () {
    if (!mv.plan || !mv.plan.views.length) return;
    if (!confirm("The arm will move to " + mv.plan.views.length
      + " positions and photograph the part at each one. Is the area clear?")) return;
    mv.auto = true; mv.autoIdx = 0;
    $("btnMvStopAuto").disabled = false;
    $("btnMvAuto").disabled = true;
    autoStep();
  });
  on("btnMvStopAuto", "click", function () { stopAuto("Stopped."); });

  function stopAuto(msg) {
    mv.auto = false;
    if (mv.autoTimer) { clearTimeout(mv.autoTimer); mv.autoTimer = null; }
    $("btnMvStopAuto").disabled = true;
    $("btnMvAuto").disabled = false;
    if (msg) say("mvRunMsg", msg, "info");
  }

  function autoStep() {
    if (!mv.auto) return;
    if (mv.autoIdx >= mv.plan.views.length) {
      stopAuto("All " + mv.plan.views.length + " views taken. Build the model.");
      return;
    }
    var v = mv.plan.views[mv.autoIdx];
    say("mvRunMsg", "Moving to view " + (mv.autoIdx + 1) + " of "
      + mv.plan.views.length + "…", "info");
    send({ type: "ur_movel", pose: v.tcp_pose, a: 0.6, v: 0.15 });
    // Settle before capturing. A depth frame taken while the arm is still
    // moving is smeared, and the voxel grid cannot tell afterwards which of
    // its points came from a smeared frame.
    mv.autoTimer = setTimeout(function () {
      if (!mv.auto) return;
      send({ type: "mv_capture", stride: 2 });
      mv.autoIdx++;
      mv.autoTimer = setTimeout(autoStep, 900);
    }, 3800);
  }

  S.on("close", function () { stopAuto(null); if (cal.timer) { clearInterval(cal.timer); cal.timer = null; } });

  S.page("camera", function () {
    if (S.connected()) { send({ type: "rs_enumerate" }); send({ type: "camera_stats" }); }
  });
  function fs3d(cls) {
    var e = $("fs3d"); if (e) e.className = "fs" + (cls ? " " + cls : "");
  }

  S.page("inspect", function () {
    // Both statuses, not just the 3D one: the single-view panel's warning
    // about missing calibration is stale the moment a calibration is solved,
    // and a stale warning is read as a live one.
    if (S.connected()) { send({ type: "mv_status" }); send({ type: "inspect_status" }); }
  });

  S.on("mv_status_res", function (d) {
    if ($("mvTag")) {
      $("mvTag").textContent = !d.available ? "unavailable"
        : !d.has_handeye ? "calibrate first"
        : d.built ? "model built" : d.views ? d.views + " views" : "not started";
    }
    if (!d.has_handeye && $("mvPlanMsg")) {
      say("mvPlanMsg", "The camera has not been calibrated to the tool yet. "
        + "Do step 5, Calibrate, first — without it the robot has no way to "
        + "know where the camera was looking.", "warn");
    }
  });
})();
