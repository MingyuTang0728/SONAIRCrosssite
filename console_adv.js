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
    // A plain D435 has no motion module and never will. Leaving the "use the
    // camera's own motion sensor" option enabled invites the operator to wait
    // for a channel that cannot exist, so it is disabled and the reason is
    // given where the choice is made rather than in a terminal check.
    var hasMotion = (d.sensors || []).some(function (s) {
      return /motion/i.test(s.name || "");
    });
    var cb = $("cbD435i");
    if (cb) {
      cb.disabled = !hasMotion;
      if (!hasMotion) {
        cb.checked = false;
        var lab = cb.parentElement;
        if (lab && !lab.dataset.noted) {
          lab.dataset.noted = "1";
          lab.style.opacity = ".55";
          lab.title = "This camera is a D435, which has no built-in motion "
            + "sensor. A D435i does.";
          lab.appendChild(document.createTextNode(" — this camera has none"));
        }
      }
    }
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
    var txt = "Last packet (" + d.bytes + " bytes, looks like "
      + (d.format || "?") + "):\n" + (d.preview || "");
    if (d.vectors && d.vectors.length) {
      // Protocol Buffers carries no field names, so the only useful view is
      // the decoded numbers and their magnitudes — which is also exactly what
      // identifies each channel.
      txt += "\n\nDecoded readings:";
      d.vectors.forEach(function (v) {
        txt += "\n  field " + v.field + "  ["
          + v.values.map(function (x) { return fmt(x, 4); }).join(", ")
          + "]   magnitude " + fmt(v.magnitude, 3);
      });
      txt += "\n\n(A magnitude near 9.81 is gravity, near 1.0 with four "
        + "numbers is an orientation.)";
    } else {
      txt += "\n\nRecognised: " + ((d.fields || []).join(", ") || "nothing")
        + (d.timestamp_found ? " · has its own timestamp"
                             : " · no timestamp, arrival time used");
    }
    box.textContent = txt;
    if (d.advice) say("imuMsg", d.advice, d.vectors ? "info" : "warn");
  });

  S.on("imu_transports_res", function (d) {
    var unit = ($("imuUnit") || {}).value || "ind0";
    var l = (d.links || {})[unit];
    if (!l || !l.running) return;

    // The failure this page used to show as nothing at all: packets arriving
    // and every one of them discarded. "Connected" plus an empty display is
    // indistinguishable from "not connected", so say which it is.
    if (!l.samples && l.bad > 20) {
      say("imuMsg", l.bad + " packets have arrived and none could be read. "
        + "The link is fine — the data is in a shape this agent does not "
        + "recognise. Press “Show what is arriving” and send me what it "
        + "prints.", "bad");
      return;
    }
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
    // The whole health block, kept for the Sensor information card. It comes
    // from a different message than the readings do, so it is held rather
    // than looked up when the card is drawn.
    linkHealth = l;

    // A binary stream with no field names has had its channels worked out
    // from the readings themselves. Show what that came to: a wrong guess is
    // otherwise only visible as orientation that does not follow the sensor.
    var m = l.protobuf_mapping;
    if (m && typeof m === "object") {
      att.pb = Object.keys(m).map(function (k) { return k + " = " + m[k]; });
      say("imuMsg", "Connected. This stream is binary with no field names, so "
        + "the channels were identified from the readings: "
        + att.pb.join(", ") + ". Times come from " + (l.protobuf_time_field
          ? "field " + l.protobuf_time_field + " in "
            + (l.protobuf_time_unit || "?") : "arrival time")
        + "."
        + (l.protobuf_partial ? " " + l.protobuf_partial : "")
        + " If the orientation does not follow the sensor, tell me and the "
        + "mapping can be pinned.", "ok");
    } else if (m === "working it out") {
      say("imuMsg", "Connected. Working out which field is which — give it a "
        + "second.", "info");
    }
  });

  S.on("imu_zero_res", function (d) {
    say("attMsg", d.ok ? "Re-levelled. " + (d.note || "") : (d.note || "Not available."),
      d.ok ? "ok" : "warn");
  });

  /* ---- attitude cube -------------------------------------------------- */
  var att = { quat: [1, 0, 0, 0], euler: [0, 0, 0], src: "", have: false,
              units: null, pb: null };

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

  /* ---- every reading, all at once -------------------------------------
     One card per measurement, each with its own chart, all drawn together.

     Drawing is decoupled from arrival deliberately. Readings land about
     twenty times a second and there are seven charts; redrawing all seven on
     every message means a hundred and forty canvas repaints a second on the
     same main thread that decodes the camera frames and runs the 3D view,
     and the page that results is slower than the robot it is meant to be
     watching. Arrival appends to a buffer and nothing else; a single
     animation frame draws whatever is in the buffers, at most fifteen times
     a second, and only while this page is actually on screen.
     --------------------------------------------------------------------- */
  var SERIES4 = ["#3987e5", "#c96f3c", "#1f9b86", "#a072c8"];

  var CHANNELS = [
    { key: "euler", cv: "chEuler", vals: "cEulerVals", names: ["Roll", "Pitch", "Yaw"],
      unit: "deg", card: "cardEuler", dp: 1,
      read: function (u, a) { return a.euler; } },
    { key: "accel", cv: "chAccel", vals: "cAccelVals", names: ["X", "Y", "Z"],
      unit: "m/s²", card: "cardAccel", dp: 2,
      read: function (u) { return u.accel || null; } },
    { key: "gyro", cv: "chGyro", vals: "cGyroVals", names: ["X", "Y", "Z"],
      unit: "deg/s", card: "cardGyro", dp: 2,
      // Held canonically in radians per second, shown in degrees per second,
      // because a teach pendant, a URScript speed and every datasheet the
      // operator has in front of them are in degrees.
      read: function (u) {
        return u.gyro ? u.gyro.map(function (r) { return r * 180 / Math.PI; }) : null;
      } },
    { key: "mag", cv: "chMag", vals: "cMagVals", names: ["X", "Y", "Z"],
      unit: "µT", card: "cardMag", dp: 1,
      read: function (u) { return u.mag || null; } },
    { key: "quat", cv: "chQuat", vals: "cQuatVals", names: ["W", "X", "Y", "Z"],
      unit: "", card: "cardQuat", dp: 4,
      read: function (u, a) { return a.quat; } },
    { key: "lin", cv: "chLin", vals: "cLinVals", names: ["X", "Y", "Z"],
      unit: "m/s²", card: "cardLin", dp: 2,
      read: function (u) { return u.linear_accel || null; } },
    { key: "env", cv: "chEnv", vals: "cEnvVals", names: [], unit: "",
      card: "cardEnv", dp: 2,
      read: function (u) {
        var out = [], names = [];
        if (u.temp_c != null) { out.push(u.temp_c); names.push("Temperature"); }
        if (u.pressure_hpa != null) { out.push(u.pressure_hpa); names.push("Pressure"); }
        if (u.humidity_pct != null) { out.push(u.humidity_pct); names.push("Humidity"); }
        this.names = names;
        this.unit = names.length === 1 && names[0] === "Temperature" ? "°C" : "";
        return out.length ? out : null;
      } }
  ];

  var charts = { span: 600, frozen: false, dirty: false, last: 0, buf: {}, seen: {} };
  CHANNELS.forEach(function (c) { charts.buf[c.key] = []; });

  seg("spanSeg", "span", function (v) {
    charts.span = Number(v);
    CHANNELS.forEach(function (c) {
      var b = charts.buf[c.key];
      while (b.length > charts.span) b.shift();
    });
    charts.dirty = true;
  });

  on("btnImuFreeze", "click", function () {
    charts.frozen = !charts.frozen;
    this.textContent = charts.frozen ? "Resume the charts" : "Pause the charts";
    this.classList.toggle("primary", charts.frozen);
    say("attMsg", charts.frozen
      ? "Charts paused. The sensor is still being read and, if a recording is "
        + "running, still being written to file — only the picture is held."
      : "Readings are arriving.", charts.frozen ? "warn" : "ok");
  });

  function pushSample(u) {
    if (charts.frozen) return;
    CHANNELS.forEach(function (c) {
      var v = c.read(u, att);
      if (!v) return;
      charts.seen[c.key] = true;
      var b = charts.buf[c.key];
      b.push(v.slice());
      while (b.length > charts.span) b.shift();
    });
    charts.dirty = true;
  }

  /* One generic chart. Any number of series, autoscaled together so that the
     axes mean the same thing across series — scaling each series to its own
     range makes a 0.01 wobble and a 10 deg swing look identical, which is the
     one thing a reader must never be shown. */
  function drawChart(c) {
    var cv = $(c.cv); if (!cv) return;
    var card = $(c.card);
    var ctx = cv.getContext("2d"), W = cv.width, H = cv.height;
    var b = charts.buf[c.key], n = b.length;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#151b23"; ctx.fillRect(0, 0, W, H);

    if (n < 2) {
      if (card) card.classList.toggle("off", !charts.seen[c.key]);
      ctx.fillStyle = "#6e7f90"; ctx.font = "12px system-ui"; ctx.textAlign = "center";
      ctx.fillText(charts.seen[c.key]
        ? "Waiting for readings"
        : (c.key === "env" ? "This sensor does not report temperature or pressure"
           : c.key === "mag" ? "This sensor does not report a magnetic field"
           : "No readings on this channel"), W / 2, H / 2);
      return;
    }
    if (card) card.classList.remove("off");

    var k = b[0].length, pad = 24;
    var lo = Infinity, hi = -Infinity;
    for (var i = 0; i < n; i++)
      for (var a = 0; a < k; a++) {
        var q = b[i][a];
        if (q < lo) lo = q; if (q > hi) hi = q;
      }
    if (!(hi > lo)) { hi = lo + 1; }
    var span = hi - lo; lo -= span * 0.14; hi += span * 0.14;
    var y = function (v) { return H - pad / 2 - (v - lo) / (hi - lo) * (H - pad); };

    if (lo < 0 && hi > 0) {
      ctx.strokeStyle = "#2b3441"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(0, y(0)); ctx.lineTo(W, y(0)); ctx.stroke();
    }
    for (var a2 = 0; a2 < k; a2++) {
      ctx.strokeStyle = SERIES4[a2 % 4]; ctx.lineWidth = 2;
      ctx.lineJoin = "round"; ctx.beginPath();
      for (var j = 0; j < n; j++) {
        var px = j / (charts.span - 1) * W, py = y(b[j][a2]);
        j ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
      }
      ctx.stroke();
      // Direct label at the live end: identity never rests on colour alone.
      var nm = (c.names[a2] || String(a2 + 1)).slice(0, 5);
      var lx = Math.min(W - 4 - nm.length * 6, (n - 1) / (charts.span - 1) * W + 6);
      ctx.fillStyle = SERIES4[a2 % 4];
      ctx.font = "bold 10px ui-monospace,monospace"; ctx.textAlign = "left";
      ctx.fillText(nm, lx, y(b[n - 1][a2]) + 3.5);
    }
    ctx.fillStyle = "#6e7f90"; ctx.font = "9.5px ui-monospace,monospace";
    ctx.textAlign = "left";
    ctx.fillText(fmt(hi, c.dp), 4, 11);
    ctx.fillText(fmt(lo, c.dp), 4, H - 4);
    ctx.textAlign = "right";
    ctx.fillText(Math.round(charts.span / 20) + " s window"
      + (c.unit ? " · " + c.unit : ""), W - 5, 11);
  }

  function drawVals(c) {
    var host = $(c.vals); if (!host) return;
    var b = charts.buf[c.key];
    if (!b.length) {
      host.innerHTML = '<div class="vv"><div class="n">&mdash;</div>'
        + '<div class="q">no data</div></div>';
      return;
    }
    var last = b[b.length - 1];
    host.innerHTML = last.map(function (v, i) {
      return '<div class="vv"><div class="n"><i style="background:'
        + SERIES4[i % 4] + '"></i>' + esc(c.names[i] || String(i + 1))
        + '</div><div class="q">' + fmt(v, c.dp) + "</div></div>";
    }).join("");
  }

  function drawCards() {
    CHANNELS.forEach(function (c) { drawVals(c); drawChart(c); });
  }

  /* The single draw loop. Cheap when nothing changed, silent when the page is
     not on screen, capped at 15 fps when it is. */
  function chartTick() {
    requestAnimationFrame(chartTick);
    var p = $("page-sensors");
    if (!p || p.hidden || !charts.dirty) return;
    var now = performance.now();
    if (now - charts.last < 66) return;
    charts.last = now;
    charts.dirty = false;
    drawCube();
    drawCards();
  }
  requestAnimationFrame(chartTick);

  /* ---- the sensor-information card ------------------------------------ */
  var linkHealth = null;

  function renderInfo(u) {
    var host = $("cInfoBody"); if (!host) return;
    var l = linkHealth || {};
    var rows = [];
    function row(k, v) { if (v != null && v !== "") rows.push([k, v]); }
    row("Sensor", $("imuUnit") ? $("imuUnit").selectedOptions[0].textContent : "—");
    row("Connection", l.kind || "—");
    row("Data format", l.format === "protobuf" ? "binary (Protocol Buffers)"
      : l.format || "—");
    row("Readings received", l.samples != null ? String(l.samples) : "—");
    row("Unreadable packets", l.bad != null ? String(l.bad) : "—");
    row("Arriving at", u && u.rate_hz != null ? fmt(u.rate_hz, 0) + " Hz" : "—");
    row("Orientation from", u && u.quat_source === "device"
      ? "the sensor's own fusion" : u && u.quat_source === "estimated"
      ? "worked out here from gyro and gravity" : "—");
    row("Turn rate units", att.units
      ? (att.units.units === "deg" ? "degrees/s" : "radians/s") : "working it out");
    row("Timestamps", l.protobuf_time_field
      ? "field " + l.protobuf_time_field + ", " + (l.protobuf_time_unit || "?")
      : "time of arrival");
    if (u && u.clock_jumps) {
      row("Clock faults", u.clock_jumps + " — the sensor's own timestamp "
        + "jumped; readings are still good, their spacing is taken from "
        + "arrival instead");
    }
    var m = l.protobuf_mapping;
    if (m && typeof m === "object") {
      Object.keys(m).forEach(function (kk) { row(kk, m[kk]); });
    }
    host.innerHTML = rows.map(function (r) {
      return '<div class="kvr"><span>' + esc(r[0]) + "</span><b>"
        + esc(String(r[1])) + "</b></div>";
    }).join("");
    if ($("cInfoTag")) {
      $("cInfoTag").textContent = l.running ? "connected"
        : l.kind ? "not running" : "—";
      $("cInfoTag").className = "tag " + (l.running ? "ok" : "");
    }
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
    // What the two orientations can honestly be compared on is the direction
    // of gravity. Our own estimate has no compass — it runs on gyroscope and
    // accelerometer alone — so its heading starts at zero and stays
    // arbitrary, while the sensor fuses a magnetometer and reports a real
    // one. Differencing the WHOLE rotation therefore reported the heading
    // offset, routinely well over a hundred degrees, as if it were an error.
    // It read as a broken sensor and was nothing of the kind.
    if (u.device_vs_estimate_tilt_deg != null) {
      chips.push([u.device_vs_estimate_tilt_deg > 3 ? "warn" : "ok",
        "sensor and our own estimate agree on which way is down to "
        + fmt(u.device_vs_estimate_tilt_deg, 1) + "°"]);
    }
    if (u.clock_jumps) {
      chips.push(["warn", "the sensor's own clock jumped " + u.clock_jumps
        + (u.clock_jumps === 1 ? " time" : " times")
        + " — spacing taken from arrival instead"]);
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
    if (att.pb) chips.push(["ok", "binary stream, channels identified"]);
    if ($("attChips")) {
      $("attChips").innerHTML = chips.map(function (c) {
        return '<span class="chip ' + c[0] + '">' + esc(c[1]) + "</span>";
      }).join("");
    }

    // Readings taken before the source settled what units its gyroscope uses
    // are not plotted, and the traces are cleared when the verdict lands. One
    // pre-verdict sample is a factor-of-57 spike that sets the whole y-axis,
    // which makes the several minutes of real data after it unreadable.
    if (u._units_pending) { charts.pending = true; }
    else if (charts.pending) {
      charts.pending = false;
      CHANNELS.forEach(function (c) { charts.buf[c.key] = []; });
    }
    if (!u._units_pending) pushSample(u);
    renderInfo(u);
    if ($("imuCardsTag")) {
      $("imuCardsTag").textContent = charts.frozen ? "paused" : "live";
      $("imuCardsTag").className = "tag " + (charts.frozen ? "warn" : "ok");
    }
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
    drawCube(); drawCards();
    if (S.connected()) {
      send({ type: "sensors_report" });
      send({ type: "imu_transports" });
      send({ type: "imu_log_status" });
    }
  });

  /* ---- getting the data out ------------------------------------------- */
  var lastExport = null;

  on("btnImuLogStart", "click", function () {
    if (!S.require("imuLogMsg")) return;
    send({ type: "imu_log_start" });
  });
  on("btnImuLogStop", "click", function () { send({ type: "imu_log_stop" }); });
  on("btnImuExport", "click", function () {
    if (!S.require("imuLogMsg")) return;
    say("imuLogMsg", "Writing out what is in memory…", "info");
    send({ type: "imu_export" });
  });

  on("btnImuDownload", "click", function () {
    if (!lastExport) return;
    // The host has already written the file; this is a second copy for the
    // operator's own machine, which is not always the machine the agent runs
    // on. A Blob rather than a data: URL because a long capture is megabytes
    // and a data: URL of that size is refused without saying so.
    var blob = new Blob([lastExport.csv], { type: "text/csv" });
    var a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = (lastExport.path || "imu.csv").split(/[\\/]/).pop();
    document.body.appendChild(a); a.click();
    setTimeout(function () { URL.revokeObjectURL(a.href); a.remove(); }, 4000);
  });

  function renderLog(d) {
    if ($("logRows")) $("logRows").textContent = d.rows != null ? d.rows : 0;
    if ($("logSecs")) $("logSecs").innerHTML = (d.seconds != null ? fmt(d.seconds, 0) : "—")
      + '<span class="u">s</span>';
    if ($("logPath")) $("logPath").textContent = d.path || "—";
    var run = !!d.running;
    if ($("btnImuLogStart")) $("btnImuLogStart").disabled = run;
    if ($("btnImuLogStop")) $("btnImuLogStop").disabled = !run;
    if ($("imuLogTag")) {
      $("imuLogTag").textContent = run ? "recording" : "not recording";
      $("imuLogTag").className = "tag " + (run ? "ok" : "");
    }
  }

  // The heartbeat already asks for bench_status once a second and the log's
  // state rides along in it, so the panel stays current with no poll of its
  // own and no stale "recording" left on screen after a stop from elsewhere.
  S.on("bench_status", function (d) {
    if (d && d.imu_log) renderLog(d.imu_log);
  });

  S.on("imu_log_res", function (d) {
    renderLog(d);
    if (d.cmd === "status") return;
    if (!d.ok) { say("imuLogMsg", d.error || "Could not do that.", "bad"); return; }
    if (d.cmd === "start") {
      say("imuLogMsg", "Recording every reading to " + esc(d.path || "")
        + ". It keeps going while you work — move the arm, run a scan, then "
        + "come back and stop it.", "ok");
    } else {
      say("imuLogMsg", d.note || "Stopped.", "ok");
    }
  });

  S.on("imu_export_res", function (d) {
    if (!d.ok) { say("imuLogMsg", d.error || "Nothing to save.", "warn"); return; }
    lastExport = d.csv ? d : null;
    if ($("btnImuDownload")) $("btnImuDownload").disabled = !lastExport;
    say("imuLogMsg", (d.note || "") + " Saved as " + esc(d.path)
      + (lastExport ? " — “Download as a spreadsheet” puts a copy on this "
        + "computer as well." : ""), "ok");
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

  on("btnCalIdentify", "click", function () {
    if (!S.require("calLiveMsg")) return;
    say("calLiveMsg", "Reading the board…", "info");
    send({ type: "handeye_identify", target: calTarget() });
  });

  S.on("handeye_identify_res", function (d) {
    if (!d.ok) { say("calLiveMsg", d.error || "Could not read the board.", "warn"); return; }
    $("calCols").value = d.cols;
    $("calRows").value = d.rows;
    var fix = $("calFixSize"); if (fix) fix.hidden = true;
    var tight = d.pitch_px != null && d.pitch_px < 15;
    say("calLiveMsg", "That is a " + d.cols + " \u00d7 " + d.rows
      + " board (" + d.corners + " inner corners), squares "
      + (d.pitch_px != null ? fmt(d.pitch_px, 0) + " px across at this distance"
         : "measured")
      + (tight ? " — that is under the 15 px the detector needs, so move the "
         + "camera closer before you start" : "")
      + ". The size has been filled in."
      + (d.max_distance_mm
         ? " With these squares the camera can read this board out to about "
           + d.max_distance_mm + " mm; past that there are too few pixels "
           + "per square whatever the lighting."
         : "")
      + " Press Start.", tight ? "warn" : "ok");
    if (cal.on) send({ type: "handeye_begin", target: calTarget() });
  });

  on("calFixSize", "click", function () {
    if (!cal.suggest) return;
    $("calCols").value = cal.suggest[0];
    $("calRows").value = cal.suggest[1];
    this.hidden = true;
    say("calLiveMsg", "Board size set to " + cal.suggest[0] + " across and "
      + cal.suggest[1] + " down. If a calibration was already started, press "
      + "Start again so the new size takes effect.", "ok");
    if (cal.on) send({ type: "handeye_begin", target: calTarget() });
    send({ type: "handeye_preview", target: calTarget(), deep: true });
  });

  on("btnCalWhy", "click", function () {
    if (!S.require("calLiveMsg")) return;
    say("calLiveMsg", "Looking at the picture properly — a few seconds.", "info");
    send({ type: "handeye_preview", target: calTarget(), deep: true });
  });

  S.on("handeye_capture_res", function (d) {
    if (!d.ok) { say("calAdvice", d.error || "Could not use that pose.", "bad"); return; }
    renderCalN(d);
  });

  S.on("handeye_preview_res", function (d) {
    cal.corners = d.ok ? d.corners : null;
    cal.outline = d.ok ? d.outline : null;
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
      $("calSeenTag").className = "tag " + (d.ok ? "ok" : "warn");
    }
    // A size the detector actually found is not advice, it is an answer, so
    // it comes with the button that applies it. Counting inner corners wrong
    // is the commonest calibration mistake there is and it is the one that
    // leaves the operator with nothing to try.
    var fix = $("calFixSize");
    if (fix) {
      var sug = d.suggested_size || d.actual_size;
      if (!d.ok && sug) {
        cal.suggest = sug;
        fix.hidden = false;
        fix.textContent = "Use " + sug[0] + " × " + sug[1] + " instead";
      } else if (d.ok) { fix.hidden = true; }
    }
    if (!d.ok && d.error) say("calLiveMsg", d.error, d.sub_grid ? "bad" : "warn");
    else if (d.ok) {
      // The square pitch is the margin the detection had. Below about 15 px
      // it stops working, and an operator watching it fall as the arm backs
      // away knows why the next pose will fail before it does.
      var tight = d.pitch_px != null && d.pitch_px < 15;
      say("calLiveMsg", "Board found, all "
        + (d.corners_total || d.n_corners || 0) + " corners"
        + (d.reprojection_px != null ? ", accurate to " + fmt(d.reprojection_px, 2)
          + " px" : "")
        + (d.pitch_px != null ? ", squares " + fmt(d.pitch_px, 0) + " px across"
          + (tight ? " — that is close to the limit; move nearer or raise the "
            + "colour resolution on the Camera page" : "") : "")
        + ". Move the arm and capture.", tight ? "warn" : "ok");
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
    // The board's real perimeter. Joining every corner in detection order
    // traced a zig-zag across the whole grid, which drew as a solid block and
    // said nothing about where the board's edges were.
    if (cal.outline && cal.outline.length > 2) {
      ctx.strokeStyle = "#e8c66a"; ctx.lineWidth = 2.5; ctx.lineJoin = "round";
      ctx.beginPath();
      cal.outline.forEach(function (p, i) {
        i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]);
      });
      ctx.closePath(); ctx.stroke();
    }
    // Corner dots, scaled to how dense the grid is on screen so a fine board
    // reads as a grid rather than as a green smear.
    var r = cal.corners.length > 150 ? 2 : 3;
    ctx.fillStyle = "#3ddc84";
    cal.corners.forEach(function (p) {
      ctx.beginPath(); ctx.arc(p[0], p[1], r, 0, 6.3); ctx.fill();
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

/* ===========================================================================
   Robot control — everything ur_control.py supports.

   These were all implemented on the agent and none of them had a control on
   the page, so the robot panel was jog and nothing else: no way to power the
   arm on, release its brakes, clear a protective stop, run a pendant program,
   hand-guide it, set the payload or zero the force sensor. The backend was
   never the limit; the page was.
   =========================================================================== */
(function () {
  "use strict";
  var S = window.SONAIR;
  if (!S) return;
  var $ = S.$, send = S.send, say = S.say, fmt = S.fmt, esc = S.esc;

  function on(id, ev, fn) { var e = $(id); if (e) e.addEventListener(ev, fn); }
  function num(id, d) { var v = parseFloat(($(id) || {}).value); return isFinite(v) ? v : d; }
  function nums(id, n, d) {
    var parts = String((($(id) || {}).value || "")).split(/[,\s]+/)
      .map(parseFloat).filter(function (v) { return isFinite(v); });
    while (parts.length < n) parts.push(d || 0);
    return parts.slice(0, n);
  }

  /* ---- starting the link -------------------------------------------- */
  on("btnUrStart", "click", function () {
    if (!S.require("urStartMsg")) return;
    var host = (($("urHost") || {}).value || "").trim();
    if (!host) { say("urStartMsg", "Enter the robot's IP address.", "bad"); return; }
    say("urStartMsg", "Connecting to " + esc(host) + "…", "info");
    // rtde_inputs stays OFF. It opens a SECOND RTDE connection, and older
    // controllers allow exactly one — the second one takes the slot and the
    // telemetry stream dies, which presents as a robot that connects and then
    // shows nothing. The agent's own comment says so; the console was passing
    // true and defeating it. The only thing it buys is the speed slider,
    // which the speed control now asks for on demand instead.
    send({ type: "ur_service_start", host: host,
           frequency: num("urRate", 125), rtde_inputs: false });
  });

  S.on("ur_service_start_res", function (d) {
    if (d.ok === false || d.error) {
      say("urStartMsg", plainLinkError(d.error || d.msg), "bad");
      return;
    }
    say("urStartMsg", "Connected. Reading the robot now.", "ok");
    send({ type: "ur_service_status" });
    send({ type: "get_urp_list" });
  });

  function plainLinkError(e) {
    e = String(e || "");
    if (/timed out|timeout|refused|unreachable|No route/i.test(e)) {
      return "Could not reach the robot at that address. Check the IP on the "
        + "pendant (Settings → Network), that this PC is on the same network, "
        + "and that the cable is in.";
    }
    if (/remote/i.test(e)) {
      return "The robot answered but refused the connection. Put the pendant "
        + "into Remote Control — it will not accept commands in Local mode.";
    }
    return "The robot link did not start. " + (e ? "The agent reported: " + e : "");
  }

  /* ---- a command that the robot refused must never be silent -------- */
  S.on("cmd_rejected", function (d) {
    var why = String(d.reason || "refused");
    var plain = /envelope/i.test(why)
      ? "That move would leave the allowed working area, so it was blocked "
        + "before it reached the robot."
      : /speed|velocity/i.test(why)
      ? "That move asked for more speed than the cell allows."
      : "The move was refused: " + why;
    say("jogMsg", plain, "bad");
    say("powerMsg", plain, "bad");
  });

  /* ---- power and recovery -------------------------------------------- */
  var POWER = [
    ["btnPowerOn", "ur_power_on", "Powering the arm on…"],
    ["btnBrakeRelease", "ur_brake_release", "Releasing the brakes — the arm will move slightly…"],
    ["btnPowerOff", "ur_power_off", "Powering the arm off…"],
    ["btnUnlockPstop", "ur_unlock_protective_stop", "Clearing the protective stop…"],
    ["btnClosePopup", "ur_close_popup", "Dismissing the pendant message…"]
  ];
  POWER.forEach(function (p) {
    on(p[0], "click", function () {
      if (!S.require("powerMsg")) return;
      if (p[1] === "ur_power_off" &&
          !confirm("Power the arm off? It will drop into its brakes.")) return;
      if (p[1] === "ur_brake_release" &&
          !confirm("Release the brakes? The arm settles under its own weight. "
                   + "Is the area clear?")) return;
      say("powerMsg", p[2], "info");
      send({ type: p[1] });
    });
  });

  /* ---- pendant programs ---------------------------------------------- */
  on("btnUrpRefresh", "click", function () {
    if (!S.require("urpMsg")) return;
    say("urpMsg", "Reading the program list…", "info");
    send({ type: "get_urp_list" });
  });

  S.on("urp_list", function (d) {
    var sel = $("urpList"); if (!sel) return;
    var list = d.list || [];
    if (!list.length) {
      sel.innerHTML = "<option>No programs found</option>";
      say("urpMsg", "No programs on the pendant, or the robot is not connected "
        + "yet.", "warn");
      return;
    }
    sel.innerHTML = list.map(function (n) {
      return "<option>" + esc(String(n)) + "</option>"; }).join("");
    say("urpMsg", list.length + " programs on the pendant.", "ok");
  });

  on("btnUrpLoad", "click", function () {
    if (!S.require("urpMsg")) return;
    var name = (($("urpList") || {}).value || "").trim();
    if (!name || /^No |^Connect /.test(name)) {
      say("urpMsg", "Pick a program first.", "warn"); return;
    }
    say("urpMsg", "Loading " + esc(name) + "…", "info");
    send({ type: "ur_load_program", name: name });
  });
  [["btnUrpPlay", "ur_play", "Starting the program…"],
   ["btnUrpPause", "ur_pause", "Pausing…"],
   ["btnUrpStop", "ur_stop_program", "Stopping the program…"]].forEach(function (p) {
    on(p[0], "click", function () {
      if (!S.require("urpMsg")) return;
      if (p[1] === "ur_play" && !confirm("Run the loaded program? The arm will "
          + "move. Is the area clear?")) return;
      say("urpMsg", p[2], "info");
      send({ type: p[1] });
    });
  });

  /* ---- hand guiding: held, never latched ------------------------------
     Freedrive makes the arm limp. A toggle that can be left on by a misclick
     is the wrong control for that, so it follows the button: pressed means
     on, released means off, and losing the window releases it too.         */
  var fd = false;
  function setFreedrive(want) {
    if (want === fd) return;
    fd = want;
    send({ type: "ur_freedrive", enable: want });
    var tag = $("fdTag");
    if (tag) { tag.textContent = want ? "ON — arm is loose" : "off"; }
    say("fdMsg", want
      ? "The arm is loose. Let go of the button to lock it again."
      : "The arm is locked.", want ? "warn" : "info");
  }
  (function () {
    var b = $("btnFreedrive"); if (!b) return;
    ["pointerdown"].forEach(function (e) {
      b.addEventListener(e, function (ev) {
        if (!S.require("fdMsg")) return;
        ev.preventDefault();
        b.setPointerCapture && b.setPointerCapture(ev.pointerId);
        setFreedrive(true);
      });
    });
    ["pointerup", "pointercancel", "pointerleave", "blur"].forEach(function (e) {
      b.addEventListener(e, function () { setFreedrive(false); });
    });
    window.addEventListener("blur", function () { setFreedrive(false); });
  })();
  S.on("close", function () { fd = false; });

  /* ---- tool settings -------------------------------------------------- */
  on("btnSetPayload", "click", function () {
    if (!S.require("toolMsg")) return;
    var cog = nums("plCog", 3, 0).map(function (v) { return v / 1000.0; });
    send({ type: "ur_set_payload", mass: num("plMass", 0), cog: cog });
    say("toolMsg", "Payload set to " + fmt(num("plMass", 0), 2) + " kg. A wrong "
      + "payload makes every force reading wrong too.", "info");
  });

  on("btnSetTcp", "click", function () {
    if (!S.require("toolMsg")) return;
    var v = nums("tcpVals", 6, 0);
    // mm and degrees on screen, metres and radians on the wire
    var pose = [v[0] / 1000, v[1] / 1000, v[2] / 1000,
                v[3] * Math.PI / 180, v[4] * Math.PI / 180, v[5] * Math.PI / 180];
    if (!confirm("Change the tool centre point? Every position the robot "
        + "reports, and every calibration made against it, is relative to "
        + "this. Continue?")) return;
    send({ type: "ur_set_tcp", pose: pose });
    say("toolMsg", "Tool centre applied. Re-do the hand-eye calibration — it "
      + "was measured against the old one.", "warn");
  });

  on("btnZeroFt", "click", function () {
    if (!S.require("toolMsg")) return;
    send({ type: "ur_zero_ft" });
    say("toolMsg", "Zeroing the force sensor…", "info");
  });

  (function () {
    var host = $("toolVSeg"); if (!host) return;
    host.addEventListener("click", function (e) {
      var b = e.target.closest(".segb"); if (!b) return;
      if (!S.require("toolMsg")) return;
      var v = Number(b.dataset.v);
      if (v > 0 && !confirm("Switch the tool output to " + v + " V? Check what "
          + "is plugged into the tool connector first.")) return;
      host.querySelectorAll(".segb").forEach(function (o) {
        o.classList.toggle("on", o === b); });
      send({ type: "ur_set_tool_voltage", volts: v });
    });
  })();

  /* ---- speed limit ----------------------------------------------------- */
  on("urSpeed", "input", function () { $("urSpeedV").textContent = this.value + "%"; });
  on("urSpeed", "change", function () {
    if (!S.require("speedMsg")) return;
    send({ type: "ur_speed_slider", fraction: Number(this.value) / 100 });
    say("speedMsg", "Speed limit set to " + this.value + "%. This scales every "
      + "movement, including the planned scan.", "info");
  });

  /* ---- results from every one of the above ----------------------------- */
  var CMD_TARGET = {
    ur_power_on: "powerMsg", ur_power_off: "powerMsg",
    ur_brake_release: "powerMsg", ur_unlock_protective_stop: "powerMsg",
    ur_close_popup: "powerMsg", ur_close_safety_popup: "powerMsg",
    ur_load_program: "urpMsg", ur_play: "urpMsg", ur_pause: "urpMsg",
    ur_stop_program: "urpMsg",
    ur_set_payload: "toolMsg", ur_set_tcp: "toolMsg", ur_zero_ft: "toolMsg",
    ur_set_tool_voltage: "toolMsg",
    ur_speed_slider: "speedMsg", ur_freedrive: "fdMsg"
  };
  var CMD_DONE = {
    ur_power_on: "Arm powered on. Release the brakes next.",
    ur_brake_release: "Brakes released — the arm is ready to move.",
    ur_power_off: "Arm powered off.",
    ur_unlock_protective_stop: "Protective stop cleared.",
    ur_close_popup: "Pendant message dismissed.",
    ur_load_program: "Program loaded.",
    ur_play: "Program running.",
    ur_pause: "Program paused.",
    ur_stop_program: "Program stopped.",
    ur_zero_ft: "Force sensor zeroed.",
    ur_set_tool_voltage: "Tool voltage set."
  };

  S.on("ur_cmd_res", function (d) {
    var where = CMD_TARGET[d.cmd];
    if (!where) return;               // the main console already reports jogs
    if (d.ok) {
      say(where, CMD_DONE[d.cmd] || "Done.", "ok");
      if (d.cmd === "ur_load_program") send({ type: "get_urp_list" });
    } else {
      say(where, plainRobotError(d.cmd, d.msg), "bad");
    }
  });

  function plainRobotError(cmd, msg) {
    var m = String(msg || "");
    if (/remote/i.test(m)) {
      return "The pendant is in Local mode. Switch it to Remote Control — the "
        + "robot ignores commands otherwise.";
    }
    if (/not started|no connection|not connected/i.test(m)) {
      return "The robot link is not running. Go to step 1 and connect first.";
    }
    if (/protective/i.test(m)) {
      return "The robot is in a protective stop. Clear it above, then try again.";
    }
    if (/emergency/i.test(m)) {
      return "The emergency stop is engaged. Release the physical button first.";
    }
    if (/File not found|no such/i.test(m)) {
      return "The robot could not find that program.";
    }
    return "The robot refused it: " + m;
  }

  /* ---- replies the page used to drop on the floor ---------------------- */
  S.on("dashboard_res", function (d) {
    say("powerMsg", "The robot replied: " + esc(String(d.res || "").trim()),
        "info");
  });

  S.on("imu_d435i_res", function (d) {
    if (d.ok) return;
    say("imuMsg", d.error
      ? "The camera's motion sensor could not start: " + d.error
      : "This camera has no built-in motion sensor.", "warn");
  });

  S.on("imu_tcp_probe_res", function (d) {
    if (!d.ok) return;
    say("imuMsg", (d.open || []).length
      ? "Ports answering on " + esc(d.host) + ": " + d.open.join(", ")
      : "Nothing is listening on " + esc(d.host) + " on any of the usual ports.",
      (d.open || []).length ? "ok" : "warn");
  });

  S.on("rs_advanced_res", function (d) {
    say("rsInfoMsg", d.ok ? "Camera preset applied."
      : (d.error || "The camera refused that preset."), d.ok ? "ok" : "warn");
  });

  S.on("bench_offset_res", function (d) {
    if (!d.ok) return;
    say("rcMsg", "Sensor timing offset recorded. Worst residual "
      + fmt(d.worst_residual_ms, 2) + " ms.", "ok");
  });

  /* Page open: ask for what this page shows. */
  S.page("robot", function () {
    if (S.connected()) {
      send({ type: "ur_service_status" });
      send({ type: "get_urp_list" });
    }
  });
})();

/* ===========================================================================
   The last four robot commands that had no control.
   =========================================================================== */
(function () {
  "use strict";
  var S = window.SONAIR;
  if (!S) return;
  var $ = S.$, send = S.send, say = S.say, esc = S.esc;
  function on(id, ev, fn) { var e = $(id); if (e) e.addEventListener(ev, fn); }

  // A safety popup is NOT the same dialog as an ordinary one and does not
  // close with the same command. Having only the ordinary one meant the
  // button appeared to do nothing on exactly the popup that matters.
  on("btnCloseSafety", "click", function () {
    if (!S.require("powerMsg")) return;
    say("powerMsg", "Dismissing the safety message…", "info");
    send({ type: "ur_close_safety_popup" });
  });

  // The commonest reason a command "does nothing" is the pendant sitting in
  // Local mode, where the robot accepts the connection and ignores the
  // instructions. Worth being able to ask directly.
  on("btnCheckRemote", "click", function () {
    if (!S.require("powerMsg")) return;
    say("powerMsg", "Asking the robot…", "info");
    send({ type: "ur_remote_control" });
  });

  S.on("ur_cmd_res", function (d) {
    if (d.cmd === "ur_close_safety_popup") {
      say("powerMsg", d.ok ? "Safety message dismissed."
        : "The robot refused: " + (d.msg || ""), d.ok ? "ok" : "bad");
    } else if (d.cmd === "ur_remote_control") {
      var yes = /true|enabled/i.test(String(d.msg || ""));
      say("powerMsg", yes
        ? "The pendant is in Remote Control — the robot will accept commands."
        : "The pendant is in LOCAL mode. It will accept a connection and "
          + "ignore every command. Switch it to Remote Control on the "
          + "pendant's top-right menu.", yes ? "ok" : "bad");
    } else if (d.cmd === "ur_set_tool_dout" || d.cmd === "ur_set_aout") {
      say("ioMsg", d.ok ? "Output set." : "The robot refused: " + (d.msg || ""),
          d.ok ? "ok" : "bad");
    }
  });

  // Tool-connector digital outputs: this is where a probe or a light gets
  // wired, so the inspection sensors arriving later land here.
  var toolOut = [false, false];
  function renderToolOut() {
    var host = $("ioToolOut"); if (!host) return;
    host.innerHTML = toolOut.map(function (v, i) {
      return '<button class="btn' + (v ? " go" : "") + '" data-tpin="' + i
        + '">Tool out ' + i + ": " + (v ? "ON" : "off") + "</button>";
    }).join("");
  }
  renderToolOut();
  (function () {
    var host = $("ioToolOut"); if (!host) return;
    host.addEventListener("click", function (e) {
      var b = e.target.closest("[data-tpin]"); if (!b) return;
      if (!S.require("ioMsg")) return;
      var pin = Number(b.dataset.tpin);
      toolOut[pin] = !toolOut[pin];
      renderToolOut();
      send({ type: "ur_set_tool_dout", pin: pin, value: toolOut[pin] });
    });
  })();

  on("btnAout", "click", function () {
    if (!S.require("ioMsg")) return;
    var v = parseFloat(($("aoutVal") || {}).value);
    if (!(v >= 0 && v <= 1)) {
      say("ioMsg", "The analogue value must be between 0 and 1.", "bad"); return;
    }
    send({ type: "ur_set_aout", pin: Number(($("aoutPin") || {}).value || 0),
           value: v });
  });
})();

/* ===========================================================================
   Automatic hand-eye calibration.

   One sighting of the board, then the arm takes the rest of the views itself.
   Two rounds, because the second cannot be planned until the first has
   answered: views around the board can only be worked out once it is known
   roughly where the camera sits, and that is the thing being measured.
   Round one is small tool rotations, which need nothing known and keep the
   board in frame whatever the mounting; round two is the wide set, planned
   from round one's rough answer.
   =========================================================================== */
(function () {
  "use strict";
  var S = window.SONAIR;
  if (!S) return;
  var $ = S.$, send = S.send, say = S.say, fmt = S.fmt;

  var run = { on: false, queue: [], i: 0, stage: "", timer: null,
              seen: 0, missed: 0 };

  function stop(msg, kind) {
    run.on = false;
    if (run.timer) { clearTimeout(run.timer); run.timer = null; }
    var b = $("btnCalAuto"), st = $("btnCalAutoStop");
    if (b) b.disabled = false;
    if (st) st.disabled = true;
    if (msg) say("calAdvice", msg, kind || "info");
  }
  S.on("close", function () { stop(null); });

  $("btnCalAuto") && $("btnCalAuto").addEventListener("click", function () {
    if (!S.require("calAdvice")) return;
    if (!confirm("The arm will move to about twenty positions around the "
        + "board, photographing it at each. Is the area clear?")) return;
    run = { on: true, queue: [], i: 0, stage: "bootstrap", timer: null,
            seen: 0, missed: 0 };
    this.disabled = true;
    $("btnCalAutoStop").disabled = false;
    say("calAdvice", "Working out the first set of positions…", "info");
    send({ type: "handeye_auto_plan", stage: "bootstrap" });
  });

  $("btnCalAutoStop") && $("btnCalAutoStop").addEventListener("click",
    function () { stop("Stopped. The poses captured so far are kept.", "warn"); });

  S.on("handeye_auto_plan_res", function (d) {
    if (!run.on) return;
    if (!d.ok) {
      // Stopping after the first round leaves the operator holding a pose
      // set the console itself will refuse, with nothing saying why. Say it
      // here, where it is still obvious what just happened.
      stop("Could not plan the next round: " + (d.error || "")
        + (run.stage === "fine"
           ? " The first round's poses are kept, but they cannot determine "
             + "the answer on their own — they only exist to plan the second "
             + "round. Put the board fully in view and press “Do it "
             + "automatically” again, or capture a dozen poses by hand with "
             + "30–60° of tilt between them."
           : ""), "bad");
      return;
    }
    run.queue = d.poses || [];
    run.i = 0;
    say("calAdvice", (d.explain || "")
      + (d.planned_from ? " Planned from " + esc(d.planned_from) + "." : ""),
      "info");
    step();
  });

  function step() {
    if (!run.on) return;
    if (run.i >= run.queue.length) {
      say("calAdvice", "Round finished — " + run.seen + " views used, "
        + run.missed + " skipped because the board was not in them"
        + (run.missed > run.seen && run.lastMiss
           ? " (" + esc(run.lastMiss.slice(0, 110)) + ")" : "")
        + ". Working it out…", run.missed > run.seen ? "warn" : "info");
      // Solve, but do not apply yet on the first round: that answer exists to
      // plan the second round, not to be used.
      send({ type: "handeye_solve", method: "all",
             apply: run.stage !== "bootstrap" });
      return;
    }
    var p = run.queue[run.i];
    say("calAdvice", "Position " + (run.i + 1) + " of " + run.queue.length
      + (run.stage === "bootstrap" ? " (first pass)" : "") + "…", "info");
    send({ type: "ur_movel", pose: p.tcp_pose, a: 0.5, v: 0.12 });
    // Settle before photographing. A board photographed while the arm is
    // still moving is blurred, and a blurred corner becomes tool error in the
    // answer rather than a failed detection you can see.
    run.timer = setTimeout(function () {
      if (!run.on) return;
      send({ type: "handeye_capture" });
      run.i++;
      run.timer = setTimeout(step, 700);
    }, 3200);
  }

  S.on("handeye_capture_res", function (d) {
    if (!run.on) return;
    if (d.ok) { run.seen++; } else { run.missed++; run.lastMiss = d.error || ""; }
  });

  S.on("handeye_solve_res", function (d) {
    if (!run.on) return;
    if (!d.ok) {
      stop("Could not solve: " + (d.error || ""), "bad");
      return;
    }
    if (run.stage === "bootstrap") {
      run.stage = "fine";
      // The first round's spread is NOT a quality figure. It is small by
      // construction, because poses that barely rotate reconstruct the board
      // consistently whatever the transform is — which is precisely why the
      // second round exists. Reporting it as though it meant something is how
      // an operator comes to trust a number that cannot be trusted yet.
      say("calAdvice", "First pass done. It is not the answer and its "
        + "accuracy figure does not mean anything yet — small rotations "
        + "always reconstruct consistently. Planning the wide set from it "
        + "now; that is the round that decides.", "info");
      send({ type: "handeye_auto_plan", stage: "fine", n_poses: 14 });
      return;
    }
    stop(null);
    say("calAdvice", "Finished: " + run.seen + " views used across both "
      + "rounds. See the result below — press Save and use it if it reads "
      + "good.", d.target_spread_mm <= 5 ? "ok" : "warn");
  });

  // the automatic button needs a session, like the manual capture buttons
  S.on("handeye_res", function (d) {
    if (d.cmd === "begin" && d.ok && $("btnCalAuto")) {
      $("btnCalAuto").disabled = false;
    }
  });
})();

/* =========================================================================
   5. AUTOMATION — the cell runs the job
   =========================================================================
   The console stops being a set of buttons somebody presses in order and
   becomes a thing that executes a declared job. Everything here is a view of
   state the agent owns: the page never tracks where a job has got to, it
   renders what the agent says. A browser that reloads mid-campaign therefore
   rejoins the running job rather than losing it.
   ====================================================================== */
(function () {
  var S = window.SONAIR;
  if (!S) return;
  var $ = S.$, send = S.send, say = S.say, fmt = S.fmt, esc = S.esc, on = bindOn;

  function bindOn(id, ev, fn) {
    var el = $(id);
    if (el) el.addEventListener(ev, fn);
  }

  var auto = { jobs: {}, pick: "", last: null, pollTimer: null };

  /* ---- pre-flight ------------------------------------------------------ */
  function renderPreflight(d) {
    var box = $("pfList"); if (!box) return;
    var checks = (d && d.checks) || [];
    if (!checks.length) return;
    box.innerHTML = checks.map(function (c) {
      return '<div class="chk-row"><span class="s ' + esc(c.state) + '">'
        + esc(c.state) + '</span><span class="l">' + esc(c.label)
        + '</span><span class="d">' + esc(c.detail || "") + "</span></div>";
    }).join("");
    var tag = $("pfTag");
    if (tag) {
      tag.textContent = d.ok ? "ready" : (d.blocking || []).length + " blocking";
      tag.className = "tag " + (d.ok ? "ok" : "bad");
    }
    say("pfMsg", d.summary || "", d.ok ? "ok" : "bad");
    var run = $("btnJobRun");
    if (run) run.disabled = !d.ok;
  }

  on("btnPreflight", "click", function () {
    if (!S.require("pfMsg")) return;
    say("pfMsg", "Checking…", "info");
    send({ type: "auto_preflight" });
  });
  S.on("auto_preflight_res", renderPreflight);

  /* ---- the job --------------------------------------------------------- */
  S.on("auto_jobs_res", function (d) {
    if (!d.ok) return;
    auto.jobs = d.jobs || {};
    var sel = $("jobPick"); if (!sel) return;
    var keys = Object.keys(auto.jobs);
    sel.innerHTML = keys.map(function (k) {
      return '<option value="' + esc(k) + '">' + esc(k.replace(/_/g, " ")) + "</option>";
    }).join("");
    if (!auto.pick || keys.indexOf(auto.pick) < 0) auto.pick = keys[0] || "";
    sel.value = auto.pick;
    renderJob();
  });

  on("jobPick", "change", function () { auto.pick = this.value; renderJob(); });
  ["jobRepeats", "jobSweep"].forEach(function (id) {
    on(id, "input", renderPlanTag);
  });

  function sweepValues() {
    return String(($("jobSweep") || {}).value || "")
      .split(/[,\s]+/).map(parseFloat)
      .filter(function (v) { return isFinite(v) && v > 0; });
  }

  function currentJob() {
    var base = auto.jobs[auto.pick];
    if (!base) return null;
    var job = JSON.parse(JSON.stringify(base));
    // Only a job that actually sweeps takes the sweep controls; applying them
    // to a one-shot job would silently turn it into a campaign.
    if (job.sweep_key) {
      job.repeats = Math.max(1, Number(($("jobRepeats") || {}).value) || 1);
      var sv = sweepValues();
      if (sv.length) job.sweep_values = sv;
    }
    return job;
  }

  function planSize(job) {
    if (!job) return 0;
    var vals = (job.sweep_key && job.sweep_values.length) ? job.sweep_values.length : 1;
    return vals * Math.max(1, job.repeats) * job.steps.length;
  }

  function renderPlanTag() {
    var job = currentJob(), tag = $("jobPlanTag");
    if (!tag) return;
    if (!job) { tag.textContent = "—"; return; }
    var vals = (job.sweep_key && job.sweep_values.length) ? job.sweep_values.length : 1;
    var iters = vals * Math.max(1, job.repeats);
    tag.textContent = iters + (iters === 1 ? " pass" : " passes") + " · "
      + planSize(job) + " steps";
  }

  var STEP_WORDS = {
    preflight: "check the cell is fit to run",
    dwell: "wait for the arm to settle",
    move: "go to one pose",
    trajectory: "run the motion",
    record_start: "start a run file",
    record_stop: "close the run file",
    imu_log_start: "start logging every inertial sample",
    imu_log_stop: "close the inertial log",
    "export": "write the dataset folder",
    message: "note in the log"
  };

  function renderJob() {
    var job = currentJob();
    var box = $("jobSteps");
    if (!job || !box) return;
    say("jobNote", job.notes || "", "info");
    box.innerHTML = job.steps.map(function (st, i) {
      var extra = [];
      if (st.seconds != null) extra.push(st.seconds + " s");
      if (st.poses) extra.push(st.poses.length + " poses");
      if (st.speed != null) extra.push(st.speed + " m/s");
      if (st.text) extra.push(st.text);
      return '<div class="stp" data-i="' + i + '"><span class="i">'
        + (i + 1) + '</span><span class="k">' + esc(st.kind) + '</span>'
        + '<span class="a">' + esc(STEP_WORDS[st.kind] || "")
        + (extra.length ? " · " + esc(extra.join(" · ")) : "") + "</span></div>";
    }).join("");
    renderPlanTag();
  }

  on("btnJobRun", "click", function () {
    if (!S.require("pfMsg")) return;
    var job = currentJob();
    if (!job) { say("pfMsg", "Pick a job first.", "warn"); return; }
    say("pfMsg", "Starting " + job.name + "…", "info");
    send({ type: "auto_start", job: job });
  });

  on("btnJobStop", "click", function () { send({ type: "auto_stop" }); });
  on("rbStop", "click", function () { send({ type: "auto_stop" }); });

  S.on("auto_res", function (d) {
    if (d.cmd === "export") {
      if (!d.ok) { say("dsMsg", d.error || "Export failed.", "bad"); return; }
      renderDataset(d);
      return;
    }
    if (!d.ok) {
      say("pfMsg", d.error || "The job would not start.", "bad");
      if (d.preflight) renderPreflight(d.preflight);
      return;
    }
    renderRun(d);
  });

  /* ---- progress -------------------------------------------------------- */
  function renderRun(d) {
    auto.last = d;
    var running = !!d.running;
    var st = d.state || "idle";

    if ($("runTag")) {
      $("runTag").textContent = st;
      $("runTag").className = "tag " + (st === "done" ? "ok"
        : st === "failed" ? "bad" : running ? "warn" : "");
    }
    if ($("runStep")) $("runStep").textContent = d.steps ? d.step + " / " + d.steps : "—";
    if ($("runTime")) $("runTime").innerHTML = fmt(d.seconds || 0, 0) + '<span class="u">s</span>';
    if ($("runFiles")) $("runFiles").textContent = (d.produced || []).length;
    if ($("btnJobStop")) $("btnJobStop").disabled = !running;
    if ($("btnJobRun")) $("btnJobRun").disabled = running;

    // the bar at the top of every page
    var bar = $("runBar");
    if (bar) {
      bar.classList.toggle("live", running);
      bar.classList.toggle("failed", st === "failed");
    }
    if ($("rbState")) $("rbState").textContent = running ? "running a job" : st;
    if ($("rbJob")) $("rbJob").textContent = d.job || "—";
    if ($("rbStep")) {
      $("rbStep").textContent = d.steps
        ? d.step + "/" + d.steps + (d.current ? " " + d.current : "") : "—";
    }
    if ($("rbTime")) $("rbTime").textContent = fmt(d.seconds || 0, 0) + "s";
    if ($("rbFill")) {
      $("rbFill").style.width = d.steps
        ? Math.round(100 * d.step / d.steps) + "%" : "0%";
    }
    if ($("rbStop")) $("rbStop").hidden = !running;

    // which step is live
    var box = $("jobSteps");
    if (box && d.steps) {
      var perPass = (currentJob() || { steps: [] }).steps.length || 1;
      var within = ((d.step - 1) % perPass + perPass) % perPass;
      box.querySelectorAll(".stp").forEach(function (el, i) {
        el.classList.toggle("now", running && i === within);
        el.classList.toggle("done", running && i < within);
      });
    }

    if ($("jobLog")) {
      var lines = d.log || [];
      $("jobLog").innerHTML = lines.length
        ? lines.map(function (l) {
            return '<span class="' + esc(l.level || "info") + '">' + esc(l.text) + "</span>";
          }).join("\n")
        : "Nothing has run yet.";
      $("jobLog").scrollTop = $("jobLog").scrollHeight;
    }

    if ((d.produced || []).length && $("dsTag")) {
      $("dsTag").textContent = d.produced.length + " files";
    }
    // While a job is running the page follows it closely; when it is not,
    // the one-second heartbeat is plenty and this timer stands down.
    if (running && !auto.pollTimer) {
      auto.pollTimer = setInterval(function () {
        if (S.connected()) send({ type: "auto_status" });
      }, 700);
    } else if (!running && auto.pollTimer) {
      clearInterval(auto.pollTimer);
      auto.pollTimer = null;
    }
  }
  S.on("auto_status_res", renderRun);

  /* ---- the dataset ----------------------------------------------------- */
  on("btnExportDs", "click", function () {
    if (!S.require("dsMsg")) return;
    say("dsMsg", "Gathering the files…", "info");
    send({ type: "auto_export", name: ($("dsName") || {}).value || "dataset" });
  });

  function renderDataset(d) {
    var box = $("dsList");
    if (box) {
      box.innerHTML = [
        ["Folder", d.path],
        ["Run files", String(d.runs)],
        ["Inertial logs", String(d.inertial)],
        ["Size", (d.bytes / 1e6).toFixed(1) + " MB"]
      ].map(function (r) {
        return '<div class="kvr"><span>' + esc(r[0]) + "</span><b>"
          + esc(r[1]) + "</b></div>";
      }).join("");
    }
    if ($("dsTag")) { $("dsTag").textContent = "exported"; $("dsTag").className = "tag ok"; }
    say("dsMsg", (d.note || "") + " It carries its own manifest and README, so "
      + "it can be read without this console.", "ok");
  }

  /* ---- recording lamp on the run bar ----------------------------------- */
  S.on("imu", function (d) {
    if ($("rbRec")) {
      var rec = !!d.rec;
      $("rbRec").textContent = rec ? "yes" : "no";
      $("rbRec").style.color = rec ? "var(--ok)" : "var(--text-2)";
    }
  });

  S.page("auto", function () {
    if (!S.connected()) return;
    send({ type: "auto_jobs" });
    send({ type: "auto_status" });
    send({ type: "auto_preflight" });
  });

  S.on("close", function () {
    if (auto.pollTimer) { clearInterval(auto.pollTimer); auto.pollTimer = null; }
  });
})();
