const fs = require('fs');
const d = require('docx');
const {
  Document, Packer, Paragraph, TextRun, HeadingLevel, AlignmentType,
  Table, TableRow, TableCell, WidthType, BorderStyle, ShadingType,
  PageBreak, TableOfContents, LevelFormat, convertInchesToTwip,
  Header, Footer, PageNumber, ExternalHyperlink
} = d;

/* ------------------------------------------------------------------ *
 * Palette and helpers
 * ------------------------------------------------------------------ */
const INK = '1A1A1A';
const MUTED = '5A6472';
const ACCENT = '1F4E79';     // UoN-ish deep blue
const WARN = '8A5A00';
const BAD = '9B2226';
const OK = '1F6F43';
const RULE = 'D5DAE0';
const HEAD_BG = 'EEF2F6';
const NOTE_BG = 'F5F7FA';
const WARN_BG = 'FDF6E7';
const BAD_BG = 'FBEDED';

const FULL = 9360;           // A4 usable width in DXA (portrait, 1" margins)

function p(text, opts = {}) {
  return new Paragraph({
    spacing: { after: opts.after == null ? 120 : opts.after, line: 276 },
    alignment: opts.align,
    indent: opts.indent,
    children: [new TextRun({
      text,
      size: opts.size || 21,
      color: opts.color || INK,
      bold: opts.bold,
      italics: opts.italics,
      font: opts.font
    })]
  });
}

// Rich paragraph from [text, {opts}] pairs
function rich(parts, opts = {}) {
  return new Paragraph({
    spacing: { after: opts.after == null ? 120 : opts.after, line: 276 },
    alignment: opts.align,
    indent: opts.indent,
    children: parts.map(function (x) {
      if (typeof x === 'string') return new TextRun({ text: x, size: 21, color: INK });
      return new TextRun({
        text: x[0],
        size: x[1] && x[1].size || 21,
        bold: x[1] && x[1].bold,
        italics: x[1] && x[1].italics,
        color: (x[1] && x[1].color) || INK,
        font: x[1] && x[1].font
      });
    })
  });
}

function h1(text) {
  return new Paragraph({
    heading: HeadingLevel.HEADING_1,
    spacing: { before: 420, after: 160 },
    border: { bottom: { style: BorderStyle.SINGLE, size: 6, color: RULE, space: 6 } },
    children: [new TextRun({ text, size: 30, bold: true, color: ACCENT })]
  });
}
function h2(text) {
  return new Paragraph({
    heading: HeadingLevel.HEADING_2,
    spacing: { before: 300, after: 120 },
    children: [new TextRun({ text, size: 24, bold: true, color: INK })]
  });
}
function h3(text) {
  return new Paragraph({
    heading: HeadingLevel.HEADING_3,
    spacing: { before: 220, after: 100 },
    children: [new TextRun({ text, size: 21, bold: true, color: MUTED })]
  });
}

function bullet(text, level) {
  return new Paragraph({
    numbering: { reference: 'dots', level: level || 0 },
    spacing: { after: 70, line: 276 },
    children: [new TextRun({ text, size: 21, color: INK })]
  });
}
function bulletRich(parts, level) {
  return new Paragraph({
    numbering: { reference: 'dots', level: level || 0 },
    spacing: { after: 70, line: 276 },
    children: parts.map(function (x) {
      if (typeof x === 'string') return new TextRun({ text: x, size: 21, color: INK });
      return new TextRun({
        text: x[0], size: 21, bold: x[1] && x[1].bold,
        italics: x[1] && x[1].italics, color: (x[1] && x[1].color) || INK,
        font: x[1] && x[1].font
      });
    })
  });
}
// Each procedure is its own numbered list. Sharing one numbering instance
// made the first step of section 6.1 read "9.", continuing the count from a
// list two sections earlier -- which in a protocol is not a cosmetic problem:
// "go to step 4" then means two different things.
let STEP_INSTANCE = 0;
function newProcedure() { STEP_INSTANCE += 1; return STEP_INSTANCE; }

function step(text, level) {
  return new Paragraph({
    numbering: { reference: 'steps', level: level || 0, instance: STEP_INSTANCE },
    spacing: { after: 80, line: 276 },
    children: [new TextRun({ text, size: 21, color: INK })]
  });
}
function stepRich(parts, level) {
  return new Paragraph({
    numbering: { reference: 'steps', level: level || 0, instance: STEP_INSTANCE },
    spacing: { after: 80, line: 276 },
    children: parts.map(function (x) {
      if (typeof x === 'string') return new TextRun({ text: x, size: 21, color: INK });
      return new TextRun({
        text: x[0], size: 21, bold: x[1] && x[1].bold,
        italics: x[1] && x[1].italics, color: (x[1] && x[1].color) || INK,
        font: x[1] && x[1].font
      });
    })
  });
}

/* Code block: monospace lines on a tinted single-cell table */
function code(lines) {
  return new Table({
    width: { size: FULL, type: WidthType.DXA },
    columnWidths: [FULL],
    borders: {
      top: { style: BorderStyle.SINGLE, size: 2, color: RULE },
      bottom: { style: BorderStyle.SINGLE, size: 2, color: RULE },
      left: { style: BorderStyle.SINGLE, size: 2, color: RULE },
      right: { style: BorderStyle.SINGLE, size: 2, color: RULE },
      insideHorizontal: { style: BorderStyle.NONE, size: 0, color: 'FFFFFF' },
      insideVertical: { style: BorderStyle.NONE, size: 0, color: 'FFFFFF' }
    },
    rows: [new TableRow({
      children: [new TableCell({
        width: { size: FULL, type: WidthType.DXA },
        shading: { type: ShadingType.CLEAR, fill: 'F4F6F8' },
        margins: { top: 140, bottom: 140, left: 180, right: 180 },
        children: lines.map(function (l) {
          return new Paragraph({
            spacing: { after: 20, line: 240 },
            children: [new TextRun({
              text: l, font: 'Consolas', size: 18,
              color: l.trim().startsWith('#') ? MUTED : INK
            })]
          });
        })
      })]
    })]
  });
}

/* Callout box */
function callout(kind, title, lines) {
  const bg = kind === 'bad' ? BAD_BG : kind === 'warn' ? WARN_BG : NOTE_BG;
  const bar = kind === 'bad' ? BAD : kind === 'warn' ? WARN : ACCENT;
  const kids = [new Paragraph({
    spacing: { after: 90 },
    children: [new TextRun({ text: title, bold: true, size: 21, color: bar })]
  })];
  lines.forEach(function (l) {
    if (Array.isArray(l)) {
      kids.push(new Paragraph({
        spacing: { after: 70, line: 276 },
        children: l.map(function (x) {
          if (typeof x === 'string') return new TextRun({ text: x, size: 21, color: INK });
          return new TextRun({
            text: x[0], size: 21, bold: x[1] && x[1].bold,
            italics: x[1] && x[1].italics, color: (x[1] && x[1].color) || INK,
            font: x[1] && x[1].font
          });
        })
      }));
    } else {
      kids.push(new Paragraph({
        spacing: { after: 70, line: 276 },
        children: [new TextRun({ text: l, size: 21, color: INK })]
      }));
    }
  });
  return new Table({
    width: { size: FULL, type: WidthType.DXA },
    columnWidths: [FULL],
    borders: {
      top: { style: BorderStyle.NONE, size: 0, color: 'FFFFFF' },
      bottom: { style: BorderStyle.NONE, size: 0, color: 'FFFFFF' },
      right: { style: BorderStyle.NONE, size: 0, color: 'FFFFFF' },
      left: { style: BorderStyle.SINGLE, size: 18, color: bar },
      insideHorizontal: { style: BorderStyle.NONE, size: 0, color: 'FFFFFF' },
      insideVertical: { style: BorderStyle.NONE, size: 0, color: 'FFFFFF' }
    },
    rows: [new TableRow({
      children: [new TableCell({
        width: { size: FULL, type: WidthType.DXA },
        shading: { type: ShadingType.CLEAR, fill: bg },
        margins: { top: 160, bottom: 160, left: 200, right: 200 },
        children: kids
      })]
    })]
  });
}

/* Quote */
function quote(text, who) {
  return new Paragraph({
    spacing: { before: 140, after: 160, line: 276 },
    indent: { left: 360 },
    border: { left: { style: BorderStyle.SINGLE, size: 12, color: ACCENT, space: 12 } },
    children: [
      new TextRun({ text: '“' + text + '”', italics: true, size: 21, color: INK }),
      new TextRun({ text: who ? '  — ' + who : '', size: 19, color: MUTED })
    ]
  });
}

/* Table */
function table(widths, header, rows, opts) {
  opts = opts || {};
  function cell(text, o) {
    o = o || {};
    return new TableCell({
      width: { size: o.w, type: WidthType.DXA },
      shading: o.fill ? { type: ShadingType.CLEAR, fill: o.fill } : undefined,
      margins: { top: 90, bottom: 90, left: 130, right: 130 },
      children: String(text).split(' ').map(function (line, i) {
        return new Paragraph({
          spacing: { after: 0, line: 252 },
          alignment: o.align,
          children: [new TextRun({
            text: line, size: o.size || 19, bold: o.bold,
            color: o.color || INK, font: o.font
          })]
        });
      })
    });
  }
  const trs = [new TableRow({
    tableHeader: true,
    children: header.map(function (t, i) {
      return cell(t, { w: widths[i], bold: true, fill: HEAD_BG, color: ACCENT, size: 18 });
    })
  })];
  rows.forEach(function (r) {
    trs.push(new TableRow({
      children: r.map(function (t, i) {
        const style = (opts.cellStyle && opts.cellStyle(t, i)) || {};
        return cell(t, Object.assign({ w: widths[i] }, style));
      })
    }));
  });
  return new Table({
    width: { size: FULL, type: WidthType.DXA },
    columnWidths: widths,
    borders: {
      top: { style: BorderStyle.SINGLE, size: 4, color: RULE },
      bottom: { style: BorderStyle.SINGLE, size: 4, color: RULE },
      left: { style: BorderStyle.NONE, size: 0, color: 'FFFFFF' },
      right: { style: BorderStyle.NONE, size: 0, color: 'FFFFFF' },
      insideHorizontal: { style: BorderStyle.SINGLE, size: 2, color: RULE },
      insideVertical: { style: BorderStyle.NONE, size: 0, color: 'FFFFFF' }
    },
    rows: trs
  });
}

const gap = function (h) { return new Paragraph({ spacing: { after: h || 160 }, children: [] }); };
const mono = { font: 'Consolas', size: 19 };

/* ------------------------------------------------------------------ *
 * Content
 * ------------------------------------------------------------------ */
const body = [];

/* ---------- Title page ---------- */
body.push(new Paragraph({ spacing: { before: 1800, after: 0 },
  children: [new TextRun({ text: 'UK OPEN MULTIMODAL AI BENCHMARK  ·  UoN × UCL',
    size: 18, bold: true, color: MUTED, characterSpacing: 40 })] }));
body.push(new Paragraph({ spacing: { before: 200, after: 0 },
  children: [new TextRun({ text: 'SONAIR', size: 72, bold: true, color: ACCENT })] }));
body.push(new Paragraph({ spacing: { before: 40, after: 260 },
  children: [new TextRun({ text: 'Sim2real Operational beNchmark for AI Robotics',
    size: 24, color: MUTED })] }));
body.push(new Paragraph({
  spacing: { before: 0, after: 100 },
  border: { top: { style: BorderStyle.SINGLE, size: 12, color: ACCENT, space: 10 } },
  children: [new TextRun({ text: 'Experimental Protocol', size: 40, bold: true, color: INK })] }));
body.push(new Paragraph({ spacing: { after: 700 },
  children: [new TextRun({
    text: 'Data collection campaign — from the first recording to a scored leaderboard',
    size: 22, color: MUTED })] }));

body.push(table([2400, 6960], ['Field', 'Value'], [
  ['Document', 'SONAIR Experimental Protocol, v1.0'],
  ['Cell', 'UR5e · Intel RealSense D435 · FusionHub industrial IMU (tool-mounted)'],
  ['Software', 'SONAIR Inspection Console + sonair_benchmark harness'],
  ['Repository', 'MingyuTang0728/SONAIRCrosssite, branch claude/sonair-benchmark-implementation-9knwbe'],
  ['Prepared for', 'Mingyu Tang (PhD, University of Nottingham)'],
  ['Status', 'Ready to execute — Phase 0 onward'],
], { cellStyle: function (t, i) { return i === 0 ? { bold: true, color: MUTED } : {}; } }));

body.push(gap(300));
body.push(callout('note', 'How to use this document', [
  'Sections 1–3 are the reasoning: what is being built and why, so that a decision taken at the machine can be checked against the intent rather than against memory.',
  'Sections 4–11 are the procedure. They are ordered and they are gated — do not skip ahead past a gate that has not been cleared, because every gate exists to stop a much more expensive mistake downstream.',
  'Appendix A is the command reference. Appendix C lists the decisions that still need Sam.'
]));

body.push(new Paragraph({ children: [new PageBreak()] }));

/* ---------- TOC ---------- */
body.push(h1('Contents'));
// A STATIC contents list, not a TOC field.
//
// A field renders blank until somebody refreshes it, and neither LibreOffice
// in headless conversion nor a phone viewer nor Google Docs will. For a
// protocol that gets opened on a shop floor and exported to PDF by whoever
// needs it, an empty Contents page reads as a broken file. Section numbers
// are stable and page numbers are not, so this navigates by the former.
const CONTENTS = [
  ['1', 'Scope'],
  ['2', 'The deliverable'],
  ['', 'What the review established \u00B7 A gap is a measurement, a benchmark is an apparatus \u00B7 The score \u00B7 Which modalities are allowed in'],
  ['3', 'Before anything moves'],
  ['', 'The cell \u00B7 The one check that must pass \u00B7 What you do NOT need yet'],
  ['4', 'Phase 0 \u2014 characterise the sensors'],
  ['', 'Smoke test \u00B7 Stationary log \u00B7 Tumble \u00B7 Tap alignment \u00B7 Physical measurements \u00B7 Gate A'],
  ['5', 'Phases 1\u20132 \u2014 the error budget and the floor'],
  ['6', 'Phase 3 \u2014 pilot campaign, and Gate B early'],
  ['7', 'The simulated side'],
  ['', 'What to feed it \u00B7 Choosing a simulator \u00B7 The four things that must match'],
  ['8', 'Phase 5 \u2014 measure the gap, and Gate C'],
  ['9', 'Phase 6 \u2014 scoring and the leaderboard'],
  ['10', 'The full campaign'],
  ['11', 'Schedule'],
  ['12', 'Known traps'],
  ['A', 'Appendix A \u2014 Command reference'],
  ['B', 'Appendix B \u2014 Run file format'],
  ['C', 'Appendix C \u2014 Open decisions'],
];
CONTENTS.forEach(function (row) {
  if (!row[0]) {
    body.push(new Paragraph({
      spacing: { after: 130, line: 252 },
      indent: { left: 700 },
      children: [new TextRun({ text: row[1], size: 18, color: MUTED, italics: true })]
    }));
    return;
  }
  body.push(new Paragraph({
    spacing: { after: 40, line: 264 },
    indent: { left: 700, hanging: 700 },
    children: [
      new TextRun({ text: row[0] + '\u00A0\u00A0\u00A0', size: 21, bold: true, color: ACCENT }),
      new TextRun({ text: row[1], size: 21, color: INK })
    ]
  }));
});

body.push(new Paragraph({ children: [new PageBreak()] }));

/* ---------- 1 ---------- */
body.push(h1('1. Scope'));
body.push(p('This protocol covers the acquisition of the SONAIR sim-to-real benchmark dataset on the UR5e cell, the generation of its simulated counterpart, and the scoring that turns the two into a published leaderboard.'));
body.push(p('It does not cover the inspection application case (defect detection, multi-view reconstruction, sensor fusion). Those are a separate track and are deliberately kept out of the benchmark — see §2.4.'));

body.push(h2('1.1 What already works, and what this protocol adds'));
body.push(p('The console, the recorder, the scoring harness and the leaderboard page are built and tested. What has not happened yet is a single real run. This document is the bridge between the two.'));
body.push(table([3400, 5960], ['Already built and verified', 'What you supply'], [
  ['Console: robot control, IMU ingestion, calibration, automated job runner, dataset export', 'A cell that passes pre-flight'],
  ['Recorder writing measured and commanded robot state plus every inertial sample', 'The runs themselves'],
  ['sonair_benchmark: phase0, budget, plan, gap, score, demo', 'The Phase 0 characterisation the rest is measured against'],
  ['benchmark.html leaderboard, reading the harness output directly', 'The simulated side'],
]));

/* ---------- 2 ---------- */
body.push(h1('2. The deliverable'));
body.push(h2('2.1 What the review established'));
body.push(p('The review recording with Sam settled the shape of the project, and it said the same thing three times:'));
body.push(quote('But that’s not a benchmark. … You need to do a benchmark. … Don’t extend.', 'Sam, review'));
body.push(p('The deliverable is not a platform that displays sensor outputs, and it is not an inspection demo. It is a scored sim-to-real benchmark: a metric matrix held privately, external teams submitting models, those models scored on how well they move simulation data onto real data, with published examples and a stated methodology — the shape of BenchCAD or GPQA Diamond.'));
body.push(p('The contributions, in his order:'));
newProcedure();
body.push(step('The benchmark. This is the main one.'));
body.push(stepRich([['A basic AI model that solves it. ', {}], ['“It doesn’t have to be a good one.”', { italics: true }]]));
body.push(step('One application domain to apply it to — inspection.'));

body.push(h2('2.2 A gap is a measurement. A benchmark is an apparatus.'));
body.push(callout('bad', 'The most common way to get this wrong', [
  ['Measuring the difference between the real UR5e and the simulated UR5e gives you ', {},
   ['a number about your cell', { bold: true }], '. It is not the benchmark. Publishing it as one is exactly what the review warned against.'],
  ['The gap is the ', {}], // placeholder replaced below
]));
// replace the malformed second line with a clean one
body.pop();
body.push(callout('bad', 'The most common way to get this wrong', [
  [['Measuring the difference between the real UR5e and the simulated UR5e gives you ', {}],
   ['a number about your cell', { bold: true }],
   ['. It is not the benchmark, and publishing it as one is exactly what the review warned against.', {}]],
  [['The raw gap is the ', {}], ['denominator', { bold: true }],
   [' of the score — the baseline that submissions are asked to close. What makes the thing a benchmark is everything around it: the held-out condition cells, the percentile at which it is scored, the measurement floor, the gates, and the two baselines.', {}]],
  [['The test is simple: can someone else submit a model and get a number comparable with yours? If not, it is a measurement.', {}]]
]));

body.push(h2('2.3 The score'));
body.push(code(['GCR = 1 - err(prediction, real) / err(simulation, real)']));
body.push(gap(100));
body.push(p('Gap Closure Ratio, reported at the median and the 95th percentile. 1.0 reproduces the real side exactly; 0.0 is no better than handing back the simulation unchanged; negative is worse than doing nothing.'));
body.push(rich([['The p95 figure is the headline, and that choice is the whole design.', { bold: true }]]));
body.push(quote('The long tail problem is when you have very small examples of training data, but very safety critical consequences… it’s those rare events that are the most critical.', 'Sam, review'));
body.push(p('A benchmark scored on means will be closed by a model that matches means. Scoring at p95 means a model that matches the centre of the error distribution but not its tails scores well on GCR-median and badly on GCR-p95 — and that visible split is the thing the benchmark exists to measure.'));

body.push(h2('2.4 Which modalities are allowed in, and why'));
body.push(p('Asked what makes a good ground truth, the answer was unambiguous:'));
body.push(quote('And the IMU? Yeah, perfect, perfect. So orientation. And everything the IMU gives you. So your orientation plus your change of orientation plus your acceleration of orientation. So all of those temporal fields of orientation. Easy ground truth.', 'Sam, review'));
body.push(p('And on what to exclude, after the probe lift-off idea was described:'));
body.push(quote('Can you simulate the defect and the response of that probe to the defect? … So therefore how would you make a sim to real gap with that?', 'Sam, review'));
body.push(rich([['The selection rule for the whole project: a modality earns its place only if it is ', {}],
  ['both cheap to ground-truth on real hardware and faithful to generate in simulation', { bold: true }], ['.', {}]]));
body.push(gap(80));
body.push(table([2600, 1500, 5260], ['Modality', 'Verdict', 'Reason'], [
  ['Orientation', 'Scored', 'IMU quaternion against simulated tool attitude; geodesic error'],
  ['Angular rate', 'Scored', 'Directly measured, directly simulated'],
  ['Acceleration', 'Scored', 'Directly measured; the channel that carries structural ringing a rigid-body simulator cannot invent'],
  ['Position', 'Scored', 'From the robot’s own forward kinematics — no camera involved'],
  ['Force / torque', 'Candidate', 'Already on RTDE at no extra cost; the proposed third modality. Confirm with Sam (Appendix C)'],
  ['Eddy current, ultrasound, thermography, Raman', 'Excluded', 'Cannot be simulated well enough for the difference to mean anything. Application case and PhD, not benchmark'],
], { cellStyle: function (t, i) {
  if (i !== 1) return {};
  if (t === 'Scored') return { bold: true, color: OK };
  if (t === 'Excluded') return { bold: true, color: BAD };
  return { bold: true, color: WARN };
} }));

/* ---------- 3 ---------- */
body.push(h1('3. Before anything moves'));
body.push(h2('3.1 The cell'));
body.push(bullet('The host agent runs on the workstation wired to the UR5e — not on a laptop, and not across the relay.'));
body.push(bullet('The teach pendant must be in Remote Control. This cannot be read back from the controller, so nothing can warn you: the robot accepts the connection and silently ignores every command.'));
body.push(bulletRich([['The robot address is entered once, on Connect → Robot link. It drives all five channels (RTDE telemetry, realtime 30003, URScript 30002, dashboard 29999, FTP program list).', {}]]));
body.push(bullet('USB 3.0 port and the cable that came with the camera, if the camera is in use.'));

body.push(h2('3.2 The one check that must pass before any run'));
body.push(callout('bad', 'target_q — without it the campaign is unusable', [
  [['A simulator must be fed the ', {}], ['commanded', { bold: true }],
   [' trajectory. Feeding it the measured trajectory returns a gap of zero by construction, because the simulator has been handed the answer.', {}]],
  [['The commanded trajectory is not the waypoints. A UR generates its own joint trajectory from a Cartesian target, with its own blending, its own acceleration limits, and whatever the speed slider was set to. None of that is recoverable afterwards.', {}]],
  [['It is ', {}], ['target_q', mono], [', the controller’s own joint setpoint stream at the control rate. The recorder now writes it into every sample, and ', {}],
   ['refuses to start a run when it is not arriving', { bold: true }],
   ['. A run file missing it looks complete, opens cleanly and plots correctly.', {}]]
]));
body.push(gap(120));
body.push(rich([['Verification: run any job, export, and open one ', {}], ['runs/*.jsonl', mono],
  [' file. Every sample line must contain ', {}], ['target_q', mono], [' beside ', {}], ['q', mono], ['.', {}]]));

body.push(h2('3.3 What you do NOT need yet'));
body.push(rich([['The hand-eye calibration is ', {}], ['not', { bold: true }],
  [' required to begin. Every scored channel — orientation, angular rate, acceleration, and the tool pose the robot computes from its own joint angles — involves no camera at all. Pre-flight reports the calibration but only blocks jobs that declare they use the camera.', {}]]));
body.push(p('Do the calibration when the application case needs it. Do not hold the campaign for it.'));

/* ---------- 4 ---------- */
body.push(new Paragraph({ children: [new PageBreak()] }));
body.push(h1('4. Phase 0 — characterise the sensors'));
body.push(quote('Your IMUs are your first bet.', 'Sam, review'));
body.push(p('Nothing else in this protocol means anything without Phase 0. It produces the noise floor that every later error claim is measured against, and it is what makes it possible to tell a real sim-to-real difference from your own sensor noise.'));

body.push(h2('4.1 Smoke test first (30 minutes, not data)'));
body.push(p('Before recording anything you intend to keep, prove the pipeline end to end.'));
newProcedure();
body.push(step('Start the agent. Connect the console. Connect → Robot link → Connect to the robot.'));
body.push(step('Sensors page: connect the industrial IMU. Confirm the update rate is not zero and the channel cards are populated.'));
body.push(step('Automate page → Check now. Expect Ready (camera and calibration appear as warnings, not blockers).'));
body.push(step('Run the checkout job, then single_run.'));
body.push(stepRich([['Export everything from this job, then open a ', {}], ['runs/*.jsonl', mono],
  [' file and confirm ', {}], ['target_q', mono], [' is present. ', {}],
  ['Do not proceed until it is.', { bold: true }]]));

body.push(h2('4.2 Stationary log — the noise floor'));
body.push(p('One hour, unit clamped, on a surface isolated from foot traffic. Use Sensors → Record every sample to a file.'));
body.push(code([
  '# The console writes imu_logs/imu_<timestamp>.csv',
  'python -m sonair_benchmark phase0 \\',
  '    --fusionhub imu_logs/imu_<timestamp>.csv \\',
  '    --unit ind0 --expected-hz 200 \\',
  '    --out phase0/ind0.json'
]));
body.push(gap(120));
body.push(p('This yields gyroscope bias and noise, accelerometer bias, noise and scale error, the orientation noise floor in deg/s, sample-rate stability and dropped fraction.'));
body.push(callout('warn', 'Watch the jitter warning', [
  'If sample jitter exceeds 2 ms the tool says so. A drifting sample rate reads downstream as a velocity-dependent gap that is not real — which is precisely the structure Gate C is looking for, so it would be mistaken for a result.'
]));

body.push(h2('4.3 Six-position tumble test'));
body.push(p('Gravity as the reference. Rest the carrier on each of six faces for roughly 20 seconds, holding still. This checks axis signs, cross-axis sensitivity and scale against a known 9.80665 m/s².'));

body.push(h2('4.4 Tap alignment'));
body.push(p('One sharp, firm tap on the carrier, seen by every inertial channel at once. Record → Verify tap alignment. The spread of arrival times across channels is the temporal row of the error budget.'));
body.push(rich([['Record the number. It becomes ', {}], ['--tap-spread-ms', mono], [' in §5.', {}]]));

body.push(h2('4.5 Physical measurements (ten minutes, needed twice)'));
body.push(bullet('Weigh the printed bracket and anything else carried at the wrist. This is the carrier mass, and it must be measured, not taken from CAD.'));
body.push(bullet('Measure the IMU mounting offset and orientation relative to the tool flange. A lever-arm error reads as an orientation gap.'));
body.push(p('Both feed the SimContract in §7.4.'));

body.push(h2('4.6 Gate A'));
body.push(table([2000, 7360], ['Gate A', 'Detail'], [
  ['Asks', 'Do the IMUs produce usable data at the declared rate?'],
  ['Pass', 'Proceed to the error budget.'],
  ['Fail', 'It is a driver or mounting problem, not a sensing one. No hardware purchase is justified yet. Reproduce the failure on the workstation before blaming the sensor.'],
], { cellStyle: function (t, i) { return i === 0 ? { bold: true, color: MUTED } : {}; } }));

/* ---------- 5 ---------- */
body.push(h1('5. Phases 1–2 — the error budget and the floor'));
body.push(p('The budget combines every measured error source in root-sum-square and produces the floor below which no gap can be interpreted.'));
body.push(code([
  'python -m sonair_benchmark budget \\',
  '    --phase0 phase0/ind0.json \\',
  '    --tap-spread-ms 0.8 \\',
  '    --tracker-mm 1.2 \\',
  '    --frame-fit-mm 0.8 \\',
  '    --calib-version calib-1 \\',
  '    --out calib/budget.json'
]));
body.push(gap(120));
body.push(rich([['Record the two numbers it prints under ', {}], ['floor', mono],
  [': ', {}], ['position_mm', mono], [' and ', {}], ['orientation_deg', mono],
  ['. Every result from here on is quoted against them, and the leaderboard flags any run whose gap falls below the floor as not interpretable.', {}]]));
body.push(callout('note', 'The budget refuses to be incomplete', [
  'Any row that has not been measured is listed as unmeasured and the budget is marked incomplete. A result cannot be published without its own floor attached — this is deliberate and should not be worked around by supplying a guess.'
]));

/* ---------- 6 ---------- */
body.push(h1('6. Phase 3 — pilot campaign, and Gate B early'));
body.push(callout('warn', 'Do not run 270 runs first', [
  [['Gate B asks whether the gap is large compared with your own measurement floor. If it is not, the benchmark is measuring itself. ', {}],
   ['Find that out after twenty runs, not after 1.9 hours of arm time and three sessions.', { bold: true }]]
]));
body.push(h2('6.1 The pilot'));
body.push(p('Two elbow velocities × two arm configurations × a few repeats, roughly twenty runs. Use the Automate page:'));
newProcedure();
body.push(step('Automate → Check now → Ready.'));
body.push(step('Job: speed_sweep. Set Repeats to 2 or 3, and Speeds to sweep to two values, e.g. 0.20, 0.60.'));
body.push(step('Clear the cell. Keep the physical e-stop within reach. Run the job.'));
body.push(step('The run bar at the top of every page tracks progress, so the campaign can be left running and checked from wherever you are.'));
body.push(step('Export everything from this job.'));
body.push(gap(100));
body.push(p('Then generate the matching simulated runs (§7) and measure:'));
body.push(code([
  'python -m sonair_benchmark gap \\',
  '    --real data/real --sim data/sim \\',
  '    --budget calib/budget.json \\',
  '    --out results/gap.json'
]));
body.push(gap(120));
body.push(h2('6.2 Gate B'));
body.push(table([2000, 7360], ['Gate B', 'Detail'], [
  ['Asks', 'Is the measured gap large compared with the calibration and sensing residual?'],
  ['Pass', 'Proceed to the full campaign.'],
  ['Marginal', 'The gap is comparable to the floor. Publish as an upper bound, or reduce the floor first — tighter carrier mounting, better time alignment, a better calibration.'],
  ['Fail', 'The benchmark is measuring itself. Stop and fix the rig before spending the arm time.'],
], { cellStyle: function (t, i) {
  if (i !== 0) return {};
  return { bold: true, color: t === 'Pass' ? OK : t === 'Fail' ? BAD : t === 'Marginal' ? WARN : MUTED };
} }));

/* ---------- 7 ---------- */
body.push(new Paragraph({ children: [new PageBreak()] }));
body.push(h1('7. The simulated side'));
body.push(h2('7.1 What to feed it'));
body.push(rich([['Feed the simulator ', {}], ['target_q', mono],
  [' as joint position targets and let it produce its own ', {}], ['q', mono], ['.', {}]]));
body.push(gap(100));
body.push(table([3600, 5760], ['Compare', 'Gives you'], [
  ['real target_q vs real q', 'How well the real robot tracks its own controller'],
  ['sim target_q vs sim q', 'How well the simulated robot tracks the same'],
  ['real q vs sim q', 'The plant gap — friction, drive flexibility, payload inertia'],
  ['real IMU vs sim IMU', 'The gap in what a sensor on the tool actually feels'],
], { cellStyle: function (t, i) { return i === 0 ? { font: 'Consolas', size: 18 } : {}; } }));
body.push(gap(140));
body.push(p('The last row is what earns the IMU its place in the benchmark. A rigid-body simulator reproduces the trajectory; it does not reproduce the structural ringing a wrist-mounted accelerometer sees when the arm stops. That difference is real, it is large, and it is exactly what a submitted model has to learn.'));
body.push(rich([['speed_scaling', mono], [' is recorded alongside. A run captured at 50% speed executed a different trajectory from the one commanded, and without that field nothing downstream can tell.', {}]]));

body.push(h2('7.2 Choosing a simulator'));
body.push(rich([['The contract names its solver in a field and the harness never imports a simulator, so this choice can change later without invalidating anything already recorded.', {}]]));
body.push(gap(100));
body.push(table([1900, 1700, 3200, 2560], ['Simulator', 'Runs on', 'Strength', 'Cost'], [
  ['MuJoCo', 'CPU, pip install', 'Joint dynamics; friction, damping and armature can be fitted to your own recordings', 'No camera worth the name'],
  ['Isaac Sim', 'RTX GPU, ~30 GB', 'RTX-accurate depth and colour cameras; GPU-parallel runs', 'Heavy install; no more accurate than MuJoCo for a serial arm with no contact'],
  ['Gazebo + ur_robot_driver', 'CPU', 'Runs the real UR control stack, so the controller stops being part of the gap', 'ROS 2 setup'],
  ['URSim (UR’s Docker image)', 'CPU', 'The actual UR controller software — same target_q generation as the real robot', 'Controller only, no plant'],
], { cellStyle: function (t, i) { return i === 0 ? { bold: true } : { size: 18 }; } }));
body.push(gap(160));
body.push(callout('note', 'Recommendation', [
  [['Start with MuJoCo.', { bold: true }], [' It installs in a minute on the machine already wired to the robot, mujoco_menagerie ships a UR5e, and it is the only one of these where the plant parameters can be ', {}],
   ['identified from the runs you have just recorded', { italics: true }], [' rather than taken from a datasheet. You will have a gap number in days.', {}]],
  [['Keep Isaac Sim as the declared simulator', { bold: true }], [' for the benchmark and the proposal — it is what the programme expects, and its cameras are needed the moment the application case rejoins the story. Because the contract carries the solver name, running MuJoCo first is a step on the path, not a detour: the same run files, the same scorer, the same leaderboard.', {}]],
  [['URSim is a refinement, not a start.', { bold: true }], [' If the controller turns out to be a large part of the gap, put URSim in front of the plant simulator and the two are separated. Find out whether it matters first.', {}]]
]));

body.push(h2('7.3 The four things that must match'));
body.push(p('These are held in the SimContract, written next to the simulated dataset and checked on import. A simulated run generated under a different contract is refused rather than silently pooled.'));
body.push(table([600, 3400, 5360], ['#', 'Must match', 'Failure if skipped'], [
  ['1', 'Commanded trajectory, units, command rate', 'You measure a different trajectory, not a gap'],
  ['2', 'Carrier mass and centre of mass at the wrist — measured, not CAD', 'Wrist dynamics differ; the gap is inflated by payload error'],
  ['3', 'Sensor rate and mounting offsets — measured in Phase 0', 'A lever-arm error reads as an orientation gap'],
  ['4', 'Simulated sensors degraded with the Phase 0 noise and bias', 'You measure “the simulator has no sensor noise”'],
], { cellStyle: function (t, i) { return i === 0 ? { bold: true, color: ACCENT, align: AlignmentType.CENTER } : {}; } }));
body.push(gap(140));
body.push(callout('warn', 'Point 4 is the one everybody skips', [
  'An ideal simulated IMU makes the gap look larger than it is, for a reason that has nothing to do with dynamics. Worse, if the simulator is allowed to invent its own noise, that noise becomes part of the thing being scored and a submission can win by modelling your random number generator. Apply the measured noise floor outside the simulator, from the Phase 0 file.'
]));

/* ---------- 8 ---------- */
body.push(h1('8. Phase 5 — measure the gap'));
body.push(code([
  'python -m sonair_benchmark gap \\',
  '    --real data/real --sim data/sim \\',
  '    --budget calib/budget.json \\',
  '    --out results/gap.json'
]));
body.push(gap(120));
body.push(p('This reports the per-cell gap, runs Gate B against the budget, and runs Gate C on the structure.'));
body.push(h2('8.1 Gate C'));
body.push(table([2000, 7360], ['Gate C', 'Detail'], [
  ['Asks', 'Does the gap vary systematically with the operating condition?'],
  ['Pass', 'There is structure to learn. Proceed to scoring.'],
  ['Fail', 'No signal. Widen the sweep before publishing — a benchmark whose gap is constant across conditions asks a model to predict a constant.'],
], { cellStyle: function (t, i) { return i === 0 ? { bold: true, color: MUTED } : {}; } }));

/* ---------- 9 ---------- */
body.push(h1('9. Phase 6 — scoring and the leaderboard'));
body.push(h2('9.1 The two baselines'));
body.push(p('Run both before publication. They set the floor the leaderboard starts from, and both go through the same code path as every submission — a benchmark whose baseline is computed separately will eventually disagree with itself.'));
body.push(bulletRich([['Identity', { bold: true }], [' — hand back the simulation unchanged. Scores 0 by construction.', {}]]));
body.push(bulletRich([['Constant offset', { bold: true }], [' — one global XYZ offset fitted on the example set. It is the dumbest thing that could possibly work, and ', {}], ['a model that does not beat it has not demonstrated anything.', { bold: true }]]));

body.push(h2('9.2 Scoring'));
body.push(code([
  'python -m sonair_benchmark score \\',
  '    --real data/real --sim data/sim \\',
  '    --budget calib/budget.json \\',
  '    --plan campaign/plan.json \\',
  '    --submission subs/<model>.jsonl --name "<model name>" \\',
  '    --out site/leaderboard.json'
]));

body.push(h2('9.3 The leaderboard page'));
body.push(rich([['benchmark.html', mono], [' is the deliverable — the page the review was pointing at when BenchCAD and GPQA Diamond were on screen. It is ', {}],
  ['not', { bold: true }], [' a simulation page. It reads ', {}], ['site/leaderboard.json', mono],
  [' and ', {}], ['results/gap.json', mono], [' straight from the harness and is never hand-maintained.', {}]]));
body.push(gap(80));
body.push(p('It renders the leaderboard ranked on GCR p95, the headline statistics, the task and I/O specification, the gap map across conditions, the measurement floor, and how to submit. It flags any run whose baseline gap falls below the rig’s floor as not interpretable, so a result cannot be presented without its own floor attached.'));
body.push(gap(80));
body.push(rich([['To see the finished shape before you have real data: ', {}], ['python -m sonair_benchmark demo --out demo/', mono],
  [', copy ', {}], ['benchmark.html', mono], [' into ', {}], ['demo/', mono], [', and serve that folder over HTTP.', {}]]));

/* ---------- 10 ---------- */
body.push(h1('10. The full campaign'));
body.push(code(['python -m sonair_benchmark plan --out campaign/plan.json --repeats 3']));
body.push(gap(120));
body.push(p('The default plan, as generated by the harness:'));
body.push(table([3400, 5960], ['Quantity', 'Value'], [
  ['Total runs', '270'],
  ['Condition cells', '54 (36 published, 18 held out)'],
  ['Runs per cell', '5'],
  ['Sessions', '3, on separate days'],
  ['Estimated arm time', '1.9 hours'],
  ['Estimated raw size', '0.17 GB'],
], { cellStyle: function (t, i) { return i === 0 ? { bold: true, color: MUTED } : {}; } }));

body.push(h2('10.1 The four factors'));
body.push(table([2400, 6960], ['Factor', 'Levels and reasoning'], [
  ['Elbow velocity', '0.2 / 0.4 / 0.5 / 0.6 / 0.7 / 0.9 rad/s — deliberately bracketing the ~0.6 rad/s region where the simulator is already known to change behaviour. Sampling either side of a known divergence turns a bug into the structure the benchmark measures.'],
  ['Arm configuration', 'near-singular / mid-workspace / extended'],
  ['Trajectory type', 'point-to-point / contour / stop-start. Stop-start is closest to real inspection scanning, which is what keeps the conditions relevant rather than arbitrary.'],
  ['Repeats', 'Spread across sessions days apart, with one deliberate carrier refit — so refit error is measured rather than assumed away.'],
], { cellStyle: function (t, i) { return i === 0 ? { bold: true } : {}; } }));

body.push(h2('10.2 The held-out set'));
body.push(rich([['Whole condition cells, not random samples.', { bold: true }],
  [' Interpolating inside a condition you have already seen is easy. The unsolved problem is generalising across conditions — to an elbow velocity or an arm configuration that was never shown. Splitting at random would measure the easy problem and report it as the hard one.', {}]]));

/* ---------- 11 ---------- */
body.push(h1('11. Schedule'));
body.push(table([1500, 3100, 4760], ['When', 'What', 'Output / gate'], [
  ['Day 0', 'Smoke test: connect, run checkout and single_run, export, verify target_q', 'A run file with commanded and measured state'],
  ['Days 1–2', 'Phase 0: one-hour stationary log, tumble test, tap test. Weigh the carrier, measure the IMU offset', 'phase0/ind0.json — Gate A'],
  ['Day 3', 'Error budget', 'calib/budget.json — the floor'],
  ['Days 4–5', 'Pilot campaign, ~20 runs, two velocities', 'data/real pilot set'],
  ['Week 2', 'MuJoCo replay of the pilot from target_q; first gap measurement', 'results/gap.json — Gate B'],
  ['Week 3+', 'Full 270-run campaign across three sessions, if Gate B passed', 'Complete data/real'],
  ['Week 4', 'Full simulated set; gap and structure', 'Gate C'],
  ['Week 5', 'Baselines, first model, leaderboard', 'site/leaderboard.json'],
], { cellStyle: function (t, i) { return i === 0 ? { bold: true, color: ACCENT } : { size: 18 }; } }));

/* ---------- 12 ---------- */
body.push(h1('12. Known traps'));
body.push(table([3000, 6360], ['Trap', 'Why it costs more than it looks'], [
  ['Recording without target_q', 'The run files look complete and cannot be replayed against anything meaningful. The recorder now refuses, but do not override it.'],
  ['Feeding the simulator the measured trajectory', 'Gap of zero by construction. The simulator has been handed the answer.'],
  ['Letting the simulator generate its own sensor noise', 'The noise becomes part of the score, and a submission can win by modelling your random number generator.'],
  ['Skipping Phase 0', 'There is no way to tell a real sim-to-real difference from sensor noise, and no floor to quote results against.'],
  ['Running the full sweep before Gate B', '1.9 hours of arm time and three sessions spent before discovering the benchmark is measuring itself.'],
  ['Splitting the held-out set at random', 'Measures interpolation inside known conditions and reports it as generalisation across them.'],
  ['Leading with the inspection demo', 'It is the application case, not the benchmark. The review warned against this twice.'],
  ['Pendant left in Local mode', 'The robot accepts the connection and ignores every command. Nothing can detect it; the first move simply does nothing.'],
], { cellStyle: function (t, i) { return i === 0 ? { bold: true, size: 18 } : { size: 18 }; } }));

/* ---------- Appendix A ---------- */
body.push(new Paragraph({ children: [new PageBreak()] }));
body.push(h1('Appendix A — Command reference'));
body.push(h2('Harness'));
body.push(code([
  '# Phase 0 — noise floor and rate stability',
  'python -m sonair_benchmark phase0 --fusionhub <csv> --unit ind0 \\',
  '    --expected-hz 200 --out phase0/ind0.json',
  '',
  '# Phase 2 — error budget',
  'python -m sonair_benchmark budget --phase0 phase0/ind0.json \\',
  '    --tap-spread-ms 0.8 --tracker-mm 1.2 --frame-fit-mm 0.8 \\',
  '    --calib-version calib-1 --out calib/budget.json',
  '',
  '# Phase 3 — condition sweep',
  'python -m sonair_benchmark plan --out campaign/plan.json --repeats 3',
  '',
  '# Phase 5 — gap, Gate B and Gate C',
  'python -m sonair_benchmark gap --real data/real --sim data/sim \\',
  '    --budget calib/budget.json --out results/gap.json',
  '',
  '# Phase 6 — scoring',
  'python -m sonair_benchmark score --real data/real --sim data/sim \\',
  '    --budget calib/budget.json --plan campaign/plan.json \\',
  '    --submission subs/<model>.jsonl --name "<name>" \\',
  '    --out site/leaderboard.json',
  '',
  '# End-to-end synthetic demonstration',
  'python -m sonair_benchmark demo --out demo/'
]));

body.push(h2('Console'));
body.push(table([2800, 6560], ['Page', 'What you do there'], [
  ['Connect', 'Link the agent; set the robot address (drives all five channels); Diagnostics shows recent faults and the streams in use'],
  ['Robot', 'Power, brake release, jog, programs, I/O, live joint data'],
  ['Camera', 'Stream configuration and depth quality'],
  ['Sensors', 'IMU link, every reading live, and the two export routes: continuous logging and an in-memory snapshot'],
  ['Calibrate', 'Hand-eye — not needed for the benchmark channels'],
  ['Automate', 'Pre-flight, jobs, campaign runner, dataset export'],
  ['Inspect / Record', 'Application case; manual single runs'],
], { cellStyle: function (t, i) { return i === 0 ? { bold: true } : { size: 18 }; } }));

/* ---------- Appendix B ---------- */
body.push(h1('Appendix B — Run file format'));
body.push(rich([['Each run is a ', {}], ['.jsonl', mono],
  [' file. The first line is the run manifest; every line after it is one sample on a fixed time grid.', {}]]));
body.push(gap(100));
body.push(code([
  'import json',
  'with open("runs/<name>.jsonl") as f:',
  '    manifest = json.loads(f.readline())["_manifest"]',
  '    samples  = [json.loads(line) for line in f]'
]));
body.push(gap(140));
body.push(table([2400, 1700, 5260], ['Field', 'Units', 'Meaning'], [
  ['t', 's', 'Host monotonic clock, shared across channels in one run'],
  ['q', 'rad × 6', 'Measured joint angles'],
  ['qd', 'rad/s × 6', 'Measured joint velocities'],
  ['target_q', 'rad × 6', 'Commanded joint angles — the simulator’s input'],
  ['target_qd', 'rad/s × 6', 'Commanded joint velocities'],
  ['target_moment', 'Nm × 6', 'Commanded joint torques'],
  ['speed_scaling', '—', 'Fraction of the commanded trajectory the controller was allowed to execute'],
  ['tcp_pos', 'm × 3', 'Tool position, robot base frame'],
  ['tcp_rot', 'rad × 3', 'Tool orientation as a rotation vector'],
  ['imu', 'per unit', 'Quaternion, angular rate, acceleration, linear acceleration, magnetic field'],
], { cellStyle: function (t, i) { return i === 0 ? { font: 'Consolas', size: 18, bold: true } : { size: 18 }; } }));
body.push(gap(140));
body.push(callout('warn', 'Clock alignment', [
  [['Run files are stamped on the host monotonic clock. Inertial CSVs are stamped on the sensor’s own clock plus whatever offset has been measured for it. Any channel listed under ', {}],
   ['clock.unaligned', mono],
   [' in a dataset manifest has never been tied to the host clock — its times are internally consistent and are ', {}],
   ['not', { italics: true }], [' comparable with another channel’s. The tap test is what measures the offset.', {}]]
]));

/* ---------- Appendix C ---------- */
body.push(h1('Appendix C — Open decisions'));
body.push(table([3400, 5960], ['Decision', 'Recommendation and reasoning'], [
  ['The third scored modality', 'Force. The UR5e already streams actual_TCP_force on RTDE at no extra cost and it meets the simulatability test. Needs Sam to confirm.'],
  ['Which simulator is declared', 'Isaac Sim for the proposal and the paper; MuJoCo as the working instrument to get the first numbers. The contract names the solver, so both are compatible with the same leaderboard.'],
  ['Who collects the data', 'The review pushed for a Master’s student to run the campaign. 1.9 hours of arm time across three sessions is a well-specified task once this protocol is followed.'],
  ['Industrial anchor for the paper', 'Rotor magnet slices are 3–4 mm thick, so a position error above 1 mm is a real failure. That is the number that makes a position-accuracy benchmark matter to someone outside the field — it belongs in the introduction.'],
], { cellStyle: function (t, i) { return i === 0 ? { bold: true } : { size: 18 }; } }));

body.push(gap(320));
body.push(new Paragraph({
  spacing: { before: 200 },
  border: { top: { style: BorderStyle.SINGLE, size: 4, color: RULE, space: 10 } },
  children: [new TextRun({
    text: 'SONAIR · University of Nottingham & University College London · UK Open Multimodal AI Benchmark programme',
    size: 17, color: MUTED, italics: true })]
}));

/* ------------------------------------------------------------------ *
 * Document
 * ------------------------------------------------------------------ */
const doc = new Document({
  // Word and LibreOffice both leave a TOC field empty until the field is
  // updated. Without this the reader opens the document and finds a blank
  // Contents page, which reads as a broken file rather than as an unrefreshed
  // field.
  features: { updateFields: true },
  creator: 'SONAIR',
  title: 'SONAIR Sim-to-Real Benchmark — Experimental Protocol',
  description: 'Data collection campaign protocol, from first recording to scored leaderboard',
  styles: {
    default: {
      document: { run: { font: 'Calibri', size: 21, color: INK } }
    }
  },
  numbering: {
    config: [
      {
        reference: 'dots',
        levels: [
          { level: 0, format: LevelFormat.BULLET, text: '•', alignment: AlignmentType.LEFT,
            style: { paragraph: { indent: { left: 420, hanging: 220 } } } },
          { level: 1, format: LevelFormat.BULLET, text: '◦', alignment: AlignmentType.LEFT,
            style: { paragraph: { indent: { left: 780, hanging: 220 } } } }
        ]
      },
      {
        reference: 'steps',
        levels: [
          { level: 0, format: LevelFormat.DECIMAL, text: '%1.', alignment: AlignmentType.START,
            style: { paragraph: { indent: { left: 460, hanging: 260 } } } }
        ]
      }
    ]
  },
  sections: [{
    properties: {
      page: {
        margin: { top: 1440, right: 1440, bottom: 1440, left: 1440 }
      }
    },
    headers: {
      default: new Header({
        children: [new Paragraph({
          alignment: AlignmentType.RIGHT,
          spacing: { after: 0 },
          children: [new TextRun({
            text: 'SONAIR · Experimental Protocol · v1.0',
            size: 16, color: MUTED })]
        })]
      })
    },
    footers: {
      default: new Footer({
        children: [new Paragraph({
          alignment: AlignmentType.CENTER,
          children: [new TextRun({ children: [PageNumber.CURRENT], size: 16, color: MUTED })]
        })]
      })
    },
    children: body
  }]
});

Packer.toBuffer(doc).then(function (buf) {
  fs.writeFileSync(process.argv[2], buf);
  console.log('written ' + process.argv[2] + '  ' + (buf.length / 1024).toFixed(0) + ' kB');
});
