import { useState, useRef, useEffect, useCallback } from "react";


const DEG = Math.PI / 180;

// Default DH parameters for a 5-DOF arm (similar to xArm5)
const DEFAULT_DH = [
  { name: "Base",     theta: 0, d: 80,  a: 0,   alpha: -90, min: -180, max: 180, color: "#ff4d4d" },
  { name: "Shoulder", theta: 0, d: 0,   a: 200, alpha: 0,   min: -130, max: 130, color: "#ff9f1a" },
  { name: "Elbow",    theta: 0, d: 0,   a: 200, alpha: 0,   min: -130, max: 130, color: "#2dd4bf" },
  { name: "Wrist 1",  theta: 0, d: 0,   a: 0,   alpha: -90, min: -180, max: 180, color: "#818cf8" },
  { name: "Wrist 2",  theta: 0, d: 80,  a: 0,   alpha: 0,   min: -180, max: 180, color: "#f472b6" },
];

// DH transformation matrix
function dhMatrix(theta, d, a, alpha) {
  const ct = Math.cos(theta), st = Math.sin(theta);
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  return [
    [ct, -st * ca,  st * sa, a * ct],
    [st,  ct * ca, -ct * sa, a * st],
    [0,   sa,       ca,      d     ],
    [0,   0,        0,       1     ],
  ];
}

function matMul(A, B) {
  const C = Array.from({ length: 4 }, () => Array(4).fill(0));
  for (let i = 0; i < 4; i++)
    for (let j = 0; j < 4; j++)
      for (let k = 0; k < 4; k++)
        C[i][j] += A[i][k] * B[k][j];
  return C;
}

function identity4() {
  return [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
}

function forwardKinematics(joints) {
  let T = identity4();
  const frames = [{ T: identity4(), pos: [0, 0, 0] }];
  for (const j of joints) {
    const dh = dhMatrix(j.theta * DEG, j.d, j.a, j.alpha * DEG);
    T = matMul(T, dh);
    frames.push({ T: T.map(r => [...r]), pos: [T[0][3], T[1][3], T[2][3]] });
  }
  return frames;
}

// Project 3D to 2D (isometric-like)
function project3D(x, y, z, viewAngle, elevation, scale, cx, cy) {
  const ca = Math.cos(viewAngle), sa = Math.sin(viewAngle);
  const ce = Math.cos(elevation), se = Math.sin(elevation);
  const rx = x * ca - y * sa;
  const ry = -(x * sa + y * ca) * se + z * ce;
  return { x: cx + rx * scale, y: cy - ry * scale };
}

// Grid component
function drawGrid(ctx, viewAngle, elevation, scale, cx, cy) {
  ctx.strokeStyle = "rgba(100, 120, 140, 0.12)";
  ctx.lineWidth = 0.5;
  const s = 400, step = 50;
  for (let i = -s; i <= s; i += step) {
    let p1 = project3D(i, -s, 0, viewAngle, elevation, scale, cx, cy);
    let p2 = project3D(i, s, 0, viewAngle, elevation, scale, cx, cy);
    ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();
    p1 = project3D(-s, i, 0, viewAngle, elevation, scale, cx, cy);
    p2 = project3D(s, i, 0, viewAngle, elevation, scale, cx, cy);
    ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();
  }
}

function drawAxes(ctx, viewAngle, elevation, scale, cx, cy, len = 60) {
  const axes = [
    { dir: [len, 0, 0], color: "#ef4444", label: "X" },
    { dir: [0, len, 0], color: "#22c55e", label: "Y" },
    { dir: [0, 0, len], color: "#3b82f6", label: "Z" },
  ];
  const o = project3D(0, 0, 0, viewAngle, elevation, scale, cx, cy);
  for (const ax of axes) {
    const p = project3D(...ax.dir, viewAngle, elevation, scale, cx, cy);
    ctx.strokeStyle = ax.color;
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(o.x, o.y); ctx.lineTo(p.x, p.y); ctx.stroke();
    ctx.fillStyle = ax.color;
    ctx.font = "bold 12px 'JetBrains Mono', monospace";
    ctx.fillText(ax.label, p.x + 4, p.y - 4);
  }
}

export default function KinematicVisualizer() {
  const [joints, setJoints] = useState(DEFAULT_DH.map(j => ({ ...j })));
  const [viewAngle, setViewAngle] = useState(-0.6);
  const [elevation, setElevation] = useState(0.5);
  const [scale, setScale] = useState(0.75);
  const [trace, setTrace] = useState([]);
  const [showTrace, setShowTrace] = useState(true);
  const [showInfo, setShowInfo] = useState(true);
  const [isDragging, setIsDragging] = useState(false);
  const [dofCount, setDofCount] = useState(5);
  const [animating, setAnimating] = useState(false);
  const [gripperOpen, setGripperOpen] = useState(18);
  const [animPhase, setAnimPhase] = useState(0);
  const canvasRef = useRef(null);
  const dragRef = useRef({ startX: 0, startY: 0, startAngle: 0, startElev: 0 });
  const animRef = useRef(null);

  const activeJoints = joints.slice(0, dofCount);
  const frames = forwardKinematics(activeJoints);
  const endEffector = frames[frames.length - 1].pos;

  const updateJoint = useCallback((idx, angle) => {
    setJoints(prev => {
      const next = prev.map(j => ({ ...j }));
      next[idx].theta = Math.max(next[idx].min, Math.min(next[idx].max, angle));
      return next;
    });
  }, []);

  // Record trace
  useEffect(() => {
    if (showTrace) {
      setTrace(prev => {
        const next = [...prev, [...endEffector]];
        return next.length > 800 ? next.slice(-800) : next;
      });
    }
  }, [endEffector[0], endEffector[1], endEffector[2], showTrace]);

  // Animation loop
  useEffect(() => {
    if (!animating) { cancelAnimationFrame(animRef.current); return; }
    let phase = animPhase;
    const tick = () => {
      phase += 0.015;
      setAnimPhase(phase);
      setJoints(prev => prev.map((j, i) => ({
        ...j,
        theta: Math.sin(phase * (1 + i * 0.3)) * (j.max * 0.6),
      })));
      animRef.current = requestAnimationFrame(tick);
    };
    animRef.current = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(animRef.current);
  }, [animating]);

  // Canvas rendering
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    const W = canvas.width, H = canvas.height;
    const cx = W / 2, cy = H * 0.6;
    ctx.clearRect(0, 0, W, H);

    // Background gradient
    const bg = ctx.createRadialGradient(cx, cy * 0.6, 0, cx, cy * 0.6, W);
    bg.addColorStop(0, "#0f1923");
    bg.addColorStop(1, "#060b11");
    ctx.fillStyle = bg;
    ctx.fillRect(0, 0, W, H);

    drawGrid(ctx, viewAngle, elevation, scale, cx, cy);
    drawAxes(ctx, viewAngle, elevation, scale, cx, cy);

    // Draw trace
    if (showTrace && trace.length > 1) {
      ctx.beginPath();
      for (let i = 0; i < trace.length; i++) {
        const p = project3D(...trace[i], viewAngle, elevation, scale, cx, cy);
        const alpha = (i / trace.length) * 0.6;
        if (i === 0) { ctx.moveTo(p.x, p.y); }
        else {
          ctx.strokeStyle = `rgba(56, 189, 248, ${alpha})`;
          ctx.lineWidth = 1.2;
          ctx.lineTo(p.x, p.y);
          ctx.stroke();
          ctx.beginPath();
          ctx.moveTo(p.x, p.y);
        }
      }
    }

    // Draw arm links
    for (let i = 0; i < frames.length - 1; i++) {
      const p1 = project3D(...frames[i].pos, viewAngle, elevation, scale, cx, cy);
      const p2 = project3D(...frames[i + 1].pos, viewAngle, elevation, scale, cx, cy);
      const color = i < activeJoints.length ? activeJoints[i].color : "#666";

      // Link shadow
      ctx.strokeStyle = "rgba(0,0,0,0.4)";
      ctx.lineWidth = 14;
      ctx.lineCap = "round";
      ctx.beginPath(); ctx.moveTo(p1.x + 2, p1.y + 3); ctx.lineTo(p2.x + 2, p2.y + 3); ctx.stroke();

      // Link body
      const grad = ctx.createLinearGradient(p1.x, p1.y, p2.x, p2.y);
      grad.addColorStop(0, color);
      grad.addColorStop(1, color + "99");
      ctx.strokeStyle = grad;
      ctx.lineWidth = 10;
      ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();

      // Link highlight
      ctx.strokeStyle = color + "44";
      ctx.lineWidth = 14;
      ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();
    }

    // Draw joints
    for (let i = 0; i < frames.length; i++) {
      const p = project3D(...frames[i].pos, viewAngle, elevation, scale, cx, cy);
      const color = i < activeJoints.length ? activeJoints[i].color : (i === frames.length - 1 ? "#38bdf8" : "#666");
      const r = i === 0 ? 10 : (i === frames.length - 1 ? 8 : 7);

      // Joint glow
      ctx.shadowColor = color;
      ctx.shadowBlur = 15;
      ctx.beginPath();
      ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
      ctx.fillStyle = color;
      ctx.fill();
      ctx.shadowBlur = 0;

      // Joint ring
      ctx.strokeStyle = "#fff3";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(p.x, p.y, r + 3, 0, Math.PI * 2);
      ctx.stroke();
    }

    // =========================
    // GRIPPER
    // =========================

    const wristFrame = frames[frames.length - 1].T;

    const ex = wristFrame[0][0];
    const ey = wristFrame[1][0];
    const ez = wristFrame[2][0];

    const px = endEffector[0];
    const py = endEffector[1];
    const pz = endEffector[2];

    // Forward direction
    const tip = [
      px + ex * 35,
      py + ey * 35,
      pz + ez * 35,
    ];

    // Side vector
    const sx = wristFrame[0][1];
    const sy = wristFrame[1][1];
    const sz = wristFrame[2][1];

    // Finger offsets
    const leftBase = [
      tip[0] + sx * gripperOpen,
      tip[1] + sy * gripperOpen,
      tip[2] + sz * gripperOpen,
    ];

    const rightBase = [
      tip[0] - sx * gripperOpen,
      tip[1] - sy * gripperOpen,
      tip[2] - sz * gripperOpen,
    ];

    // Finger tips
    const leftTip = [
      leftBase[0] + ex * 25,
      leftBase[1] + ey * 25,
      leftBase[2] + ez * 25,
    ];

    const rightTip = [
      rightBase[0] + ex * 25,
      rightBase[1] + ey * 25,
      rightBase[2] + ez * 25,
    ];

    // Project to screen
    const ep         = project3D(...endEffector, viewAngle, elevation, scale, cx, cy); // FIX: was undeclared `ep`
    const pLeftBase  = project3D(...leftBase,    viewAngle, elevation, scale, cx, cy);
    const pRightBase = project3D(...rightBase,   viewAngle, elevation, scale, cx, cy);
    const pLeftTip   = project3D(...leftTip,     viewAngle, elevation, scale, cx, cy);
    const pRightTip  = project3D(...rightTip,    viewAngle, elevation, scale, cx, cy);
    const pTip       = project3D(...tip,         viewAngle, elevation, scale, cx, cy);

    // Wrist connector
    ctx.strokeStyle = "#38bdf8";
    ctx.lineWidth = 5;
    ctx.beginPath();
    ctx.moveTo(ep.x, ep.y);
    ctx.lineTo(pTip.x, pTip.y);
    ctx.stroke();

    // Left finger
    ctx.strokeStyle = "#f8fafc";
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(pLeftBase.x, pLeftBase.y);
    ctx.lineTo(pLeftTip.x, pLeftTip.y);
    ctx.stroke();

    // Right finger
    ctx.beginPath();
    ctx.moveTo(pRightBase.x, pRightBase.y);
    ctx.lineTo(pRightTip.x, pRightTip.y);
    ctx.stroke();

    // Finger connector
    ctx.strokeStyle = "#38bdf8";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(pLeftBase.x, pLeftBase.y);
    ctx.lineTo(pRightBase.x, pRightBase.y);
    ctx.stroke();

  }, [frames, trace, showTrace, viewAngle, elevation, scale, gripperOpen, activeJoints]); // FIX: properly closed useEffect

  // FIX: event handlers moved out of useEffect into component body
  const handleMouseDown = (e) => {
    if (e.target !== canvasRef.current) return;
    setIsDragging(true);
    dragRef.current = { startX: e.clientX, startY: e.clientY, startAngle: viewAngle, startElev: elevation };
  };
  const handleMouseMove = (e) => {
    if (!isDragging) return;
    const dx = e.clientX - dragRef.current.startX;
    const dy = e.clientY - dragRef.current.startY;
    setViewAngle(dragRef.current.startAngle - dx * 0.005);
    setElevation(Math.max(0.1, Math.min(1.4, dragRef.current.startElev + dy * 0.005)));
  };
  const handleMouseUp = () => setIsDragging(false);
  const handleWheel = (e) => {
    e.preventDefault();
    setScale(prev => Math.max(0.2, Math.min(2, prev - e.deltaY * 0.001)));
  };

  const resetArm = () => {
    setJoints(DEFAULT_DH.map(j => ({ ...j, theta: 0 })));
    setTrace([]);
  };

  return (
    <div style={{
      width: "100%", height: "100vh", display: "flex", fontFamily: "'JetBrains Mono', 'Fira Code', monospace",
      background: "#060b11", color: "#c8d6e5", overflow: "hidden",
    }}>
      {/* Sidebar Controls */}
      <div style={{
        width: 320, minWidth: 320, background: "linear-gradient(180deg, #0d1520 0%, #0a1018 100%)",
        borderRight: "1px solid #1a2a3a", display: "flex", flexDirection: "column", overflowY: "auto",
      }}>
        {/* Header */}
        <div style={{
          padding: "20px 20px 16px", borderBottom: "1px solid #1a2a3a",
          background: "linear-gradient(135deg, #0f1a28 0%, #0d1520 100%)",
        }}>
          <div style={{ display: "flex", alignItems: "center", gap: 10, marginBottom: 6 }}>
            <div style={{
              width: 10, height: 10, borderRadius: "50%", background: "#2dd4bf",
              boxShadow: "0 0 8px #2dd4bf88", animation: "pulse 2s infinite",
            }} />
            <span style={{ fontSize: 11, color: "#2dd4bf", letterSpacing: 3, textTransform: "uppercase" }}>
              Kinematic Visualizer
            </span>
          </div>
          <h1 style={{ fontSize: 18, fontWeight: 700, color: "#e8f0f8", margin: "4px 0 0", letterSpacing: -0.5 }}>
            Robot Arm FK
          </h1>
          <p style={{ fontSize: 10, color: "#5a7a96", margin: "4px 0 0" }}>
            Forward Kinematics • DH Parameters
          </p>
        </div>

        {/* DOF Selector */}
        <div style={{ padding: "14px 20px", borderBottom: "1px solid #1a2a3a" }}>
          <div style={{ display: "flex", alignItems: "center", justifyContent: "space-between", marginBottom: 10 }}>
            <span style={{ fontSize: 10, color: "#5a7a96", letterSpacing: 2, textTransform: "uppercase" }}>
              Degrees of Freedom
            </span>
            <span style={{ fontSize: 14, fontWeight: 700, color: "#38bdf8" }}>{dofCount}</span>
          </div>
          <input type="range" min={2} max={5} step={1} value={dofCount}
            onChange={e => { setDofCount(+e.target.value); setTrace([]); }}
            style={{ width: "100%", accentColor: "#38bdf8" }}
          />
        </div>

        {/* Joint Sliders */}
        <div style={{ padding: "10px 20px", flex: 1, overflowY: "auto" }}>
          <span style={{ fontSize: 10, color: "#5a7a96", letterSpacing: 2, textTransform: "uppercase", display: "block", marginBottom: 12 }}>
            Joint Angles
          </span>
          {activeJoints.map((j, i) => (
            <div key={i} style={{ marginBottom: 16 }}>
              <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: 4 }}>
                <div style={{ display: "flex", alignItems: "center", gap: 6 }}>
                  <div style={{ width: 8, height: 8, borderRadius: 2, background: j.color }} />
                  <span style={{ fontSize: 11, color: "#a0b4c8" }}>J{i + 1} · {j.name}</span>
                </div>
                <span style={{
                  fontSize: 12, fontWeight: 700, color: j.color,
                  background: j.color + "15", padding: "2px 8px", borderRadius: 4,
                }}>
                  {j.theta.toFixed(1)}°
                </span>
              </div>
              <input type="range" min={j.min} max={j.max} step={0.5} value={j.theta}
                onChange={e => updateJoint(i, +e.target.value)}
                style={{ width: "100%", accentColor: j.color }}
              />
              <div style={{ display: "flex", justifyContent: "space-between", fontSize: 9, color: "#3a5a76", marginTop: 2 }}>
                <span>{j.min}°</span><span>{j.max}°</span>
              </div>
            </div>
          ))}

          {/* Gripper slider */}
          <div style={{ marginBottom: 16, marginTop: 8, paddingTop: 12, borderTop: "1px solid #1a2a3a" }}>
            <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: 4 }}>
              <div style={{ display: "flex", alignItems: "center", gap: 6 }}>
                <div style={{ width: 8, height: 8, borderRadius: 2, background: "#38bdf8" }} />
                <span style={{ fontSize: 11, color: "#a0b4c8" }}>Gripper</span>
              </div>
              <span style={{
                fontSize: 12, fontWeight: 700, color: "#38bdf8",
                background: "#38bdf815", padding: "2px 8px", borderRadius: 4,
              }}>
                {gripperOpen.toFixed(0)} mm
              </span>
            </div>
            <input type="range" min={2} max={40} step={1} value={gripperOpen}
              onChange={e => setGripperOpen(+e.target.value)}
              style={{ width: "100%", accentColor: "#38bdf8" }}
            />
            <div style={{ display: "flex", justifyContent: "space-between", fontSize: 9, color: "#3a5a76", marginTop: 2 }}>
              <span>Closed</span><span>Open</span>
            </div>
          </div>
        </div>

        {/* DH Parameters Table */}
        {showInfo && (
          <div style={{ padding: "12px 20px", borderTop: "1px solid #1a2a3a" }}>
            <span style={{ fontSize: 10, color: "#5a7a96", letterSpacing: 2, textTransform: "uppercase", display: "block", marginBottom: 8 }}>
              DH Parameters
            </span>
            <table style={{ width: "100%", fontSize: 10, borderCollapse: "collapse" }}>
              <thead>
                <tr style={{ color: "#5a7a96" }}>
                  <th style={{ textAlign: "left", padding: "3px 4px" }}>Joint</th>
                  <th style={{ textAlign: "right", padding: "3px 4px" }}>θ</th>
                  <th style={{ textAlign: "right", padding: "3px 4px" }}>d</th>
                  <th style={{ textAlign: "right", padding: "3px 4px" }}>a</th>
                  <th style={{ textAlign: "right", padding: "3px 4px" }}>α</th>
                </tr>
              </thead>
              <tbody>
                {activeJoints.map((j, i) => (
                  <tr key={i} style={{ color: j.color + "cc" }}>
                    <td style={{ padding: "2px 4px" }}>J{i + 1}</td>
                    <td style={{ textAlign: "right", padding: "2px 4px" }}>{j.theta.toFixed(1)}°</td>
                    <td style={{ textAlign: "right", padding: "2px 4px" }}>{j.d}</td>
                    <td style={{ textAlign: "right", padding: "2px 4px" }}>{j.a}</td>
                    <td style={{ textAlign: "right", padding: "2px 4px" }}>{j.alpha}°</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        )}

        {/* Buttons */}
        <div style={{ padding: "14px 20px", borderTop: "1px solid #1a2a3a", display: "flex", gap: 8, flexWrap: "wrap" }}>
          <button onClick={() => setAnimating(a => !a)} style={{
            flex: 1, padding: "8px 0", borderRadius: 6, border: "1px solid #2dd4bf44",
            background: animating ? "#2dd4bf22" : "transparent", color: "#2dd4bf",
            fontSize: 11, cursor: "pointer", fontFamily: "inherit", letterSpacing: 1,
          }}>
            {animating ? "■ STOP" : "▶ ANIMATE"}
          </button>
          <button onClick={resetArm} style={{
            flex: 1, padding: "8px 0", borderRadius: 6, border: "1px solid #ff4d4d44",
            background: "transparent", color: "#ff4d4d",
            fontSize: 11, cursor: "pointer", fontFamily: "inherit", letterSpacing: 1,
          }}>
            ↺ RESET
          </button>
          <button onClick={() => setShowTrace(t => !t)} style={{
            flex: 1, padding: "8px 0", borderRadius: 6, border: "1px solid #38bdf844",
            background: showTrace ? "#38bdf822" : "transparent", color: "#38bdf8",
            fontSize: 11, cursor: "pointer", fontFamily: "inherit", letterSpacing: 1,
          }}>
            {showTrace ? "◉ TRACE" : "○ TRACE"}
          </button>
          <button onClick={() => setTrace([])} style={{
            flex: 1, padding: "8px 0", borderRadius: 6, border: "1px solid #818cf844",
            background: "transparent", color: "#818cf8",
            fontSize: 11, cursor: "pointer", fontFamily: "inherit", letterSpacing: 1,
          }}>
            ✕ CLEAR
          </button>
          <button onClick={() => setShowInfo(s => !s)} style={{
            flex: "1 1 100%", padding: "8px 0", borderRadius: 6, border: "1px solid #5a7a9644",
            background: showInfo ? "#5a7a9622" : "transparent", color: "#5a7a96",
            fontSize: 11, cursor: "pointer", fontFamily: "inherit", letterSpacing: 1,
          }}>
            {showInfo ? "▾ HIDE DH TABLE" : "▸ SHOW DH TABLE"}
          </button>
        </div>
      </div>

      {/* Canvas Area */}
      <div style={{ flex: 1, position: "relative" }}
        onMouseDown={handleMouseDown} onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp} onMouseLeave={handleMouseUp}
      >
        <canvas ref={canvasRef} width={900} height={700}
          onWheel={handleWheel}
          style={{ width: "100%", height: "100%", cursor: isDragging ? "grabbing" : "grab" }}
        />

        {/* End-effector HUD */}
        <div style={{
          position: "absolute", top: 20, right: 20,
          background: "rgba(10, 16, 24, 0.85)", backdropFilter: "blur(8px)",
          border: "1px solid #1a2a3a", borderRadius: 10, padding: "14px 18px",
          minWidth: 200,
        }}>
          <span style={{ fontSize: 10, color: "#38bdf8", letterSpacing: 2, textTransform: "uppercase", display: "block", marginBottom: 8 }}>
            End-Effector Position
          </span>
          <div style={{ display: "grid", gridTemplateColumns: "auto 1fr", gap: "4px 12px", fontSize: 13 }}>
            <span style={{ color: "#ef4444", fontWeight: 700 }}>X</span>
            <span style={{ textAlign: "right", fontWeight: 600 }}>{endEffector[0].toFixed(2)} mm</span>
            <span style={{ color: "#22c55e", fontWeight: 700 }}>Y</span>
            <span style={{ textAlign: "right", fontWeight: 600 }}>{endEffector[1].toFixed(2)} mm</span>
            <span style={{ color: "#3b82f6", fontWeight: 700 }}>Z</span>
            <span style={{ textAlign: "right", fontWeight: 600 }}>{endEffector[2].toFixed(2)} mm</span>
          </div>
          <div style={{ marginTop: 10, paddingTop: 8, borderTop: "1px solid #1a2a3a", fontSize: 11 }}>
            <span style={{ color: "#5a7a96" }}>Reach: </span>
            <span style={{ fontWeight: 700, color: "#ff9f1a" }}>
              {Math.sqrt(endEffector[0]**2 + endEffector[1]**2 + endEffector[2]**2).toFixed(1)} mm
            </span>
          </div>
        </div>

        {/* View controls hint */}
        <div style={{
          position: "absolute", bottom: 20, left: 20,
          fontSize: 10, color: "#3a5a76", letterSpacing: 1,
        }}>
          DRAG to rotate • SCROLL to zoom
        </div>

        {/* Scale indicator */}
        <div style={{
          position: "absolute", bottom: 20, right: 20,
          fontSize: 10, color: "#3a5a76", letterSpacing: 1,
        }}>
          Scale: {(scale * 100).toFixed(0)}%
        </div>

        <style>{`
          @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.4; }
          }

          input[type="range"] {
            -webkit-appearance: none;
            height: 4px;
            border-radius: 2px;
            background: #1a2a3a;
            outline: none;
          }

          input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 14px;
            height: 14px;
            border-radius: 50%;
            cursor: pointer;
            border: 2px solid currentColor;
            background: #0d1520;
          }

          ::-webkit-scrollbar {
            width: 4px;
          }

          ::-webkit-scrollbar-track {
            background: transparent;
          }

          ::-webkit-scrollbar-thumb {
            background: #1a2a3a;
            border-radius: 2px;
          }
        `}</style>
      </div>
    </div>
  );
}