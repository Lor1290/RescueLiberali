import React, {
  useState,
  useEffect,
  useRef,
  useCallback,
  useMemo,
  type ReactNode,
} from "react";

// ─────────────────────────────────────────────────────────────
//  TYPE DEFINITIONS
// ─────────────────────────────────────────────────────────────

interface GPSData {
  x: number;
  y: number;
  z: number;
}

interface GyroData {
  x: number;
  y: number;
  z: number;
}

interface AccelerometerData {
  x: number;
  y: number;
  z: number;
}

interface LidarPoint {
  x: number;
  y: number;
  z: number;
  layer: number;
}

interface GlobalMapPoint {
  x: number;
  z: number;
  layer: number;
}

interface LidarData {
  range_image: number[];
  horizontal_resolution: number;
  num_layers: number;
  point_cloud: LidarPoint[];
}

interface DistanceSensors {
  ds0: number;
  ds1: number;
  ds2: number;
  ds3: number;
  ds4: number;
  ds5: number;
  ds6: number;
  ds7: number;
}

interface ColorSensor {
  r: number;
  g: number;
  b: number;
}

interface CameraData {
  left?: string;
  center?: string;
  right?: string;
}

interface GameData {
  score: number;
  time_elapsed: number;
  time_remaining: number;
  victims_found: number;
}

interface SensorFrame {
  gps?: GPSData;
  gyro?: GyroData;
  accelerometer?: AccelerometerData;
  lidar?: LidarData;
  distance_sensors?: DistanceSensors;
  color_sensor?: ColorSensor;
  camera?: CameraData;
  game?: GameData;
  robot_config?: Record<string, unknown>;
}

// ─────────────────────────────────────────────────────────────
//  CONSTANTS
// ─────────────────────────────────────────────────────────────

const WS_URL = "ws://localhost:8765";
const MAX_TRAIL = 200;
const MAX_IMU_SAMPLES = 100;
const CLOUD_GRID_SIZE = 0.02; // 2 cm grid for dedup
const RECONNECT_INTERVAL = 2000;

const ACCENT = "#38bdf8";
const CARD_BG = "#1e293b";
const BORDER_COLOR = "#334155";

// ─────────────────────────────────────────────────────────────
//  CIRCULAR BUFFER UTILITY
// ─────────────────────────────────────────────────────────────

class CircularBuffer<T> {
  private buf: T[];
  private head = 0;
  private count = 0;
  private cap: number;
  constructor(cap: number) {
    this.cap = cap;
    this.buf = new Array(cap);
  }
  push(item: T) {
    this.buf[this.head] = item;
    this.head = (this.head + 1) % this.cap;
    if (this.count < this.cap) this.count++;
  }
  toArray(): T[] {
    if (this.count < this.cap) return this.buf.slice(0, this.count);
    return [...this.buf.slice(this.head), ...this.buf.slice(0, this.head)];
  }
  get length() {
    return this.count;
  }
  clear() {
    this.head = 0;
    this.count = 0;
  }
}

// ─────────────────────────────────────────────────────────────
//  useWebSocket HOOK
// ─────────────────────────────────────────────────────────────

type WSStatus = "connected" | "disconnected" | "reconnecting";

function useWebSocket(url: string) {
  const wsRef = useRef<WebSocket | null>(null);
  const [status, setStatus] = useState<WSStatus>("disconnected");
  const [lastFrame, setLastFrame] = useState<SensorFrame | null>(null);
  const reconnectTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const connectRef = useRef<() => void>(() => {});

  // FPS tracking
  const frameCountRef = useRef(0);
  const [fps, setFps] = useState(0);

  const connect = useCallback(() => {
    if (
      wsRef.current &&
      (wsRef.current.readyState === WebSocket.OPEN ||
        wsRef.current.readyState === WebSocket.CONNECTING)
    )
      return;

    setStatus("reconnecting");
    const ws = new WebSocket(url);

    ws.onopen = () => {
      setStatus("connected");
      if (reconnectTimer.current) {
        clearTimeout(reconnectTimer.current);
        reconnectTimer.current = null;
      }
    };

    ws.onmessage = (ev) => {
      try {
        const data = JSON.parse(ev.data) as SensorFrame;
        setLastFrame(data);
        frameCountRef.current++;
      } catch {
        // ignore malformed frames
      }
    };

    ws.onclose = () => {
      setStatus("disconnected");
      wsRef.current = null;
      reconnectTimer.current = setTimeout(() => {
        connectRef.current();
      }, RECONNECT_INTERVAL);
    };

    ws.onerror = () => {
      ws.close();
    };

    wsRef.current = ws;
  }, [url]);

  // Keep connectRef in sync
  useEffect(() => {
    connectRef.current = connect;
  }, [connect]);

  // FPS counter interval
  useEffect(() => {
    const id = setInterval(() => {
      setFps(frameCountRef.current);
      frameCountRef.current = 0;
    }, 1000);
    return () => clearInterval(id);
  }, []);

  useEffect(() => {
    connect();
    return () => {
      if (reconnectTimer.current) clearTimeout(reconnectTimer.current);
      wsRef.current?.close();
    };
  }, [connect]);

  const sendMessage = useCallback((cmd: object) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(cmd));
    }
  }, []);

  return { status, lastFrame, fps, sendMessage };
}

// ─────────────────────────────────────────────────────────────
//  HELPER: setupCanvas (HiDPI-aware)
// ─────────────────────────────────────────────────────────────

function setupCanvas(
  canvas: HTMLCanvasElement,
  width: number,
  height: number
): CanvasRenderingContext2D | null {
  const dpr = window.devicePixelRatio || 1;
  canvas.width = width * dpr;
  canvas.height = height * dpr;
  canvas.style.width = `${width}px`;
  canvas.style.height = `${height}px`;
  const ctx = canvas.getContext("2d");
  if (ctx) ctx.scale(dpr, dpr);
  return ctx;
}

// ─────────────────────────────────────────────────────────────
//  ERROR BOUNDARY
// ─────────────────────────────────────────────────────────────

class PanelErrorBoundary extends React.Component<
  { children: ReactNode; name: string },
  { hasError: boolean; error?: Error }
> {
  constructor(props: { children: ReactNode; name: string }) {
    super(props);
    this.state = { hasError: false };
  }
  static getDerivedStateFromError(error: Error) {
    return { hasError: true, error };
  }
  render() {
    if (this.state.hasError) {
      return (
        <div
          className="rounded-lg p-4"
          style={{ background: CARD_BG, border: `1px solid ${BORDER_COLOR}` }}
        >
          <h3 className="text-red-400 font-semibold mb-1">
            {this.props.name} — Error
          </h3>
          <p className="text-sm text-gray-400">
            {this.state.error?.message ?? "Unknown error"}
          </p>
        </div>
      );
    }
    return this.props.children;
  }
}

// ─────────────────────────────────────────────────────────────
//  PANEL WRAPPER COMPONENT
// ─────────────────────────────────────────────────────────────

function Panel({
  title,
  icon,
  children,
  className = "",
  collapsible = false,
}: {
  title: string;
  icon: string;
  children: ReactNode;
  className?: string;
  collapsible?: boolean;
}) {
  const [collapsed, setCollapsed] = useState(false);
  return (
    <div
      className={`rounded-lg overflow-hidden ${className}`}
      style={{ background: CARD_BG, border: `1px solid ${BORDER_COLOR}` }}
    >
      <div
        className="flex items-center justify-between px-4 py-2 border-b"
        style={{ borderColor: BORDER_COLOR }}
      >
        <div className="flex items-center gap-2">
          <span className="text-lg">{icon}</span>
          <h2 className="text-sm font-semibold tracking-wide uppercase" style={{ color: ACCENT }}>
            {title}
          </h2>
        </div>
        {collapsible && (
          <button
            onClick={() => setCollapsed(!collapsed)}
            className="text-xs text-gray-400 hover:text-white transition-colors px-2 py-1"
          >
            {collapsed ? "▼ Show" : "▲ Hide"}
          </button>
        )}
      </div>
      {!collapsed && <div className="p-4">{children}</div>}
    </div>
  );
}

// ─────────────────────────────────────────────────────────────
//  1. CONNECTION STATUS BAR
// ─────────────────────────────────────────────────────────────

function ConnectionStatusBar({
  status,
  fps,
  robotConfig,
}: {
  status: WSStatus;
  fps: number;
  robotConfig?: Record<string, unknown>;
}) {
  const statusColor =
    status === "connected"
      ? "#22c55e"
      : status === "reconnecting"
        ? "#eab308"
        : "#ef4444";
  const robotName =
    (robotConfig?.name as string) ??
    (robotConfig?.robotName as string) ??
    "Unknown Robot";

  return (
    <div
      className="flex items-center justify-between px-6 py-3 rounded-lg mb-4"
      style={{ background: CARD_BG, border: `1px solid ${BORDER_COLOR}` }}
    >
      <div className="flex items-center gap-3">
        <div
          className="w-3 h-3 rounded-full"
          style={{ background: statusColor, boxShadow: `0 0 8px ${statusColor}` }}
        />
        <span className="text-sm font-medium capitalize">{status}</span>
        <span className="text-xs text-gray-500">|</span>
        <span className="text-sm text-gray-300">🤖 {robotName}</span>
      </div>
      <div className="flex items-center gap-6">
        <span className="text-sm text-gray-400">
          FPS: <span className="font-mono text-white">{fps}</span>
        </span>
        <span className="text-sm text-gray-400">
          {new Date().toLocaleTimeString()}
        </span>
      </div>
    </div>
  );
}

// ─────────────────────────────────────────────────────────────
//  2. GPS PANEL
// ─────────────────────────────────────────────────────────────

function GPSPanel({ gps, trail }: { gps?: GPSData; trail: GPSData[] }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const animRef = useRef<number>(0);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const W = 280,
      H = 220;
    const ctx = setupCanvas(canvas, W, H);
    if (!ctx) return;

    const draw = () => {
      ctx.clearRect(0, 0, W, H);
      ctx.fillStyle = "#0f172a";
      ctx.fillRect(0, 0, W, H);

      if (trail.length === 0) {
        ctx.fillStyle = "#64748b";
        ctx.font = "12px sans-serif";
        ctx.textAlign = "center";
        ctx.fillText("Waiting for GPS data…", W / 2, H / 2);
        animRef.current = requestAnimationFrame(draw);
        return;
      }

      // Compute bounds with padding
      let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
      for (const p of trail) {
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
        if (p.z < minZ) minZ = p.z;
        if (p.z > maxZ) maxZ = p.z;
      }
      const rangeX = Math.max(maxX - minX, 0.5);
      const rangeZ = Math.max(maxZ - minZ, 0.5);
      const pad = 20;
      const scale = Math.min((W - 2 * pad) / rangeX, (H - 2 * pad) / rangeZ);
      const cx = W / 2, cy = H / 2;
      const midX = (minX + maxX) / 2, midZ = (minZ + maxZ) / 2;

      const toScreen = (x: number, z: number): [number, number] => [
        cx + (x - midX) * scale,
        cy + (z - midZ) * scale,
      ];

      // Grid lines
      ctx.strokeStyle = "#1e293b";
      ctx.lineWidth = 1;
      for (let i = 0; i <= 4; i++) {
        const y2 = pad + ((H - 2 * pad) * i) / 4;
        ctx.beginPath();
        ctx.moveTo(0, y2);
        ctx.lineTo(W, y2);
        ctx.stroke();
      }

      // Trail
      ctx.beginPath();
      ctx.strokeStyle = ACCENT + "60";
      ctx.lineWidth = 1.5;
      for (let i = 0; i < trail.length; i++) {
        const [sx, sz] = toScreen(trail[i].x, trail[i].z);
        if (i === 0) ctx.moveTo(sx, sz);
        else ctx.lineTo(sx, sz);
      }
      ctx.stroke();

      // Trail dots with fading opacity
      for (let i = 0; i < trail.length; i++) {
        const alpha = 0.15 + (0.85 * i) / trail.length;
        const [sx, sz] = toScreen(trail[i].x, trail[i].z);
        ctx.fillStyle = `rgba(56,189,248,${alpha})`;
        ctx.beginPath();
        ctx.arc(sx, sz, 1.5, 0, Math.PI * 2);
        ctx.fill();
      }

      // Current position
      if (gps) {
        const [sx, sz] = toScreen(gps.x, gps.z);
        ctx.fillStyle = "#fff";
        ctx.beginPath();
        ctx.arc(sx, sz, 5, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = ACCENT;
        ctx.beginPath();
        ctx.arc(sx, sz, 3, 0, Math.PI * 2);
        ctx.fill();
      }

      animRef.current = requestAnimationFrame(draw);
    };

    draw();
    return () => cancelAnimationFrame(animRef.current);
  }, [gps, trail]);

  return (
    <Panel title="GPS" icon="📍">
      <div className="flex flex-col gap-3">
        <div className="grid grid-cols-3 gap-2 text-center">
          {(["x", "y", "z"] as const).map((axis) => (
            <div key={axis}>
              <div className="text-xs text-gray-500 uppercase">{axis}</div>
              <div className="font-mono text-sm">
                {gps ? gps[axis].toFixed(4) : "—"}
              </div>
            </div>
          ))}
        </div>
        <canvas ref={canvasRef} className="w-full rounded" style={{ maxWidth: 280 }} />
      </div>
    </Panel>
  );
}

// ─────────────────────────────────────────────────────────────
//  3. LIDAR PANEL
// ─────────────────────────────────────────────────────────────

function LidarPanel({
  lidar,
  pointCloud,
}: {
  lidar?: LidarData;
  pointCloud: GlobalMapPoint[];
}) {
  const radarRef = useRef<HTMLCanvasElement>(null);
  const cloudRef = useRef<HTMLCanvasElement>(null);
  const radarAnimRef = useRef<number>(0);
  const cloudAnimRef = useRef<number>(0);
  const [showRange, setShowRange] = useState(true);
  const [showCloud, setShowCloud] = useState(true);

  // Radar visualization (polar)
  useEffect(() => {
    if (!showRange) return;
    const canvas = radarRef.current;
    if (!canvas) return;
    const S = 420;
    const ctx = setupCanvas(canvas, S, S);
    if (!ctx) return;

    const draw = () => {
      const cx = S / 2, cy = S / 2;
      const maxR = S / 2 - 30;
      ctx.clearRect(0, 0, S, S);
      ctx.fillStyle = "#0c1222";
      ctx.fillRect(0, 0, S, S);

      // Reference rings
      for (let i = 1; i <= 4; i++) {
        const r = (maxR * i) / 4;
        ctx.strokeStyle = "#1e293b";
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.arc(cx, cy, r, 0, Math.PI * 2);
        ctx.stroke();
        // Label
        ctx.fillStyle = "#475569";
        ctx.font = "10px sans-serif";
        ctx.textAlign = "left";
        ctx.fillText(`${((i / 4) * 1.0).toFixed(1)}m`, cx + r + 2, cy - 2);
      }

      // Cross hair lines
      ctx.strokeStyle = "#1e293b";
      ctx.beginPath();
      ctx.moveTo(cx, 10);
      ctx.lineTo(cx, S - 10);
      ctx.moveTo(10, cy);
      ctx.lineTo(S - 10, cy);
      ctx.stroke();

      // Cardinal labels
      ctx.fillStyle = "#64748b";
      ctx.font = "11px sans-serif";
      ctx.textAlign = "center";
      ctx.fillText("Front", cx, 22);
      ctx.fillText("Back", cx, S - 10);
      ctx.textAlign = "left";
      ctx.fillText("Right", S - 28, cy - 5);
      ctx.textAlign = "right";
      ctx.fillText("Left", 30, cy - 5);

      if (lidar && lidar.range_image.length > 0) {
        const n = lidar.range_image.length;
        const maxDist = 1.0; // reference max

        // Draw range rays
        for (let i = 0; i < n; i++) {
          const angle = (i / n) * Math.PI * 2 - Math.PI / 2; // 0=front (top)
          const dist = Math.min(lidar.range_image[i], maxDist);
          const r = (dist / maxDist) * maxR;

          // Color by distance
          let color: string;
          const ratio = dist / maxDist;
          if (ratio < 0.3) color = "#ef4444"; // red close
          else if (ratio < 0.6) color = "#eab308"; // yellow mid
          else color = "#22c55e"; // green far

          ctx.strokeStyle = color + "b0";
          ctx.lineWidth = 1.5;
          ctx.beginPath();
          ctx.moveTo(cx, cy);
          ctx.lineTo(cx + Math.cos(angle) * r, cy + Math.sin(angle) * r);
          ctx.stroke();

          // Endpoint dot
          ctx.fillStyle = color;
          ctx.beginPath();
          ctx.arc(
            cx + Math.cos(angle) * r,
            cy + Math.sin(angle) * r,
            2,
            0,
            Math.PI * 2
          );
          ctx.fill();
        }
      }

      // Robot triangle at center
      ctx.fillStyle = ACCENT;
      ctx.beginPath();
      ctx.moveTo(cx, cy - 8);
      ctx.lineTo(cx - 5, cy + 5);
      ctx.lineTo(cx + 5, cy + 5);
      ctx.closePath();
      ctx.fill();

      radarAnimRef.current = requestAnimationFrame(draw);
    };

    draw();
    return () => cancelAnimationFrame(radarAnimRef.current);
  }, [lidar, showRange]);

  // Point cloud 2D scatter (X-Z plane)
  useEffect(() => {
    if (!showCloud) return;
    const canvas = cloudRef.current;
    if (!canvas) return;
    const W = 420, H = 300;
    const ctx = setupCanvas(canvas, W, H);
    if (!ctx) return;

    const draw = () => {
      ctx.clearRect(0, 0, W, H);
      ctx.fillStyle = "#0c1222";
      ctx.fillRect(0, 0, W, H);

      if (pointCloud.length === 0) {
        ctx.fillStyle = "#64748b";
        ctx.font = "12px sans-serif";
        ctx.textAlign = "center";
        ctx.fillText("No global point cloud data", W / 2, H / 2);
        cloudAnimRef.current = requestAnimationFrame(draw);
        return;
      }

      const points = pointCloud;

      // Find bounds
      let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
      for (const p of points) {
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
        if (p.z < minZ) minZ = p.z;
        if (p.z > maxZ) maxZ = p.z;
      }
      const rangeX = Math.max(maxX - minX, 0.1);
      const rangeZ = Math.max(maxZ - minZ, 0.1);
      const pad = 20;
      const scale = Math.min((W - 2 * pad) / rangeX, (H - 2 * pad) / rangeZ);
      const centerX = W / 2, centerZ = H / 2;
      const midX = (minX + maxX) / 2, midZ = (minZ + maxZ) / 2;

      const layerColors = ["#38bdf8", "#a78bfa", "#34d399", "#fb923c", "#f87171"];

      for (const p of points) {
        const sx = centerX + (p.x - midX) * scale;
        const sz = centerZ + (p.z - midZ) * scale;
        ctx.fillStyle = layerColors[p.layer % layerColors.length];
        ctx.beginPath();
        ctx.arc(sx, sz, 2, 0, Math.PI * 2);
        ctx.fill();
      }

      // World origin marker
      const [ox, oz] = [centerX + (0 - midX) * scale, centerZ + (0 - midZ) * scale];
      ctx.fillStyle = "#ffffffaa";
      ctx.beginPath();
      ctx.arc(ox, oz, 3, 0, Math.PI * 2);
      ctx.fill();

      // Title overlay
      ctx.fillStyle = "#94a3b8";
      ctx.font = "11px sans-serif";
      ctx.textAlign = "left";
      ctx.fillText("Global Cloud (X-Z world frame)", 8, 16);

      cloudAnimRef.current = requestAnimationFrame(draw);
    };

    draw();
    return () => cancelAnimationFrame(cloudAnimRef.current);
  }, [pointCloud, showCloud]);

  return (
    <Panel title="LIDAR" icon="🔴" className="col-span-1 md:col-span-2">
      <div className="flex flex-col gap-3">
        {/* Stats */}
        <div className="flex gap-4 text-xs text-gray-400 flex-wrap">
          <span>
            Resolution:{" "}
            <span className="text-white font-mono">
              {lidar?.horizontal_resolution ?? "—"}
            </span>
          </span>
          <span>
            Layers:{" "}
            <span className="text-white font-mono">
              {lidar?.num_layers ?? "—"}
            </span>
          </span>
          <span>
            Points:{" "}
            <span className="text-white font-mono">
              {pointCloud.length}
            </span>
          </span>
        </div>

        {/* Toggles */}
        <div className="flex gap-4">
          <label className="flex items-center gap-2 text-xs text-gray-300 cursor-pointer">
            <input
              type="checkbox"
              checked={showRange}
              onChange={(e) => setShowRange(e.target.checked)}
              className="accent-sky-400"
            />
            Range Radar
          </label>
          <label className="flex items-center gap-2 text-xs text-gray-300 cursor-pointer">
            <input
              type="checkbox"
              checked={showCloud}
              onChange={(e) => setShowCloud(e.target.checked)}
              className="accent-sky-400"
            />
            Global Point Cloud
          </label>
        </div>

        <div className="flex flex-col md:flex-row gap-4 items-start">
          {showRange && (
            <canvas
              ref={radarRef}
              className="rounded"
              style={{ width: 420, height: 420, maxWidth: "100%" }}
            />
          )}
          {showCloud && (
            <canvas
              ref={cloudRef}
              className="rounded"
              style={{ width: 420, height: 300, maxWidth: "100%" }}
            />
          )}
        </div>
      </div>
    </Panel>
  );
}

// ─────────────────────────────────────────────────────────────
//  4. DISTANCE SENSORS PANEL
// ─────────────────────────────────────────────────────────────

function DistanceSensorsPanel({ sensors }: { sensors?: DistanceSensors }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const animRef = useRef<number>(0);

  // Sensor positions around the robot (angles in radians, 0 = front)
  // ds0=front-left, ds1=left, ds2=back-left, ds3=back, ds4=back-right, ds5=right, ds6=front-right, ds7=front
  const sensorAngles = useMemo(
    () => [
      -Math.PI / 4,      // ds0: front-left
      -Math.PI / 2,      // ds1: left
      -(3 * Math.PI) / 4, // ds2: back-left
      Math.PI,            // ds3: back
      (3 * Math.PI) / 4,  // ds4: back-right
      Math.PI / 2,        // ds5: right
      Math.PI / 4,        // ds6: front-right
      0,                  // ds7: front
    ],
    []
  );

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const S = 260;
    const ctx = setupCanvas(canvas, S, S);
    if (!ctx) return;

    const draw = () => {
      const cx = S / 2,
        cy = S / 2;
      ctx.clearRect(0, 0, S, S);
      ctx.fillStyle = "#0c1222";
      ctx.fillRect(0, 0, S, S);

      // Robot body circle
      ctx.strokeStyle = "#475569";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(cx, cy, 25, 0, Math.PI * 2);
      ctx.stroke();
      // Robot direction arrow
      ctx.fillStyle = ACCENT;
      ctx.beginPath();
      ctx.moveTo(cx, cy - 20);
      ctx.lineTo(cx - 6, cy - 8);
      ctx.lineTo(cx + 6, cy - 8);
      ctx.closePath();
      ctx.fill();

      const keys = ["ds0", "ds1", "ds2", "ds3", "ds4", "ds5", "ds6", "ds7"] as const;
      const maxBar = 80;

      for (let i = 0; i < 8; i++) {
        const val = sensors ? sensors[keys[i]] : 0;
        const angle = sensorAngles[i] - Math.PI / 2; // rotate so 0 = up
        const barLen = (Math.min(val, 1.0) / 1.0) * maxBar;

        // Color by distance
        let color: string;
        if (val < 0.1) color = "#ef4444";
        else if (val < 0.5) color = "#eab308";
        else color = "#22c55e";

        const startR = 30;
        const x0 = cx + Math.cos(angle) * startR;
        const y0 = cy + Math.sin(angle) * startR;
        const x1 = cx + Math.cos(angle) * (startR + barLen);
        const y1 = cy + Math.sin(angle) * (startR + barLen);

        ctx.strokeStyle = color;
        ctx.lineWidth = 6;
        ctx.lineCap = "round";
        ctx.beginPath();
        ctx.moveTo(x0, y0);
        ctx.lineTo(x1, y1);
        ctx.stroke();

        // Value label
        const labelR = startR + maxBar + 15;
        const lx = cx + Math.cos(angle) * labelR;
        const ly = cy + Math.sin(angle) * labelR;
        ctx.fillStyle = "#94a3b8";
        ctx.font = "10px monospace";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(keys[i], lx, ly - 8);
        ctx.fillStyle = color;
        ctx.fillText(sensors ? val.toFixed(2) : "—", lx, ly + 5);
      }

      animRef.current = requestAnimationFrame(draw);
    };

    draw();
    return () => cancelAnimationFrame(animRef.current);
  }, [sensors, sensorAngles]);

  return (
    <Panel title="Distance Sensors" icon="📏">
      <canvas ref={canvasRef} className="rounded mx-auto" style={{ width: 260, height: 260 }} />
    </Panel>
  );
}

// ─────────────────────────────────────────────────────────────
//  5. IMU PANEL (Gyro + Accelerometer)
// ─────────────────────────────────────────────────────────────

function IMUPanel({
  gyro,
  accel,
  gyroHistory,
  accelHistory,
}: {
  gyro?: GyroData;
  accel?: AccelerometerData;
  gyroHistory: number[];
  accelHistory: number[];
}) {
  const gyroCanvasRef = useRef<HTMLCanvasElement>(null);
  const accelCanvasRef = useRef<HTMLCanvasElement>(null);
  const gyroAnimRef = useRef<number>(0);
  const accelAnimRef = useRef<number>(0);

  const drawChart = useCallback(
    (
      canvas: HTMLCanvasElement,
      data: number[],
      label: string,
      color: string,
      unit: string
    ) => {
      const W = 280,
        H = 100;
      const ctx = setupCanvas(canvas, W, H);
      if (!ctx) return;

      ctx.clearRect(0, 0, W, H);
      ctx.fillStyle = "#0c1222";
      ctx.fillRect(0, 0, W, H);

      if (data.length < 2) return;

      // Find range
      let min = Infinity,
        max = -Infinity;
      for (const v of data) {
        if (v < min) min = v;
        if (v > max) max = v;
      }
      const range = Math.max(max - min, 0.01);
      const pad = 5;

      // Zero line
      const zeroY = H - pad - ((0 - min) / range) * (H - 2 * pad);
      ctx.strokeStyle = "#334155";
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(0, zeroY);
      ctx.lineTo(W, zeroY);
      ctx.stroke();
      ctx.setLineDash([]);

      // Data line
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i = 0; i < data.length; i++) {
        const x = (i / (MAX_IMU_SAMPLES - 1)) * W;
        const y = H - pad - ((data[i] - min) / range) * (H - 2 * pad);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();

      // Label
      ctx.fillStyle = color;
      ctx.font = "10px sans-serif";
      ctx.textAlign = "left";
      ctx.fillText(`${label} (${unit})`, 4, 12);

      // Current value
      const last = data[data.length - 1];
      ctx.textAlign = "right";
      ctx.fillText(last.toFixed(3), W - 4, 12);
    },
    []
  );

  useEffect(() => {
    const canvas = gyroCanvasRef.current;
    if (!canvas) return;
    const animate = () => {
      drawChart(canvas, gyroHistory, "Gyro Z", "#a78bfa", "rad/s");
      gyroAnimRef.current = requestAnimationFrame(animate);
    };
    animate();
    return () => cancelAnimationFrame(gyroAnimRef.current);
  }, [gyroHistory, drawChart]);

  useEffect(() => {
    const canvas = accelCanvasRef.current;
    if (!canvas) return;
    const animate = () => {
      drawChart(canvas, accelHistory, "Accel Y", "#fb923c", "m/s²");
      accelAnimRef.current = requestAnimationFrame(animate);
    };
    animate();
    return () => cancelAnimationFrame(accelAnimRef.current);
  }, [accelHistory, drawChart]);

  return (
    <Panel title="IMU" icon="🧭">
      <div className="flex flex-col gap-3">
        {/* Gyro Readout */}
        <div>
          <div className="text-xs text-gray-500 mb-1">Gyroscope (rad/s)</div>
          <div className="grid grid-cols-3 gap-2 text-center">
            {(["x", "y", "z"] as const).map((a) => (
              <div key={a}>
                <span className="text-xs text-gray-500 uppercase">{a}</span>
                <div className="font-mono text-sm">
                  {gyro ? gyro[a].toFixed(4) : "—"}
                </div>
              </div>
            ))}
          </div>
        </div>

        {/* Accel Readout */}
        <div>
          <div className="text-xs text-gray-500 mb-1">Accelerometer (m/s²)</div>
          <div className="grid grid-cols-3 gap-2 text-center">
            {(["x", "y", "z"] as const).map((a) => (
              <div key={a}>
                <span className="text-xs text-gray-500 uppercase">{a}</span>
                <div className="font-mono text-sm">
                  {accel ? accel[a].toFixed(4) : "—"}
                </div>
              </div>
            ))}
          </div>
        </div>

        {/* Charts */}
        <canvas ref={gyroCanvasRef} className="rounded w-full" style={{ maxWidth: 280 }} />
        <canvas ref={accelCanvasRef} className="rounded w-full" style={{ maxWidth: 280 }} />
      </div>
    </Panel>
  );
}

// ─────────────────────────────────────────────────────────────
//  6. COLOR SENSOR PANEL
// ─────────────────────────────────────────────────────────────

function classifyColor(r: number, g: number, b: number): string {
  const max = Math.max(r, g, b);
  const min = Math.min(r, g, b);
  if (max < 40) return "BLACK";
  if (min > 200) return "WHITE";
  if (r > 180 && g < 80 && b < 80) return "RED";
  if (g > 150 && r < 100 && b < 100) return "GREEN";
  if (b > 150 && r < 100 && g < 100) return "BLUE";
  if (r > 200 && g > 200 && b < 60) return "YELLOW";
  return "UNKNOWN";
}

function ColorSensorPanel({ color }: { color?: ColorSensor }) {
  const label = color ? classifyColor(color.r, color.g, color.b) : "—";
  const swatchColor = color ? `rgb(${color.r},${color.g},${color.b})` : "#333";

  return (
    <Panel title="Color Sensor" icon="🎨">
      <div className="flex items-center gap-4">
        <div
          className="w-16 h-16 rounded-lg border-2"
          style={{ background: swatchColor, borderColor: BORDER_COLOR }}
        />
        <div>
          <div className="text-lg font-bold" style={{ color: ACCENT }}>
            {label}
          </div>
          <div className="text-xs text-gray-400 font-mono mt-1">
            R:{color?.r ?? "—"} G:{color?.g ?? "—"} B:{color?.b ?? "—"}
          </div>
        </div>
      </div>
    </Panel>
  );
}

// ─────────────────────────────────────────────────────────────
//  7. CAMERA FEEDS PANEL
// ─────────────────────────────────────────────────────────────

function CameraPanel({ camera }: { camera?: CameraData }) {
  const feeds = useMemo(() => {
    if (!camera) return [];
    const result: { label: string; src: string }[] = [];
    if (camera.left)
      result.push({ label: "Left", src: `data:image/bmp;base64,${camera.left}` });
    if (camera.center)
      result.push({ label: "Center", src: `data:image/bmp;base64,${camera.center}` });
    if (camera.right)
      result.push({ label: "Right", src: `data:image/bmp;base64,${camera.right}` });
    return result;
  }, [camera]);

  return (
    <Panel title="Camera Feeds" icon="📷" collapsible className="col-span-1 md:col-span-3">
      {feeds.length === 0 ? (
        <div className="text-gray-500 text-sm">No camera data</div>
      ) : (
        <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
          {feeds.map((f) => (
            <div
              key={f.label}
              className="rounded border p-2"
              style={{ borderColor: BORDER_COLOR, background: "#0f172a" }}
            >
              <span className="text-xs text-gray-400 font-semibold uppercase block mb-2">
                {f.label} Camera
              </span>
              <img
                src={f.src}
                alt={f.label}
                className="rounded w-full object-contain"
                style={{ borderColor: BORDER_COLOR, height: 220 }}
              />
            </div>
          ))}
        </div>
      )}
    </Panel>
  );
}

// ─────────────────────────────────────────────────────────────
//  8. GAME STATE PANEL
// ─────────────────────────────────────────────────────────────

function formatTime(seconds: number): string {
  const m = Math.floor(seconds / 60);
  const s = Math.floor(seconds % 60);
  return `${m.toString().padStart(2, "0")}:${s.toString().padStart(2, "0")}`;
}

function GamePanel({ game }: { game?: GameData }) {
  let timeColor = "#22c55e";
  if (game) {
    if (game.time_remaining < 120) timeColor = "#ef4444";
    else if (game.time_remaining < 300) timeColor = "#eab308";
  }

  return (
    <Panel title="Game State" icon="🏆">
      <div className="grid grid-cols-2 gap-4">
        <div>
          <div className="text-xs text-gray-500">Score</div>
          <div className="text-2xl font-bold font-mono" style={{ color: ACCENT }}>
            {game?.score ?? "—"}
          </div>
        </div>
        <div>
          <div className="text-xs text-gray-500">Victims Found</div>
          <div className="text-2xl font-bold font-mono" style={{ color: ACCENT }}>
            {game?.victims_found ?? "—"}
          </div>
        </div>
        <div>
          <div className="text-xs text-gray-500">Elapsed</div>
          <div className="text-lg font-mono">
            {game ? formatTime(game.time_elapsed) : "—"}
          </div>
        </div>
        <div>
          <div className="text-xs text-gray-500">Remaining</div>
          <div className="text-lg font-mono font-bold" style={{ color: timeColor }}>
            {game ? formatTime(game.time_remaining) : "—"}
          </div>
        </div>
      </div>
    </Panel>
  );
}

// ─────────────────────────────────────────────────────────────
//  9. VARIABLE CONTROLS PANEL
// ─────────────────────────────────────────────────────────────

function VariableControlsPanel({
  sendMessage,
}: {
  sendMessage: (cmd: object) => void;
}) {
  const [maxSpeed, setMaxSpeed] = useState(6.28);
  const [debugMode, setDebugMode] = useState(false);
  const [autoNav, setAutoNav] = useState(true);
  const [customKey, setCustomKey] = useState("");
  const [customVal, setCustomVal] = useState("");
  const [lastAck, setLastAck] = useState<string | null>(null);

  const send = useCallback(
    (key: string, value: unknown) => {
      sendMessage({ cmd: "set_variable", key, value });
      setLastAck(`Set ${key} = ${JSON.stringify(value)}`);
      setTimeout(() => setLastAck(null), 2000);
    },
    [sendMessage]
  );

  const sendCmd = useCallback(
    (cmd: string) => {
      sendMessage({ cmd });
      setLastAck(`Sent: ${cmd}`);
      setTimeout(() => setLastAck(null), 2000);
    },
    [sendMessage]
  );

  const toggleBtn =
    "px-3 py-1.5 rounded text-xs font-semibold transition-colors cursor-pointer select-none";

  return (
    <Panel title="Controls" icon="🎮">
      <div className="flex flex-col gap-4">
        {/* LoP Trigger Button */}
        <button
          className="w-full py-2 rounded text-sm font-semibold bg-red-700 hover:bg-red-600 transition-colors text-white cursor-pointer"
          onClick={() => sendCmd("lop_trigger")}
        >
          Trigger LoP (Lack Of Progress)
        </button>

        {/* Auto Nav Controls */}
        <div className="flex items-center justify-between gap-2">
          <span className="text-sm text-gray-300">Autonomous Navigation</span>
          <button
            className={`${toggleBtn} ${autoNav ? "bg-green-600 text-white" : "bg-gray-700 text-gray-300"}`}
            onClick={() => {
              setAutoNav(false);
              send("auto_nav", false);
            }}
          >
            Stop Auto
          </button>
          <button
            className={`${toggleBtn} ${autoNav ? "bg-gray-700 text-gray-300" : "bg-sky-600 text-white"}`}
            onClick={() => {
              setAutoNav(true);
              send("auto_nav", true);
              sendCmd("restart_navigation");
            }}
          >
            Resume Auto
          </button>
        </div>

        {/* Max Speed Slider */}
        <div>
          <div className="flex items-center justify-between mb-1">
            <span className="text-sm text-gray-300">Max Speed</span>
            <span className="text-xs font-mono text-white">
              {maxSpeed.toFixed(2)} rad/s
            </span>
          </div>
          <input
            type="range"
            min={0}
            max={6.28}
            step={0.01}
            value={maxSpeed}
            onChange={(e) => {
              const v = parseFloat(e.target.value);
              setMaxSpeed(v);
              send("max_speed", v);
            }}
            className="w-full accent-sky-400"
          />
        </div>

        {/* Debug Mode Toggle */}
        <div className="flex items-center justify-between">
          <span className="text-sm text-gray-300">Debug Mode</span>
          <button
            className={`${toggleBtn} ${debugMode ? "bg-amber-600 text-white" : "bg-gray-700 text-gray-300"}`}
            onClick={() => {
              const v = !debugMode;
              setDebugMode(v);
              send("debug_mode", v);
            }}
          >
            {debugMode ? "ON" : "OFF"}
          </button>
        </div>

        {/* WASD Manual Controls */}
        <div>
          <div className="text-sm text-gray-300 mb-2">Manual Drive (WASD)</div>
          <div className="grid grid-cols-3 gap-2 w-48 mx-auto">
            <div />
            <button
              className="py-2 rounded bg-gray-700 hover:bg-sky-700 text-white font-semibold"
              onMouseDown={() => sendMessage({ cmd: "manual_drive", direction: "w" })}
              onMouseUp={() => sendCmd("manual_stop")}
              onTouchStart={() => sendMessage({ cmd: "manual_drive", direction: "w" })}
              onTouchEnd={() => sendCmd("manual_stop")}
            >
              W
            </button>
            <div />
            <button
              className="py-2 rounded bg-gray-700 hover:bg-sky-700 text-white font-semibold"
              onMouseDown={() => sendMessage({ cmd: "manual_drive", direction: "a" })}
              onMouseUp={() => sendCmd("manual_stop")}
              onTouchStart={() => sendMessage({ cmd: "manual_drive", direction: "a" })}
              onTouchEnd={() => sendCmd("manual_stop")}
            >
              A
            </button>
            <button
              className="py-2 rounded bg-gray-700 hover:bg-sky-700 text-white font-semibold"
              onMouseDown={() => sendMessage({ cmd: "manual_drive", direction: "s" })}
              onMouseUp={() => sendCmd("manual_stop")}
              onTouchStart={() => sendMessage({ cmd: "manual_drive", direction: "s" })}
              onTouchEnd={() => sendCmd("manual_stop")}
            >
              S
            </button>
            <button
              className="py-2 rounded bg-gray-700 hover:bg-sky-700 text-white font-semibold"
              onMouseDown={() => sendMessage({ cmd: "manual_drive", direction: "d" })}
              onMouseUp={() => sendCmd("manual_stop")}
              onTouchStart={() => sendMessage({ cmd: "manual_drive", direction: "d" })}
              onTouchEnd={() => sendCmd("manual_stop")}
            >
              D
            </button>
          </div>
        </div>

        {/* Manual Stop */}
        <div className="flex items-center justify-between">
          <span className="text-sm text-gray-300">Manual Stop</span>
          <button
            className={`${toggleBtn} bg-gray-700 text-gray-300 hover:bg-gray-600`}
            onClick={() => sendCmd("manual_stop")}
          >
            STOP
          </button>
        </div>

        {/* Restart Navigation Button */}
        <button
          className="w-full py-2 rounded text-sm font-semibold bg-sky-700 hover:bg-sky-600 transition-colors text-white cursor-pointer"
          onClick={() => sendCmd("restart_navigation")}
        >
          ⟳ Restart Navigation
        </button>

        {/* Custom Variable */}
        <div className="flex gap-2">
          <input
            type="text"
            placeholder="key"
            value={customKey}
            onChange={(e) => setCustomKey(e.target.value)}
            className="flex-1 px-2 py-1.5 rounded text-sm bg-gray-800 border text-white placeholder-gray-500 outline-none focus:border-sky-500"
            style={{ borderColor: BORDER_COLOR }}
          />
          <input
            type="text"
            placeholder="value"
            value={customVal}
            onChange={(e) => setCustomVal(e.target.value)}
            className="flex-1 px-2 py-1.5 rounded text-sm bg-gray-800 border text-white placeholder-gray-500 outline-none focus:border-sky-500"
            style={{ borderColor: BORDER_COLOR }}
          />
          <button
            className="px-3 py-1.5 rounded text-sm font-semibold bg-sky-700 hover:bg-sky-600 transition-colors text-white cursor-pointer"
            onClick={() => {
              if (!customKey.trim()) return;
              // Try to parse as JSON value (number, bool, string)
              let parsed: unknown = customVal;
              try {
                parsed = JSON.parse(customVal);
              } catch {
                // keep as string
              }
              send(customKey.trim(), parsed);
              setCustomKey("");
              setCustomVal("");
            }}
          >
            Set
          </button>
        </div>

        {/* Ack feedback */}
        {lastAck && (
          <div className="text-xs text-green-400 bg-green-900/30 rounded px-2 py-1">
            ✓ {lastAck}
          </div>
        )}
      </div>
    </Panel>
  );
}

// ─────────────────────────────────────────────────────────────
//  10. ROBOT CONFIG VIEWER
// ─────────────────────────────────────────────────────────────

function JsonTree({ data, depth = 0 }: { data: unknown; depth?: number }) {
  const [expanded, setExpanded] = useState(depth < 2);

  if (data === null || data === undefined)
    return <span className="text-gray-500">null</span>;
  if (typeof data === "boolean")
    return <span className="text-purple-400">{data.toString()}</span>;
  if (typeof data === "number")
    return <span className="text-amber-400">{data}</span>;
  if (typeof data === "string")
    return <span className="text-green-400">"{data}"</span>;

  if (Array.isArray(data)) {
    if (data.length === 0) return <span className="text-gray-500">[]</span>;
    return (
      <span>
        <button
          onClick={() => setExpanded(!expanded)}
          className="text-gray-400 hover:text-white text-xs cursor-pointer"
        >
          {expanded ? "▾" : "▸"} [{data.length}]
        </button>
        {expanded && (
          <div className="ml-4 border-l border-gray-700 pl-2">
            {data.map((item, i) => (
              <div key={i} className="text-xs">
                <span className="text-gray-600 mr-1">{i}:</span>
                <JsonTree data={item} depth={depth + 1} />
              </div>
            ))}
          </div>
        )}
      </span>
    );
  }

  if (typeof data === "object") {
    const entries = Object.entries(data as Record<string, unknown>);
    if (entries.length === 0) return <span className="text-gray-500">{"{}"}</span>;
    return (
      <span>
        <button
          onClick={() => setExpanded(!expanded)}
          className="text-gray-400 hover:text-white text-xs cursor-pointer"
        >
          {expanded ? "▾" : "▸"} {"{"}
          {entries.length}
          {"}"}
        </button>
        {expanded && (
          <div className="ml-4 border-l border-gray-700 pl-2">
            {entries.map(([key, val]) => (
              <div key={key} className="text-xs">
                <span className="text-sky-400">"{key}"</span>
                <span className="text-gray-500">: </span>
                <JsonTree data={val} depth={depth + 1} />
              </div>
            ))}
          </div>
        )}
      </span>
    );
  }

  return <span className="text-gray-400">{String(data)}</span>;
}

function RobotConfigPanel({
  config,
}: {
  config?: Record<string, unknown>;
}) {
  // Extract active sensors from config
  const activeSensors = useMemo(() => {
    if (!config) return [];
    const sensors: string[] = [];
    const walk = (obj: unknown, prefix = "") => {
      if (obj && typeof obj === "object" && !Array.isArray(obj)) {
        for (const [k, v] of Object.entries(obj as Record<string, unknown>)) {
          const sensorKeywords = [
            "lidar", "gps", "gyro", "accelerometer", "camera",
            "distance", "color", "sensor", "imu",
          ];
          if (sensorKeywords.some((s) => k.toLowerCase().includes(s))) {
            sensors.push(prefix ? `${prefix}.${k}` : k);
          }
          walk(v, prefix ? `${prefix}.${k}` : k);
        }
      }
    };
    walk(config);
    return sensors;
  }, [config]);

  return (
    <Panel title="Robot Config" icon="⚙️" collapsible>
      <div className="flex flex-col gap-3">
        {activeSensors.length > 0 && (
          <div>
            <div className="text-xs text-gray-500 mb-1">Detected Sensors</div>
            <div className="flex flex-wrap gap-1">
              {activeSensors.map((s) => (
                <span
                  key={s}
                  className="text-xs px-2 py-0.5 rounded-full"
                  style={{ background: "#0f172a", color: ACCENT }}
                >
                  {s}
                </span>
              ))}
            </div>
          </div>
        )}
        <div className="max-h-80 overflow-auto text-xs font-mono">
          {config ? (
            <JsonTree data={config} />
          ) : (
            <span className="text-gray-500">
              Waiting for robot_config…
            </span>
          )}
        </div>
      </div>
    </Panel>
  );
}

// ─────────────────────────────────────────────────────────────
//  MAIN DASHBOARD COMPONENT
// ─────────────────────────────────────────────────────────────

export default function Dashboard() {
  const { status, lastFrame, fps, sendMessage } = useWebSocket(WS_URL);

  // GPS trail buffer (persisted across frames)
  const gpsTrailRef = useRef(new CircularBuffer<GPSData>(MAX_TRAIL));
  const [gpsTrail, setGpsTrail] = useState<GPSData[]>([]);

  // IMU history buffers
  const gyroZHistRef = useRef(new CircularBuffer<number>(MAX_IMU_SAMPLES));
  const accelYHistRef = useRef(new CircularBuffer<number>(MAX_IMU_SAMPLES));
  const [gyroHistory, setGyroHistory] = useState<number[]>([]);
  const [accelHistory, setAccelHistory] = useState<number[]>([]);
  // Unbounded map: key = snapped grid coords, value = point. Points NEVER removed.
  const globalCloudRef = useRef<Map<string, GlobalMapPoint>>(new Map());
  const cloudFrameCountRef = useRef(0);
  const [globalCloud, setGlobalCloud] = useState<GlobalMapPoint[]>([]);

  // Robot config (sent once)
  const [robotConfig, setRobotConfig] = useState<Record<string, unknown> | undefined>();

  // Update rolling buffers when new frames arrive
  useEffect(() => {
    if (!lastFrame) return;

    if (lastFrame.gps) {
      gpsTrailRef.current.push(lastFrame.gps);
      setGpsTrail(gpsTrailRef.current.toArray());
    }

    if (lastFrame.gyro) {
      gyroZHistRef.current.push(lastFrame.gyro.z);
      setGyroHistory(gyroZHistRef.current.toArray());
    }

    if (lastFrame.accelerometer) {
      accelYHistRef.current.push(lastFrame.accelerometer.y);
      setAccelHistory(accelYHistRef.current.toArray());
    }

    // Build a world-frame cloud map — points are NEVER removed, only added.
    if (lastFrame.lidar?.point_cloud && lastFrame.gps && lastFrame.gyro) {
      const yaw = lastFrame.gyro.z;
      const cosY = Math.cos(yaw);
      const sinY = Math.sin(yaw);
      for (const p of lastFrame.lidar.point_cloud) {
        const gx = lastFrame.gps.x + (p.x * cosY - p.z * sinY);
        const gz = lastFrame.gps.z + (p.x * sinY + p.z * cosY);
        // Snap to grid to deduplicate nearby points
        const key = `${Math.round(gx / CLOUD_GRID_SIZE)},${Math.round(gz / CLOUD_GRID_SIZE)}`;
        globalCloudRef.current.set(key, { x: gx, z: gz, layer: p.layer });
      }
      cloudFrameCountRef.current++;
      if (cloudFrameCountRef.current % 15 === 0) {
        setGlobalCloud(Array.from(globalCloudRef.current.values()));
      }
    }

    if (lastFrame.robot_config && !robotConfig) {
      setRobotConfig(lastFrame.robot_config);
    }
  }, [lastFrame, robotConfig]);

  return (
    <div className="min-h-screen p-4 md:p-6" style={{ background: "#0f172a" }}>
      {/* Title */}
      <h1 className="text-xl font-bold mb-4 flex items-center gap-2">
        <span style={{ color: ACCENT }}>⬡</span>
        <span className="text-white">Erebus Debug Dashboard</span>
        <span className="text-xs text-gray-500 font-normal ml-2">RoboCup Junior Rescue Sim</span>
      </h1>

      {/* 1. Connection Status */}
      <PanelErrorBoundary name="Connection Status">
        <ConnectionStatusBar status={status} fps={fps} robotConfig={robotConfig} />
      </PanelErrorBoundary>

      {/* Main Grid */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        {/* 3. LIDAR — spans 2 columns */}
        <PanelErrorBoundary name="LIDAR">
          <LidarPanel lidar={lastFrame?.lidar} pointCloud={globalCloud} />
        </PanelErrorBoundary>

        {/* Right column first row */}
        <div className="flex flex-col gap-4">
          {/* 8. Game State */}
          <PanelErrorBoundary name="Game State">
            <GamePanel game={lastFrame?.game} />
          </PanelErrorBoundary>

          {/* 6. Color Sensor */}
          <PanelErrorBoundary name="Color Sensor">
            <ColorSensorPanel color={lastFrame?.color_sensor} />
          </PanelErrorBoundary>
        </div>

        {/* 2. GPS */}
        <PanelErrorBoundary name="GPS">
          <GPSPanel gps={lastFrame?.gps} trail={gpsTrail} />
        </PanelErrorBoundary>

        {/* 4. Distance Sensors */}
        <PanelErrorBoundary name="Distance Sensors">
          <DistanceSensorsPanel sensors={lastFrame?.distance_sensors} />
        </PanelErrorBoundary>

        {/* 5. IMU */}
        <PanelErrorBoundary name="IMU">
          <IMUPanel
            gyro={lastFrame?.gyro}
            accel={lastFrame?.accelerometer}
            gyroHistory={gyroHistory}
            accelHistory={accelHistory}
          />
        </PanelErrorBoundary>

        {/* 9. Controls */}
        <PanelErrorBoundary name="Controls">
          <VariableControlsPanel sendMessage={sendMessage} />
        </PanelErrorBoundary>

        {/* 10. Robot Config */}
        <PanelErrorBoundary name="Robot Config">
          <RobotConfigPanel config={robotConfig} />
        </PanelErrorBoundary>

        {/* Spacer for grid alignment */}
        <div />

        {/* 7. Camera Feeds — spans full width */}
        <PanelErrorBoundary name="Camera Feeds">
          <CameraPanel camera={lastFrame?.camera} />
        </PanelErrorBoundary>
      </div>
    </div>
  );
}
