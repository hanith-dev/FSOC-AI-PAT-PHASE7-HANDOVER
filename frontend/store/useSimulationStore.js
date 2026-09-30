import { create } from 'zustand';

const DEFAULT_SIM_STATE = {
  isPlaying: true,
  speed: 1.0,
  time: 0.0,
  dt: 0.016, // ~60fps step
  mode: 'linear',
  
  // Beacon properties
  beaconId: 'beacon-01',
  beaconPosition: [10.0, 5.0, -100.0],
  beaconVelocity: [-2.0, 0.2, 5.0], // Moves across x, slowly up/down y, and closer along z
  beaconSize: 1.5,
  beaconIntensity: 2.0,
  beaconColor: '#10B981', // glowing emerald beacon
  beaconActive: true,
  
  // Gimbal state
  gimbalPan: 0.0,  // degrees
  gimbalTilt: 0.0, // degrees
  panMin: -180.0,
  panMax: 180.0,
  tiltMin: -45.0,
  tiltMax: 45.0,
  maxPanSpeed: 30.0,  // deg/s
  maxTiltSpeed: 30.0, // deg/s
  
  // Camera state
  hfov: 10.0, // degrees
  vfov: 7.5,  // degrees
  resolutionWidth: 640,
  resolutionHeight: 480,
  cameraPosition: [0.0, 1.5, 0.0], // Positioned 1.5m high on the local terminal base
  
  // Tracking Telemetry
  targetInsideFOV: false,
  pixelErrorX: 0.0,
  pixelErrorY: 0.0,
  angularErrorTotal: 0.0,
  angularErrorH: 0.0,
  angularErrorV: 0.0,
  trackedPixelX: 320.0,
  trackedPixelY: 240.0,
  
  // WebSocket / Streaming metrics
  wsConnected: false,
  wsFps: 0,
  wsBytesReceived: 0,
  wsAcksCount: 0,
  wsLatency: 0,
  capturedFrame: null, // Last captured frame for UI thumbnail
};

export const useSimulationStore = create((set, get) => ({
  ...DEFAULT_SIM_STATE,
  
  togglePlay: () => set((state) => ({ isPlaying: !state.isPlaying })),
  
  setSpeed: (speed) => set({ speed }),
  
  resetSimulation: () => set({
    time: 0.0,
    beaconPosition: [10.0, 5.0, -100.0],
    beaconVelocity: [-2.0, 0.2, 5.0],
    gimbalPan: 0.0,
    gimbalTilt: 0.0,
    targetInsideFOV: false,
    pixelErrorX: 0.0,
    pixelErrorY: 0.0,
    angularErrorTotal: 0.0,
    angularErrorH: 0.0,
    angularErrorV: 0.0,
  }),
  
  setGimbalAngles: (pan, tilt) => {
    const { panMin, panMax, tiltMin, tiltMax } = get();
    // Clamp to mechanical limits
    const clampedPan = Math.max(panMin, Math.min(panMax, pan));
    const clampedTilt = Math.max(tiltMin, Math.min(tiltMax, tilt));
    set({ gimbalPan: clampedPan, gimbalTilt: clampedTilt });
  },
  
  manualGimbalMove: (deltaPan, deltaTilt) => {
    const { gimbalPan, gimbalTilt, maxPanSpeed, maxTiltSpeed, dt } = get();
    // Speed limit check: delta is degrees to move this frame
    const maxPanMove = maxPanSpeed * dt;
    const maxTiltMove = maxTiltSpeed * dt;
    
    const clampedDeltaPan = Math.max(-maxPanMove, Math.min(maxPanMove, deltaPan));
    const clampedDeltaTilt = Math.max(-maxTiltMove, Math.min(maxTiltMove, deltaTilt));
    
    get().setGimbalAngles(gimbalPan + clampedDeltaPan, gimbalTilt + clampedDeltaTilt);
  },
  
  updateBeaconPosition: (customDt) => {
    const { isPlaying, speed, beaconPosition, beaconVelocity, mode } = get();
    if (!isPlaying) return;
    
    const stepDt = customDt || get().dt;
    const dtScaled = stepDt * speed;
    
    let [x, y, z] = beaconPosition;
    let [vx, vy, vz] = beaconVelocity;
    
    if (mode === 'linear') {
      x += vx * dtScaled;
      y += vy * dtScaled;
      z += vz * dtScaled;
      
      // Boundary checks to bounce the beacon so it stays in sight
      // Z-boundary: 10m to 150m away
      if (z > -10.0) {
        z = -10.0;
        vz = -Math.abs(vz); // Bounce back deep
      } else if (z < -150.0) {
        z = -150.0;
        vz = Math.abs(vz); // Bounce forward
      }
      
      // X-boundary: -40m to 40m left/right
      if (Math.abs(x) > 40.0) {
        vx = -vx;
        x = Math.sign(x) * 40.0;
      }
      
      // Y-boundary: 1m (above ground) to 25m high
      if (y < 1.0) {
        vy = Math.abs(vy);
        y = 1.0;
      } else if (y > 25.0) {
        vy = -Math.abs(vy);
        y = 25.0;
      }
    }
    
    set((state) => ({
      time: state.time + stepDt,
      beaconPosition: [x, y, z],
      beaconVelocity: [vx, vy, vz]
    }));
  },
  
  updateTrackingStats: (u, v, isInside, totalErr, hErr, vErr) => {
    const { resolutionWidth, resolutionHeight } = get();
    // Compute pixel error relative to sensor center
    const cx = resolutionWidth / 2;
    const cy = resolutionHeight / 2;
    
    set({
      targetInsideFOV: isInside,
      trackedPixelX: u,
      trackedPixelY: v,
      pixelErrorX: isInside ? (u - cx) : 0,
      pixelErrorY: isInside ? (v - cy) : 0,
      angularErrorTotal: totalErr,
      angularErrorH: hErr,
      angularErrorV: vErr
    });
  },
  
  setWsStatus: (connected) => set({ wsConnected: connected }),
  
  updateWsMetrics: (fps, bytes, acks, latency) => set({
    wsFps: fps,
    wsBytesReceived: bytes,
    wsAcksCount: acks,
    wsLatency: latency
  }),
  
  setCapturedFrame: (capturedFrame) => set({ capturedFrame })
}));
