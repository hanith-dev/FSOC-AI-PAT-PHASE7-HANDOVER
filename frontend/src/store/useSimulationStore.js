import { create } from 'zustand';

export const getApiBase = () => {
  if (import.meta.env.VITE_API_URL) {
    return import.meta.env.VITE_API_URL;
  }
  if (typeof window !== 'undefined' && window.location) {
    if (window.location.port !== '5173') {
      return window.location.origin;
    }
    return `${window.location.protocol}//${window.location.hostname}:8000`;
  }
  return 'http://127.0.0.1:8000';
};

const DEFAULT_SIM_STATE = {
  isPlaying: true,
  speed: 1.0,
  time: 0.0,
  dt: 0.016, // ~60fps step
  mode: 'linear',
  
  // Multiple beacons tracking list
  beaconId: 'BEACON-01',
  beaconPosition: [10.0, 5.0, -100.0],
  beaconVelocity: [-2.0, 0.2, 5.0],
  beaconSize: 1.5,
  beaconIntensity: 2.0,
  beaconColor: '#10B981',
  beaconActive: true,
  
  beacons: [
    { id: 'BEACON-01', position: [10.0, 5.0, -100.0], start_position: [10.0, 5.0, -100.0], velocity: [-2.0, 0.2, 5.0], trajectory: 'linear', speed: 5.0, color: '#10B981', intensity: 2.0, size: 1.5, designated: true, active: true },
    { id: 'BEACON-02', position: [-15.0, 10.0, -120.0], start_position: [-15.0, 10.0, -120.0], velocity: [0.0, 0.0, 0.0], trajectory: 'circular', speed: 0.5, color: '#3B82F6', intensity: 1.5, size: 1.2, designated: false, active: true },
    { id: 'BEACON-03', position: [20.0, 8.0, -80.0], start_position: [20.0, 8.0, -80.0], velocity: [1.0, 0.0, 2.0], trajectory: 'sinewave', speed: 3.0, color: '#F59E0B', intensity: 1.8, size: 1.0, designated: false, active: true },
    { id: 'BEACON-04', position: [0.0, 12.0, -110.0], start_position: [0.0, 12.0, -110.0], velocity: [-1.0, 0.5, 1.0], trajectory: 'random', speed: 2.0, color: '#EC4899', intensity: 1.6, size: 1.3, designated: false, active: true }
  ],
  
  // Search parameters for state = SEARCHING
  searchTime: 0.0,
  searchConfig: {
    pattern: 'raster',
    speed: 10.0,
    rangeH: 20.0,
    rangeV: 15.0,
    stepSize: 3.0
  },
  
  // Disturbances
  disturbances: {
    platform_vibration: 0,
    camera_jitter: 0,
    image_noise: 0,
    blur: 0,
    atmospheric_turbulence: 0,
    illumination_variation: 0
  },
  
  // Live analytics metrics from backend
  liveMetrics: null,
  
  // Gimbal state
  gimbalPan: 0.0,  // degrees
  gimbalTilt: 0.0, // degrees
  panCommand: 0.0, // deg/s
  tiltCommand: 0.0, // deg/s
  controllerTelemetry: null,
  patEvents: [],
  panMin: -360.0,
  panMax: 360.0,
  tiltMin: -90.0,
  tiltMax: 90.0,
  maxPanSpeed: 180.0,  // deg/s
  maxTiltSpeed: 180.0, // deg/s
  
  // Camera state
  hfov: 10.0, // degrees
  vfov: 7.5,  // degrees
  resolutionWidth: 640,
  resolutionHeight: 480,
  cameraPosition: [0.0, 1.5, 0.0],
  
  // Tracking Telemetry
  targetInsideFOV: false,
  pixelErrorX: 0.0,
  pixelErrorY: 0.0,
  angularErrorTotal: 0.0,
  angularErrorH: 0.0,
  angularErrorV: 0.0,
  trackedPixelX: 320.0,
  trackedPixelY: 240.0,
  
  trackingState: 'SEARCHING',
  activeDetectorMode: 'opencv',
  fallbackWarning: false,
  inferenceTime: 0.0,
  confidence: 0.0,
  bbox: [0, 0, 0, 0],
  rawCenter: [0.0, 0.0],
  
  kalmanState: {
    active: false,
    x: 320.0,
    y: 240.0,
    vx: 0.0,
    vy: 0.0,
    timestamp: 0.0
  },
  
  pidConfig: {
    pan: { kp: 2.5, ki: 0.05, kd: 0.15 },
    tilt: { kp: 2.5, ki: 0.05, kd: 0.15 }
  },

  // WebSocket / Streaming metrics
  wsConnected: false,
  wsFps: 0,
  wsBytesReceived: 0,
  wsAcksCount: 0,
  wsLatency: 0,
  capturedFrame: null,
  wsSender: null,

  // Ablation Testing
  ablation: {
    ai_detection: true,
    kalman_filter: true,
    pid_controller: true,
    disturbances: true
  },

  // Real-World Simulation & Environment State
  targetPlatformModel: 'satellite', // 'satellite' | 'uav' | 'calibration_tower'
  environmentProfile: 'leo_downlink', // 'leo_downlink' | 'inter_satellite' | 'uav_tactical' | 'terrestrial_range'
  laserBeamVisible: true,
  timeOfDay: 'night', // 'night' | 'twilight' | 'day'
  
  // Real-World Atmospheric & Optical Link Parameters
  environmentalParameters: {
    cn2: 1e-15,               // Atmospheric turbulence structure constant (m^-2/3)
    friedR0Cm: 7.2,           // Fried coherence diameter (cm)
    scintillationIndex: 0.14, // Normalized optical intensity variance
    beamWanderUrad: 12.5,     // RMS beam wander (urad)
    cloudExtinctionDbKm: 0.25,// Atmospheric aerosol/fog extinction (dB/km)
    cloudDensity: 0.15,       // Visual cloud cover density (0.0 to 1.0)
    cloudAltitudeM: 14.0,     // Cloud base altitude (m)
    windSpeedMs: 5.5,         // Ground wind speed (m/s)
    windDirectionDeg: 220,    // Wind direction from North (deg)
    laserWavelengthNm: 1550,  // Wavelength (nm, eye-safe C-band)
    laserPowerMw: 500,        // Transmit power (mW)
    beamDivergenceUrad: 100,  // Full-angle beam divergence (urad)
    linkDistanceKm: 480.0,    // Slant range link distance (km)
    zenithAngleDeg: 34.0,     // Zenith angle (deg)
    airMass: 1.21,            // Optical air mass (1/cos(zenith))
    receivedPowerDbm: -31.4,  // Received optical power at OGS aperture (dBm)
    linkMarginDb: 13.6,       // Link power margin above -45 dBm sensitivity (dB)
    snrDb: 25.2,              // Optical detector SNR (dB)
    berEstimate: 1e-9,        // Estimated Bit Error Rate
    solarSeparationDeg: 62.0, // Angular separation from Sun
    solarBlinding: false      // Solar exclusion violation flag (< 15 deg)
  },

  // Experiment Engine
  experimentActive: false,
  experimentId: null,
  experimentTimeRemaining: 0,
  activeScenarioId: null,
  experimentResult: null,
  experimentsList: [],
  comparedExperiments: [],
  scenariosList: []
};

export const useSimulationStore = create((set, get) => ({
  ...DEFAULT_SIM_STATE,
  
  togglePlay: () => set((state) => ({ isPlaying: !state.isPlaying })),
  
  setSpeed: (speed) => set({ speed }),
  
  resetSimulation: () => {
    // Reset all beacons back to start positions
    const resetBeacons = get().beacons.map(b => ({
      ...b,
      position: [...b.start_position],
      velocity: b.trajectory === 'linear' ? [-2.0, 0.2, 5.0] : [0, 0, 0]
    }));
    const designated = resetBeacons.find(b => b.designated) || resetBeacons[0];
    
    set({
      time: 0.0,
      searchTime: 0.0,
      beacons: resetBeacons,
      beaconId: designated.id,
      beaconPosition: designated.position,
      beaconVelocity: designated.velocity,
      beaconSize: designated.size,
      beaconIntensity: designated.intensity,
      beaconColor: designated.color,
      beaconActive: designated.active,
      
      gimbalPan: 0.0,
      gimbalTilt: 0.0,
      targetInsideFOV: false,
      pixelErrorX: 0.0,
      pixelErrorY: 0.0,
      angularErrorTotal: 0.0,
      angularErrorH: 0.0,
      angularErrorV: 0.0,
      trackingState: 'SEARCHING',
      rawCenter: [0.0, 0.0],
      bbox: [0, 0, 0, 0],
      kalmanState: {
        active: false,
        x: 320.0,
        y: 240.0,
        vx: 0.0,
        vy: 0.0,
        timestamp: 0.0
      }
    });
  },
  
  setGimbalAngles: (pan, tilt) => {
    const { panMin, panMax, tiltMin, tiltMax } = get();
    const clampedPan = Math.max(panMin, Math.min(panMax, pan));
    const clampedTilt = Math.max(tiltMin, Math.min(tiltMax, tilt));
    set({ gimbalPan: clampedPan, gimbalTilt: clampedTilt });
  },
  
  manualGimbalMove: (deltaPan, deltaTilt) => {
    const { gimbalPan, gimbalTilt, maxPanSpeed, maxTiltSpeed, dt } = get();
    const maxPanMove = maxPanSpeed * dt;
    const maxTiltMove = maxTiltSpeed * dt;
    
    const clampedDeltaPan = Math.max(-maxPanMove, Math.min(maxPanMove, deltaPan));
    const clampedDeltaTilt = Math.max(-maxTiltMove, Math.min(maxTiltMove, deltaTilt));
    
    get().setGimbalAngles(gimbalPan + clampedDeltaPan, gimbalTilt + clampedDeltaTilt);
  },
  
  updateBeaconPosition: (customDt) => {
    const { isPlaying, speed, beacons } = get();
    if (!isPlaying) return;
    
    const stepDt = customDt || get().dt;
    const dtScaled = stepDt * speed;
    
    const updatedBeacons = beacons.map((b) => {
      let [x, y, z] = b.position;
      let [vx, vy, vz] = b.velocity || [0, 0, 0];
      
      const t = get().time + stepDt;
      
      let isActive = b.active;
      if (b.occlusion_window) {
        if (t >= b.occlusion_window[0] && t <= b.occlusion_window[1]) {
          isActive = false;
        } else {
          isActive = true;
        }
      }
      
      if (b.trajectory === 'linear') {
        x += vx * dtScaled;
        y += vy * dtScaled;
        z += vz * dtScaled;
        
        // Bounce bounds
        if (z > -10.0) { z = -10.0; vz = -Math.abs(vz); }
        else if (z < -150.0) { z = -150.0; vz = Math.abs(vz); }
        if (Math.abs(x) > 40.0) { vx = -vx; x = Math.sign(x) * 40.0; }
        if (y < 1.0) { vy = Math.abs(vy); y = 1.0; }
        else if (y > 25.0) { vy = -Math.abs(vy); y = 25.0; }
      } else if (b.trajectory === 'circular') {
        const radius = 15.0;
        const omega = b.speed || 0.5;
        const startPos = b.start_position || b.position;
        x = startPos[0] + radius * Math.cos(omega * t);
        y = startPos[1] + radius * Math.sin(omega * t) * 0.3;
        z = startPos[2] + Math.sin(omega * t * 0.5) * 10.0;
      } else if (b.trajectory === 'sinewave') {
        const startPos = b.start_position || b.position;
        const speedVal = b.speed || 3.0;
        x = startPos[0] + (b.velocity[0] || -1.0) * t;
        z = startPos[2] + (b.velocity[2] || 2.0) * t;
        y = startPos[1] + 8.0 * Math.sin(speedVal * t);
        
        if (z > -10.0) { z = -10.0; }
        else if (z < -150.0) { z = -150.0; }
        if (Math.abs(x) > 40.0) { x = Math.sign(x) * 40.0; }
        if (y < 1.0) y = 1.0;
      } else if (b.trajectory === 'random') {
        const maxWalkSpeed = b.speed || 3.0;
        const ax = (Math.random() - 0.5) * 4.0;
        const ay = (Math.random() - 0.5) * 2.0;
        const az = (Math.random() - 0.5) * 4.0;
        
        vx += ax * dtScaled;
        vy += ay * dtScaled;
        vz += az * dtScaled;
        
        const vMag = Math.sqrt(vx*vx + vy*vy + vz*vz);
        if (vMag > maxWalkSpeed) {
          vx = (vx / vMag) * maxWalkSpeed;
          vy = (vy / vMag) * maxWalkSpeed;
          vz = (vz / vMag) * maxWalkSpeed;
        }
        
        x += vx * dtScaled;
        y += vy * dtScaled;
        z += vz * dtScaled;
        
        if (z > -10.0) { z = -10.0; vz = -Math.abs(vz); }
        else if (z < -150.0) { z = -150.0; vz = Math.abs(vz); }
        if (Math.abs(x) > 40.0) { vx = -vx; x = Math.sign(x) * 40.0; }
        if (y < 1.0) { vy = Math.abs(vy); y = 1.0; }
        else if (y > 25.0) { vy = -Math.abs(vy); y = 25.0; }
      }
      
      return {
        ...b,
        position: [x, y, z],
        velocity: [vx, vy, vz],
        active: isActive
      };
    });
    
    const designated = updatedBeacons.find(b => b.designated) || updatedBeacons[0];
    
    // Dynamic Real-World Optical Link Geometry & Link Budget
    const camPos = get().cameraPosition || [0, 1.5, 0];
    const dx = designated.position[0] - camPos[0];
    const dy = designated.position[1] - camPos[1];
    const dz = designated.position[2] - camPos[2];
    const dist3d = Math.sqrt(dx * dx + dy * dy + dz * dz);
    
    const slantRangeKm = Math.max(1.0, dist3d * 5.0);
    const elevRad = Math.asin(Math.max(-1.0, Math.min(1.0, dy / Math.max(0.01, dist3d))));
    const elevDeg = (elevRad * 180.0) / Math.PI;
    const zenithDeg = Math.max(0.0, 90.0 - elevDeg);
    const airMass = Math.min(10.0, 1.0 / Math.max(0.1, Math.cos((zenithDeg * Math.PI) / 180.0)));
    
    const env = get().environmentalParameters || {};
    const lambdaMeters = (env.laserWavelengthNm || 1550) * 1e-9;
    const fsplDb = 20.0 * Math.log10((4.0 * Math.PI * slantRangeKm * 1000.0) / lambdaMeters);
    const atmLossDb = (env.cloudExtinctionDbKm || 0.25) * Math.min(15.0, slantRangeKm) * airMass;
    const ptxDbm = 10.0 * Math.log10(Math.max(1.0, env.laserPowerMw || 500));
    const prxDbm = ptxDbm + 75.0 + 90.0 - fsplDb - atmLossDb;
    const linkMarginDb = prxDbm - (-45.0);
    const snrDb = Math.max(0.0, linkMarginDb + 12.0);
    
    const k = (2.0 * Math.PI) / lambdaMeters;
    const cn2Val = Math.max(1e-17, env.cn2 || 1e-15);
    const friedR0Cm = Math.min(50.0, Math.max(0.5, Math.pow(0.423 * k * k * cn2Val * (slantRangeKm * 1000.0), -0.6) * 100.0));
    const scintIdx = Math.min(1.5, 1.23 * cn2Val * Math.pow(k, 7.0 / 6.0) * Math.pow(slantRangeKm * 1000.0, 11.0 / 6.0));

    // Accumulate search time if searching
    const currentTrackingState = get().trackingState;
    const currentSearchTime = (currentTrackingState === 'SEARCHING' || currentTrackingState === 'SEARCH') ? get().searchTime + stepDt : 0.0;
    
    set((state) => ({
      time: state.time + stepDt,
      searchTime: currentSearchTime,
      beacons: updatedBeacons,
      beaconId: designated.id,
      beaconPosition: designated.position,
      beaconVelocity: designated.velocity,
      beaconSize: designated.size,
      beaconIntensity: designated.intensity,
      beaconColor: designated.color,
      beaconActive: designated.active,
      environmentalParameters: {
        ...state.environmentalParameters,
        linkDistanceKm: parseFloat(slantRangeKm.toFixed(1)),
        zenithAngleDeg: parseFloat(zenithDeg.toFixed(1)),
        airMass: parseFloat(airMass.toFixed(2)),
        receivedPowerDbm: parseFloat(prxDbm.toFixed(1)),
        linkMarginDb: parseFloat(linkMarginDb.toFixed(1)),
        snrDb: parseFloat(snrDb.toFixed(1)),
        friedR0Cm: parseFloat(friedR0Cm.toFixed(1)),
        scintillationIndex: parseFloat(scintIdx.toFixed(3))
      }
    }));
  },
  
  updateTrackingStats: (u, v, isInside, totalErr, hErr, vErr) => {
    const { resolutionWidth, resolutionHeight } = get();
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
  
  setDetectorMode: (mode) => {
    const { wsConnected, wsSender } = get();
    set({ activeDetectorMode: mode });
    if (wsConnected && wsSender) {
      wsSender(JSON.stringify({
        type: 'set_detector',
        mode: mode
      }));
    }
  },
  
  setDesignatedTarget: (id) => {
    const { wsConnected, wsSender, beacons } = get();
    const updatedBeacons = beacons.map(b => ({
      ...b,
      designated: b.id === id
    }));
    const designated = updatedBeacons.find(b => b.designated) || updatedBeacons[0];
    
    set({
      beacons: updatedBeacons,
      beaconId: designated.id,
      beaconPosition: designated.position,
      beaconVelocity: designated.velocity
    });
    
    if (wsConnected && wsSender) {
      wsSender(JSON.stringify({
        type: 'set_designated_target',
        target_id: id
      }));
    }
  },
  
  setTrajectoryConfig: (id, trajectory, speed, startPos) => {
    const { wsConnected, wsSender, beacons } = get();
    const updatedBeacons = beacons.map(b => {
      if (b.id === id) {
        const nextStart = startPos || b.start_position;
        return {
          ...b,
          trajectory: trajectory,
          speed: parseFloat(speed),
          start_position: nextStart,
          position: [...nextStart]
        };
      }
      return b;
    });
    
    set({ beacons: updatedBeacons });
    
    if (wsConnected && wsSender) {
      wsSender(JSON.stringify({
        type: 'set_trajectory',
        beacon_id: id,
        trajectory: trajectory,
        speed: parseFloat(speed),
        start_position: startPos
      }));
    }
  },
  
  setSearchConfig: (config) => {
    const { wsConnected, wsSender, searchConfig } = get();
    const newConfig = { ...searchConfig, ...config };
    set({ searchConfig: newConfig });
    
    if (wsConnected && wsSender) {
      wsSender(JSON.stringify({
        type: 'set_search_config',
        config: newConfig
      }));
    }
  },
  
  setDisturbances: (config) => {
    const { wsConnected, wsSender, disturbances } = get();
    const newDisturbances = { ...disturbances, ...config };
    set({ disturbances: newDisturbances });
    
    if (wsConnected && wsSender) {
      wsSender(JSON.stringify({
        type: 'set_disturbances',
        disturbances: newDisturbances
      }));
    }
  },
  
  resetDisturbances: () => {
    const { wsConnected, wsSender } = get();
    const cleared = {
      platform_vibration: 0,
      camera_jitter: 0,
      image_noise: 0,
      blur: 0,
      atmospheric_turbulence: 0,
      illumination_variation: 0
    };
    set({ disturbances: cleared });
    
    if (wsConnected && wsSender) {
      wsSender(JSON.stringify({
        type: 'reset_disturbances'
      }));
    }
  },
  
  randomizeDisturbances: () => {
    const { wsConnected, wsSender } = get();
    const randomized = {
      platform_vibration: Math.floor(Math.random() * 80),
      camera_jitter: Math.floor(Math.random() * 80),
      image_noise: Math.floor(Math.random() * 80),
      blur: Math.floor(Math.random() * 80),
      atmospheric_turbulence: Math.floor(Math.random() * 80),
      illumination_variation: Math.floor(Math.random() * 80)
    };
    set({ disturbances: randomized });
    
    if (wsConnected && wsSender) {
      wsSender(JSON.stringify({
        type: 'set_disturbances',
        disturbances: randomized
      }));
    }
  },

  setTargetPlatformModel: (model) => set({ targetPlatformModel: model }),
  
  setLaserBeamVisible: (visible) => set({ laserBeamVisible: visible }),
  
  setTimeOfDay: (tod) => set({ timeOfDay: tod }),
  
  setEnvironmentProfile: (profile) => {
    const { setDisturbances } = get();
    if (profile === 'leo_downlink') {
      set({
        environmentProfile: profile,
        targetPlatformModel: 'satellite',
        timeOfDay: 'night',
        environmentalParameters: {
          ...get().environmentalParameters,
          cn2: 1e-15,
          friedR0Cm: 7.5,
          scintillationIndex: 0.15,
          cloudExtinctionDbKm: 0.25,
          cloudDensity: 0.15,
          windSpeedMs: 5.5,
          linkDistanceKm: 500.0
        }
      });
      setDisturbances({ atmospheric_turbulence: 25, platform_vibration: 15 });
    } else if (profile === 'inter_satellite') {
      set({
        environmentProfile: profile,
        targetPlatformModel: 'satellite',
        timeOfDay: 'night',
        environmentalParameters: {
          ...get().environmentalParameters,
          cn2: 1e-17,
          friedR0Cm: 50.0,
          scintillationIndex: 0.01,
          cloudExtinctionDbKm: 0.0,
          cloudDensity: 0.0,
          windSpeedMs: 0.0,
          linkDistanceKm: 12000.0
        }
      });
      setDisturbances({ atmospheric_turbulence: 0, platform_vibration: 20 });
    } else if (profile === 'uav_tactical') {
      set({
        environmentProfile: profile,
        targetPlatformModel: 'uav',
        timeOfDay: 'day',
        environmentalParameters: {
          ...get().environmentalParameters,
          cn2: 5e-15,
          friedR0Cm: 4.2,
          scintillationIndex: 0.35,
          cloudExtinctionDbKm: 1.2,
          cloudDensity: 0.35,
          windSpeedMs: 12.0,
          linkDistanceKm: 25.0
        }
      });
      setDisturbances({ atmospheric_turbulence: 55, platform_vibration: 40, camera_jitter: 25 });
    } else if (profile === 'terrestrial_range') {
      set({
        environmentProfile: profile,
        targetPlatformModel: 'calibration_tower',
        timeOfDay: 'twilight',
        environmentalParameters: {
          ...get().environmentalParameters,
          cn2: 2e-14,
          friedR0Cm: 2.1,
          scintillationIndex: 0.65,
          cloudExtinctionDbKm: 2.5,
          cloudDensity: 0.45,
          windSpeedMs: 8.0,
          linkDistanceKm: 5.2
        }
      });
      setDisturbances({ atmospheric_turbulence: 75, blur: 30, image_noise: 20 });
    }
  },

  updateEnvironmentalParameters: (params) => {
    const updated = { ...get().environmentalParameters, ...params };
    set({ environmentalParameters: updated });
    if (params.cn2 !== undefined) {
      const turbPercent = Math.min(100, Math.max(0, Math.round(((Math.log10(params.cn2) + 17) / 4.0) * 100)));
      get().setDisturbances({ atmospheric_turbulence: turbPercent });
    }
    if (params.cloudExtinctionDbKm !== undefined) {
      const blurVal = Math.min(100, Math.max(0, Math.round((params.cloudExtinctionDbKm / 10.0) * 100)));
      get().setDisturbances({ blur: blurVal });
    }
  },
  
  
  updatePidConfig: (axis, coef, value) => set((state) => ({
    pidConfig: {
      ...state.pidConfig,
      [axis]: {
        ...state.pidConfig[axis],
        [coef]: parseFloat(value)
      }
    }
  })),
  
  setWsSender: (sender) => set({ wsSender: sender }),
  
  setWsStatus: (connected) => set({ wsConnected: connected }),
  
  updateWsMetrics: (fps, bytes, acks, latency) => set({
    wsFps: fps,
    wsBytesReceived: bytes,
    wsAcksCount: acks,
    wsLatency: latency
  }),
  
  setTelemetry: (data) => {
    // Process backend telemetry updates
    const currentState = get().trackingState;
    const newState = data.tracking_state;
    let patEvents = get().patEvents || [];
    if (newState && currentState && newState !== currentState) {
      const newEvent = {
        id: Date.now() + Math.random(),
        time: new Date().toLocaleTimeString(),
        from: currentState,
        to: newState,
        reason: data.pat_fsm?.transition_reason || data.pat_telemetry?.transition_reason || "STATE_UPDATE"
      };
      patEvents = [newEvent, ...patEvents].slice(0, 15);
    }

    const updates = {
      trackingState: newState,
      fallbackWarning: data.fallback_warning,
      inferenceTime: data.inference_time_ms,
      confidence: data.confidence !== undefined ? data.confidence : (data.detected ? 0.9 : 0.0),
      bbox: data.bbox || [0, 0, 0, 0],
      rawCenter: data.raw_center || [0.0, 0.0],
      searchOffsetPan: data.search_offset_pan_deg || 0.0,
      searchOffsetTilt: data.search_offset_tilt_deg || 0.0,
      panCommand: data.pan_command !== undefined ? data.pan_command : 0.0,
      tiltCommand: data.tilt_command !== undefined ? data.tilt_command : 0.0,
      controllerTelemetry: data.controller_telemetry || null,
      patEvents: patEvents,
      patTelemetry: data.pat_telemetry || null,
      kalmanState: {
        active: Boolean(data.kalman?.active),
        x: data.kalman?.x || 320.0,
        y: data.kalman?.y || 240.0,
        vx: data.kalman?.vx || 0.0,
        vy: data.kalman?.vy || 0.0,
        timestamp: data.kalman?.timestamp || 0.0
      }
    };
    
    if (data.live_metrics) {
      updates.liveMetrics = data.live_metrics;
    }

    if (data.ablation) {
      updates.ablation = data.ablation;
    }

    if (data.experiment_active) {
      updates.experimentActive = true;
      updates.experimentId = data.experiment_id;
      updates.experimentTimeRemaining = data.experiment_time_remaining;
    } else if (data.type === "experiment_complete") {
      updates.experimentActive = false;
      updates.experimentId = null;
      updates.experimentTimeRemaining = 0;
      updates.experimentResult = data.experiment_result;
      get().fetchResults();
    } else {
      updates.experimentActive = false;
    }
    
    // Sync backend sync configurations if present
    if (data.config_sync) {
      if (data.config_sync.beacons) {
        // Only override static details, keeping current simulation coordinate updates
        const syncedBeacons = get().beacons.map(b => {
          const match = data.config_sync.beacons.find(x => x.id === b.id);
          if (match) {
            return {
              ...b,
              trajectory: match.trajectory,
              speed: match.speed,
              designated: match.designated,
              color: match.color,
              size: match.size,
              intensity: match.intensity,
              active: match.active,
              occlusion_window: match.occlusion_window
            };
          }
          return b;
        });
        updates.beacons = syncedBeacons;
        
        const designated = syncedBeacons.find(b => b.designated) || syncedBeacons[0];
        updates.beaconId = designated.id;
      }
      if (data.config_sync.search) {
        updates.searchConfig = data.config_sync.search;
      }
      if (data.config_sync.disturbances) {
        updates.disturbances = data.config_sync.disturbances;
      }
    }
    
    set(updates);
  },
  
  setCapturedFrame: (capturedFrame) => set({ capturedFrame }),

  setAblation: (config) => {
    const { wsConnected, wsSender, ablation } = get();
    const newAblation = { ...ablation, ...config };
    set({ ablation: newAblation });
    if (wsConnected && wsSender) {
      wsSender(JSON.stringify({
        type: 'set_ablation',
        toggles: newAblation
      }));
    }
  },

  fetchScenarios: async () => {
    const apiBase = getApiBase();
    try {
      const res = await fetch(`${apiBase}/api/scenarios`);
      const data = await res.json();
      set({ scenariosList: data });
    } catch (e) {
      console.error("Error fetching scenarios", e);
    }
  },

  fetchResults: async () => {
    const apiBase = getApiBase();
    try {
      const res = await fetch(`${apiBase}/api/experiments/results`);
      const data = await res.json();
      set({ experimentsList: data });
    } catch (e) {
      console.error("Error fetching experiment results", e);
    }
  },

  startExperiment: async (scenarioId) => {
    const apiBase = getApiBase();
    try {
      const payload = {
        scenario_id: scenarioId,
        client_config: {
          ablation: get().ablation,
          pid_config: get().pidConfig,
          detector_mode: get().activeDetectorMode
        }
      };
      
      const res = await fetch(`${apiBase}/api/experiments/start`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      const data = await res.json();
      if (data.status === 'success') {
        const exp = data.experiment;
        set({
          experimentActive: true,
          experimentId: exp.experiment_id,
          experimentTimeRemaining: exp.duration,
          activeScenarioId: scenarioId,
          experimentResult: null
        });
        // Reset simulation coordinates & time
        get().resetSimulation();
        // Force play simulation
        set({ isPlaying: true });
      }
    } catch (e) {
      console.error("Error starting experiment", e);
    }
  },

  stopExperiment: async () => {
    const apiBase = getApiBase();
    try {
      const res = await fetch(`${apiBase}/api/experiments/stop`, { method: 'POST' });
      const data = await res.json();
      if (data.status === 'success') {
        set({
          experimentActive: false,
          experimentId: null,
          experimentTimeRemaining: 0,
          experimentResult: data.experiment
        });
        get().fetchResults();
      }
    } catch (e) {
      console.error("Error stopping experiment", e);
    }
  },

  deleteExperiment: async (id) => {
    const apiBase = getApiBase();
    try {
      await fetch(`${apiBase}/api/experiments/results/${id}`, { method: 'DELETE' });
      get().fetchResults();
    } catch (e) {
      console.error("Error deleting experiment", e);
    }
  },

  fetchComparison: async (ids) => {
    if (!ids || ids.length === 0) {
      set({ comparedExperiments: [] });
      return;
    }
    const apiBase = getApiBase();
    try {
      const res = await fetch(`${apiBase}/api/experiments/compare?ids=${ids.join(',')}`);
      const data = await res.json();
      set({ comparedExperiments: data });
    } catch (e) {
      console.error("Error comparing experiments", e);
    }
  },

  simulateOcclusion: () => {
    const designatedId = get().beaconId;
    const updated = get().beacons.map(b => b.id === designatedId ? { ...b, active: false } : b);
    set({ beacons: updated, beaconActive: false });
  },

  restoreBeacon: () => {
    const designatedId = get().beaconId;
    const updated = get().beacons.map(b => b.id === designatedId ? { ...b, active: true } : b);
    set({ beacons: updated, beaconActive: true });
  },

  addDistractor: () => {
    const currentBeacons = get().beacons;
    const distractorNum = currentBeacons.length + 1;
    const distractorId = `BEACON-${distractorNum < 10 ? '0' + distractorNum : distractorNum}`;
    const newDistractor = {
      id: distractorId,
      position: [12.0, 7.0, -95.0],
      start_position: [12.0, 7.0, -95.0],
      velocity: [-1.2, 0.1, 3.5],
      trajectory: 'linear',
      speed: 3.5,
      color: '#3B82F6',
      intensity: 2.0,
      size: 1.3,
      designated: false,
      active: true
    };
    set({ beacons: [...currentBeacons, newDistractor] });
  }
}));

