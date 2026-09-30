import React, { useRef, useEffect } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import { PerspectiveCamera, Line } from '@react-three/drei';
import * as THREE from 'three';
import { useSimulationStore } from '../store/useSimulationStore';
import { GimbalController } from '../utils/pid';

export default function GimbalCamera({ sendFrame }) {
  const gimbalBaseRef = useRef(); // Pan (Y-rotation)
  const gimbalHeadRef = useRef(); // Tilt (X-rotation)
  const cameraRef = useRef();
  const renderTargetRef = useRef();
  
  // Offscreen canvas elements for flipping and compression
  const offscreenCanvasRef = useRef(null);
  const tempCanvasRef = useRef(null);
  const bufferRef = useRef(null);
  
  // Client-side 60Hz PID controllers (Phase 6.5 certified filtered_pid)
  const panPid = useRef(new GimbalController({ mode: 'filtered_pid' }));
  const tiltPid = useRef(new GimbalController({ mode: 'filtered_pid' }));

  // Search and Reacquisition state
  const searchTimeRef = useRef(0.0);
  const reacquireAnchorRef = useRef({ pan: 0.0, tilt: 0.0 });
  
  // Specific selectors to avoid re-rendering GimbalCamera on every 60Hz state change
  const setCapturedFrame = useSimulationStore((s) => s.setCapturedFrame);
  const setGimbalAngles = useSimulationStore((s) => s.setGimbalAngles);
  const updateTrackingStats = useSimulationStore((s) => s.updateTrackingStats);
  const resolutionWidth = useSimulationStore((s) => s.resolutionWidth);
  const resolutionHeight = useSimulationStore((s) => s.resolutionHeight);
  const hfov = useSimulationStore((s) => s.hfov);
  const vfov = useSimulationStore((s) => s.vfov);
  const pidConfig = useSimulationStore((s) => s.pidConfig);
  const kalmanActive = useSimulationStore((s) => s.kalmanState.active);
  const kalmanTimestamp = useSimulationStore((s) => s.kalmanState.timestamp);
  
  const { gl, scene } = useThree();

  // Track the timestamp when the last Kalman state update was received
  const lastTelemetryTime = useRef(0);
  const prevKalmanTimestamp = useRef(0);

  // Initialize offscreen canvases
  useEffect(() => {
    offscreenCanvasRef.current = document.createElement('canvas');
    offscreenCanvasRef.current.width = resolutionWidth;
    offscreenCanvasRef.current.height = resolutionHeight;
    
    tempCanvasRef.current = document.createElement('canvas');
    tempCanvasRef.current.width = resolutionWidth;
    tempCanvasRef.current.height = resolutionHeight;
    
    renderTargetRef.current = new THREE.WebGLRenderTarget(resolutionWidth, resolutionHeight, {
      minFilter: THREE.LinearFilter,
      magFilter: THREE.LinearFilter,
      format: THREE.RGBAFormat,
    });
    
    return () => {
      if (renderTargetRef.current) {
        renderTargetRef.current.dispose();
      }
    };
  }, [resolutionWidth, resolutionHeight]);

  // Sync PID tuning settings
  useEffect(() => {
    panPid.current.tune(pidConfig.pan.kp, pidConfig.pan.ki, pidConfig.pan.kd);
    tiltPid.current.tune(pidConfig.tilt.kp, pidConfig.tilt.ki, pidConfig.tilt.kd);
  }, [pidConfig]);

  // Handle Kalman timestamp changes to sync clock
  useEffect(() => {
    if (kalmanActive && kalmanTimestamp !== prevKalmanTimestamp.current) {
      prevKalmanTimestamp.current = kalmanTimestamp;
      // Record local time when we got this update
      lastTelemetryTime.current = performance.now() / 1000;
    }
  }, [kalmanActive, kalmanTimestamp]);

  // 60Hz rendering and closed-loop control update
  useFrame((state, delta) => {
    if (!gimbalBaseRef.current || !gimbalHeadRef.current || !cameraRef.current) return;
    
    // Fetch absolute latest values from the store to avoid stale closures
    const store = useSimulationStore.getState();
    const currentPan = store.gimbalPan;
    const currentTilt = store.gimbalTilt;
    const currentIsPlaying = store.isPlaying;
    const currentKalmanState = store.kalmanState;
    const currentAblation = store.ablation;
    const currentTrackingState = store.trackingState;
    const currentSearchConfig = store.searchConfig;
    
    // Direct drive of physical 3D scene representation from store angles
    gimbalBaseRef.current.rotation.y = THREE.MathUtils.degToRad(currentPan);
    gimbalHeadRef.current.rotation.x = THREE.MathUtils.degToRad(currentTilt);
    
    if (!currentIsPlaying) return;

    // Compute focal length in pixel dimensions from FOV config
    const fx = resolutionWidth / (2.0 * Math.tan(THREE.MathUtils.degToRad(hfov) / 2.0));
    const fy = resolutionHeight / (2.0 * Math.tan(THREE.MathUtils.degToRad(vfov) / 2.0));
    
    let u = resolutionWidth / 2;
    let v = resolutionHeight / 2;
    let isInside = false;
    let totalErrorDeg = 0.0;
    let hErrorDeg = 0.0;
    let vErrorDeg = 0.0;

    if (currentKalmanState.active) {
      // Target active and tracked
      searchTimeRef.current = 0.0;
      reacquireAnchorRef.current = { pan: currentPan, tilt: currentTilt };

      // 1. High-rate prediction (60Hz extrapolation using velocities)
      const timeNow = performance.now() / 1000;
      const elapsed = lastTelemetryTime.current > 0 ? (timeNow - lastTelemetryTime.current) : 0;
      
      const xPred = currentKalmanState.x + currentKalmanState.vx * elapsed;
      const yPred = currentKalmanState.y + currentKalmanState.vy * elapsed;
      
      u = xPred;
      v = yPred;
      isInside = (0 <= u && u <= resolutionWidth) && (0 <= v && v <= resolutionHeight);
      
      // 2. Compute pixel error relative to center
      const dx = u - (resolutionWidth / 2);
      const dy = (resolutionHeight / 2) - v; // Invert y: pixel y goes down, tilting +y is up
      
      // 3. Compute angular errors in degrees
      hErrorDeg = THREE.MathUtils.radToDeg(Math.atan2(dx, fx));
      vErrorDeg = THREE.MathUtils.radToDeg(Math.atan2(dy, fy));
      totalErrorDeg = Math.sqrt(hErrorDeg * hErrorDeg + vErrorDeg * vErrorDeg);

      // 4. Closed-loop PID control (runs continuously to maintain lock & keep error < 5°)
      if (currentAblation.pid_controller) {
        // Primary: Authoritative rate command from certified backend GimbalController
        let panSpeedCmd = store.panCommand !== undefined ? store.panCommand : 0.0;
        let tiltSpeedCmd = store.tiltCommand !== undefined ? store.tiltCommand : 0.0;

        // Fallback to local PID if backend hasn't emitted commands yet
        if (panSpeedCmd === 0.0 && tiltSpeedCmd === 0.0 && totalErrorDeg > 0.02) {
          panSpeedCmd = panPid.current.update(-hErrorDeg, delta);
          tiltSpeedCmd = tiltPid.current.update(vErrorDeg, delta);
        }

        // Integrate speeds to drive gimbal angles
        const nextPan = currentPan + panSpeedCmd * delta;
        const nextTilt = currentTilt + tiltSpeedCmd * delta;
        
        setGimbalAngles(nextPan, nextTilt);
      } else {
        panPid.current.reset();
        tiltPid.current.reset();
      }
    } else if (currentTrackingState === 'REACQUIRE') {
      // Reacquisition search guided by covariance offsets
      u = resolutionWidth / 2;
      v = resolutionHeight / 2;
      isInside = false;
      totalErrorDeg = store.angularErrorTotal || 0.0;
      hErrorDeg = store.angularErrorH || 0.0;
      vErrorDeg = store.angularErrorV || 0.0;
      panPid.current.reset();
      tiltPid.current.reset();

      if (store.panCommand !== 0.0 || store.tiltCommand !== 0.0) {
        const nextPan = currentPan + (store.panCommand || 0.0) * delta;
        const nextTilt = currentTilt + (store.tiltCommand || 0.0) * delta;
        setGimbalAngles(nextPan, nextTilt);
      } else {
        const offsetPan = store.searchOffsetPan || 0.0;
        const offsetTilt = store.searchOffsetTilt || 0.0;
        const nextPan = reacquireAnchorRef.current.pan + offsetPan;
        const nextTilt = reacquireAnchorRef.current.tilt + offsetTilt;
        setGimbalAngles(nextPan, nextTilt);
      }
    } else {
      // Autonomous wide-area search scan in SEARCH state
      u = resolutionWidth / 2;
      v = resolutionHeight / 2;
      isInside = false;
      totalErrorDeg = 0.0;
      hErrorDeg = 0.0;
      vErrorDeg = 0.0;
      panPid.current.reset();
      tiltPid.current.reset();

      searchTimeRef.current += delta;
      const tSearch = searchTimeRef.current;
      const speed = currentSearchConfig.speed || 10.0;
      const rH = (currentSearchConfig.rangeH || 20.0) / 2.0; // 10 deg half-range
      const rV = (currentSearchConfig.rangeV || 15.0) / 2.0; // 7.5 deg half-range
      const stepSize = currentSearchConfig.stepSize || 3.0;

      const timePerRow = (2.0 * rH) / speed;
      const rowIdx = Math.floor(tSearch / timePerRow);
      const rowT = tSearch % timePerRow;
      const frac = rowT / timePerRow;
      const sweepPan = (rowIdx % 2 === 0) ? (-rH + frac * 2.0 * rH) : (rH - frac * 2.0 * rH);
      const numRows = Math.floor((2.0 * rV) / stepSize) + 1;
      const activeRow = rowIdx % numRows;
      const sweepTilt = rV - (activeRow * stepSize);

      setGimbalAngles(sweepPan, sweepTilt);
    }
    
    // Update store stats for UI rendering
    const finalTotalErr = (store.trackingState !== 'SEARCH' && store.angularErrorTotal !== undefined) ? store.angularErrorTotal : totalErrorDeg;
    const finalHErr = (store.trackingState !== 'SEARCH' && store.angularErrorH !== undefined) ? store.angularErrorH : hErrorDeg;
    const finalVErr = (store.trackingState !== 'SEARCH' && store.angularErrorV !== undefined) ? store.angularErrorV : vErrorDeg;
    updateTrackingStats(u, v, isInside, finalTotalErr, finalHErr, finalVErr);
  });

  // Decoupled Frame Capture Loop at 15Hz
  const lastCaptureTime = useRef(0);
  useFrame((state) => {
    if (!cameraRef.current || !renderTargetRef.current) return;
    
    const time = state.clock.getElapsedTime();
    if (time - lastCaptureTime.current < 1.0 / 15.0) return;
    lastCaptureTime.current = time;
    
    const rt = renderTargetRef.current;
    const camera = cameraRef.current;
    const currentRenderTarget = gl.getRenderTarget();
    
    gl.setRenderTarget(rt);
    gl.render(scene, camera);
    gl.setRenderTarget(currentRenderTarget);
    
    const width = rt.width;
    const height = rt.height;
    if (!bufferRef.current || bufferRef.current.length !== width * height * 4) {
      bufferRef.current = new Uint8Array(width * height * 4);
    }
    const buffer = bufferRef.current;
    gl.readRenderTargetPixels(rt, 0, 0, width, height, buffer);
    
    const canvas = offscreenCanvasRef.current;
    const tempCanvas = tempCanvasRef.current;
    if (!canvas || !tempCanvas) return;
    
    const ctx = canvas.getContext('2d');
    const tempCtx = tempCanvas.getContext('2d');
    
    const imgData = new ImageData(new Uint8ClampedArray(buffer.buffer), width, height);
    tempCtx.putImageData(imgData, 0, 0);
    
    ctx.clearRect(0, 0, width, height);
    ctx.save();
    ctx.translate(0, height);
    ctx.scale(1, -1);
    ctx.drawImage(tempCanvas, 0, 0);
    ctx.restore();
    
    // Single JPEG compression and direct base64 transfer
    const dataUrl = canvas.toDataURL('image/jpeg', 0.6);
    setCapturedFrame(dataUrl);
    
    if (sendFrame) {
      const storeState = useSimulationStore.getState();
      const metadata = {
        gimbal_pan: storeState.gimbalPan,
        gimbal_tilt: storeState.gimbalTilt,
        camera_position: storeState.cameraPosition,
        camera_orientation: cameraRef.current ? [
          cameraRef.current.quaternion.x,
          cameraRef.current.quaternion.y,
          cameraRef.current.quaternion.z,
          cameraRef.current.quaternion.w
        ] : [0, 0, 0, 1],
        designated_target_id: storeState.beaconId,
        beacons: storeState.beacons.map(b => ({
          id: b.id,
          position: b.position,
          velocity: b.velocity,
          size: b.size,
          color: b.color,
          intensity: b.intensity,
          designated: b.designated,
          active: b.active
        }))
      };
      sendFrame(dataUrl, metadata);
    }
  });

  // visual lines for the FOV frustum/cone
  const fLength = 5.0; // visual length of frustum lines
  const hfovRad = THREE.MathUtils.degToRad(hfov);
  const vfovRad = THREE.MathUtils.degToRad(vfov);
  const wHalf = fLength * Math.tan(hfovRad / 2);
  const hHalf = fLength * Math.tan(vfovRad / 2);

  const frustumLines = [
    [[0, 0, 0], [0, 0, -fLength]],
    [[0, 0, 0], [-wHalf, hHalf, -fLength]],
    [[0, 0, 0], [wHalf, hHalf, -fLength]],
    [[0, 0, 0], [wHalf, -hHalf, -fLength]],
    [[0, 0, 0], [-wHalf, -hHalf, -fLength]],
    [[-wHalf, hHalf, -fLength], [wHalf, hHalf, -fLength]],
    [[wHalf, hHalf, -fLength], [wHalf, -hHalf, -fLength]],
    [[wHalf, -hHalf, -fLength], [-wHalf, -hHalf, -fLength]],
    [[-wHalf, -hHalf, -fLength], [-wHalf, hHalf, -fLength]],
  ];

  return (
    <group ref={gimbalBaseRef} position={[0, 0, 0]}>
      {/* ======================================================== */}
      {/* 1. VIBRATION-ISOLATED OGS PIER & AZIMUTH MOTOR BASE      */}
      {/* ======================================================== */}
      {/* Structural Steel Pier Base Flange */}
      <mesh position={[0, 0.1, 0]}>
        <cylinderGeometry args={[0.55, 0.65, 0.2, 16]} />
        <meshStandardMaterial color="#0F172A" roughness={0.8} metalness={0.4} />
      </mesh>
      
      {/* Main Steel Telescope Pier Column */}
      <mesh position={[0, 0.75, 0]}>
        <cylinderGeometry args={[0.3, 0.42, 1.3, 16]} />
        <meshStandardMaterial color="#1E293B" roughness={0.5} metalness={0.7} />
      </mesh>

      {/* Azimuth Bearing & Rotary Encoder Housing */}
      <mesh position={[0, 1.42, 0]}>
        <cylinderGeometry args={[0.42, 0.42, 0.14, 24]} />
        <meshStandardMaterial color="#334155" roughness={0.3} metalness={0.8} />
      </mesh>

      {/* Azimuth Graduation Ring */}
      <mesh position={[0, 1.45, 0]}>
        <torusGeometry args={[0.43, 0.015, 12, 32]} />
        <meshStandardMaterial color="#0284C7" metalness={0.9} roughness={0.1} />
      </mesh>
      
      {/* ======================================================== */}
      {/* 2. AZIMUTH FORK & ALTITUDE AXIS (Rotates in Pan Y)       */}
      {/* ======================================================== */}
      <group position={[0, 1.5, 0]}>
        {/* Pan Turret Head Plate */}
        <mesh>
          <cylinderGeometry args={[0.4, 0.4, 0.1, 24]} />
          <meshStandardMaterial color="#1E293B" roughness={0.4} metalness={0.7} />
        </mesh>
        
        {/* Dual Altitude Fork Arms */}
        <group ref={gimbalHeadRef}>
          {/* Port (Left) Fork Arm */}
          <group position={[-0.48, 0.35, 0]}>
            <mesh>
              <boxGeometry args={[0.14, 0.75, 0.3]} />
              <meshStandardMaterial color="#334155" roughness={0.4} metalness={0.6} />
            </mesh>
            {/* Altitude Direct-Drive Motor Housing */}
            <mesh position={[-0.08, 0.2, 0]} rotation={[0, 0, Math.PI / 2]}>
              <cylinderGeometry args={[0.18, 0.18, 0.1, 16]} />
              <meshStandardMaterial color="#0F172A" metalness={0.9} roughness={0.2} />
            </mesh>
          </group>

          {/* Starboard (Right) Fork Arm */}
          <group position={[0.48, 0.35, 0]}>
            <mesh>
              <boxGeometry args={[0.14, 0.75, 0.3]} />
              <meshStandardMaterial color="#334155" roughness={0.4} metalness={0.6} />
            </mesh>
            {/* Altitude Rotary Encoder Housing */}
            <mesh position={[0.08, 0.2, 0]} rotation={[0, 0, Math.PI / 2]}>
              <cylinderGeometry args={[0.16, 0.16, 0.08, 16]} />
              <meshStandardMaterial color="#0F172A" metalness={0.9} roughness={0.2} />
            </mesh>
          </group>
          
          {/* ======================================================== */}
          {/* 3. RITCHEY-CHRETIEN TELESCOPE OPTICAL TUBE ASSEMBLY (OTA)*/}
          {/* ======================================================== */}
          <group position={[0, 0.55, 0]}>
            {/* Primary Mirror Cell (Aperture D=200mm) */}
            <mesh position={[0, 0, 0.15]} rotation={[Math.PI / 2, 0, 0]}>
              <cylinderGeometry args={[0.34, 0.34, 0.35, 24]} />
              <meshStandardMaterial color="#0F172A" roughness={0.3} metalness={0.8} />
            </mesh>

            {/* Primary Mirror Surface (Inside Cell) */}
            <mesh position={[0, 0, 0.05]} rotation={[Math.PI / 2, 0, 0]}>
              <circleGeometry args={[0.3, 24]} />
              <meshStandardMaterial color="#E2E8F0" metalness={0.95} roughness={0.05} />
            </mesh>

            {/* Carbon-Fiber Open Truss Poles (4 Truss Struts) */}
            {[
              [-0.22, 0.22],
              [0.22, 0.22],
              [-0.22, -0.22],
              [0.22, -0.22]
            ].map((p, idx) => (
              <mesh key={idx} position={[p[0], p[1], -0.25]} rotation={[Math.PI / 2, 0, 0]}>
                <cylinderGeometry args={[0.015, 0.015, 0.5, 8]} />
                <meshStandardMaterial color="#1E293B" roughness={0.2} metalness={0.9} />
              </mesh>
            ))}

            {/* Front Aperture Ring & Dew Shield Baffle */}
            <mesh position={[0, 0, -0.55]} rotation={[Math.PI / 2, 0, 0]}>
              <cylinderGeometry args={[0.34, 0.34, 0.25, 24, 1, true]} />
              <meshStandardMaterial color="#0F172A" roughness={0.4} metalness={0.7} side={2} />
            </mesh>

            {/* 4-Vane Spider Support for Secondary Mirror */}
            <mesh position={[0, 0, -0.55]}>
              <boxGeometry args={[0.66, 0.01, 0.04]} />
              <meshStandardMaterial color="#475569" />
            </mesh>
            <mesh position={[0, 0, -0.55]}>
              <boxGeometry args={[0.01, 0.66, 0.04]} />
              <meshStandardMaterial color="#475569" />
            </mesh>

            {/* Secondary Mirror Cell & Central Baffle */}
            <mesh position={[0, 0, -0.55]} rotation={[Math.PI / 2, 0, 0]}>
              <cylinderGeometry args={[0.09, 0.09, 0.12, 16]} />
              <meshStandardMaterial color="#334155" metalness={0.8} />
            </mesh>

            {/* Anodized Aperture Rim Ring */}
            <mesh position={[0, 0, -0.67]} rotation={[Math.PI / 2, 0, 0]}>
              <torusGeometry args={[0.34, 0.015, 12, 32]} />
              <meshStandardMaterial color="#0284C7" metalness={0.9} roughness={0.1} />
            </mesh>

            {/* Co-Aligned Wide-Field Acquisition Camera (WFOV) */}
            <group position={[0.38, 0.15, -0.3]}>
              <mesh rotation={[Math.PI / 2, 0, 0]}>
                <cylinderGeometry args={[0.06, 0.06, 0.3, 12]} />
                <meshStandardMaterial color="#1E293B" metalness={0.8} />
              </mesh>
              <mesh position={[0, 0, -0.16]} rotation={[Math.PI / 2, 0, 0]}>
                <circleGeometry args={[0.05, 12]} />
                <meshStandardMaterial color="#10B981" metalness={0.9} />
              </mesh>
            </group>

            {/* Co-Aligned Laser Collimator Transmitter Projector */}
            <group position={[-0.38, 0.15, -0.3]}>
              <mesh rotation={[Math.PI / 2, 0, 0]}>
                <cylinderGeometry args={[0.07, 0.07, 0.35, 12]} />
                <meshStandardMaterial color="#1E293B" metalness={0.8} />
              </mesh>
              <mesh position={[0, 0, -0.18]} rotation={[Math.PI / 2, 0, 0]}>
                <circleGeometry args={[0.06, 12]} />
                <meshStandardMaterial color="#06B6D4" emissive="#0284C7" emissiveIntensity={0.6} />
              </mesh>
            </group>
            
            {/* Virtual Camera Sensor Position (Essential for Image Stream) */}
            <PerspectiveCamera
              ref={cameraRef}
              fov={vfov}
              aspect={resolutionWidth / resolutionHeight}
              near={0.1}
              far={10000}
              position={[0, 0, -0.2]}
            />
            
            {/* Visual Tracking Optical Frustum Lines */}
            {frustumLines.map((points, index) => (
              <Line
                key={index}
                points={points.map(p => new THREE.Vector3(...p))}
                color={index === 0 ? "#EF4444" : "#0284C7"}
                lineWidth={index === 0 ? 1.5 : 0.8}
                opacity={index === 0 ? 0.8 : 0.25}
                transparent
              />
            ))}
          </group>
        </group>
      </group>
    </group>
  );
}
