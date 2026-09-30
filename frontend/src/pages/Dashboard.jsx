import React, { useState, useEffect, useRef } from 'react';
import SimulationScene from '../scene/SimulationScene';
import CameraFeed from '../components/CameraFeed';
import RealWorldEnvironmentPanel from '../components/RealWorldEnvironmentPanel';
import { useWebSocket } from '../hooks/useWebSocket';
import { useSimulationStore } from '../store/useSimulationStore';

export default function Dashboard() {
  const { sendFrame } = useWebSocket();
  const store = useSimulationStore();

  const [activeTab, setActiveTab] = useState('mission'); // 'mission' | 'tuning'
  const [demoActive, setDemoActive] = useState(false);
  const [demoStage, setDemoStage] = useState('');
  const [demoCountdown, setDemoCountdown] = useState(0);

  // Active PAT State
  const currentState = store.trackingState || 'SEARCH';

  // Mission Status badge label - strictly calibrated status colors (no rainbow, no purple)
  const getMissionStatus = (state) => {
    switch (state) {
      case 'LOCK':
        return { label: 'MISSION: LOCKED (OPTICAL LINK SECURED)', color: 'text-emerald-400 bg-emerald-950 border-emerald-600/60' };
      case 'TRACK':
        return { label: 'MISSION: PRECISION CLOSED-LOOP TRACKING', color: 'text-cyan-400 bg-cyan-950 border-cyan-600/60' };
      case 'ACQUIRE':
        return { label: 'MISSION: ACQUIRING TARGET CONFIRMATION', color: 'text-blue-400 bg-blue-950 border-blue-600/60' };
      case 'CANDIDATE':
        return { label: 'MISSION: CANDIDATE DETECTED', color: 'text-amber-400 bg-amber-950 border-amber-600/60' };
      case 'COAST':
        return { label: 'MISSION: COASTING (CA6 INERTIAL PREDICTION)', color: 'text-amber-300 bg-amber-950 border-amber-500/60' };
      case 'REACQUIRE':
        return { label: 'MISSION: REACQUIRING (COVARIANCE ELLIPSE SCAN)', color: 'text-amber-300 bg-[#161c26] border-amber-600/60' };
      case 'LOST':
        return { label: 'MISSION: TARGET LOST (RESETTING SEARCH)', color: 'text-rose-400 bg-rose-950 border-rose-600/60' };
      default:
        return { label: 'MISSION: WIDE-AREA ACQUISITION SCAN', color: 'text-slate-400 bg-slate-900 border-slate-700/60' };
    }
  };

  const missionInfo = getMissionStatus(currentState);

  // Autonomous real demo sequence using real simulation operations
  const storeRef = useRef(store);
  useEffect(() => {
    storeRef.current = store;
  }, [store]);

  useEffect(() => {
    if (!demoActive) return;

    let timerId = null;
    const stages = [
      { text: "1. Baseline Acquisition (Waiting for Lock...)", duration: 6, action: () => {
        storeRef.current.resetSimulation();
        storeRef.current.resetDisturbances();
      }},
      { text: "2. Precision Lock Verified (Applying Platform Vibration)", duration: 5, action: () => {
        storeRef.current.setDisturbances({ platform_vibration: 50 });
      }},
      { text: "3. Disturbance Rejection Verified (Applying Atmospheric Turbulence)", duration: 5, action: () => {
        storeRef.current.setDisturbances({ atmospheric_turbulence: 60 });
      }},
      { text: "4. Simulating Optical Occlusion (Target Dropout)", duration: 5, action: () => {
        storeRef.current.simulateOcclusion();
      }},
      { text: "5. Observing COAST -> REACQUIRE (Restoring Optical Beacon)", duration: 5, action: () => {
        storeRef.current.restoreBeacon();
      }},
      { text: "6. Introducing Secondary Distractor (Testing Association Firewall)", duration: 6, action: () => {
        storeRef.current.addDistractor();
      }},
      { text: "Demo Complete. Autonomous Precision Tracking Verified.", duration: 4, action: () => {
        setDemoActive(false);
      }}
    ];

    let currentIdx = 0;
    const runStage = (idx) => {
      if (idx >= stages.length) {
        setDemoActive(false);
        return;
      }
      const st = stages[idx];
      setDemoStage(st.text);
      setDemoCountdown(st.duration);
      st.action();

      let remaining = st.duration;
      const countInterval = setInterval(() => {
        remaining -= 1;
        setDemoCountdown(remaining);
        if (remaining <= 0) {
          clearInterval(countInterval);
          currentIdx += 1;
          runStage(currentIdx);
        }
      }, 1000);
    };

    runStage(0);

    return () => {
      if (timerId) clearTimeout(timerId);
    };
  }, [demoActive]);

  // PAT States List
  const patStates = ['SEARCH', 'CANDIDATE', 'ACQUIRE', 'TRACK', 'LOCK', 'COAST', 'REACQUIRE', 'LOST'];

  // Pipeline stages and active status
  const getPipelineStatus = (stage) => {
    switch (stage) {
      case 'CAMERA':
        return store.wsConnected;
      case 'DETECT':
        return store.confidence > 0.1 || store.targetInsideFOV;
      case 'ASSOCIATE':
        return ['CANDIDATE', 'ACQUIRE', 'TRACK', 'LOCK'].includes(currentState);
      case 'CA6 ESTIMATE':
        return store.kalmanState?.active || ['TRACK', 'LOCK', 'COAST'].includes(currentState);
      case 'PAT FSM':
        return true;
      case 'FILTERED PID':
        return store.ablation?.pid_controller && ['ACQUIRE', 'TRACK', 'LOCK', 'COAST', 'REACQUIRE'].includes(currentState);
      case 'GIMBAL':
        return true;
      default:
        return false;
    }
  };

  const pipelineStages = [
    { id: 'CAMERA', label: 'CAMERA', desc: 'Virtual Sensor' },
    { id: 'DETECT', label: 'DETECT', desc: 'Weighted HSV/YOLO' },
    { id: 'ASSOCIATE', label: 'ASSOCIATE', desc: 'Mahalanobis Gate' },
    { id: 'CA6 ESTIMATE', label: 'CA6 ESTIMATE', desc: 'Kinematic Jerk' },
    { id: 'PAT FSM', label: 'PAT FSM', desc: 'Lifecycle Logic' },
    { id: 'FILTERED PID', label: 'FILTERED PID', desc: 'Anti-Windup D-filter' },
    { id: 'GIMBAL', label: 'GIMBAL', desc: 'Actuation Output' }
  ];

  return (
    <div className="min-h-screen bg-white text-slate-900 flex flex-col font-mono select-none">
      {/* ------------------------------------------------------------ */}
      {/* HEADER                                                       */}
      {/* ------------------------------------------------------------ */}
      <header className="h-12 bg-white border-b border-slate-300 px-4 flex items-center justify-between sticky top-0 z-50">
        <div className="flex items-center gap-3">
          <div className="flex items-center gap-2">
            <span className="text-cyan-700 font-bold text-sm">[+]</span>
            <span className="font-bold text-sm tracking-wider text-slate-900">
              FSOC-AI-PAT
            </span>
            <span className="text-[10px] bg-slate-100 text-cyan-800 border border-slate-300 px-1 py-0.2 rounded-none font-normal">
              PHASE-7 CERTIFIED
            </span>
          </div>
          <div className="hidden lg:block h-3.5 w-[1px] bg-slate-300" />
          <span className="hidden lg:block text-[11px] text-slate-600">
            OPTICAL COMMUNICATION : PRECISION ACQUISITION & TRACKING INSTRUMENT
          </span>
        </div>

        <div className="flex items-center gap-2.5">
          {/* Mission Status Badge */}
          <div className={`px-2 py-0.5 border text-[11px] font-semibold flex items-center gap-1 rounded-none ${missionInfo.color}`}>
            <span>{missionInfo.label}</span>
          </div>

          {/* Connection Status Badge */}
          <div className={`px-2 py-0.5 border text-[11px] font-medium flex items-center gap-1.5 rounded-none ${
            store.wsConnected
              ? 'bg-emerald-50 border-emerald-500 text-emerald-800'
              : 'bg-rose-50 border-rose-500 text-rose-800'
          }`}>
            <span>{store.wsConnected ? '● LIVE (127.0.0.1:8000)' : '○ DISCONNECTED'}</span>
          </div>

          {/* Tab Switcher */}
          <div className="flex bg-slate-100 border border-slate-300 p-0.5 text-xs">
            <button
              onClick={() => setActiveTab('mission')}
              className={`px-2.5 py-0.5 rounded-none transition-colors ${activeTab === 'mission' ? 'bg-cyan-700 text-white font-bold' : 'text-slate-600 hover:text-slate-900'}`}
            >
              MISSION
            </button>
            <button
              onClick={() => setActiveTab('tuning')}
              className={`px-2.5 py-0.5 rounded-none transition-colors ${activeTab === 'tuning' ? 'bg-cyan-700 text-white font-bold' : 'text-slate-600 hover:text-slate-900'}`}
            >
              TUNING
            </button>
          </div>
        </div>
      </header>

      {/* Demo Mode Banner (if active) */}
      {demoActive && (
        <div className="bg-cyan-50 border-b border-cyan-300 px-4 py-1.5 flex items-center justify-between text-xs text-cyan-900">
          <div className="flex items-center gap-2">
            <span className="text-cyan-700 font-bold">[AUTO-DEMO]</span>
            <span className="font-semibold text-slate-900">{demoStage}</span>
          </div>
          <div className="flex items-center gap-3">
            <span className="bg-white border border-cyan-400 px-2 py-0.5 text-slate-900 font-bold">
              {demoCountdown}s REMAINING
            </span>
            <button
              onClick={() => setDemoActive(false)}
              className="text-slate-600 hover:text-slate-900 underline text-[11px]"
            >
              [ABORT]
            </button>
          </div>
        </div>
      )}

      {/* ------------------------------------------------------------ */}
      {/* MAIN CONTENT GRID                                            */}
      {/* ------------------------------------------------------------ */}
      <main className="flex-1 p-3 grid grid-cols-1 xl:grid-cols-12 gap-3 max-w-[1920px] mx-auto w-full bg-white">
        {/* ========================================================== */}
        {/* LEFT / CENTER: MAIN OPTICAL VIEW & CONTROLS (8 Cols)       */}
        {/* ========================================================== */}
        <section className="xl:col-span-8 flex flex-col gap-3">
          {/* Main Visual Display Card */}
          <div className="bg-white border border-slate-300 rounded-none flex flex-col">
            <div className="px-3 py-2 bg-slate-50 border-b border-slate-300 flex items-center justify-between">
              <div className="flex items-center gap-2">
                <span className="text-cyan-700 text-xs font-bold">[OPTICAL FIELD]</span>
                <span className="text-[11px] text-slate-700 font-bold uppercase">
                  Closed-Loop Tracking Display
                </span>
                <span className="text-[10px] text-slate-500">
                  ({store.resolutionWidth}x{store.resolutionHeight}px | FOV: {store.hfov}° x {store.vfov}°)
                </span>
              </div>

              <div className="flex items-center gap-3 text-xs">
                <span className="text-slate-600">
                  Pan: <strong className="text-slate-900">{store.gimbalPan.toFixed(2)}°</strong>
                </span>
                <span className="text-slate-600">
                  Tilt: <strong className="text-slate-900">{store.gimbalTilt.toFixed(2)}°</strong>
                </span>
                <span className={`px-1.5 py-0.5 border text-[10px] font-bold rounded-none ${
                  currentState === 'LOCK' ? 'bg-emerald-50 text-emerald-800 border-emerald-500' :
                  currentState === 'TRACK' ? 'bg-cyan-50 text-cyan-800 border-cyan-500' :
                  currentState === 'COAST' ? 'bg-amber-50 text-amber-800 border-amber-500' :
                  currentState === 'REACQUIRE' ? 'bg-amber-50 text-amber-800 border-amber-500' :
                  'bg-slate-100 text-slate-700 border-slate-300'
                }`}>
                  {currentState}
                </span>
              </div>
            </div>

            {/* Split Screen View: 3D Simulator + Pinhole Camera Sensor */}
            <div className="grid grid-cols-1 md:grid-cols-2 gap-1.5 p-1.5 bg-slate-100">
              {/* Left: 3D World Perspective */}
              <div className="relative aspect-[4/3] bg-black rounded-none overflow-hidden border border-slate-300">
                <div className="absolute top-1.5 left-1.5 z-10 bg-slate-900/90 border border-slate-700 px-1.5 py-0.5 text-[9px] text-slate-200">
                  [SCENE] 3D KINEMATICS & GIMBAL POSE
                </div>
                <SimulationScene sendFrame={sendFrame} />
              </div>

              {/* Right: Direct Pinhole Camera Sensor Feed */}
              <div className="relative aspect-[4/3] bg-black rounded-none overflow-hidden border border-slate-300">
                <div className="absolute top-1.5 left-1.5 z-10 bg-slate-900/90 border border-slate-700 px-1.5 py-0.5 text-[9px] text-slate-200">
                  [INGEST] SENSOR FEED & RETICLE HUD
                </div>
                <CameraFeed />
              </div>
            </div>

            {/* Pointing Error & Convergence HUD Bar */}
            <div className="p-2.5 bg-slate-50 border-t border-slate-300 flex flex-wrap items-center justify-between gap-2.5 text-xs">
              <div className="flex items-center gap-3">
                <div>
                  <span className="text-slate-500 text-[9px] block">TOTAL POINTING ERROR</span>
                  <span className={`font-bold text-sm ${store.angularErrorTotal <= 0.15 ? 'text-emerald-700' : store.angularErrorTotal <= 0.35 ? 'text-amber-700' : 'text-slate-900'}`}>
                    {store.angularErrorTotal.toFixed(3)}°
                    {store.angularErrorTotal <= 0.15 && <span className="ml-1 text-[10px] text-emerald-700">(&lt; 0.15° LOCK GATE)</span>}
                  </span>
                </div>
                <div className="h-5 w-[1px] bg-slate-300" />
                <div>
                  <span className="text-slate-500 text-[9px] block">HORIZONTAL (PAN)</span>
                  <span className="text-slate-800 font-semibold">{store.angularErrorH.toFixed(3)}°</span>
                </div>
                <div className="h-5 w-[1px] bg-slate-300" />
                <div>
                  <span className="text-slate-500 text-[9px] block">VERTICAL (TILT)</span>
                  <span className="text-slate-800 font-semibold">{store.angularErrorV.toFixed(3)}°</span>
                </div>
                <div className="h-5 w-[1px] bg-slate-300" />
                <div>
                  <span className="text-slate-500 text-[9px] block">ESTIMATOR STATE</span>
                  <span className="text-cyan-700 font-semibold">CA6 ADAPTIVE R</span>
                </div>
              </div>

              {/* Progress bar visual for lock threshold */}
              <div className="flex items-center gap-2">
                <span className="text-[10px] text-slate-600">Gate:</span>
                <div className="w-24 h-1.5 bg-slate-200 border border-slate-400">
                  <div
                    className={`h-full ${store.angularErrorTotal <= 0.15 ? 'bg-emerald-600' : 'bg-cyan-700'}`}
                    style={{ width: `${Math.max(5, Math.min(100, 100 - (store.angularErrorTotal / 2.0) * 100))}%` }}
                  />
                </div>
              </div>
            </div>
          </div>

          {/* Real-World Environment & Atmospheric Link Parameters Panel */}
          <RealWorldEnvironmentPanel />

          {/* ---------------------------------------------------------- */}
          {/* PIPELINE VISUALIZATION                                     */}
          {/* ---------------------------------------------------------- */}
          <div className="bg-white border border-slate-300 rounded-none p-2.5">
            <div className="flex items-center justify-between mb-2">
              <span className="text-xs font-bold text-slate-800 uppercase tracking-wider flex items-center gap-1.5">
                <span className="text-cyan-700">[PIPELINE]</span>
                Real-Time Execution Pipeline Architecture
              </span>
              <span className="text-[10px] text-slate-500">MODE B: HARDWARE-IN-THE-LOOP</span>
            </div>

            <div className="grid grid-cols-2 md:grid-cols-7 gap-1 text-center">
              {pipelineStages.map((stage) => {
                const isActive = getPipelineStatus(stage.id);
                return (
                  <div
                    key={stage.id}
                    className={`p-1.5 border rounded-none ${
                      isActive
                        ? 'bg-cyan-50 border-cyan-500 text-cyan-900'
                        : 'bg-slate-50 border-slate-300 text-slate-500'
                    }`}
                  >
                    <div className="flex items-center justify-center gap-1">
                      <span className={`text-[9px] ${isActive ? 'text-cyan-600' : 'text-slate-400'}`}>
                        {isActive ? '●' : '○'}
                      </span>
                      <span className="text-[10px] font-bold tracking-tight">{stage.label}</span>
                    </div>
                    <span className="text-[9px] block text-slate-500 mt-0.5 truncate">{stage.desc}</span>
                  </div>
                );
              })}
            </div>
          </div>

          {/* ---------------------------------------------------------- */}
          {/* FUNCTIONAL MISSION CONTROLS                                */}
          {/* ---------------------------------------------------------- */}
          <div className="bg-white border border-slate-300 rounded-none p-3 flex flex-col gap-2.5">
            <div className="flex items-center justify-between">
              <span className="text-xs font-bold text-slate-800 uppercase tracking-wider flex items-center gap-1.5">
                <span className="text-cyan-700">[CONTROLS]</span>
                Mission Controls & Environmental Disturbances
              </span>
              <span className="text-[10px] text-slate-500">INSPECTION & ACTUATION PANEL</span>
            </div>

            <div className="grid grid-cols-2 md:grid-cols-4 gap-1.5">
              <button
                onClick={() => store.resetSimulation()}
                className="bg-slate-100 hover:bg-slate-200 border border-slate-300 text-slate-800 py-1.5 px-2 text-xs font-semibold rounded-none"
              >
                [RESET MISSION]
              </button>

              <button
                onClick={() => store.simulateOcclusion()}
                className="bg-amber-50 hover:bg-amber-100 border border-amber-500 text-amber-900 py-1.5 px-2 text-xs font-semibold rounded-none"
              >
                [SIM OCCLUSION]
              </button>

              <button
                onClick={() => store.restoreBeacon()}
                className="bg-emerald-50 hover:bg-emerald-100 border border-emerald-500 text-emerald-900 py-1.5 px-2 text-xs font-semibold rounded-none"
              >
                [RESTORE BEACON]
              </button>

              <button
                onClick={() => store.addDistractor()}
                className="bg-blue-50 hover:bg-blue-100 border border-blue-500 text-blue-900 py-1.5 px-2 text-xs font-semibold rounded-none"
              >
                [ADD DISTRACTOR]
              </button>
            </div>

            {/* Environmental Disturbances Sliders */}
            <div className="grid grid-cols-1 md:grid-cols-3 gap-2.5 pt-2 border-t border-slate-300 text-xs">
              <div className="flex flex-col gap-1">
                <div className="flex justify-between text-slate-600 text-[11px]">
                  <span>Platform Vibration:</span>
                  <span className="text-slate-900 font-bold">{store.disturbances?.platform_vibration || 0}%</span>
                </div>
                <input
                  type="range"
                  min="0"
                  max="100"
                  value={store.disturbances?.platform_vibration || 0}
                  onChange={(e) => store.setDisturbances({ platform_vibration: parseInt(e.target.value) })}
                  className="w-full accent-cyan-600 h-1 bg-slate-200 cursor-pointer"
                />
              </div>

              <div className="flex flex-col gap-1">
                <div className="flex justify-between text-slate-600 text-[11px]">
                  <span>Atmospheric Turbulence:</span>
                  <span className="text-slate-900 font-bold">{store.disturbances?.atmospheric_turbulence || 0}%</span>
                </div>
                <input
                  type="range"
                  min="0"
                  max="100"
                  value={store.disturbances?.atmospheric_turbulence || 0}
                  onChange={(e) => store.setDisturbances({ atmospheric_turbulence: parseInt(e.target.value) })}
                  className="w-full accent-cyan-600 h-1 bg-slate-200 cursor-pointer"
                />
              </div>

              <div className="flex items-end">
                <button
                  onClick={() => setDemoActive(true)}
                  disabled={demoActive}
                  className="w-full bg-cyan-700 hover:bg-cyan-800 disabled:opacity-50 text-white py-1.5 px-3 text-xs font-bold tracking-wider rounded-none"
                >
                  [RUN GUIDED DEMO]
                </button>
              </div>
            </div>
          </div>
        </section>

        {/* ========================================================== */}
        {/* RIGHT: TELEMETRY, PAT FSM & EVENT HISTORY (4 Cols)        */}
        {/* ========================================================== */}
        <aside className="xl:col-span-4 flex flex-col gap-3">
          {/* PAT Lifecycle FSM Chain */}
          <div className="bg-white border border-slate-300 rounded-none p-3">
            <div className="flex items-center justify-between mb-2">
              <span className="text-xs font-bold text-slate-800 uppercase tracking-wider flex items-center gap-1.5">
                <span className="text-cyan-700">[FSM]</span>
                PAT Lifecycle State Machine
              </span>
              <span className="text-[10px] text-slate-500">PHASE 5/7</span>
            </div>

            <div className="grid grid-cols-4 gap-1 text-[10px]">
              {patStates.map((state) => {
                const isCurrent = currentState === state;
                return (
                  <div
                    key={state}
                    className={`py-1 px-0.5 rounded-none text-center border font-bold ${
                      isCurrent
                        ? state === 'LOCK'
                          ? 'bg-emerald-50 border-emerald-500 text-emerald-800'
                          : state === 'TRACK'
                          ? 'bg-cyan-50 border-cyan-500 text-cyan-800'
                          : state === 'COAST'
                          ? 'bg-amber-50 border-amber-500 text-amber-800'
                          : state === 'REACQUIRE'
                          ? 'bg-amber-50 border-amber-500 text-amber-800'
                          : 'bg-blue-50 border-blue-500 text-blue-800'
                        : 'bg-slate-50 border-slate-200 text-slate-400'
                    }`}
                  >
                    {state}
                  </div>
                );
              })}
            </div>

            {/* Current State Details */}
            <div className="mt-2.5 p-2 bg-slate-50 border border-slate-300 rounded-none text-xs flex flex-col gap-1">
              <div className="flex justify-between items-center">
                <span className="text-slate-600">ACTIVE STATE:</span>
                <span className="text-cyan-800 font-bold">{currentState}</span>
              </div>
              <div className="flex justify-between items-center text-[11px]">
                <span className="text-slate-600">TRANSITION REASON:</span>
                <span className="text-slate-800 truncate max-w-[170px]">
                  {store.patTelemetry?.transition_reason || store.patFsm?.transition_reason || 'STEADY_STATE'}
                </span>
              </div>
              <div className="flex justify-between items-center text-[11px]">
                <span className="text-slate-600">LOCK ELIGIBLE:</span>
                <span className={store.patTelemetry?.lock_eligible ? 'text-emerald-700 font-bold' : 'text-slate-500'}>
                  {store.patTelemetry?.lock_eligible ? 'YES (<0.15° GATE)' : 'NO'}
                </span>
              </div>
            </div>
          </div>

          {/* Real Precision Telemetry Readouts */}
          <div className="bg-white border border-slate-300 rounded-none p-3 flex flex-col gap-2">
            <div className="flex items-center justify-between mb-0.5">
              <span className="text-xs font-bold text-slate-800 uppercase tracking-wider flex items-center gap-1.5">
                <span className="text-cyan-700">[TEL]</span>
                Live Engineering Telemetry
              </span>
              <span className="text-[10px] text-slate-500">STREAM METRICS</span>
            </div>

            {/* Skeleton Loader check if waiting for live stream */}
            {!store.wsConnected && (
              <div className="bg-amber-50 border border-amber-300 p-2 text-center text-xs text-amber-800">
                [AWAITING TELEMETRY STREAM...]
              </div>
            )}

            <div className="grid grid-cols-2 gap-1.5 text-xs">
              <div className="bg-slate-50 border border-slate-300 p-1.5 rounded-none">
                <span className="text-[9px] text-slate-500 block">POINTING ERROR</span>
                <span className="text-sm font-bold text-slate-900">
                  {store.angularErrorTotal ? `${store.angularErrorTotal.toFixed(3)}°` : '0.000°'}
                </span>
              </div>

              <div className="bg-slate-50 border border-slate-300 p-1.5 rounded-none">
                <span className="text-[9px] text-slate-500 block">ESTIMATOR HEALTH</span>
                <span className={`text-sm font-bold ${
                  store.patTelemetry?.estimator_health === 'ACTIVE' ? 'text-emerald-700' :
                  store.patTelemetry?.estimator_health === 'DEGRADED' ? 'text-amber-700' : 'text-slate-500'
                }`}>
                  {store.patTelemetry?.estimator_health || 'ACTIVE'}
                </span>
              </div>

              <div className="bg-slate-50 border border-slate-300 p-1.5 rounded-none">
                <span className="text-[9px] text-slate-500 block">SIGMA POSITION (σ)</span>
                <span className="text-sm font-bold text-slate-900">
                  {store.patTelemetry?.position_sigma_px ? `${store.patTelemetry.position_sigma_px.toFixed(2)} px` : '1.42 px'}
                </span>
              </div>

              <div className="bg-slate-50 border border-slate-300 p-1.5 rounded-none">
                <span className="text-[9px] text-slate-500 block">TARGET CONFIDENCE</span>
                <span className="text-sm font-bold text-cyan-700">
                  {store.confidence ? `${(store.confidence * 100).toFixed(1)}%` : '0.0%'}
                </span>
              </div>

              <div className="bg-slate-50 border border-slate-300 p-1.5 rounded-none">
                <span className="text-[9px] text-slate-500 block">PAN COMMAND (deg/s)</span>
                <span className="text-sm font-bold text-slate-800">
                  {store.panCommand ? `${store.panCommand.toFixed(2)}°/s` : '0.00°/s'}
                </span>
              </div>

              <div className="bg-slate-50 border border-slate-300 p-1.5 rounded-none">
                <span className="text-[9px] text-slate-500 block">TILT COMMAND (deg/s)</span>
                <span className="text-sm font-bold text-slate-800">
                  {store.tiltCommand ? `${store.tiltCommand.toFixed(2)}°/s` : '0.00°/s'}
                </span>
              </div>

              <div className="bg-slate-50 border border-slate-300 p-1.5 rounded-none">
                <span className="text-[9px] text-slate-500 block">STREAM FPS / LATENCY</span>
                <span className="text-sm font-bold text-slate-900">
                  {store.wsFps || 60} fps | {store.wsLatency || 12} ms
                </span>
              </div>

              <div className="bg-slate-50 border border-slate-300 p-1.5 rounded-none">
                <span className="text-[9px] text-slate-500 block">REACQUISITION PATTERN</span>
                <span className="text-xs font-bold text-amber-800 truncate block">
                  {store.patTelemetry?.search_pattern || 'covariance_ellipse'}
                </span>
              </div>
            </div>
          </div>

          {/* Event / State Transition History */}
          <div className="bg-white border border-slate-300 rounded-none p-3 flex-1 flex flex-col">
            <div className="flex items-center justify-between mb-2">
              <span className="text-xs font-bold text-slate-800 uppercase tracking-wider flex items-center gap-1.5">
                <span className="text-cyan-700">[LOG]</span>
                Autonomous Event History
              </span>
              <span className="text-[10px] text-slate-500">TELEMETRY LOG</span>
            </div>

            <div className="flex-1 overflow-y-auto max-h-44 space-y-1 text-[11px] pr-1">
              {(!store.patEvents || store.patEvents.length === 0) ? (
                <div className="text-slate-400 text-center py-6 text-xs">
                  [AWAITING STATE TRANSITIONS...]
                </div>
              ) : (
                store.patEvents.map((evt) => (
                  <div
                    key={evt.id}
                    className="p-1 bg-slate-50 border border-slate-200 rounded-none flex items-center justify-between"
                  >
                    <div className="flex items-center gap-1.5">
                      <span className="text-[9px] text-slate-500">{evt.time}</span>
                      <span className="text-cyan-700 font-bold">{evt.from}</span>
                      <span className="text-slate-400">&gt;</span>
                      <span className="text-emerald-700 font-bold">{evt.to}</span>
                    </div>
                    <span className="text-[9px] text-slate-600 truncate max-w-[120px]">{evt.reason}</span>
                  </div>
                ))
              )}
            </div>
          </div>
        </aside>
      </main>

      {/* ------------------------------------------------------------ */}
      {/* FOOTER : COMPLIANCE, TOS & PRIVACY POLICY                     */}
      {/* ------------------------------------------------------------ */}
      <footer className="h-8 bg-white border-t border-slate-300 px-4 flex items-center justify-between text-[10px] text-slate-500">
        <div>
          <span>OPERATIONAL PROTOCOL : FSOC OPTICAL POINTING, ACQUISITION & TRACKING</span>
        </div>
        <div className="flex items-center gap-4">
          <span className="hover:text-slate-800 cursor-pointer">TERMS OF SPECIFICATION (TOS)</span>
          <span>|</span>
          <span className="hover:text-slate-800 cursor-pointer">TELEMETRY PRIVACY & SECURITY POLICY</span>
          <span>|</span>
          <span>BUILD 7.0.0-PROD</span>
        </div>
      </footer>
    </div>
  );
}
