import React from 'react';
import { useSimulationStore } from '../store/useSimulationStore';

export default function ControlPanel() {
  const {
    isPlaying,
    speed,
    gimbalPan,
    gimbalTilt,
    panMin,
    panMax,
    tiltMin,
    tiltMax,
    togglePlay,
    setSpeed,
    resetSimulation,
    setGimbalAngles,
    
    // BATCH 2 States
    activeDetectorMode,
    setDetectorMode,
    pidConfig,
    updatePidConfig,
    beaconActive
  } = useSimulationStore();

  const handlePanChange = (e) => {
    setGimbalAngles(parseFloat(e.target.value), gimbalTilt);
  };

  const handleTiltChange = (e) => {
    setGimbalAngles(gimbalPan, parseFloat(e.target.value));
  };

  const toggleBeacon = () => {
    useSimulationStore.setState({ beaconActive: !beaconActive });
  };

  return (
    <div className="w-full bg-white border border-slate-300 p-3 rounded-sm flex flex-col gap-3">
      <h3 className="text-xs font-semibold text-slate-800 uppercase tracking-wider font-mono">
        Simulation & Tracking Configuration
      </h3>
      
      {/* Playback Controls & Occlusion Simulator */}
      <div className="flex gap-2 items-center flex-wrap">
        <button
          onClick={togglePlay}
          className={`flex-1 min-w-[120px] flex items-center justify-center gap-1.5 py-1.5 rounded-sm font-semibold text-xs font-mono transition-colors ${
            isPlaying
              ? 'bg-amber-700 hover:bg-amber-800 text-white'
              : 'bg-emerald-700 hover:bg-emerald-800 text-white'
          }`}
        >
          {isPlaying ? '[PAUSE]' : '[RESUME]'}
        </button>
        
        {/* Simulate Occlusion Toggle */}
        <button
          onClick={toggleBeacon}
          className={`flex items-center gap-1.5 px-3 py-1.5 rounded-sm text-xs font-mono transition-colors font-semibold ${
            beaconActive 
              ? 'bg-slate-100 hover:bg-rose-50 text-slate-700 border border-slate-300' 
              : 'bg-rose-50 text-rose-800 border border-rose-500 animate-pulse'
          }`}
          title={beaconActive ? "Occlude Beacon" : "Restore Beacon"}
        >
          {beaconActive ? "[OCCLUDE]" : "[RESTORE]"}
        </button>

        <button
          onClick={resetSimulation}
          className="px-3 py-1.5 bg-slate-100 hover:bg-slate-200 text-slate-800 rounded-sm text-xs font-mono transition-colors font-semibold border border-slate-300"
          title="Reset Simulation"
        >
          [RESET]
        </button>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        {/* Speed Slider */}
        <div className="flex flex-col gap-1">
          <div className="flex justify-between text-xs font-mono text-slate-600">
            <span>Simulation Speed</span>
            <span className="text-cyan-700 font-bold">{speed.toFixed(1)}x</span>
          </div>
          <input
            type="range"
            min="0.1"
            max="3.0"
            step="0.1"
            value={speed}
            onChange={(e) => setSpeed(parseFloat(e.target.value))}
            className="w-full accent-cyan-600 h-1.5 bg-slate-200 rounded-sm appearance-none cursor-pointer"
          />
        </div>

        {/* Detector Mode Selector */}
        <div className="flex flex-col gap-1.5">
          <span className="text-xs font-mono text-slate-600 flex items-center gap-1">
            Detector Mode
          </span>
          <div className="flex bg-slate-100 p-1 rounded-sm border border-slate-300">
            {['opencv', 'yolo', 'hybrid'].map((mode) => (
              <button
                key={mode}
                onClick={() => setDetectorMode(mode)}
                className={`flex-1 text-[10px] py-1 rounded-sm font-mono font-semibold transition-colors ${
                  activeDetectorMode === mode
                    ? 'bg-cyan-700 text-white font-bold'
                    : 'text-slate-600 hover:text-slate-900'
                }`}
              >
                {mode.toUpperCase()}
              </button>
            ))}
          </div>
        </div>
      </div>

      <hr className="border-slate-300" />

      {/* PID Tuning Controls */}
      <div className="flex flex-col gap-2">
        <span className="text-xs font-semibold text-slate-800 uppercase tracking-wider font-mono flex items-center gap-1.5">
          PID Control Tuning
        </span>
        <div className="grid grid-cols-2 gap-4">
          {/* Pan PID */}
          <div className="bg-slate-50 p-3 rounded-sm border border-slate-300 flex flex-col gap-2">
            <div className="text-[10px] font-mono text-emerald-700 font-bold uppercase">
              Pan PID (Yaw Loop)
            </div>
            <div className="grid grid-cols-3 gap-2">
              <div className="flex flex-col gap-0.5">
                <label className="text-[9px] text-slate-500 font-mono">Kp</label>
                <input
                  type="number"
                  step="0.1"
                  value={pidConfig.pan.kp}
                  onChange={(e) => updatePidConfig('pan', 'kp', e.target.value)}
                  className="bg-white border border-slate-300 text-slate-900 text-xs px-1.5 py-1 rounded-sm text-center focus:outline-none focus:border-cyan-600"
                />
              </div>
              <div className="flex flex-col gap-0.5">
                <label className="text-[9px] text-slate-500 font-mono">Ki</label>
                <input
                  type="number"
                  step="0.01"
                  value={pidConfig.pan.ki}
                  onChange={(e) => updatePidConfig('pan', 'ki', e.target.value)}
                  className="bg-white border border-slate-300 text-slate-900 text-xs px-1.5 py-1 rounded-sm text-center focus:outline-none focus:border-cyan-600"
                />
              </div>
              <div className="flex flex-col gap-0.5">
                <label className="text-[9px] text-slate-500 font-mono">Kd</label>
                <input
                  type="number"
                  step="0.05"
                  value={pidConfig.pan.kd}
                  onChange={(e) => updatePidConfig('pan', 'kd', e.target.value)}
                  className="bg-white border border-slate-300 text-slate-900 text-xs px-1.5 py-1 rounded-sm text-center focus:outline-none focus:border-cyan-600"
                />
              </div>
            </div>
          </div>

          {/* Tilt PID */}
          <div className="bg-slate-50 p-3 rounded-sm border border-slate-300 flex flex-col gap-2">
            <div className="text-[10px] font-mono text-emerald-700 font-bold uppercase">
              Tilt PID (Pitch Loop)
            </div>
            <div className="grid grid-cols-3 gap-2">
              <div className="flex flex-col gap-0.5">
                <label className="text-[9px] text-slate-500 font-mono">Kp</label>
                <input
                  type="number"
                  step="0.1"
                  value={pidConfig.tilt.kp}
                  onChange={(e) => updatePidConfig('tilt', 'kp', e.target.value)}
                  className="bg-white border border-slate-300 text-slate-900 text-xs px-1.5 py-1 rounded-sm text-center focus:outline-none focus:border-cyan-600"
                />
              </div>
              <div className="flex flex-col gap-0.5">
                <label className="text-[9px] text-slate-500 font-mono">Ki</label>
                <input
                  type="number"
                  step="0.01"
                  value={pidConfig.tilt.ki}
                  onChange={(e) => updatePidConfig('tilt', 'ki', e.target.value)}
                  className="bg-white border border-slate-300 text-slate-900 text-xs px-1.5 py-1 rounded-sm text-center focus:outline-none focus:border-cyan-600"
                />
              </div>
              <div className="flex flex-col gap-0.5">
                <label className="text-[9px] text-slate-500 font-mono">Kd</label>
                <input
                  type="number"
                  step="0.05"
                  value={pidConfig.tilt.kd}
                  onChange={(e) => updatePidConfig('tilt', 'kd', e.target.value)}
                  className="bg-white border border-slate-300 text-slate-900 text-xs px-1.5 py-1 rounded-sm text-center focus:outline-none focus:border-cyan-600"
                />
              </div>
            </div>
          </div>
        </div>
      </div>

      <hr className="border-slate-300" />

      {/* Manual Gimbal Jog Controls */}
      <div className="flex flex-col gap-2">
        <span className="text-[10px] font-semibold text-slate-500 uppercase font-mono text-center">
          Manual Gimbal Jog (Yaw/Pitch)
        </span>
        <div className="flex justify-between items-center text-xs">
          {/* Pan Slider */}
          <div className="flex-1 flex flex-col gap-1 pr-4 border-r border-slate-300">
            <div className="flex justify-between font-mono text-[10px] text-slate-600">
              <span>Pan Jog</span>
              <span className="text-slate-900 font-semibold">{gimbalPan.toFixed(1)}°</span>
            </div>
            <input
              type="range"
              min={panMin}
              max={panMax}
              step="0.5"
              value={gimbalPan}
              onChange={handlePanChange}
              className="w-full accent-cyan-600 h-1 bg-slate-200 rounded-none appearance-none cursor-pointer"
            />
          </div>

          {/* Tilt Slider */}
          <div className="flex-1 flex flex-col gap-1 pl-4">
            <div className="flex justify-between font-mono text-[10px] text-slate-600">
              <span>Tilt Jog</span>
              <span className="text-slate-900 font-semibold">{gimbalTilt.toFixed(1)}°</span>
            </div>
            <input
              type="range"
              min={tiltMin}
              max={tiltMax}
              step="0.5"
              value={gimbalTilt}
              onChange={handleTiltChange}
              className="w-full accent-cyan-600 h-1 bg-slate-200 rounded-none appearance-none cursor-pointer"
            />
          </div>
        </div>
      </div>
    </div>
  );
}
