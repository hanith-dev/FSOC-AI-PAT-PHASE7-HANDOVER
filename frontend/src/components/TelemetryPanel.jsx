import React from 'react';
import { useSimulationStore } from '../store/useSimulationStore';

export default function TelemetryPanel() {
  const {
    beaconPosition,
    gimbalPan,
    gimbalTilt,
    targetInsideFOV,
    pixelErrorX,
    pixelErrorY,
    angularErrorTotal,
    angularErrorH,
    angularErrorV,
    wsConnected,
    wsFps,
    wsBytesReceived,
    wsAcksCount,
    wsLatency,
    trackingState,
    activeDetectorMode,
    inferenceTime
  } = useSimulationStore();

  const [bx, by, bz] = beaconPosition;

  // Determine state machine style
  const getStateStyle = (state) => {
    switch (state) {
      case 'LOCKED':
      case 'LOCK':
        return 'bg-emerald-50 text-emerald-800 border-emerald-500';
      case 'TRACKING':
      case 'TRACK':
        return 'bg-cyan-50 text-cyan-800 border-cyan-500';
      case 'ACQUIRING':
      case 'ACQUIRE':
        return 'bg-blue-50 text-blue-800 border-blue-500';
      case 'DETECTED':
      case 'CANDIDATE':
        return 'bg-amber-50 text-amber-800 border-amber-500';
      case 'COAST':
        return 'bg-amber-50 text-amber-800 border-amber-500 animate-pulse';
      case 'REACQUIRE':
        return 'bg-amber-50 text-amber-800 border-amber-500 animate-pulse';
      case 'TARGET_LOST':
      case 'LOST':
        return 'bg-rose-50 text-rose-800 border-rose-500 font-bold';
      case 'SEARCHING':
      case 'SEARCH':
      default:
        return 'bg-slate-50 text-slate-700 border-slate-300';
    }
  };

  return (
    <div className="w-full bg-white border border-slate-300 p-3 rounded-sm flex flex-col gap-3">
      {/* Header with Connection Status */}
      <div className="flex justify-between items-center">
        <h3 className="text-xs font-semibold text-slate-800 uppercase tracking-wider font-mono">
          System Telemetry
        </h3>
        
        <div className={`flex items-center gap-1.5 px-2 py-0.5 rounded-sm text-[10px] font-mono font-semibold ${
          wsConnected
            ? 'bg-emerald-50 text-emerald-800 border border-emerald-500'
            : 'bg-rose-50 text-rose-800 border border-rose-500'
        }`}>
          <span>{wsConnected ? '● WS STREAMING' : '○ WS OFFLINE'}</span>
        </div>
      </div>

      {/* State Machine Status */}
      <div className={`flex flex-col gap-1 p-2.5 rounded-sm border ${getStateStyle(trackingState)}`}>
        <div className="flex justify-between items-center">
          <span className="text-[10px] font-mono font-bold uppercase tracking-wider opacity-85">Alignment State</span>
          <span className="text-xs font-mono font-semibold opacity-75">
            {targetInsideFOV ? 'IN FOV' : 'OUT FOV'}
          </span>
        </div>
        <div className="text-base font-bold font-mono tracking-wide mt-0.5">
          {trackingState}
        </div>
      </div>

      {/* Numerical Data Grid */}
      <div className="grid grid-cols-2 gap-2 text-xs">
        {/* Beacon coordinates */}
        <div className="bg-slate-50 p-2 rounded-sm border border-slate-300 col-span-2">
          <div className="text-slate-500 font-mono text-[10px] mb-1 uppercase">Beacon Position (World)</div>
          <div className="grid grid-cols-3 text-center font-mono font-semibold text-slate-800 text-xs">
            <div>X: <span className="text-cyan-700">{bx.toFixed(2)}m</span></div>
            <div>Y: <span className="text-cyan-700">{by.toFixed(2)}m</span></div>
            <div>Z: <span className="text-cyan-700">{bz.toFixed(2)}m</span></div>
          </div>
        </div>

        {/* Gimbal orientation */}
        <div className="bg-slate-50 p-2 rounded-sm border border-slate-300">
          <div className="text-slate-500 font-mono text-[10px] mb-1 uppercase">Gimbal Pan (Yaw)</div>
          <div className="font-mono text-sm font-bold text-slate-900">
            {gimbalPan.toFixed(2)}°
          </div>
        </div>

        <div className="bg-slate-50 p-2 rounded-sm border border-slate-300">
          <div className="text-slate-500 font-mono text-[10px] mb-1 uppercase">Gimbal Tilt (Pitch)</div>
          <div className="font-mono text-sm font-bold text-slate-900">
            {gimbalTilt.toFixed(2)}°
          </div>
        </div>

        {/* Angular Error details */}
        <div className="bg-slate-50 p-2 rounded-sm border border-slate-300 col-span-2">
          <div className="text-slate-500 font-mono text-[10px] mb-1 uppercase">Tracking Angular Errors</div>
          <div className="grid grid-cols-3 gap-1 font-mono text-xs mt-1">
            <div className="flex flex-col bg-white p-1 rounded-sm border border-slate-300">
              <span className="text-[9px] text-slate-500">TOTAL</span>
              <span className="font-bold text-emerald-700">{angularErrorTotal.toFixed(3)}°</span>
            </div>
            <div className="flex flex-col bg-white p-1 rounded-sm border border-slate-300">
              <span className="text-[9px] text-slate-500">YAW (H)</span>
              <span className={`font-bold ${Math.abs(angularErrorH) < 0.1 ? 'text-slate-600' : angularErrorH > 0 ? 'text-cyan-700' : 'text-amber-700'}`}>
                {angularErrorH > 0 ? '+' : ''}{angularErrorH.toFixed(3)}°
              </span>
            </div>
            <div className="flex flex-col bg-white p-1 rounded-sm border border-slate-300">
              <span className="text-[9px] text-slate-500">PITCH (V)</span>
              <span className={`font-bold ${Math.abs(angularErrorV) < 0.1 ? 'text-slate-600' : angularErrorV > 0 ? 'text-cyan-700' : 'text-amber-700'}`}>
                {angularErrorV > 0 ? '+' : ''}{angularErrorV.toFixed(3)}°
              </span>
            </div>
          </div>
        </div>

        {/* Pixel Error details */}
        <div className="bg-slate-50 p-2 rounded-sm border border-slate-300 col-span-2">
          <div className="text-slate-500 font-mono text-[10px] mb-1 uppercase">Focal Plane Pixel Error</div>
          <div className="grid grid-cols-2 text-center font-mono font-semibold text-slate-800 text-xs py-0.5">
            <div>ΔX: <span className={Math.abs(pixelErrorX) > 0.1 ? 'text-amber-700 font-bold' : 'text-slate-600'}>{pixelErrorX.toFixed(1)} px</span></div>
            <div>ΔY: <span className={Math.abs(pixelErrorY) > 0.1 ? 'text-amber-700 font-bold' : 'text-slate-600'}>{pixelErrorY.toFixed(1)} px</span></div>
          </div>
        </div>
      </div>

      <hr className="border-slate-300" />

      {/* AI Processing Telemetry */}
      <div className="flex flex-col gap-2">
        <span className="text-[10px] font-semibold text-slate-600 uppercase font-mono flex items-center gap-1">
          Inference Telemetry
        </span>
        <div className="grid grid-cols-2 gap-2 text-[11px] font-mono text-slate-600 bg-slate-50 p-2 rounded-sm border border-slate-300">
          <div className="flex items-center justify-between col-span-2 border-b border-slate-200 pb-1">
            <span className="text-slate-500">Active Mode:</span>
            <span className="font-bold text-cyan-800 uppercase">{activeDetectorMode}</span>
          </div>
          <div className="flex items-center justify-between mt-1">
            <span className="text-slate-500">Latency:</span>
            <span className="font-bold text-slate-900">
              {inferenceTime.toFixed(1)} ms
            </span>
          </div>
          <div className="flex items-center justify-between mt-1">
            <span className="text-slate-500">Rate:</span>
            <span className="font-bold text-slate-900">~15 Hz</span>
          </div>
        </div>
      </div>

      <hr className="border-slate-300" />

      {/* Network / Streaming Stats */}
      <div className="flex flex-col gap-2">
        <span className="text-[10px] font-semibold text-slate-500 uppercase font-mono">
          Network Stream Statistics
        </span>
        
        <div className="grid grid-cols-2 gap-2 text-[11px] font-mono text-slate-600">
          <div className="flex justify-between bg-slate-50 px-2 py-1.5 rounded-sm border border-slate-300">
            <span className="text-slate-500">FPS:</span>
            <span className="font-bold text-cyan-800">{wsFps} Hz</span>
          </div>
          <div className="flex justify-between bg-slate-50 px-2 py-1.5 rounded-sm border border-slate-300">
            <span className="text-slate-500">Latency:</span>
            <span className={`font-bold ${wsLatency > 100 ? 'text-rose-700' : 'text-emerald-700'}`}>
              {wsLatency} ms
            </span>
          </div>
          <div className="flex justify-between bg-slate-50 px-2 py-1.5 rounded-sm border border-slate-300">
            <span className="text-slate-500">Payload:</span>
            <span className="font-bold text-slate-900">{(wsBytesReceived / 1024).toFixed(1)} KB</span>
          </div>
          <div className="flex justify-between bg-slate-50 px-2 py-1.5 rounded-sm border border-slate-300">
            <span className="text-slate-500">Acks:</span>
            <span className="font-bold text-slate-900">{wsAcksCount}</span>
          </div>
        </div>
      </div>
    </div>
  );
}
