import React from 'react';
import { useSimulationStore } from '../store/useSimulationStore';

export default function CameraFeed() {
  const {
    capturedFrame,
    targetInsideFOV,
    trackedPixelX,
    trackedPixelY,
    resolutionWidth,
    resolutionHeight,
    activeDetectorMode,
    fallbackWarning,
    bbox,
    rawCenter,
    trackingState,
    confidence,
    environmentalParameters
  } = useSimulationStore();

  const env = environmentalParameters || {};

  // Project tracked pixel coordinates (640x480) to percentage of container
  const targetXPercent = (trackedPixelX / resolutionWidth) * 100;
  const targetYPercent = (trackedPixelY / resolutionHeight) * 100;

  // Project raw detection bounding box and center
  const isDetected = trackingState !== 'SEARCHING' && trackingState !== 'SEARCH' && trackingState !== 'TARGET_LOST' && trackingState !== 'LOST' && rawCenter[0] > 0;
  
  const boxLeft = (bbox[0] / resolutionWidth) * 100;
  const boxTop = (bbox[1] / resolutionHeight) * 100;
  const boxWidth = ((bbox[2] - bbox[0]) / resolutionWidth) * 100;
  const boxHeight = ((bbox[3] - bbox[1]) / resolutionHeight) * 100;

  const rawXPercent = (rawCenter[0] / resolutionWidth) * 100;
  const rawYPercent = (rawCenter[1] / resolutionHeight) * 100;

  return (
    <div className="w-full bg-white border border-slate-300 p-3 rounded-none flex flex-col gap-2 font-mono">
      {/* Header */}
      <div className="flex justify-between items-center">
        <h3 className="text-xs font-semibold text-slate-800 uppercase tracking-wider flex items-center gap-1.5">
          {targetInsideFOV ? <span className="text-emerald-700 font-bold">[●]</span> : <span className="text-rose-700 font-bold">[○]</span>}
          Optical Sensor Focal Plane (InGaAs FPA)
        </h3>
        <div className="flex items-center gap-2">
          {fallbackWarning && (
            <span className="flex items-center gap-1 text-[9px] bg-amber-50 text-amber-800 border border-amber-400 px-1.5 py-0.5 rounded-none font-bold">
              [!] FALLBACK
            </span>
          )}
          <span className="text-[10px] text-slate-600">
            DET: {activeDetectorMode.toUpperCase()}
          </span>
        </div>
      </div>
      
      {/* Video display box */}
      <div className="relative aspect-square w-full bg-black rounded-none overflow-hidden border border-slate-300 flex items-center justify-center">
        {capturedFrame ? (
          <img
            src={capturedFrame}
            alt="Virtual camera sensor view"
            className="w-full h-full object-cover"
          />
        ) : (
          <div className="flex flex-col items-center justify-center text-center p-4">
            <div className="w-8 h-8 rounded-full border-2 border-cyan-500/20 border-t-cyan-500 animate-spin mb-2" />
            <span className="text-[10px] text-gray-500">Awaiting optical frame capture...</span>
          </div>
        )}
        
        {/* Real-World Optical Reticle with Sub-Pixel Graduation Marks */}
        <div className="absolute inset-0 pointer-events-none">
          {/* Vertical Center Axis */}
          <div className="absolute left-1/2 top-0 bottom-0 w-[0.5px] bg-cyan-500/40 -translate-x-1/2" />
          {/* Horizontal Center Axis */}
          <div className="absolute top-1/2 left-0 right-0 h-[0.5px] bg-cyan-500/40 -translate-y-1/2" />
          
          {/* Concentric Angular Field Rings (1 mrad, 2.5 mrad, 5 mrad) */}
          <div className="absolute left-1/2 top-1/2 w-10 h-10 rounded-full border border-cyan-500/30 -translate-x-1/2 -translate-y-1/2" />
          <div className="absolute left-1/2 top-1/2 w-24 h-24 rounded-full border border-cyan-500/20 -translate-x-1/2 -translate-y-1/2" />
          <div className="absolute left-1/2 top-1/2 w-48 h-48 rounded-full border border-cyan-500/10 -translate-x-1/2 -translate-y-1/2" />

          {/* Vernier Graduation Tick Marks along Axes */}
          {[-120, -80, -40, 40, 80, 120].map((offset) => (
            <React.Fragment key={offset}>
              {/* Horizontal Ticks */}
              <div
                className="absolute top-1/2 w-[0.5px] h-2 bg-cyan-500/50 -translate-y-1/2"
                style={{ left: `calc(50% + ${offset}px)` }}
              />
              {/* Vertical Ticks */}
              <div
                className="absolute left-1/2 h-[0.5px] w-2 bg-cyan-500/50 -translate-x-1/2"
                style={{ top: `calc(50% + ${offset}px)` }}
              />
            </React.Fragment>
          ))}
        </div>

        {/* Airy Disk Diffraction Ring Pattern (Centered on Target Spot when Locked) */}
        {targetInsideFOV && (
          <div
            className="absolute pointer-events-none -translate-x-1/2 -translate-y-1/2"
            style={{ left: `${targetXPercent}%`, top: `${targetYPercent}%` }}
          >
            {/* Primary Airy Ring (First Dark Ring) */}
            <div className="w-6 h-6 rounded-full border border-emerald-400/60" />
            {/* Secondary Airy Ring */}
            <div className="absolute -inset-1.5 rounded-full border border-emerald-400/25" />
          </div>
        )}

        {/* Bounding Box overlay (from active detector) */}
        {isDetected && boxWidth > 0 && boxHeight > 0 && (
          <div
            className="absolute border border-cyan-500 rounded-none bg-cyan-500/5 pointer-events-none transition-all duration-75"
            style={{
              left: `${boxLeft}%`,
              top: `${boxTop}%`,
              width: `${boxWidth}%`,
              height: `${boxHeight}%`
            }}
          >
            {/* Target tag */}
            <span className="absolute -top-3.5 left-0 text-[8px] bg-cyan-700 text-white px-1 py-0.5 rounded-none font-bold leading-none">
              OPT-TARGET ({confidence.toFixed(2)})
            </span>
          </div>
        )}

        {/* Raw detected centroid dot */}
        {isDetected && (
          <div
            className="absolute w-2 h-2 -translate-x-1/2 -translate-y-1/2 bg-cyan-400 rounded-none border border-black pointer-events-none transition-all duration-75"
            style={{ left: `${rawXPercent}%`, top: `${rawYPercent}%` }}
          />
        )}

        {/* Kalman Estimated Position Indicator (Emerald Green Reticle) */}
        {targetInsideFOV && (
          <div
            className="absolute w-4 h-4 -translate-x-1/2 -translate-y-1/2 pointer-events-none transition-all duration-75"
            style={{ left: `${targetXPercent}%`, top: `${targetYPercent}%` }}
          >
            <div className="absolute inset-0 rounded-none border border-emerald-400 opacity-90" />
            <div className="absolute inset-1 rounded-none bg-emerald-400/80" />
          </div>
        )}

        {/* Error Vector Line Overlay */}
        <svg className="absolute inset-0 w-full h-full pointer-events-none">
          {targetInsideFOV && (
            <line
              x1="50%"
              y1="50%"
              x2={`${targetXPercent}%`}
              y2={`${targetYPercent}%`}
              stroke="#10b981"
              strokeWidth="1.2"
              strokeDasharray="3,3"
            />
          )}
        </svg>
        
        {/* Physical Optical Sensor Telemetry Bar (Bottom HUD) */}
        <div className="absolute bottom-2 left-2 right-2 flex justify-between items-center pointer-events-none bg-black/90 px-2 py-1 text-[9px] text-slate-300 border border-slate-700">
          <div className="flex gap-2">
            <span>FPA: 640x480</span>
            <span className="text-slate-500">|</span>
            <span>Tgt: {targetInsideFOV ? `(${Math.round(trackedPixelX)}, ${Math.round(trackedPixelY)})` : 'SEARCHING'}</span>
            <span className="text-slate-500">|</span>
            <span>SNR: {env.snrDb || 25.2}dB</span>
          </div>
          <span className={`font-bold ${targetInsideFOV ? 'text-emerald-400' : 'text-amber-400'}`}>
            {trackingState}
          </span>
        </div>
      </div>

      {/* Optical Sensor Technical Specifications */}
      <div className="grid grid-cols-2 md:grid-cols-4 gap-1.5 text-[9px] bg-slate-50 p-1.5 border border-slate-300 text-slate-600">
        <div><span>OTA: </span><span className="font-bold text-slate-800">D=200mm, f=1200mm (f/6)</span></div>
        <div><span>WAVELENGTH: </span><span className="font-bold text-slate-800">1550nm C-Band</span></div>
        <div><span>EXPOSURE: </span><span className="font-bold text-slate-800">2.5ms (400Hz FPA)</span></div>
        <div><span>ELLIPTICITY: </span><span className="font-bold text-emerald-700">0.98 (Nominal)</span></div>
      </div>
    </div>
  );
}
