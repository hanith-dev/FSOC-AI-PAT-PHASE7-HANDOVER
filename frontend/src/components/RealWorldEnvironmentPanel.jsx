import React from 'react';
import { useSimulationStore } from '../store/useSimulationStore';

export default function RealWorldEnvironmentPanel() {
  const {
    targetPlatformModel,
    environmentProfile,
    laserBeamVisible,
    timeOfDay,
    environmentalParameters,
    setTargetPlatformModel,
    setEnvironmentProfile,
    setLaserBeamVisible,
    setTimeOfDay,
    updateEnvironmentalParameters
  } = useSimulationStore();

  const env = environmentalParameters || {};

  // Formatted calculations
  const linkMargin = env.linkMarginDb || 0;
  const isHealthyMargin = linkMargin >= 10.0;
  const isDegradedMargin = linkMargin > 3.0 && linkMargin < 10.0;

  return (
    <div className="w-full bg-white border border-slate-300 p-3 rounded-none flex flex-col gap-3 font-mono">
      {/* Header */}
      <div className="flex justify-between items-center border-b border-slate-300 pb-2">
        <div className="flex items-center gap-2">
          <span className="text-cyan-700 font-bold text-xs">[PHYSICS & ENVIRONMENT]</span>
          <span className="text-xs font-bold text-slate-800 uppercase tracking-wider">
            High-Grade Optical Link Simulation
          </span>
        </div>
        <div className="flex items-center gap-1.5 text-[10px]">
          <span className="text-slate-500">OPERATIONAL TESTBED:</span>
          <span className="font-bold text-slate-900 uppercase">
            {environmentProfile.replace('_', ' ')}
          </span>
        </div>
      </div>

      {/* 1. Operational Environment Scenarios (4 Real-World Profiles) */}
      <div className="flex flex-col gap-1.5">
        <span className="text-[10px] font-semibold text-slate-500 uppercase">
          Real-World Operational Link Profiles
        </span>
        <div className="grid grid-cols-2 md:grid-cols-4 gap-1.5">
          {[
            { id: 'leo_downlink', label: '[LEO DOWNLINK]', desc: '500km LEO to OGS' },
            { id: 'inter_satellite', label: '[ISL VACUUM]', desc: '12,000km Space Link' },
            { id: 'uav_tactical', label: '[UAV TACTICAL]', desc: '25km Airborne Link' },
            { id: 'terrestrial_range', label: '[TERRESTRIAL]', desc: '5.2km Mountain Range' }
          ].map((profile) => (
            <button
              key={profile.id}
              onClick={() => setEnvironmentProfile(profile.id)}
              className={`py-1.5 px-2 text-left rounded-none border text-xs transition-colors flex flex-col ${
                environmentProfile === profile.id
                  ? 'bg-cyan-700 text-white border-cyan-800 font-bold'
                  : 'bg-slate-50 hover:bg-slate-100 text-slate-700 border-slate-300'
              }`}
            >
              <span>{profile.label}</span>
              <span className={`text-[9px] ${environmentProfile === profile.id ? 'text-cyan-100' : 'text-slate-500'}`}>
                {profile.desc}
              </span>
            </button>
          ))}
        </div>
      </div>

      {/* 2. Target Platform Model Selector & Visual Environmental Elements */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-3 pt-1 border-t border-slate-200">
        {/* Target 3D Model */}
        <div className="flex flex-col gap-1.5">
          <span className="text-[10px] font-semibold text-slate-500 uppercase">
            Target Platform Geometry
          </span>
          <div className="flex gap-1.5">
            {[
              { id: 'satellite', label: '[SATELLITE BUS]' },
              { id: 'uav', label: '[UAV DRONE]' },
              { id: 'calibration_tower', label: '[CALIB TOWER]' }
            ].map((tgt) => (
              <button
                key={tgt.id}
                onClick={() => setTargetPlatformModel(tgt.id)}
                className={`flex-1 py-1 px-1.5 text-center text-xs border rounded-none ${
                  targetPlatformModel === tgt.id
                    ? 'bg-slate-800 text-white border-slate-900 font-bold'
                    : 'bg-slate-50 hover:bg-slate-100 text-slate-600 border-slate-300'
                }`}
              >
                {tgt.label}
              </button>
            ))}
          </div>
        </div>

        {/* Environmental Celestial & Beam Toggles */}
        <div className="flex flex-col gap-1.5">
          <span className="text-[10px] font-semibold text-slate-500 uppercase">
            Celestial Lighting & Beam Emission
          </span>
          <div className="flex gap-1.5">
            {/* Time of Day */}
            <div className="flex-1 flex border border-slate-300 bg-slate-100 p-0.5">
              {['night', 'twilight', 'day'].map((tod) => (
                <button
                  key={tod}
                  onClick={() => setTimeOfDay(tod)}
                  className={`flex-1 text-[10px] py-0.5 uppercase ${
                    timeOfDay === tod ? 'bg-white text-slate-900 font-bold border border-slate-300' : 'text-slate-600'
                  }`}
                >
                  {tod}
                </button>
              ))}
            </div>

            {/* Laser Beam Visibility */}
            <button
              onClick={() => setLaserBeamVisible(!laserBeamVisible)}
              className={`px-3 py-1 text-xs border rounded-none font-bold ${
                laserBeamVisible
                  ? 'bg-emerald-50 text-emerald-800 border-emerald-500'
                  : 'bg-slate-100 text-slate-500 border-slate-300'
              }`}
            >
              {laserBeamVisible ? '[BEAM: ARMED]' : '[BEAM: MUTED]'}
            </button>
          </div>
        </div>
      </div>

      {/* 3. Real Physical Channel Sliders */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-3 pt-2 border-t border-slate-200">
        {/* Atmospheric Turbulence Cn2 */}
        <div className="bg-slate-50 p-2.5 border border-slate-300 flex flex-col gap-1">
          <div className="flex justify-between text-xs text-slate-700">
            <span>Turbulence Index (Cn^2):</span>
            <span className="font-bold text-cyan-800">
              {(env.cn2 || 1e-15).toExponential(1)} m^-2/3
            </span>
          </div>
          <input
            type="range"
            min="13"
            max="17"
            step="0.1"
            value={-(Math.log10(env.cn2 || 1e-15))}
            onChange={(e) => updateEnvironmentalParameters({ cn2: Math.pow(10, -parseFloat(e.target.value)) })}
            className="w-full accent-cyan-600 h-1 bg-slate-200 cursor-pointer"
          />
          <div className="flex justify-between text-[9px] text-slate-500">
            <span>Severe (1e-13)</span>
            <span>Fried r0: {env.friedR0Cm || 7.2} cm</span>
            <span>Weak (1e-17)</span>
          </div>
        </div>

        {/* Cloud / Aerosol Extinction */}
        <div className="bg-slate-50 p-2.5 border border-slate-300 flex flex-col gap-1">
          <div className="flex justify-between text-xs text-slate-700">
            <span>Cloud/Aerosol Extinction:</span>
            <span className="font-bold text-cyan-800">
              {(env.cloudExtinctionDbKm || 0.25).toFixed(2)} dB/km
            </span>
          </div>
          <input
            type="range"
            min="0"
            max="10"
            step="0.1"
            value={env.cloudExtinctionDbKm || 0.25}
            onChange={(e) => updateEnvironmentalParameters({
              cloudExtinctionDbKm: parseFloat(e.target.value),
              cloudDensity: Math.min(1.0, parseFloat(e.target.value) / 5.0)
            })}
            className="w-full accent-cyan-600 h-1 bg-slate-200 cursor-pointer"
          />
          <div className="flex justify-between text-[9px] text-slate-500">
            <span>Clear Sky (0.0)</span>
            <span>Density: {Math.round((env.cloudDensity || 0.15) * 100)}%</span>
            <span>Dense Fog (10.0)</span>
          </div>
        </div>

        {/* Laser Transmit Power */}
        <div className="bg-slate-50 p-2.5 border border-slate-300 flex flex-col gap-1">
          <div className="flex justify-between text-xs text-slate-700">
            <span>Laser Transmit Power:</span>
            <span className="font-bold text-cyan-800">
              {env.laserPowerMw || 500} mW
            </span>
          </div>
          <input
            type="range"
            min="50"
            max="2000"
            step="50"
            value={env.laserPowerMw || 500}
            onChange={(e) => updateEnvironmentalParameters({ laserPowerMw: parseInt(e.target.value) })}
            className="w-full accent-cyan-600 h-1 bg-slate-200 cursor-pointer"
          />
          <div className="flex justify-between text-[9px] text-slate-500">
            <span>50 mW</span>
            <span>Div: {env.beamDivergenceUrad || 100} urad</span>
            <span>2000 mW</span>
          </div>
        </div>
      </div>

      {/* 4. Real-World Optical Link Budget Telemetry HUD */}
      <div className="bg-slate-50 border border-slate-300 p-2.5 flex flex-col gap-1.5">
        <div className="flex justify-between items-center">
          <span className="text-[10px] font-bold text-slate-700 uppercase">
            Optical Link Budget & Atmospheric Telemetry
          </span>
          <span className={`text-[10px] font-bold px-1.5 py-0.5 rounded-none border ${
            isHealthyMargin
              ? 'bg-emerald-50 text-emerald-800 border-emerald-400'
              : (isDegradedMargin
                ? 'bg-amber-50 text-amber-800 border-amber-400'
                : 'bg-rose-50 text-rose-800 border-rose-400')
          }`}>
            LINK MARGIN: {(env.linkMarginDb || 0).toFixed(1)} dB
          </span>
        </div>

        <div className="grid grid-cols-2 md:grid-cols-6 gap-2 text-xs">
          <div className="flex flex-col">
            <span className="text-[9px] text-slate-500">Slant Range (L):</span>
            <span className="font-bold text-slate-800">{env.linkDistanceKm || 480.0} km</span>
          </div>
          <div className="flex flex-col">
            <span className="text-[9px] text-slate-500">Zenith / Air Mass:</span>
            <span className="font-bold text-slate-800">{env.zenithAngleDeg || 34.0}° (AM {env.airMass || 1.2})</span>
          </div>
          <div className="flex flex-col">
            <span className="text-[9px] text-slate-500">Scintillation Index:</span>
            <span className="font-bold text-slate-800">{env.scintillationIndex || 0.14}</span>
          </div>
          <div className="flex flex-col">
            <span className="text-[9px] text-slate-500">Received Power (Prx):</span>
            <span className="font-bold text-slate-800">{env.receivedPowerDbm || -31.4} dBm</span>
          </div>
          <div className="flex flex-col">
            <span className="text-[9px] text-slate-500">Detector SNR:</span>
            <span className="font-bold text-slate-800">{env.snrDb || 25.2} dB</span>
          </div>
          <div className="flex flex-col">
            <span className="text-[9px] text-slate-500">Estimated BER:</span>
            <span className="font-bold text-emerald-700">{(env.berEstimate || 1e-9).toExponential(0)}</span>
          </div>
        </div>
      </div>
    </div>
  );
}
