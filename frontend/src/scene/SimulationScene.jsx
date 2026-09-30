import React from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Stars, Grid } from '@react-three/drei';
import GimbalCamera from './GimbalCamera';
import Beacon from './Beacon';
import EnvironmentPad from './EnvironmentPad';
import LaserBeam from './LaserBeam';
import { useSimulationStore } from '../store/useSimulationStore';

// Decoupled physics clock loop component
function SimulationLoop() {
  const updateBeaconPosition = useSimulationStore((state) => state.updateBeaconPosition);
  
  useFrame((state, delta) => {
    // Limit delta to prevent massive jumps when tab is minimized
    const dt = Math.min(delta, 0.1);
    updateBeaconPosition(dt);
  });
  
  return null;
}

export default function SimulationScene({ sendFrame }) {
  const beacons = useSimulationStore((state) => state.beacons);
  const timeOfDay = useSimulationStore((state) => state.timeOfDay || 'night');
  const targetPlatformModel = useSimulationStore((state) => state.targetPlatformModel || 'satellite');
  const environmentProfile = useSimulationStore((state) => state.environmentProfile || 'leo_downlink');
  const trackingState = useSimulationStore((state) => state.trackingState);
  const laserBeamVisible = useSimulationStore((state) => state.laserBeamVisible !== false);

  const isDay = timeOfDay === 'day';
  const isTwilight = timeOfDay === 'twilight';
  const bgColor = isDay ? '#0F1E36' : (isTwilight ? '#0A1224' : '#050811');

  return (
    <div className="w-full h-full relative bg-[#050811] rounded-none overflow-hidden border border-slate-300">
      <Canvas
        camera={{ position: [22, 16, 32], fov: 50 }}
        gl={{ preserveDrawingBuffer: true }}
      >
        {/* Sky / Environment depth */}
        <color attach="background" args={[bgColor]} />
        <Stars
          radius={120}
          depth={60}
          count={isDay ? 200 : (isTwilight ? 800 : 2500)}
          factor={4}
          saturation={0.4}
          fade
          speed={0.8}
        />
        
        {/* Lights (Sun & Ambient Celestial Illumination) */}
        <ambientLight intensity={isDay ? 0.8 : (isTwilight ? 0.5 : 0.35)} />
        <directionalLight
          position={isDay ? [25, 40, 15] : (isTwilight ? [35, 10, -20] : [15, 25, 15])}
          intensity={isDay ? 1.4 : (isTwilight ? 0.9 : 0.65)}
          castShadow
          shadow-mapSize-width={1024}
          shadow-mapSize-height={1024}
        />
        
        {/* Ground Reference Grid */}
        <Grid
          position={[0, 0, 0]}
          args={[160, 160]}
          cellSize={5}
          cellThickness={0.5}
          cellColor="#1E293B"
          sectionSize={25}
          sectionThickness={1.0}
          sectionColor="#0284C7"
          fadeDistance={110}
        />

        {/* Real-World Ground Station Testing Facility Environment */}
        <EnvironmentPad />

        {/* Precision Ritchey-Chrétien Gimbal & Camera System */}
        <GimbalCamera sendFrame={sendFrame} />
        
        {/* Remote Real-World Optical Beacons (Satellite / UAV / Tower) */}
        {beacons.map((b) => (
          <Beacon key={b.id} id={b.id} />
        ))}

        {/* Physical Gaussian Laser Beam Propagation */}
        {laserBeamVisible && <LaserBeam />}
        
        {/* Simulation Physics Clock Updater */}
        <SimulationLoop />
        
        {/* Interactive 3D Camera Controls */}
        <OrbitControls
          makeDefault
          enableDamping
          dampingFactor={0.05}
          maxPolarAngle={Math.PI / 2 + 0.02} // Restrain orbit above ground pad
          minDistance={2}
          maxDistance={180}
        />
      </Canvas>
      
      {/* Real-World Telemetry HUD Overlay Indicators */}
      <div className="absolute top-3 left-3 flex flex-col gap-1.5 pointer-events-none">
        <div className="flex items-center gap-1.5 px-2.5 py-1 bg-white/95 border border-slate-300 text-xs font-mono text-slate-800 rounded-none">
          <span className="h-2 w-2 rounded-none bg-emerald-600" />
          <span className="font-bold">OGS TESTBED ACTIVE</span>
          <span className="text-[10px] text-slate-500">[{environmentProfile.toUpperCase()}]</span>
        </div>
        <div className="flex items-center gap-1.5 px-2 py-0.5 bg-white/90 border border-slate-300 text-[10px] font-mono text-slate-700">
          <span>PLATFORM:</span>
          <span className="font-bold text-cyan-700">{targetPlatformModel.toUpperCase()}</span>
          <span className="text-slate-400">|</span>
          <span>LINK:</span>
          <span className={`font-bold ${trackingState === 'LOCKED' || trackingState === 'TRACKING' ? 'text-emerald-700' : 'text-amber-700'}`}>
            {trackingState}
          </span>
        </div>
      </div>
    </div>
  );
}
