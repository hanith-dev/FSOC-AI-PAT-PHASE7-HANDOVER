import React, { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { useSimulationStore } from '../store/useSimulationStore';

export default function EnvironmentPad() {
  const anemometerRef = useRef();
  const windVaneRef = useRef();
  const cloudsRef = useRef();

  const windSpeedMs = useSimulationStore((s) => s.environmentalParameters?.windSpeedMs || 5.0);
  const windDirectionDeg = useSimulationStore((s) => s.environmentalParameters?.windDirectionDeg || 220);
  const cloudDensity = useSimulationStore((s) => s.environmentalParameters?.cloudDensity || 0.15);
  const cloudAltitudeM = useSimulationStore((s) => s.environmentalParameters?.cloudAltitudeM || 14.0);

  useFrame((state, delta) => {
    // 1. Anemometer 3-cup rotation proportional to wind speed
    if (anemometerRef.current) {
      anemometerRef.current.rotation.y += windSpeedMs * 1.5 * delta;
    }

    // 2. Wind direction vane orientation
    if (windVaneRef.current) {
      const targetRad = (windDirectionDeg * Math.PI) / 180.0;
      windVaneRef.current.rotation.y = targetRad + Math.sin(state.clock.getElapsedTime() * 1.5) * 0.05;
    }

    // 3. Dynamic atmospheric cloud / fog layer drift across optical path
    if (cloudsRef.current) {
      cloudsRef.current.position.x += (windSpeedMs * 0.4 * delta);
      cloudsRef.current.position.z += (windSpeedMs * 0.2 * delta);
      // Wrap around bounding box to maintain infinite atmospheric drift
      if (cloudsRef.current.position.x > 80) cloudsRef.current.position.x = -80;
      if (cloudsRef.current.position.z > 80) cloudsRef.current.position.z = -80;
    }
  });

  return (
    <group>
      {/* ======================================================== */}
      {/* 1. HEAVY REINFORCED CONCRETE GROUND STATION TEST PAD     */}
      {/* ======================================================== */}
      {/* Main Octagonal Base Foundation (Radius 24m) */}
      <mesh position={[0, 0.02, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[22, 24, 0.1, 8]} />
        <meshStandardMaterial color="#0F172A" roughness={0.9} metalness={0.1} />
      </mesh>

      {/* Vibration-Isolated Central Instrument Plinth (Radius 4.5m) */}
      <mesh position={[0, 0.08, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[4.2, 4.5, 0.12, 16]} />
        <meshStandardMaterial color="#1E293B" roughness={0.7} metalness={0.3} />
      </mesh>

      {/* Safety Perimeter Warning Stanchion Ring */}
      <mesh position={[0, 0.1, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <ringGeometry args={[4.45, 4.55, 32]} />
        <meshBasicMaterial color="#F59E0B" />
      </mesh>

      {/* ======================================================== */}
      {/* 2. ENGRAVED 360-DEGREE AZIMUTH COMPASS ROSE & RANGE RINGS*/}
      {/* ======================================================== */}
      {/* Concentric Range Rings: 10m, 20m, 35m, 60m, 100m */}
      {[10, 20, 35, 60, 100].map((radius, idx) => (
        <mesh key={idx} position={[0, 0.03, 0]} rotation={[-Math.PI / 2, 0, 0]}>
          <ringGeometry args={[radius - 0.06, radius + 0.06, 64]} />
          <meshBasicMaterial color="#334155" opacity={0.6} transparent />
        </mesh>
      ))}

      {/* Cardinal Direction Radial Tick Lines (N, E, S, W) */}
      {[0, Math.PI / 2, Math.PI, (3 * Math.PI) / 2].map((angle, idx) => (
        <mesh key={idx} position={[0, 0.04, 0]} rotation={[-Math.PI / 2, 0, angle]}>
          <planeGeometry args={[0.15, 40]} />
          <meshBasicMaterial color="#0284C7" opacity={0.5} transparent />
        </mesh>
      ))}

      {/* Cardinal Direction Markers */}
      {/* North Marker (000 deg) */}
      <group position={[0, 0.15, -21]}>
        <mesh>
          <boxGeometry args={[1.4, 0.05, 0.6]} />
          <meshStandardMaterial color="#1E293B" />
        </mesh>
        <mesh position={[0, 0.03, 0]} rotation={[-Math.PI / 2, 0, 0]}>
          <planeGeometry args={[1.2, 0.4]} />
          <meshBasicMaterial color="#0284C7" />
        </mesh>
      </group>
      {/* South Marker (180 deg) */}
      <group position={[0, 0.15, 21]}>
        <mesh>
          <boxGeometry args={[1.4, 0.05, 0.6]} />
          <meshStandardMaterial color="#1E293B" />
        </mesh>
      </group>
      {/* East Marker (090 deg) */}
      <group position={[21, 0.15, 0]}>
        <mesh>
          <boxGeometry args={[0.6, 0.05, 1.4]} />
          <meshStandardMaterial color="#1E293B" />
        </mesh>
      </group>
      {/* West Marker (270 deg) */}
      <group position={[-21, 0.15, 0]}>
        <mesh>
          <boxGeometry args={[0.6, 0.05, 1.4]} />
          <meshStandardMaterial color="#1E293B" />
        </mesh>
      </group>

      {/* ======================================================== */}
      {/* 3. METEOROLOGICAL WEATHER SENSOR MAST (OGS Standards)   */}
      {/* ======================================================== */}
      <group position={[12, 0, 12]}>
        {/* Mast Base Plate */}
        <mesh position={[0, 0.05, 0]}>
          <boxGeometry args={[1.2, 0.1, 1.2]} />
          <meshStandardMaterial color="#1E293B" />
        </mesh>

        {/* 5-Meter Weather Mast Pole */}
        <mesh position={[0, 2.5, 0]}>
          <cylinderGeometry args={[0.06, 0.08, 5.0, 12]} />
          <meshStandardMaterial color="#64748B" metalness={0.7} roughness={0.3} />
        </mesh>

        {/* Barometric & Temperature Radiation Shield */}
        <mesh position={[0, 3.2, 0]}>
          <cylinderGeometry args={[0.2, 0.2, 0.6, 12]} />
          <meshStandardMaterial color="#CBD5E1" />
        </mesh>

        {/* Crossbar at Summit */}
        <mesh position={[0, 4.8, 0]}>
          <boxGeometry args={[1.2, 0.04, 0.04]} />
          <meshStandardMaterial color="#475569" />
        </mesh>

        {/* Anemometer (Left Arm) */}
        <group position={[-0.5, 4.8, 0]}>
          <mesh position={[0, 0.1, 0]}>
            <cylinderGeometry args={[0.02, 0.02, 0.2, 8]} />
            <meshStandardMaterial color="#334155" />
          </mesh>
          {/* 3-Cup Rotor */}
          <group ref={anemometerRef} position={[0, 0.2, 0]}>
            <mesh>
              <cylinderGeometry args={[0.04, 0.04, 0.04, 8]} />
              <meshStandardMaterial color="#0F172A" />
            </mesh>
            {[0, (2 * Math.PI) / 3, (4 * Math.PI) / 3].map((angle, idx) => (
              <group key={idx} rotation={[0, angle, 0]}>
                <mesh position={[0.15, 0, 0]}>
                  <boxGeometry args={[0.3, 0.01, 0.01]} />
                  <meshStandardMaterial color="#475569" />
                </mesh>
                <mesh position={[0.3, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
                  <sphereGeometry args={[0.06, 8, 8, 0, Math.PI]} />
                  <meshStandardMaterial color="#D97706" metalness={0.8} />
                </mesh>
              </group>
            ))}
          </group>
        </group>

        {/* Wind Direction Vane (Right Arm) */}
        <group position={[0.5, 4.8, 0]} ref={windVaneRef}>
          <mesh position={[0, 0.1, 0]}>
            <cylinderGeometry args={[0.02, 0.02, 0.2, 8]} />
            <meshStandardMaterial color="#334155" />
          </mesh>
          {/* Vane Fin & Pointer */}
          <mesh position={[0, 0.2, 0]}>
            <boxGeometry args={[0.02, 0.02, 0.6]} />
            <meshStandardMaterial color="#0F172A" />
          </mesh>
          <mesh position={[0, 0.2, 0.3]}>
            <coneGeometry args={[0.05, 0.15, 4]} rotation={[Math.PI / 2, 0, 0]} />
            <meshStandardMaterial color="#0284C7" />
          </mesh>
          <mesh position={[0, 0.25, -0.25]}>
            <boxGeometry args={[0.01, 0.2, 0.2]} />
            <meshStandardMaterial color="#0284C7" />
          </mesh>
        </group>
      </group>

      {/* ======================================================== */}
      {/* 4. DYNAMIC ATMOSPHERIC CLOUD / FOG EXTINCTION LAYER     */}
      {/* ======================================================== */}
      {cloudDensity > 0.02 && (
        <group ref={cloudsRef} position={[0, cloudAltitudeM, -20]}>
          {/* Drifting semi-transparent volumetric cloud decks */}
          {[
            [-25, 0, -10, 35, 25],
            [15, 2, -30, 45, 30],
            [-5, -1, 15, 40, 28],
            [30, 1, 10, 30, 22]
          ].map((cloud, idx) => (
            <mesh key={idx} position={[cloud[0], cloud[1], cloud[2]]} rotation={[-Math.PI / 2, 0, 0]}>
              <planeGeometry args={[cloud[3], cloud[4]]} />
              <meshBasicMaterial
                color="#64748B"
                transparent
                opacity={Math.min(0.65, cloudDensity * 1.2)}
                depthWrite={false}
              />
            </mesh>
          ))}
        </group>
      )}

      {/* ======================================================== */}
      {/* 5. DISTANT TERRAIN HORIZON & BOUNDARY LIMB LAYER         */}
      {/* ======================================================== */}
      <mesh position={[0, -0.2, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <ringGeometry args={[50, 180, 48]} />
        <meshBasicMaterial color="#05080E" />
      </mesh>
    </group>
  );
}
