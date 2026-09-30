import React, { useRef } from 'react';
import { useFrame } from '@react-three/fiber';

export default function SatelliteModel({
  size = 1.5,
  color = '#10B981',
  intensity = 2.0,
  active = true,
  designated = true,
  _velocity = [0, 0, 0]
}) {
  const satelliteGroupRef = useRef();
  const solarWingsRef = useRef();
  const thrusterGlowRef = useRef();

  // Subtle orbital rotation and solar wing tracking
  useFrame((state) => {
    if (!satelliteGroupRef.current) return;
    
    // Slow attitude stabilization drift
    satelliteGroupRef.current.rotation.z += 0.002;
    
    // Solar panel articulation to face sun
    if (solarWingsRef.current) {
      solarWingsRef.current.rotation.y = Math.sin(state.clock.getElapsedTime() * 0.2) * 0.15;
    }

    // RCS thruster micro-pulse
    if (thrusterGlowRef.current) {
      const pulse = 0.5 + 0.5 * Math.sin(state.clock.getElapsedTime() * 3.0);
      thrusterGlowRef.current.intensity = pulse * 1.5;
    }
  });

  return (
    <group ref={satelliteGroupRef}>
      {/* ======================================================== */}
      {/* 1. MAIN AVIONICS CHASSIS / SATELLITE BUS (Gold MLI)      */}
      {/* ======================================================== */}
      <group>
        {/* Core Bus Box */}
        <mesh position={[0, 0, 0]}>
          <boxGeometry args={[1.2, 1.6, 1.2]} />
          <meshStandardMaterial
            color="#D97706"
            metalness={0.88}
            roughness={0.22}
          />
        </mesh>

        {/* Structural Edge Framing (Graphite-Epoxy Matte Rails) */}
        <mesh position={[0, 0, 0]}>
          <boxGeometry args={[1.22, 1.62, 1.22]} />
          <meshBasicMaterial color="#0F172A" wireframe />
        </mesh>

        {/* Top Equipment Deck Plate */}
        <mesh position={[0, 0.82, 0]}>
          <boxGeometry args={[1.25, 0.05, 1.25]} />
          <meshStandardMaterial color="#334155" metalness={0.7} roughness={0.4} />
        </mesh>

        {/* Bottom Propulsion Deck Plate */}
        <mesh position={[0, -0.82, 0]}>
          <boxGeometry args={[1.25, 0.05, 1.25]} />
          <meshStandardMaterial color="#1E293B" metalness={0.8} roughness={0.3} />
        </mesh>
      </group>

      {/* ======================================================== */}
      {/* 2. ARTICULATED SOLAR ARRAY WINGS (Port & Starboard)      */}
      {/* ======================================================== */}
      <group ref={solarWingsRef}>
        {/* Port (Left) Solar Array Wing */}
        <group position={[-0.6, 0, 0]}>
          {/* Main Extension Boom */}
          <mesh position={[-0.45, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
            <cylinderGeometry args={[0.04, 0.04, 0.9, 8]} />
            <meshStandardMaterial color="#64748B" metalness={0.6} roughness={0.3} />
          </mesh>
          {/* Panel Inner */}
          <mesh position={[-1.6, 0, 0]}>
            <boxGeometry args={[1.3, 0.9, 0.04]} />
            <meshStandardMaterial color="#0A192F" metalness={0.9} roughness={0.15} />
          </mesh>
          {/* Panel Inner Photovoltaic Grid Wireframe */}
          <mesh position={[-1.6, 0, 0.025]}>
            <planeGeometry args={[1.26, 0.86, 6, 4]} />
            <meshBasicMaterial color="#0284C7" wireframe opacity={0.4} transparent />
          </mesh>
          {/* Panel Outer */}
          <mesh position={[-3.0, 0, 0]}>
            <boxGeometry args={[1.3, 0.9, 0.04]} />
            <meshStandardMaterial color="#0A192F" metalness={0.9} roughness={0.15} />
          </mesh>
          {/* Panel Outer Photovoltaic Grid Wireframe */}
          <mesh position={[-3.0, 0, 0.025]}>
            <planeGeometry args={[1.26, 0.86, 6, 4]} />
            <meshBasicMaterial color="#0284C7" wireframe opacity={0.4} transparent />
          </mesh>
          {/* Solar Wing Gold Busbars */}
          <mesh position={[-2.3, 0, 0.022]}>
            <boxGeometry args={[2.7, 0.02, 0.01]} />
            <meshStandardMaterial color="#F59E0B" metalness={0.9} roughness={0.1} />
          </mesh>
        </group>

        {/* Starboard (Right) Solar Array Wing */}
        <group position={[0.6, 0, 0]}>
          {/* Main Extension Boom */}
          <mesh position={[0.45, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
            <cylinderGeometry args={[0.04, 0.04, 0.9, 8]} />
            <meshStandardMaterial color="#64748B" metalness={0.6} roughness={0.3} />
          </mesh>
          {/* Panel Inner */}
          <mesh position={[1.6, 0, 0]}>
            <boxGeometry args={[1.3, 0.9, 0.04]} />
            <meshStandardMaterial color="#0A192F" metalness={0.9} roughness={0.15} />
          </mesh>
          {/* Panel Inner Grid */}
          <mesh position={[1.6, 0, 0.025]}>
            <planeGeometry args={[1.26, 0.86, 6, 4]} />
            <meshBasicMaterial color="#0284C7" wireframe opacity={0.4} transparent />
          </mesh>
          {/* Panel Outer */}
          <mesh position={[3.0, 0, 0]}>
            <boxGeometry args={[1.3, 0.9, 0.04]} />
            <meshStandardMaterial color="#0A192F" metalness={0.9} roughness={0.15} />
          </mesh>
          {/* Panel Outer Grid */}
          <mesh position={[3.0, 0, 0.025]}>
            <planeGeometry args={[1.26, 0.86, 6, 4]} />
            <meshBasicMaterial color="#0284C7" wireframe opacity={0.4} transparent />
          </mesh>
          {/* Busbars */}
          <mesh position={[2.3, 0, 0.022]}>
            <boxGeometry args={[2.7, 0.02, 0.01]} />
            <meshStandardMaterial color="#F59E0B" metalness={0.9} roughness={0.1} />
          </mesh>
        </group>
      </group>

      {/* ======================================================== */}
      {/* 3. OPTICAL COMMUNICATION PAYLOAD / PAT TERMINAL APERTURE */}
      {/* ======================================================== */}
      <group position={[0, 0, 0.65]}>
        {/* Optical Baffle Shroud Cylinder */}
        <mesh rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[0.26, 0.32, 0.45, 24]} />
          <meshStandardMaterial color="#0F172A" metalness={0.9} roughness={0.2} />
        </mesh>

        {/* Optical Primary Lens / Aperture Window */}
        <mesh position={[0, 0, 0.23]} rotation={[Math.PI / 2, 0, 0]}>
          <circleGeometry args={[0.22, 24]} />
          <meshStandardMaterial
            color="#06B6D4"
            emissive={color}
            emissiveIntensity={intensity * 0.8}
            metalness={0.95}
            roughness={0.05}
          />
        </mesh>

        {/* Laser Emitter Core (Essential for Virtual Camera Sensor) */}
        {active && (
          <group position={[0, 0, 0.25]}>
            <mesh>
              <sphereGeometry args={[size * 0.25, 16, 16]} />
              <meshBasicMaterial color={color} />
            </mesh>
            <pointLight
              color={color}
              intensity={intensity * 15}
              distance={250}
              decay={1.2}
            />
          </group>
        )}
      </group>

      {/* ======================================================== */}
      {/* 4. RF HIGH-GAIN PARABOLIC DISH & TELEMETRY BOOMS         */}
      {/* ======================================================== */}
      <group position={[0, 0.95, -0.3]} rotation={[-0.4, 0.3, 0]}>
        {/* Dish Mount Arm */}
        <mesh position={[0, 0.15, 0]}>
          <cylinderGeometry args={[0.03, 0.03, 0.3, 8]} />
          <meshStandardMaterial color="#475569" />
        </mesh>
        {/* Parabolic Reflector Dish */}
        <mesh position={[0, 0.35, 0]} rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[0.45, 0.1, 0.12, 24, 1, true]} />
          <meshStandardMaterial color="#CBD5E1" metalness={0.7} roughness={0.3} side={2} />
        </mesh>
        {/* Subreflector Feedhorn */}
        <mesh position={[0, 0.35, 0.2]}>
          <cylinderGeometry args={[0.04, 0.02, 0.15, 12]} />
          <meshStandardMaterial color="#D97706" metalness={0.9} roughness={0.2} />
        </mesh>
      </group>

      {/* Star Tracker Baffles */}
      <group position={[0.4, 0.85, 0.3]} rotation={[0.3, -0.2, 0]}>
        <mesh>
          <cylinderGeometry args={[0.07, 0.08, 0.25, 12]} />
          <meshStandardMaterial color="#0F172A" />
        </mesh>
      </group>

      {/* ======================================================== */}
      {/* 5. 4-QUADRANT RCS ATTITUDE THRUSTERS                     */}
      {/* ======================================================== */}
      {[
        [-0.55, -0.75, 0.55],
        [0.55, -0.75, 0.55],
        [-0.55, -0.75, -0.55],
        [0.55, -0.75, -0.55]
      ].map((pos, idx) => (
        <group key={idx} position={pos}>
          {/* Thruster Nozzle Block */}
          <mesh>
            <boxGeometry args={[0.1, 0.1, 0.1]} />
            <meshStandardMaterial color="#334155" metalness={0.7} />
          </mesh>
          {/* Micro-Nozzle Bell */}
          <mesh position={[0, -0.08, 0]} rotation={[Math.PI, 0, 0]}>
            <coneGeometry args={[0.05, 0.1, 8]} />
            <meshStandardMaterial color="#D97706" metalness={0.9} roughness={0.2} />
          </mesh>
        </group>
      ))}

      {/* Main Apogee Engine Nozzle (Base) */}
      <mesh position={[0, -0.95, 0]} rotation={[Math.PI, 0, 0]}>
        <coneGeometry args={[0.22, 0.35, 16]} />
        <meshStandardMaterial color="#1E293B" metalness={0.8} roughness={0.4} />
      </mesh>

      {/* ======================================================== */}
      {/* 6. TARGET IDENTIFIER HUD BADGE                           */}
      {/* ======================================================== */}
      {designated && (
        <group position={[0, 1.4, 0]}>
          <mesh>
            <planeGeometry args={[1.6, 0.3]} />
            <meshBasicMaterial color="#0F172A" opacity={0.85} transparent />
          </mesh>
          <mesh position={[0, 0, 0.01]}>
            <boxGeometry args={[1.62, 0.32, 0.01]} />
            <meshBasicMaterial color={color} wireframe />
          </mesh>
        </group>
      )}
    </group>
  );
}
