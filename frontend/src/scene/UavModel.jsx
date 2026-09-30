import React, { useRef } from 'react';
import { useFrame } from '@react-three/fiber';

export default function UavModel({
  size = 1.5,
  color = '#10B981',
  intensity = 2.0,
  active = true,
  _designated = true
}) {
  const uavRef = useRef();
  const leftPropRef = useRef();
  const rightPropRef = useRef();
  const navStrobeRef = useRef();

  useFrame((state, delta) => {
    if (!uavRef.current) return;
    
    // Slight banking and pitch aerodynamic movement
    uavRef.current.rotation.z = Math.sin(state.clock.getElapsedTime() * 0.8) * 0.05;
    uavRef.current.rotation.x = Math.sin(state.clock.getElapsedTime() * 0.5) * 0.03;

    // High-speed propeller spin
    if (leftPropRef.current) leftPropRef.current.rotation.z += 25.0 * delta;
    if (rightPropRef.current) rightPropRef.current.rotation.z += 25.0 * delta;

    // Aircraft navigation strobe pulse (1 Hz)
    if (navStrobeRef.current) {
      const flash = Math.sin(state.clock.getElapsedTime() * 6.28) > 0.85 ? 2.5 : 0.1;
      navStrobeRef.current.intensity = flash;
    }
  });

  return (
    <group ref={uavRef}>
      {/* ======================================================== */}
      {/* 1. AERODYNAMIC COMPOSITE FUSELAGE                        */}
      {/* ======================================================== */}
      <group>
        {/* Main Body */}
        <mesh position={[0, 0, 0]} rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[0.22, 0.15, 3.2, 16]} />
          <meshStandardMaterial color="#334155" metalness={0.4} roughness={0.5} />
        </mesh>
        {/* Streamlined Nose Cone */}
        <mesh position={[0, 0, 1.7]} rotation={[Math.PI / 2, 0, 0]}>
          <coneGeometry args={[0.22, 0.6, 16]} />
          <meshStandardMaterial color="#1E293B" metalness={0.5} roughness={0.4} />
        </mesh>
      </group>

      {/* ======================================================== */}
      {/* 2. HIGH-ASPECT-RATIO SWEPT WINGS & WINGLETS              */}
      {/* ======================================================== */}
      <group position={[0, 0.05, 0.2]}>
        {/* Main Wing Span */}
        <mesh position={[0, 0, 0]}>
          <boxGeometry args={[7.2, 0.04, 0.55]} />
          <meshStandardMaterial color="#475569" metalness={0.3} roughness={0.6} />
        </mesh>
        {/* Port Winglet (Left) */}
        <mesh position={[-3.6, 0.2, 0]}>
          <boxGeometry args={[0.04, 0.4, 0.35]} />
          <meshStandardMaterial color="#0F172A" />
        </mesh>
        {/* Starboard Winglet (Right) */}
        <mesh position={[3.6, 0.2, 0]}>
          <boxGeometry args={[0.04, 0.4, 0.35]} />
          <meshStandardMaterial color="#0F172A" />
        </mesh>

        {/* Wing Navigation Lights */}
        {/* Port Red Light */}
        <mesh position={[-3.6, 0.4, 0.1]}>
          <sphereGeometry args={[0.04, 8, 8]} />
          <meshBasicMaterial color="#EF4444" />
        </mesh>
        {/* Starboard Green Light */}
        <mesh position={[3.6, 0.4, 0.1]}>
          <sphereGeometry args={[0.04, 8, 8]} />
          <meshBasicMaterial color="#10B981" />
        </mesh>
      </group>

      {/* ======================================================== */}
      {/* 3. TWIN ELECTRIC PROPULSION NACELLES & PROPELLERS        */}
      {/* ======================================================== */}
      {[-1.4, 1.4].map((xOffset, idx) => (
        <group key={idx} position={[xOffset, 0, 0.2]}>
          {/* Motor Nacelle */}
          <mesh rotation={[Math.PI / 2, 0, 0]}>
            <cylinderGeometry args={[0.12, 0.1, 0.7, 12]} />
            <meshStandardMaterial color="#1E293B" metalness={0.6} />
          </mesh>
          {/* Propeller Hub */}
          <group position={[0, 0, -0.4]} ref={idx === 0 ? leftPropRef : rightPropRef}>
            <mesh>
              <sphereGeometry args={[0.06, 8, 8]} />
              <meshStandardMaterial color="#64748B" />
            </mesh>
            {/* Propeller Blade 1 */}
            <mesh position={[0, 0.3, 0]}>
              <boxGeometry args={[0.04, 0.6, 0.01]} />
              <meshStandardMaterial color="#0F172A" />
            </mesh>
            {/* Propeller Blade 2 */}
            <mesh position={[0, -0.3, 0]}>
              <boxGeometry args={[0.04, 0.6, 0.01]} />
              <meshStandardMaterial color="#0F172A" />
            </mesh>
          </group>
        </group>
      ))}

      {/* ======================================================== */}
      {/* 4. V-TAIL EMPENNAGE STABILIZER FINS                      */}
      {/* ======================================================== */}
      <group position={[0, 0.1, -1.5]}>
        {/* Left V-Fin (Angled 45 deg) */}
        <mesh position={[-0.35, 0.35, 0]} rotation={[0, 0, -0.7]}>
          <boxGeometry args={[0.04, 0.8, 0.35]} />
          <meshStandardMaterial color="#334155" />
        </mesh>
        {/* Right V-Fin (Angled -45 deg) */}
        <mesh position={[0.35, 0.35, 0]} rotation={[0, 0, 0.7]}>
          <boxGeometry args={[0.04, 0.8, 0.35]} />
          <meshStandardMaterial color="#334155" />
        </mesh>
        {/* Tail White Anti-Collision Strobe */}
        <pointLight
          ref={navStrobeRef}
          position={[0, 0.6, 0]}
          color="#FFFFFF"
          intensity={1.0}
          distance={50}
        />
      </group>

      {/* ======================================================== */}
      {/* 5. VENTRAL OPTICAL TURRET GIMBAL & LASER EMITTER         */}
      {/* ======================================================== */}
      <group position={[0, -0.25, 0.8]}>
        {/* Gimbal Turret Sphere */}
        <mesh>
          <sphereGeometry args={[0.22, 16, 16]} />
          <meshStandardMaterial color="#0F172A" metalness={0.8} roughness={0.2} />
        </mesh>
        {/* Sapphire Optical Window */}
        <mesh position={[0, -0.05, 0.18]} rotation={[0.3, 0, 0]}>
          <circleGeometry args={[0.12, 16]} />
          <meshStandardMaterial
            color="#06B6D4"
            emissive={color}
            emissiveIntensity={intensity * 0.7}
            metalness={0.9}
            roughness={0.1}
          />
        </mesh>

        {/* Laser Emitter Core */}
        {active && (
          <group position={[0, -0.05, 0.2]}>
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
    </group>
  );
}
