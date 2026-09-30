import React from 'react';

export default function CalibrationTowerModel({
  size = 1.5,
  color = '#10B981',
  intensity = 2.0,
  active = true
}) {
  return (
    <group>
      {/* ======================================================== */}
      {/* 1. STEEL LATTICE STRUCTURAL TOWER                        */}
      {/* ======================================================== */}
      {/* Vertical Legs */}
      {[
        [-0.8, 0, -0.8],
        [0.8, 0, -0.8],
        [-0.8, 0, 0.8],
        [0.8, 0, 0.8]
      ].map((pos, idx) => (
        <mesh key={idx} position={[pos[0], -5.0, pos[2]]}>
          <cylinderGeometry args={[0.05, 0.08, 10.0, 8]} />
          <meshStandardMaterial color="#475569" metalness={0.6} roughness={0.4} />
        </mesh>
      ))}

      {/* Horizontal & Diagonal Bracing Rings */}
      {[-8.0, -6.0, -4.0, -2.0, -0.5].map((yLevel, idx) => (
        <group key={idx} position={[0, yLevel, 0]}>
          <mesh>
            <boxGeometry args={[1.65, 0.05, 1.65]} />
            <meshStandardMaterial color="#334155" />
          </mesh>
        </group>
      ))}

      {/* Summit Service Platform */}
      <mesh position={[0, 0, 0]}>
        <boxGeometry args={[2.0, 0.1, 2.0]} />
        <meshStandardMaterial color="#1E293B" metalness={0.7} />
      </mesh>

      {/* Platform Safety Railings */}
      <mesh position={[0, 0.45, 0]}>
        <boxGeometry args={[2.0, 0.9, 2.0]} />
        <meshBasicMaterial color="#64748B" wireframe />
      </mesh>

      {/* ======================================================== */}
      {/* 2. OPTICAL COLLIMATOR TEST INSTRUMENT HEAD               */}
      {/* ======================================================== */}
      <group position={[0, 0.5, 0]}>
        {/* Instrument Pier */}
        <mesh position={[0, 0.25, 0]}>
          <cylinderGeometry args={[0.2, 0.25, 0.5, 12]} />
          <meshStandardMaterial color="#0F172A" />
        </mesh>

        {/* Collimator Telescope Barrel */}
        <mesh position={[0, 0.6, 0]} rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[0.18, 0.18, 0.8, 16]} />
          <meshStandardMaterial color="#334155" metalness={0.8} roughness={0.3} />
        </mesh>

        {/* Front Lens Aperture Ring */}
        <mesh position={[0, 0.6, 0.42]} rotation={[Math.PI / 2, 0, 0]}>
          <torusGeometry args={[0.16, 0.03, 12, 24]} />
          <meshStandardMaterial color="#D97706" metalness={0.9} />
        </mesh>

        {/* Optical Test Source Laser Core */}
        {active && (
          <group position={[0, 0.6, 0.45]}>
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
