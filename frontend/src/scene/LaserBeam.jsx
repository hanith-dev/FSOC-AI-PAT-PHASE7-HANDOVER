import React, { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { useSimulationStore } from '../store/useSimulationStore';

export default function LaserBeam() {
  const beamCoreRef = useRef();
  const beamEnvelopeRef = useRef();
  const rayleighRef = useRef();

  const beaconPosition = useSimulationStore((s) => s.beaconPosition);
  const beaconActive = useSimulationStore((s) => s.beaconActive);
  const trackingState = useSimulationStore((s) => s.trackingState);
  const laserBeamVisible = useSimulationStore((s) => s.laserBeamVisible !== false);
  const cn2 = useSimulationStore((s) => s.environmentalParameters?.cn2 || 1e-15);

  // Optical Ground Station Aperture origin position
  const groundOrigin = new THREE.Vector3(0, 1.7, 0);

  useFrame((state) => {
    if (!beamCoreRef.current || !beaconActive || !laserBeamVisible) return;

    const targetPos = new THREE.Vector3(...beaconPosition);
    const direction = new THREE.Vector3().subVectors(targetPos, groundOrigin);
    const distance = direction.length();

    // Position cylinder midway between ground station and target
    const midpoint = new THREE.Vector3().addVectors(groundOrigin, targetPos).multiplyScalar(0.5);
    beamCoreRef.current.position.copy(midpoint);
    beamCoreRef.current.scale.set(1, distance, 1);

    // Orient cylinder along the link direction vector
    const orientation = new THREE.Quaternion();
    const upAxis = new THREE.Vector3(0, 1, 0);
    orientation.setFromUnitVectors(upAxis, direction.clone().normalize());
    beamCoreRef.current.quaternion.copy(orientation);

    if (beamEnvelopeRef.current) {
      beamEnvelopeRef.current.position.copy(midpoint);
      beamEnvelopeRef.current.scale.set(1, distance, 1);
      beamEnvelopeRef.current.quaternion.copy(orientation);
    }

    if (rayleighRef.current) {
      // Atmospheric boundary layer Rayleigh backscatter (first 25m)
      const rayleighDist = Math.min(distance, 25.0);
      const rayleighMid = new THREE.Vector3().addVectors(
        groundOrigin,
        groundOrigin.clone().add(direction.clone().normalize().multiplyScalar(rayleighDist))
      ).multiplyScalar(0.5);
      rayleighRef.current.position.copy(rayleighMid);
      rayleighRef.current.scale.set(1, rayleighDist, 1);
      rayleighRef.current.quaternion.copy(orientation);
    }

    // Turbulence Scintillation Intensity Modulation
    const turbFactor = Math.min(2.0, Math.max(0.2, (Math.log10(cn2) + 17) / 4.0));
    const scintillation = 1.0 + 0.25 * turbFactor * Math.sin(state.clock.getElapsedTime() * 18.0);
    
    if (beamCoreRef.current.material) {
      beamCoreRef.current.material.opacity = Math.min(0.95, 0.7 * scintillation);
    }
  });

  if (!beaconActive || !laserBeamVisible) return null;

  // Tracking state status: GREEN when LOCKED/TRACKING, AMBER when COASTING, CYAN for SEARCHING/NOMINAL
  const isLocked = trackingState === 'LOCKED' || trackingState === 'TRACKING';
  const isCoasting = trackingState === 'COASTING' || trackingState === 'COAST';
  const beamColor = isLocked ? '#10B981' : (isCoasting ? '#F59E0B' : '#06B6D4');

  return (
    <group>
      {/* 1. Core High-Intensity Collimated Laser Beam */}
      <mesh ref={beamCoreRef}>
        <cylinderGeometry args={[0.04, 0.08, 1.0, 16, 1, true]} />
        <meshBasicMaterial
          color={beamColor}
          transparent
          opacity={0.75}
          side={THREE.DoubleSide}
        />
      </mesh>

      {/* 2. Micro-Radian Beam Divergence Envelope Cone */}
      <mesh ref={beamEnvelopeRef}>
        <cylinderGeometry args={[0.06, 0.45, 1.0, 16, 1, true]} />
        <meshBasicMaterial
          color={beamColor}
          transparent
          opacity={0.15}
          side={THREE.DoubleSide}
          wireframe
        />
      </mesh>

      {/* 3. Atmospheric Rayleigh Backscatter Boundary Layer Glow */}
      <mesh ref={rayleighRef}>
        <cylinderGeometry args={[0.08, 0.2, 1.0, 16, 1, true]} />
        <meshBasicMaterial
          color="#38BDF8"
          transparent
          opacity={0.3}
          side={THREE.DoubleSide}
        />
      </mesh>

      {/* 4. Terminal Beam Emitter Glow at Aperture */}
      <mesh position={[0, 1.7, 0]}>
        <sphereGeometry args={[0.15, 16, 16]} />
        <meshBasicMaterial color={beamColor} transparent opacity={0.6} />
      </mesh>
    </group>
  );
}
