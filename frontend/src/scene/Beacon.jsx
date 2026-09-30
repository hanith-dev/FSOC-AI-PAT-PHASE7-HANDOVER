import React from 'react';
import { useSimulationStore } from '../store/useSimulationStore';
import SatelliteModel from './SatelliteModel';
import UavModel from './UavModel';
import CalibrationTowerModel from './CalibrationTowerModel';

export default function Beacon({ id }) {
  const beacon = useSimulationStore((state) => 
    state.beacons.find(b => b.id === id)
  );
  const targetPlatformModel = useSimulationStore((state) => 
    state.targetPlatformModel || 'satellite'
  );

  if (!beacon || !beacon.active) return null;

  return (
    <group position={beacon.position}>
      {targetPlatformModel === 'uav' ? (
        <UavModel
          size={beacon.size}
          color={beacon.color}
          intensity={beacon.intensity}
          active={beacon.active}
          designated={beacon.designated}
        />
      ) : targetPlatformModel === 'calibration_tower' ? (
        <CalibrationTowerModel
          size={beacon.size}
          color={beacon.color}
          intensity={beacon.intensity}
          active={beacon.active}
        />
      ) : (
        <SatelliteModel
          size={beacon.size}
          color={beacon.color}
          intensity={beacon.intensity}
          active={beacon.active}
          designated={beacon.designated}
          velocity={beacon.velocity}
        />
      )}
    </group>
  );
}
