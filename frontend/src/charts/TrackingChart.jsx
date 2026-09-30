import React from 'react';
import { ResponsiveContainer, LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend } from 'recharts';

export default function TrackingChart({ data }) {
  return (
    <div className="w-full h-[220px] bg-white border border-slate-300 p-3 rounded-sm">
      <h3 className="text-xs font-semibold text-slate-700 mb-2 uppercase tracking-wider font-mono">
        Error Dynamics (Yaw/Pitch/Total)
      </h3>
      <ResponsiveContainer width="100%" height="90%">
        <LineChart data={data} margin={{ top: 5, right: 10, left: -20, bottom: 0 }}>
          <CartesianGrid strokeDasharray="3 3" stroke="#e2e8f0" />
          <XAxis dataKey="timeIndex" stroke="#64748B" fontSize={10} tickLine={false} />
          <YAxis stroke="#64748B" fontSize={10} tickLine={false} domain={[-6, 6]} />
          <Tooltip
            contentStyle={{ backgroundColor: '#ffffff', borderColor: '#cbd5e1', borderRadius: '2px', color: '#0f172a' }}
            labelStyle={{ color: '#475569', fontSize: '10px' }}
            itemStyle={{ fontSize: '12px' }}
          />
          <Legend iconType="circle" wrapperStyle={{ fontSize: '10px', marginTop: '5px' }} />
          <Line
            type="monotone"
            dataKey="totalError"
            name="Total Error (°)"
            stroke="#10B981" // Emerald
            strokeWidth={1.5}
            dot={false}
            activeDot={{ r: 4 }}
          />
          <Line
            type="monotone"
            dataKey="hError"
            name="Yaw Error (°)"
            stroke="#3B82F6" // Blue
            strokeWidth={1.2}
            dot={false}
          />
          <Line
            type="monotone"
            dataKey="vError"
            name="Pitch Error (°)"
            stroke="#EC4899" // Pink
            strokeWidth={1.2}
            dot={false}
          />
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}
