import { useEffect, useRef, useCallback } from 'react';
import { useSimulationStore } from '../store/useSimulationStore';

export function useWebSocket() {
  const socketRef = useRef(null);
  const reconnectTimeoutRef = useRef(null);
  const frameCountRef = useRef(0);
  const sentTimesRef = useRef(new Map());
  
  const { setWsStatus, updateWsMetrics, setTelemetry, setWsSender } = useSimulationStore();
  const connectRef = useRef(null);
  
  const connect = useCallback(() => {
    // Dynamically resolve WebSocket URL based on host/port
    let wsUrl = 'ws://127.0.0.1:8000/ws/tracking';
    if (import.meta.env.VITE_WS_URL) {
      wsUrl = `${import.meta.env.VITE_WS_URL.replace(/^http/, 'ws')}/ws/tracking`;
    } else if (typeof window !== 'undefined' && window.location) {
      const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
      if (window.location.port !== '5173') {
        wsUrl = `${protocol}//${window.location.host}/ws/tracking`;
      } else {
        wsUrl = `${protocol}//${window.location.hostname}:8000/ws/tracking`;
      }
    }
    
    console.log(`Connecting to WebSocket: ${wsUrl}`);
    const socket = new WebSocket(wsUrl);
    socketRef.current = socket;
    
    socket.onopen = () => {
      console.log('WebSocket Connection Established');
      setWsStatus(true);
      // Bind sender
      setWsSender((msg) => {
        if (socketRef.current && socketRef.current.readyState === WebSocket.OPEN) {
          socketRef.current.send(msg);
        }
      });
    };
    
    socket.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        
        if (message.type === 'telemetry') {
          // Update tracker state machine and Kalman prediction metrics
          setTelemetry(message);
          
          // Also handle network metrics
          const { frame_index, bytes_received, fps } = message;
          const sentTime = sentTimesRef.current.get(frame_index);
          let latency = 0;
          if (sentTime) {
            latency = performance.now() - sentTime;
            sentTimesRef.current.delete(frame_index);
          }
          updateWsMetrics(fps, bytes_received, frame_index, Math.round(latency));
        } else if (message.type === 'ack') {
          const { frame_index, bytes_received, fps } = message;
          const sentTime = sentTimesRef.current.get(frame_index);
          let latency = 0;
          if (sentTime) {
            latency = performance.now() - sentTime;
            sentTimesRef.current.delete(frame_index);
          }
          updateWsMetrics(fps, bytes_received, frame_index, Math.round(latency));
        } else if (message.type === 'connection_status') {
          console.log('Server message:', message.message);
        }
      } catch (e) {
        console.error('Error parsing WS message:', e);
      }
    };
    
    socket.onclose = () => {
      console.log('WebSocket Connection Closed. Attempting reconnect...');
      setWsStatus(false);
      setWsSender(null);
      
      // Auto-reconnect after 3 seconds
      reconnectTimeoutRef.current = setTimeout(() => {
        if (connectRef.current) {
          connectRef.current();
        }
      }, 3000);
    };
    
    socket.onerror = (error) => {
      console.error('WebSocket Error:', error);
      socket.close();
    };
  }, [setWsStatus, updateWsMetrics, setTelemetry, setWsSender]);

  useEffect(() => {
    connectRef.current = connect;
  }, [connect]);

  useEffect(() => {
    connect();
    
    return () => {
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
      }
      if (socketRef.current) {
        socketRef.current.close();
      }
    };
  }, [connect]);

  // Send frame with optional metadata
  const sendFrame = useCallback((blobOrDataUrl, metadata = null) => {
    const socket = socketRef.current;
    if (!socket || socket.readyState !== WebSocket.OPEN) return;
    
    frameCountRef.current += 1;
    const currentFrameIndex = frameCountRef.current;
    
    // Save sent timestamp
    sentTimesRef.current.set(currentFrameIndex, performance.now());
    
    // Cap Map size to prevent memory leaks if ACKs are missed
    if (sentTimesRef.current.size > 50) {
      const oldestKey = sentTimesRef.current.keys().next().value;
      sentTimesRef.current.delete(oldestKey);
    }
    
    if (typeof blobOrDataUrl === 'string') {
      // Direct base64 Data URL send - zero async FileReader overhead!
      socket.send(JSON.stringify({
        type: 'frame',
        frame_index: currentFrameIndex,
        image: blobOrDataUrl,
        metadata: metadata
      }));
      return;
    }
    
    const reader = new FileReader();
    if (metadata) {
      reader.onload = () => {
        if (socket.readyState === WebSocket.OPEN) {
          socket.send(JSON.stringify({
            type: 'frame',
            frame_index: currentFrameIndex,
            image: reader.result, // base64 Data URL
            metadata: metadata
          }));
        }
      };
      reader.readAsDataURL(blobOrDataUrl);
    } else {
      // Fallback binary send
      reader.onload = () => {
        if (socket.readyState === WebSocket.OPEN) {
          socket.send(reader.result);
        }
      };
      reader.readAsArrayBuffer(blobOrDataUrl);
    }
  }, []);

  return { sendFrame };
}
