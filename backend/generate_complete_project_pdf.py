import os
import sys
import math
from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, KeepTogether, HRFlowable
)
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import inch
from reportlab.pdfgen import canvas

class NumberedCanvas(canvas.Canvas):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._saved_page_states = []

    def showPage(self):
        self._saved_page_states.append(dict(self.__dict__))
        self._startPage()

    def save(self):
        num_pages = len(self._saved_page_states)
        for state in self._saved_page_states:
            self.__dict__.update(state)
            self.draw_page_decorations(num_pages)
            super().showPage()
        super().save()

    def draw_page_decorations(self, page_count):
        self.saveState()
        
        # Suppress headers/footers on first cover page if desired, but here page 1 is title block
        page_w, page_h = letter
        
        # Top Header line & text
        self.setStrokeColor(colors.HexColor("#1E293B"))
        self.setLineWidth(0.75)
        self.line(54, page_h - 40, page_w - 54, page_h - 40)
        
        self.setFont("Helvetica-Bold", 8)
        self.setFillColor(colors.HexColor("#3B82F6"))
        self.drawString(54, page_h - 34, "FSOC AI-BASED VIRTUAL CAMERA TRACKING SIMULATOR")
        
        self.setFont("Helvetica", 8)
        self.setFillColor(colors.HexColor("#64748B"))
        self.drawRightString(page_w - 54, page_h - 34, "COMPLETE SYSTEM GUIDE & WORKFLOW MANUAL")
        
        # Bottom Footer line & text
        self.line(54, 45, page_w - 54, 45)
        self.setFont("Helvetica", 8)
        self.setFillColor(colors.HexColor("#64748B"))
        self.drawString(54, 32, "Confidential & Proprietary — Aerospace Pointing, Acquisition & Tracking (PAT) System")
        
        page_text = f"Page {self._pageNumber} of {page_count}"
        self.drawRightString(page_w - 54, 32, page_text)
        
        self.restoreState()

def build_pdf(filename):
    doc = SimpleDocTemplate(
        filename,
        pagesize=letter,
        leftMargin=54,
        rightMargin=54,
        topMargin=54,
        bottomMargin=54
    )

    styles = getSampleStyleSheet()

    # Custom Color Palette
    PRIMARY = colors.HexColor("#0F172A")    # Dark Navy Slate
    SECONDARY = colors.HexColor("#1E293B")  # Slate Accent
    ACCENT_BLUE = colors.HexColor("#2563EB")# Royal Blue
    EMERALD_GREEN = colors.HexColor("#059669")# Emerald Green
    AMBER_GOLD = colors.HexColor("#D97706") # Amber
    PINK_MAGENTA = colors.HexColor("#DB2777")# Pink/Magenta
    TEXT_DARK = colors.HexColor("#1E293B")  # Dark Text
    TEXT_MUTED = colors.HexColor("#475569") # Muted Text
    BG_LIGHT = colors.HexColor("#F8FAFC")   # Soft Grey Background
    BORDER_COLOR = colors.HexColor("#E2E8F0")

    # Typography Styles
    title_style = ParagraphStyle(
        'DocTitle',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=24,
        leading=28,
        textColor=PRIMARY,
        spaceAfter=6
    )

    subtitle_style = ParagraphStyle(
        'DocSubTitle',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=12,
        leading=16,
        textColor=ACCENT_BLUE,
        spaceAfter=15
    )

    h1_style = ParagraphStyle(
        'SectionH1',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=15,
        leading=19,
        textColor=PRIMARY,
        spaceBefore=16,
        spaceAfter=8,
        keepWithNext=True
    )

    h2_style = ParagraphStyle(
        'SectionH2',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=11,
        leading=15,
        textColor=ACCENT_BLUE,
        spaceBefore=12,
        spaceAfter=6,
        keepWithNext=True
    )

    body_style = ParagraphStyle(
        'BodyDark',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=9.5,
        leading=13.5,
        textColor=TEXT_DARK,
        spaceAfter=6
    )

    bullet_style = ParagraphStyle(
        'BulletDark',
        parent=body_style,
        leftIndent=12,
        firstLineIndent=-8,
        spaceAfter=4
    )

    code_style = ParagraphStyle(
        'CodeSnippet',
        parent=styles['Normal'],
        fontName='Courier',
        fontSize=8.5,
        leading=11.5,
        textColor=colors.HexColor("#0F172A"),
        backColor=colors.HexColor("#F1F5F9"),
        borderColor=colors.HexColor("#CBD5E1"),
        borderWidth=0.5,
        borderPadding=6,
        spaceBefore=4,
        spaceAfter=6,
        borderRadius=4
    )

    callout_style = ParagraphStyle(
        'CalloutText',
        parent=styles['Normal'],
        fontName='Helvetica-Oblique',
        fontSize=9,
        leading=13,
        textColor=colors.HexColor("#1E3A8A"),
        backColor=colors.HexColor("#EFF6FF"),
        borderColor=colors.HexColor("#BFDBFE"),
        borderWidth=0.5,
        borderPadding=8,
        spaceBefore=6,
        spaceAfter=8
    )

    table_cell = ParagraphStyle(
        'TableCell',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=8.5,
        leading=11.5,
        textColor=TEXT_DARK
    )

    table_header = ParagraphStyle(
        'TableHeader',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=8.5,
        leading=11.5,
        textColor=colors.white
    )

    story = []

    # Title Block Banner
    story.append(Paragraph("FSOC AI-Based Virtual Camera Tracking", title_style))
    story.append(Paragraph("Complete Technical Reference, Target Color Identification & Operating Workflow Manual", subtitle_style))
    story.append(HRFlowable(width="100%", thickness=1.5, color=ACCENT_BLUE, spaceAfter=14))

    # Executive Overview
    story.append(Paragraph("1. Executive Summary & System Purpose", h1_style))
    story.append(Paragraph(
        "The <b>Free-Space Optical Communications (FSOC) AI-Based Virtual Camera Tracking Simulator</b> is an aerospace-grade closed-loop testing platform. Mobile FSOC terminals require extreme pointing precision (micro-radians) to establish and maintain directional optical laser links between ground stations, unmanned aerial vehicles (UAVs), satellites, and aircraft. This simulator emulates a 2-axis optical gimbal system operating under harsh environmental perturbations.",
        body_style
    ))
    story.append(Paragraph(
        "The platform bridges <b>dynamic 3D WebGL physics</b>, <b>synthetic pinhole camera projection</b>, <b>high-speed WebSocket streaming</b>, <b>hybrid neural detection (YOLOv8 + OpenCV fallback)</b>, <b>2D Kalman filter state estimation</b>, and <b>anti-windup PID actuator control</b> into a unified real-time suite.",
        body_style
    ))

    # Key Objectives Callout Box
    key_objs = Paragraph(
        "<b>Core Operational Objectives:</b><br/>"
        "• <b>Rapid Coarse Alignment:</b> Detect and center dynamic optical targets within camera FOV under 3 seconds.<br/>"
        "• <b>Occlusion & Noise Resilience:</b> Maintain trajectory tracking during temporary signal loss using Kalman prediction.<br/>"
        "• <b>Disturbance Rejection:</b> Suppress platform vibrations, camera jitter, atmospheric turbulence, and optical blur.<br/>"
        "• <b>Ablation & Benchmarking:</b> Evaluate system performance across 10 standardized aerospace testing scenarios.",
        callout_style
    )
    story.append(key_objs)

    # End to End Workflow Architecture
    story.append(Paragraph("2. End-to-End System Workflow Architecture", h1_style))
    story.append(Paragraph(
        "The end-to-end data pipeline forms a closed loop operating at up to 60 frames per second (FPS). Below is the step-by-step breakdown of how data flows through the system components:",
        body_style
    ))

    workflow_steps = [
        ("Step 1: 3D Scene Dynamics", "The WebGL engine (Three.js / React Three Fiber) simulates 4 distinct optical beacon targets moving along user-defined trajectories (Linear, Circular, Sinewave, Random, Spiral) in 3D world space (X, Y, Z)."),
        ("Step 2: Virtual Camera Projection", "The Virtual Camera Model (pinhole optics) projects 3D beacon position vectors into 2D camera sensor pixel coordinates (u, v) based on current Horizontal FOV (10.0°), Vertical FOV (7.5°), and 640x480 resolution."),
        ("Step 3: Bi-Directional WebSocket Stream", "The frontend transmits real-time frame telemetry to the Uvicorn FastAPI backend at `ws://localhost:8000/ws/tracking`. The backend returns state metrics, detection coordinates, and PID correction commands."),
        ("Step 4: Disturbance Engine Injection", "Before AI detection, 6-DoF environmental perturbations are synthetically injected into the frame: platform vibration, CCD sensor noise, optical blur, camera jitter, atmospheric turbulence, and illumination fluctuation."),
        ("Step 5: Hybrid AI Target Detection", "The backend passes frame data to YOLOv8 deep learning model. If confidence drops below threshold or GPU is unavailable, the system instantly triggers an automatic fail-safe fallback to fast OpenCV HSV color contour thresholding."),
        ("Step 6: 2D Kalman Filter Estimation", "Raw pixel coordinates are processed through a 4D Kalman filter state vector [x, y, vx, vy]^T. The filter smoothes out sensor noise and predicts target position during temporary occlusions/line-of-sight loss."),
        ("Step 7: PAT State Machine Transition", "The Pointing, Acquisition & Tracking (PAT) state machine evaluates error thresholds and updates the current state (SEARCHING → DETECTED → ACQUIRING → TRACKING → LOCKED → TARGET_LOST)."),
        ("Step 8: Discrete Anti-Windup PID Control", "Dual-axis discrete PID controllers compute proportional, integral, and derivative angular velocity commands for Yaw (Pan) and Pitch (Tilt) gimbals with anti-windup clamping to prevent actuator saturation.")
    ]

    for step_title, step_desc in workflow_steps:
        story.append(Paragraph(f"<b>{step_title}:</b> {step_desc}", bullet_style))

    story.append(Spacer(1, 10))

    # Target Identification & Color Coding Reference (USER SPECIFICALLY ASKED FOR THIS)
    story.append(Paragraph("3. Target Beacon & HUD Color Coding Guide", h1_style))
    story.append(Paragraph(
        "To ensure clear visual distinction during complex multi-target operations and telemetry debugging, specific colors have been assigned to 3D beacon targets, camera overlays, HUD crosshairs, and system status indicators:",
        body_style
    ))

    # Target Table
    target_data = [
        [Paragraph("Target / Element Name", table_header), Paragraph("Color Name & Hex Code", table_header), Paragraph("Role & Description", table_header)],
        
        [Paragraph("<b>BEACON-01</b><br/>(Primary Target)", table_cell), 
         Paragraph("<font color='#059669'><b>Emerald Green</b></font><br/><code>#10B981</code>", table_cell), 
         Paragraph("<b>Default Primary Beacon</b> under active closed-loop tracking. Represents the main optical laser transmitter.", table_cell)],
        
        [Paragraph("<b>BEACON-02</b><br/>(Secondary Target)", table_cell), 
         Paragraph("<font color='#2563EB'><b>Azure Blue</b></font><br/><code>#3B82F6</code>", table_cell), 
         Paragraph("Auxiliary target for multi-beacon acquisition and target switching stress tests.", table_cell)],
        
        [Paragraph("<b>BEACON-03</b><br/>(Tertiary Target)", table_cell), 
         Paragraph("<font color='#D97706'><b>Amber Yellow</b></font><br/><code>#F59E0B</code>", table_cell), 
         Paragraph("Interference target / solar glare decoy used to test background false positive rejection.", table_cell)],
        
        [Paragraph("<b>BEACON-04</b><br/>(Quaternary Target)", table_cell), 
         Paragraph("<font color='#DB2777'><b>Hot Pink</b></font><br/><code>#EC4899</code>", table_cell), 
         Paragraph("High-velocity dynamic target for testing rapid gimbal slewing limits.", table_cell)],

        [Paragraph("<b>Optical Boresight Reticle</b>", table_cell), 
         Paragraph("<font color='#EF4444'><b>Muted Red</b></font><br/><code>#EF4444</code> (20% Opacity)", table_cell), 
         Paragraph("Center crosshair & alignment circle on camera feed representing zero-error boresight center (320, 240).", table_cell)],

        [Paragraph("<b>AI Detector Bounding Box</b>", table_cell), 
         Paragraph("<font color='#2563EB'><b>Royal Blue Box</b></font><br/><code>#3B82F6</code>", table_cell), 
         Paragraph("Surrounds detected beacon in camera feed with live confidence tag (e.g. BEACON 0.95) and raw center dot.", table_cell)],

        [Paragraph("<b>Kalman Predicted Target</b>", table_cell), 
         Paragraph("<font color='#059669'><b>Pulsating Emerald</b></font><br/><code>#10B981 / #34D399</code>", table_cell), 
         Paragraph("Pulsating green ring and dot indicating filtered/predicted target position used by the PID controller.", table_cell)],

        [Paragraph("<b>Angular Error Vector Line</b>", table_cell), 
         Paragraph("<font color='#059669'><b>Dotted Green Line</b></font><br/><code>#34D399</code>", table_cell), 
         Paragraph("Dashed vector connecting central boresight reticle to target pixel, representing current tracking error.", table_cell)],

        [Paragraph("<b>Fallback Warning Badge</b>", table_cell), 
         Paragraph("<font color='#D97706'><b>Pulsating Amber</b></font><br/><code>#F59E0B</code>", table_cell), 
         Paragraph("Animated warning pill displayed on HUD when system switches from YOLOv8 to OpenCV HSV thresholding.", table_cell)]
    ]

    t_target = Table(target_data, colWidths=[1.5*inch, 1.6*inch, 3.9*inch])
    t_target.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), PRIMARY),
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('GRID', (0,0), (-1,-1), 0.5, BORDER_COLOR),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, BG_LIGHT]),
        ('TOPPADDING', (0,0), (-1,-1), 5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 5),
    ]))

    story.append(t_target)
    story.append(Spacer(1, 10))

    # PAT State Machine & Hysteresis Rules
    story.append(Paragraph("4. Pointing, Acquisition & Tracking (PAT) State Transitions", h1_style))
    story.append(Paragraph(
        "The system enforces a robust finite state machine to manage state transitions and avoid rapid oscillation (chattering) between states:",
        body_style
    ))

    pat_states = [
        ("1. SEARCHING", "Default initial state. The gimbal executes a pre-programmed raster scan pattern (Speed: 10°/s, Span: ±20° H / ±15° V) until a beacon enters the optical Field of View."),
        ("2. DETECTED", "Triggered immediately when AI detector detects target pixels within camera FOV. Initializes tracking candidate counter."),
        ("3. ACQUIRING", "Transitioned when target is consistently detected across 3 consecutive frames. Kalman filter initializes state vector [x, y, 0, 0]^T."),
        ("4. TRACKING", "Active closed-loop control. Discrete PID controller actively drives Pan and Tilt gimbals to align target with optical boresight center (320, 240)."),
        ("5. LOCKED", "Achieved when Total Angular Error remains below 0.15° continuously for at least 0.5 seconds. Represents optical communication link lock."),
        ("6. TARGET_LOST", "Triggered when beacon is occluded or lost. Kalman filter predicts trajectory for up to 3.0 seconds. If re-detected within 3s, system returns to TRACKING; otherwise returns to SEARCHING.")
    ]

    for st_title, st_desc in pat_states:
        story.append(Paragraph(f"<b>{st_title}:</b> {st_desc}", bullet_style))

    story.append(Spacer(1, 10))

    # System Architecture Components & Stack
    story.append(Paragraph("5. Technical Component Architecture & Tech Stack", h1_style))
    story.append(Paragraph(
        "The software architecture is split into high-performance Frontend and Backend subsystems designed for zero-lag real-time execution:",
        body_style
    ))

    tech_stack_data = [
        [Paragraph("Subsystem", table_header), Paragraph("Technologies & Libraries", table_header), Paragraph("Key Modules & Responsibilities", table_header)],
        
        [Paragraph("<b>Frontend UI & 3D Simulation</b>", table_cell), 
         Paragraph("React 18, Vite, Three.js, React Three Fiber, TailwindCSS, Zustand, Lucide Icons, Recharts", table_cell), 
         Paragraph("• <code>Dashboard.jsx</code>: Core application container.<br/>"
                   "• <code>SimulationScene.jsx</code>: 3D WebGL Canvas.<br/>"
                   "• <code>GimbalCamera.jsx</code>: 2-Axis gimbal mount model.<br/>"
                   "• <code>Beacon.jsx</code>: Multi-beacon spatial renderer.<br/>"
                   "• <code>CameraFeed.jsx</code>: Real-time HUD overlay.<br/>"
                   "• <code>TelemetryPanel.jsx</code>: Real-time charts.<br/>"
                   "• <code>ControlPanel.jsx</code>: Interactive tuning sliders.<br/>"
                   "• <code>useSimulationStore.js</code>: Global state store.", table_cell)],

        [Paragraph("<b>Backend Engine & AI Analytics</b>", table_cell), 
         Paragraph("Python 3.10+, FastAPI, Uvicorn, WebSockets, OpenCV (cv2), Ultralytics YOLOv8, NumPy, SciPy, ReportLab, Matplotlib", table_cell), 
         Paragraph("• <code>main.py</code>: FastAPI REST & WebSocket server.<br/>"
                   "• <code>camera/model.py</code>: Pinhole camera math.<br/>"
                   "• <code>detection/detector.py</code>: YOLO / HSV detector.<br/>"
                   "• <code>detection/disturbances.py</code>: 6-DoF disturbance engine.<br/>"
                   "• <code>tracking/kalman.py</code>: 2D linear Kalman filter.<br/>"
                   "• <code>tracking/state_machine.py</code>: PAT FSM transition logic.<br/>"
                   "• <code>tracking/metrics.py</code>: Benchmark metrics logging.<br/>"
                   "• <code>experiments/manager.py</code>: Scenario evaluator.<br/>"
                   "• <code>experiments/reports.py</code>: PDF report compiler.", table_cell)]
    ]

    t_stack = Table(tech_stack_data, colWidths=[1.5*inch, 2.2*inch, 3.3*inch])
    t_stack.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), PRIMARY),
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'TOP'),
        ('GRID', (0,0), (-1,-1), 0.5, BORDER_COLOR),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, BG_LIGHT]),
        ('TOPPADDING', (0,0), (-1,-1), 6),
        ('BOTTOMPADDING', (0,0), (-1,-1), 6),
    ]))

    story.append(t_stack)
    story.append(Spacer(1, 10))

    # Control Parameters & Tuning Specifications
    story.append(Paragraph("6. Control & Estimation Math Specifications", h1_style))
    story.append(Paragraph("<b>A. Discrete Anti-Windup PID Controller Equations:</b>", h2_style))
    story.append(Paragraph(
        "The gimbal pan and tilt angular error signals e_pan(t) and e_tilt(t) are processed via discrete PID control with anti-windup clamping to prevent integrator saturation during large step changes:",
        body_style
    ))
    story.append(Paragraph(
        "<code>u(t) = K_p · e(t) + K_i · Σ [e(t) · Δt] + K_d · [(e(t) - e(t - Δt)) / Δt]</code>",
        code_style
    ))
    story.append(Paragraph(
        "<b>Default Gain Constants:</b> Proportional K_p = 2.5, Integral K_i = 0.05, Derivative K_d = 0.15. Max actuator speed limit: ±180.0 °/s.",
        body_style
    ))

    story.append(Paragraph("<b>B. 2D Kalman Filter State Representation:</b>", h2_style))
    story.append(Paragraph(
        "State vector X = [x, y, v_x, v_y]^T. The state transition matrix F assumes constant velocity motion model between time steps Δt:",
        body_style
    ))
    story.append(Paragraph(
        "<code>F = [[1, 0, Δt, 0], [0, 1, 0, Δt], [0, 0, 1, 0], [0, 0, 0, 1]]</code>",
        code_style
    ))

    story.append(Spacer(1, 10))

    # Step-by-Step How to Use & Operations Guide
    story.append(Paragraph("7. Operating Manual & User Execution Workflow", h1_style))
    story.append(Paragraph(
        "Follow these step-by-step instructions to run, operate, test, and evaluate the FSOC Virtual Camera Tracking Simulator:",
        body_style
    ))

    op_steps = [
        ("Step 1: Start Backend Server", "Open Terminal 1 in the project directory:<br/>"
                                         "<code>cd backend</code><br/>"
                                         "<code>.venv\\Scripts\\activate</code> (Windows) or <code>source .venv/bin/activate</code> (Linux/Mac)<br/>"
                                         "<code>python -m uvicorn app.main:app --reload</code><br/>"
                                         "<i>Server starts on http://127.0.0.1:8000 with WebSocket active.</i>"),
        
        ("Step 2: Start Frontend Application", "Open Terminal 2 in the project directory:<br/>"
                                          "<code>cd frontend</code><br/>"
                                          "<code>npm run dev</code><br/>"
                                          "<i>Open http://localhost:5173 in Google Chrome or Microsoft Edge.</i>"),

        ("Step 3: Operating Dashboard & Target Selection", "• <b>Play / Pause:</b> Toggle simulation clock playback.<br/>"
                                                           "• <b>Target Beacon Selection:</b> Select active beacon (BEACON-01 Green, BEACON-02 Blue, BEACON-03 Amber, BEACON-04 Pink).<br/>"
                                                           "• <b>Trajectory Mode:</b> Switch target motion profiles (Linear, Circular, Sinewave, Random, Spiral).<br/>"
                                                           "• <b>Target Speed Slider:</b> Adjust velocity from 0.5 m/s up to 20.0 m/s."),

        ("Step 4: Disturbances & Environmental Stress Testing", "In the <b>Control Panel</b>, adjust disturbance sliders to evaluate system robustness:<br/>"
                                                                "• <b>Platform Vibration:</b> High-frequency mechanical jitter (0 - 100%).<br/>"
                                                                "• <b>Camera Jitter:</b> Structural wind buffeting.<br/>"
                                                                "• <b>Image Noise / Blur:</b> Atmospheric haze & CCD thermal noise.<br/>"
                                                                "• <b>Atmospheric Turbulence:</b> Optical refraction fluctuations."),

        ("Step 5: Ablation Studies & Feature Toggles", "Toggle individual system components on/off to isolate performance impact:<br/>"
                                                        "• <b>AI Detection Toggle:</b> Disables YOLO/HSV detector (forces open-loop mode).<br/>"
                                                        "• <b>Kalman Filter Toggle:</b> Disables state smoothing & occlusion prediction.<br/>"
                                                        "• <b>PID Controller Toggle:</b> Disables active gimbal correction.<br/>"
                                                        "• <b>Disturbances Toggle:</b> Disables environmental perturbations."),

        ("Step 6: Running Benchmarks & Generating Reports", "• Navigate to <b>Experiment Benchmarks Tab</b>.<br/>"
                                                             "• Select any of 10 aerospace scenarios (e.g. High Jitter, Dense Occlusion, Supersonic Slew).<br/>"
                                                             "• Click <b>Run Scenario</b> to initiate automated evaluation.<br/>"
                                                             "• Click <b>Export PDF Report</b> to download a formal 2-page evaluation document with Matplotlib charts.")
    ]

    for op_title, op_desc in op_steps:
        story.append(Paragraph(f"<b>{op_title}:</b>", body_style))
        story.append(Paragraph(op_desc, bullet_style))
        story.append(Spacer(1, 4))

    story.append(Spacer(1, 10))

    # Predefined Scenarios Table
    story.append(Paragraph("8. Predefined Aerospace Benchmark Scenarios", h1_style))
    story.append(Paragraph(
        "The platform includes 10 standardized test scenarios to validate tracking precision under stress:",
        body_style
    ))

    scenarios_data = [
        [Paragraph("ID", table_header), Paragraph("Scenario Name", table_header), Paragraph("Test Purpose & Configuration", table_header)],
        [Paragraph("EXP-01", table_cell), Paragraph("Baseline Nominal Tracking", table_cell), Paragraph("Zero disturbances, linear trajectory at 5.0 m/s. Establishes baseline PID response.", table_cell)],
        [Paragraph("EXP-02", table_cell), Paragraph("High Mechanical Jitter", table_cell), Paragraph("Vibration = 80%, Jitter = 70%. Tests Kalman filter noise suppression capabilities.", table_cell)],
        [Paragraph("EXP-03", table_cell), Paragraph("Severe Atmospheric Blur", table_cell), Paragraph("Blur = 90%, Image Noise = 60%. Tests OpenCV color thresholding & YOLO robustness.", table_cell)],
        [Paragraph("EXP-04", table_cell), Paragraph("Dense Signal Occlusion", table_cell), Paragraph("Beacon occluded for 2.5s intervals. Tests Kalman trajectory velocity prediction.", table_cell)],
        [Paragraph("EXP-05", table_cell), Paragraph("Supersonic Target Slew", table_cell), Paragraph("Target speed = 25.0 m/s with sharp 90° direction changes. Tests PID slewing response.", table_cell)],
        [Paragraph("EXP-06", table_cell), Paragraph("Multi-Beacon Interference", table_cell), Paragraph("BEACON-03 (Amber) crosses path of BEACON-01 (Green). Tests decoy rejection.", table_cell)],
        [Paragraph("EXP-07", table_cell), Paragraph("Low Illumination Solar Dusk", table_cell), Paragraph("Illumination variation = 95%. Tests optical sensor gain & exposure handling.", table_cell)],
        [Paragraph("EXP-08", table_cell), Paragraph("Raster Search Recovery", table_cell), Paragraph("Target starts 40° out of FOV. Tests automatic raster search scan pattern efficiency.", table_cell)],
        [Paragraph("EXP-09", table_cell), Paragraph("Full Environmental Stress", table_cell), Paragraph("All 6 disturbance sliders at 75%. Evaluates overall system break-lock limits.", table_cell)],
        [Paragraph("EXP-10", table_cell), Paragraph("Ablation Baseline (No Kalman)", table_cell), Paragraph("Kalman filter disabled under jitter. Quantifies exact RMSE degradation without Kalman.", table_cell)]
    ]

    t_scenarios = Table(scenarios_data, colWidths=[0.8*inch, 2.0*inch, 4.2*inch])
    t_scenarios.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), PRIMARY),
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('GRID', (0,0), (-1,-1), 0.5, BORDER_COLOR),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, BG_LIGHT]),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
    ]))

    story.append(t_scenarios)
    story.append(Spacer(1, 14))

    # Document Conclusion Block
    story.append(Paragraph("9. Verification & Conclusion", h1_style))
    story.append(Paragraph(
        "This project successfully demonstrates a complete end-to-end Free-Space Optical Communications tracking simulator. By combining deep learning AI, state estimation, and physical gimbal dynamics, the system achieves sub-pixel tracking accuracy and resilient closed-loop lock retention under extreme environmental disturbances.",
        body_style
    ))
    
    doc.build(story, canvasmaker=NumberedCanvas)
    print(f"PDF successfully generated at: {filename}")

if __name__ == "__main__":
    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "FSOC_Project_Complete_Guide_and_Workflow.pdf")
    build_pdf(out_path)
    
    # Also save copy in documentation directory
    doc_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "documentation", "FSOC_Project_Complete_Guide_and_Workflow.pdf")
    build_pdf(doc_path)
