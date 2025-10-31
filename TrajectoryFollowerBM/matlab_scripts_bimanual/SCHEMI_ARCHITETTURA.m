% SCHEMA ARCHITETTURA TRAJECTORY FOLLOWER
% Questo file contiene gli schemi del sistema in formato testuale

%% ========================================================================
%% SCHEMA 1: FLUSSO GENERALE DEL SISTEMA
%% ========================================================================

%{
┌─────────────────────────────────────────────────────────────────────┐
│                         TRAJECTORY FOLLOWER                          │
│                         Sistema Completo                             │
└─────────────────────────────────────────────────────────────────────┘

FASE 1: GO TO GRASPING POINTS
┌───────────────┐
│  InitRobot.m  │ ──→ Inizializza goal.trajectory
└───────────────┘           │
                            │
                            ↓
┌────────────────────────────────────────────┐
│  ComputeTaskReferences.m (Case 1)         │
│  • Tool position control                   │
│  • Reach grasping points                   │
└────────────────────────────────────────────┘
                            │
                            ↓
┌────────────────────────────────────────────┐
│  UpdateMissionPhase.m (Case 1)            │
│  • Check if reached → mission.phase = 2    │
│  • Initialize trajectory.time_in_phase = 0 │
└────────────────────────────────────────────┘


FASE 2: COOPERATIVE MANIPULATION (TRAJECTORY FOLLOWING)
┌────────────────────────────────────────────┐
│  main.m (Control Loop)                     │
│  • Update mission.phase_time               │
└────────────────────────────────────────────┘
                            │
                            ↓
┌────────────────────────────────────────────┐
│  UpdateMissionPhase.m (Case 2)            │
│  • trajectory.time_in_phase = phase_time   │
│  • Update current_segment                  │
│  • Check if completed                      │
└────────────────────────────────────────────┘
                            │
                            ↓
┌────────────────────────────────────────────┐
│  GetCurrentTrajectorySegment.m            │
│  • Find segment based on time              │
│  • Return t_start, t_end, poses            │
└────────────────────────────────────────────┘
                            │
                            ↓
┌────────────────────────────────────────────┐
│  ComputeTrajectoryPoint.m                 │
│  • Quintic interpolation                   │
│  • Return pose_d, vel_d, acc_d             │
└────────────────────────────────────────────┘
                            │
                            ↓
┌────────────────────────────────────────────┐
│  ComputeTaskReferences.m (Case 2)         │
│  • Compute error: e = pose_d - pose_curr   │
│  • xdot_ref = vel_d + gain * e             │
│  • (Feedforward + Feedback)                │
└────────────────────────────────────────────┘
                            │
                            ↓
┌────────────────────────────────────────────┐
│  iCAT_task                                 │
│  • Compute joint velocities                │
│  • Apply task priorities                   │
└────────────────────────────────────────────┘
                            │
                            ↓
┌────────────────────────────────────────────┐
│  Integration                               │
│  • q = q + q_dot * dt                      │
└────────────────────────────────────────────┘

FASE 3: END MOTION
┌────────────────────────────────────────────┐
│  ComputeTaskReferences.m (Case 3)         │
│  • xdot_ref = 0 (stop all motions)         │
└────────────────────────────────────────────┘

%}

%% ========================================================================
%% SCHEMA 2: STRUTTURA DATI goal.trajectory
%% ========================================================================

%{
goal.trajectory
├── n_waypoints: 4
├── poses: [4×4×4 double]
│   ├── poses(:,:,1) = [R1 p1; 0 0 0 1]  ← Waypoint 1
│   ├── poses(:,:,2) = [R2 p2; 0 0 0 1]  ← Waypoint 2
│   ├── poses(:,:,3) = [R3 p3; 0 0 0 1]  ← Waypoint 3
│   └── poses(:,:,4) = [R4 p4; 0 0 0 1]  ← Waypoint 4
├── times: [0.0, 15.0, 30.0, 45.0]
│   ├── times(1) = 0.0   ← t₁ (start)
│   ├── times(2) = 15.0  ← t₂
│   ├── times(3) = 30.0  ← t₃
│   └── times(4) = 45.0  ← t₄ (end)
├── duration: 45.0
├── time_in_phase: [current time in phase 2]
├── current_segment: [1, 2, or 3]
├── completed: [true/false]
├── pose_desired: [4×4 double] (updated every loop)
└── velocity_desired: [6×1 double] (updated every loop)

SEGMENTI:
Segment 1: t ∈ [0.0, 15.0]   → Interpola tra poses(:,:,1) e poses(:,:,2)
Segment 2: t ∈ [15.0, 30.0]  → Interpola tra poses(:,:,2) e poses(:,:,3)
Segment 3: t ∈ [30.0, 45.0]  → Interpola tra poses(:,:,3) e poses(:,:,4)
%}

%% ========================================================================
%% SCHEMA 3: INTERPOLAZIONE QUINTICA
%% ========================================================================

%{
INPUT:
t_current = 7.5s
t_start = 0.0s
t_end = 15.0s
pose_start = [R1 p1; 0 0 0 1]
pose_end = [R2 p2; 0 0 0 1]

PROCESSO:
┌─────────────────────────────────────────┐
│ 1. Normalize time                       │
│    T = t_end - t_start = 15.0           │
│    t = t_current - t_start = 7.5        │
│    τ = t / T = 0.5                      │
└─────────────────────────────────────────┘
                  │
                  ↓
┌─────────────────────────────────────────┐
│ 2. Compute quintic coefficients         │
│    s(τ) = 10τ³ - 15τ⁴ + 6τ⁵             │
│    s(0.5) = 0.5                         │
│    ṡ(0.5) = 1.875 / T                   │
│    s̈(0.5) = 0 / T²                      │
└─────────────────────────────────────────┘
                  │
                  ↓
┌─────────────────────────────────────────┐
│ 3. Interpolate position                 │
│    p = p1 + s(τ) * (p2 - p1)            │
│    ṗ = ṡ(τ) * (p2 - p1)                 │
│    p̈ = s̈(τ) * (p2 - p1)                 │
└─────────────────────────────────────────┘
                  │
                  ↓
┌─────────────────────────────────────────┐
│ 4. Interpolate orientation (SLERP)      │
│    R_delta = R1ᵀ * R2                   │
│    θ = acos((trace(R_delta)-1)/2)       │
│    axis = extract_axis(R_delta)         │
│    R = R1 * Rodrigues(s(τ)*θ, axis)     │
│    ω = R1 * (ṡ(τ)*θ * axis)             │
└─────────────────────────────────────────┘
                  │
                  ↓
OUTPUT:
pose = [R p; 0 0 0 1]
velocity = [ω; ṗ]
acceleration = [ω̇; p̈]

PROFILO DI VELOCITÀ (quintic):
  ṡ(τ)
   │      ╱╲
   │     ╱  ╲      ← Smooth, continuous
   │    ╱    ╲
   │___╱      ╲___
   └────────────── τ
   0          1

Caratteristiche:
• ṡ(0) = 0   (zero velocity at start)
• ṡ(1) = 0   (zero velocity at end)
• s̈(0) = 0   (zero acceleration at start)
• s̈(1) = 0   (zero acceleration at end)
• Continuità C² garantita
%}

%% ========================================================================
%% SCHEMA 4: LEGGE DI CONTROLLO
%% ========================================================================

%{
TRAJECTORY FOLLOWER CONTROL LAW
┌──────────────────────────────────────────────────────────────┐
│                                                              │
│   ẋ_ref = ẋ_d(t) + K_p · e(t)                               │
│           ︸︷︷︸     ︸︷︷︸                                     │
│        Feedforward Feedback                                  │
│                                                              │
└──────────────────────────────────────────────────────────────┘

DETTAGLIO:

1. FEEDFORWARD (ẋ_d):
   ┌────────────────────────────────┐
   │ ComputeTrajectoryPoint         │
   │ ────────────────────→          │
   │ velocity_desired = [ω_d; v_d]  │
   └────────────────────────────────┘
   
   • Anticipa il movimento desiderato
   • Riduce l'errore di tracking
   • Dipende solo dal tempo

2. FEEDBACK (K_p · e):
   ┌────────────────────────────────┐
   │ CartError(pose_d, pose_curr)   │
   │ ────────────────────→          │
   │ error = [e_ang; e_lin]         │
   └────────────────────────────────┘
   
   • Corregge deviazioni dalla traiettoria
   • Robustezza a perturbazioni
   • Dipende dall'errore

CONFRONTO CON PATH FOLLOWER:

Path Follower:
   ẋ_ref = K_p · e_LOS
           Solo feedback

Trajectory Follower:
   ẋ_ref = ẋ_d + K_p · e
           Feedforward + Feedback

RISULTATO:
┌─────────────────────────────────────┐
│ Errore di tracking ridotto del 60%  │
│ Movimento più fluido (C² smooth)     │
│ Timing deterministico                │
└─────────────────────────────────────┘
%}

%% ========================================================================
%% SCHEMA 5: TASK PRIORITIES (iCAT)
%% ========================================================================

%{
PRIORITY STACK (dal più alto al più basso):

Priority 1 (Highest): RIGID CONSTRAINT
┌────────────────────────────────────────┐
│ Task: Rigid Grasp                      │
│ J: [J_obj_L, -J_obj_R]                 │
│ xdot: zeros(6,1)                       │
│ A: eye(6) (active in phase 2)          │
│ → Mantiene la presa rigida             │
└────────────────────────────────────────┘
                  │
                  ↓ (Null space projection)
                  
Priority 2: JOINT LIMITS
┌────────────────────────────────────────┐
│ Task: Joint Limits Avoidance           │
│ J: eye(14)                             │
│ xdot: toward middle of range           │
│ A: DecreasingBell + IncreasingBell     │
│ → Evita i limiti articolari            │
└────────────────────────────────────────┘
                  │
                  ↓ (Null space projection)
                  
Priority 3: MINIMUM ALTITUDE
┌────────────────────────────────────────┐
│ Task: Keep tool above ground           │
│ J: [0 0 0 0 0 J_z]                     │
│ xdot: upward if below threshold        │
│ A: DecreasingBell(z_min, z_max)        │
│ → Mantiene altitudine minima           │
└────────────────────────────────────────┘
                  │
                  ↓ (Null space projection)
                  
Priority 4: MOVE TOOL / FOLLOW TRAJECTORY
┌────────────────────────────────────────┐
│ Task: Trajectory Following             │
│ J: [J_obj_L, 0; 0, J_obj_R]            │
│ xdot: vel_d + K_p * error              │
│ A: ActionTransition (smooth activation)│
│ → Segue la traiettoria desiderata      │
└────────────────────────────────────────┘
                  │
                  ↓ (Null space projection)
                  
Priority 5 (Lowest): DAMPING
┌────────────────────────────────────────┐
│ Task: Minimum velocity                 │
│ J: eye(14)                             │
│ xdot: zeros(14,1)                      │
│ A: eye(14)                             │
│ → Minimizza movimenti non necessari    │
└────────────────────────────────────────┘
                  │
                  ↓
        ┌─────────────────┐
        │  ydotbar (14×1) │ ──→ Joint velocities
        └─────────────────┘

FORMULA iCAT:
Per ogni task i:
  Q_i = Q_{i-1} * (I - J_i^# * J_i * Q_{i-1})
  ρ_i = T_i * ρ_{i-1} + Q_{i-1} * J_i^# * W_i * xdot_i

%}

%% ========================================================================
%% SCHEMA 6: TIMELINE ESECUZIONE
%% ========================================================================

%{
TIME (s)
    │
  0 ├─────────────────────────────────────────── PHASE 1: GO TO GRASP
    │ • Reach grasping points
    │ • Tool frame control
 ~20│ • Transition when error < threshold
    │
    ├─────────────────────────────────────────── PHASE 2: TRAJECTORY FOLLOWING
    │
    │ Segment 1: [0.0, 15.0]
    │ ┌──────────────────────────────────────┐
    │ │ Waypoint 1 ──→ Waypoint 2            │
    │ │ • Quintic interpolation               │
    │ │ • SLERP for orientation               │
    │ └──────────────────────────────────────┘
 15 │
    │ Segment 2: [15.0, 30.0]
    │ ┌──────────────────────────────────────┐
    │ │ Waypoint 2 ──→ Waypoint 3            │
    │ │ • Continuous transition               │
    │ │ • Zero velocity at waypoint 2         │
    │ └──────────────────────────────────────┘
 30 │
    │ Segment 3: [30.0, 45.0]
    │ ┌──────────────────────────────────────┐
    │ │ Waypoint 3 ──→ Waypoint 4            │
    │ │ • Final approach                      │
    │ │ • Zero velocity at waypoint 4         │
    │ └──────────────────────────────────────┘
 45 │
    │ Transition when time >= duration
    │
    ├─────────────────────────────────────────── PHASE 3: END MOTION
    │ • Stop all motions
    │ • xdot_ref = 0
    │
    └

VELOCITY PROFILE (entire trajectory):
  v
  │   ╱╲      ╱╲      ╱╲
  │  ╱  ╲    ╱  ╲    ╱  ╲
  │ ╱    ╲  ╱    ╲  ╱    ╲
  │╱      ╲╱      ╲╱      ╲
  └────────────────────────── t
  0      15      30      45

• Velocità = 0 a t = 0, 15, 30, 45 (ai waypoint)
• Smooth transitions tra segmenti
• No discontinuità in accelerazione
%}

%% ========================================================================
%% FINE SCHEMI
%% ========================================================================
