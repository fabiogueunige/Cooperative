# Trajectory Follower - Documentazione

## Trasformazione da Path Follower a Trajectory Follower

Il sistema è stato modificato da **Path Follower** (basato su Line-of-Sight) a **Trajectory Follower** (basato su timing parametrico).

---

## Differenze Principali

### Path Follower (Vecchio Sistema)
- **Approccio**: Seguiva geometricamente una linea tra waypoint
- **Metodo**: Line-of-Sight (LOS) - proiezione sulla linea
- **Controllo**: Solo feedback sulla posizione
- **Velocità**: Non controllata esplicitamente, dipendeva dal gain
- **Timing**: Non deterministico, dipendeva dall'errore

### Trajectory Follower (Nuovo Sistema)
- **Approccio**: Segue una traiettoria temporale pianificata
- **Metodo**: Interpolazione polinomiale quintica
- **Controllo**: Feedforward (velocità desiderata) + Feedback (correzione errore)
- **Velocità**: Esplicitamente controllata tramite interpolazione
- **Timing**: Deterministico, ogni waypoint ha un tempo specifico

---

## Nuovi File Creati

### 1. `ComputeTrajectoryPoint.m`
**Funzione**: Calcola pose, velocità e accelerazione desiderate usando interpolazione polinomiale quintica.

**Input**:
- `t_current`: tempo corrente
- `t_start`, `t_end`: inizio e fine del segmento
- `pose_start`, `pose_end`: pose iniziale e finale (4x4)

**Output**:
- `pose`: pose desiderata (4x4)
- `velocity`: velocità desiderata [ω; v] (6x1)
- `acceleration`: accelerazione desiderata [ω̇; v̇] (6x1)

**Caratteristiche**:
- Interpolazione quintica per la posizione: s(τ) = 10τ³ - 15τ⁴ + 6τ⁵
- SLERP per l'orientamento (Spherical Linear Interpolation)
- Garantisce continuità C² (posizione, velocità, accelerazione continue)
- Condizioni al contorno: velocità e accelerazione zero agli estremi

### 2. `GetCurrentTrajectorySegment.m`
**Funzione**: Identifica il segmento di traiettoria corrente in base al tempo.

**Input**:
- `t_current`: tempo corrente
- `trajectory`: struttura contenente waypoint e timing

**Output**:
- `segment_idx`: indice del segmento corrente
- `t_start`, `t_end`: tempi di inizio e fine
- `pose_start`, `pose_end`: pose iniziale e finale del segmento

---

## File Modificati

### 1. `InitRobot.m`
**Modifiche**:
- Aggiunta struttura `goal.trajectory` con:
  - `n_waypoints`: numero di waypoint
  - `poses`: array 4x4xN di pose
  - `times`: array 1xN di tempi assoluti
  - `duration`: durata totale della traiettoria
  - `current_segment`: segmento corrente
  - `completed`: flag di completamento

**Timing predefinito**:
```matlab
goal.trajectory.times(1) = 0.0;   % Inizio
goal.trajectory.times(2) = 15.0;  % Secondo waypoint dopo 15s
goal.trajectory.times(3) = 30.0;  % Terzo waypoint dopo 30s
goal.trajectory.times(4) = 45.0;  % Quarto waypoint dopo 45s
```

### 2. `ComputeTaskReferences.m`
**Modifiche nel Case 2 (Cooperative Manipulation)**:

**Prima (Path Follower)**:
```matlab
[ang, lin] = CartError(goal.los, pandaArm.ArmL.wTo); 
pandaArm.ArmL.xdot.tool = gain_obj * [ang; lin];
```

**Dopo (Trajectory Follower)**:
```matlab
[seg_idx, t_start, t_end, pose_start, pose_end] = 
    GetCurrentTrajectorySegment(goal.trajectory.time_in_phase, goal.trajectory);

[pose_desired, velocity_desired, ~] = 
    ComputeTrajectoryPoint(goal.trajectory.time_in_phase, t_start, t_end, pose_start, pose_end);

[ang, lin] = CartError(pose_desired, pandaArm.ArmL.wTo); 
pandaArm.ArmL.xdot.tool = velocity_desired + gain_obj * [ang; lin];
```

**Vantaggi**:
- `velocity_desired`: termine feedforward che anticipa il movimento
- `gain_obj * [ang; lin]`: termine feedback che corregge gli errori
- Tracking più preciso e fluido

### 3. `UpdateMissionPhase.m`
**Modifiche nel Case 1**:
- Inizializza `goal.trajectory.time_in_phase = 0.0`

**Modifiche nel Case 2**:
- Aggiorna `goal.trajectory.time_in_phase = mission.phase_time`
- Transizione basata su tempo: `time_in_phase >= duration`
- Aggiorna automaticamente `current_segment` in base al tempo

**Prima**: Transizione basata su errore quando il LOS raggiungeva il waypoint

**Dopo**: Transizione basata su tempo quando la traiettoria è completata

### 4. `main.m`
**Modifiche**:
- Rimossi riferimenti a `GetLosPoint`
- Rimossi calcoli geometrici di linee A-B
- Aggiornati debug prints per mostrare:
  - `desired_point`: punto desiderato dalla traiettoria
  - Errori di tracking invece che errori LOS

---

## Equazioni Matematiche

### Interpolazione Polinomiale Quintica
Per un parametro normalizzato τ ∈ [0,1]:

**Posizione**:
$$s(τ) = 10τ^3 - 15τ^4 + 6τ^5$$

**Velocità**:
$$\dot{s}(τ) = \frac{30τ^2 - 60τ^3 + 30τ^4}{T}$$

**Accelerazione**:
$$\ddot{s}(τ) = \frac{60τ - 180τ^2 + 120τ^3}{T^2}$$

Dove T è la durata del segmento.

### Legge di Controllo
**Trajectory Follower**:
$$\dot{x}_{ref} = \dot{x}_d(t) + K_p \cdot e(t)$$

Dove:
- $\dot{x}_d(t)$: velocità desiderata (feedforward)
- $e(t)$: errore di tracking
- $K_p$: guadagno proporzionale (gain_obj)

**Path Follower** (vecchio):
$$\dot{x}_{ref} = K_p \cdot e_{LOS}$$

Solo controllo feedback, nessun feedforward.

---

## Parametri da Regolare

### 1. Timing dei Waypoint
In `InitRobot.m`, modifica:
```matlab
goal.trajectory.times(1) = 0.0;
goal.trajectory.times(2) = 15.0;  % <-- Modifica questi valori
goal.trajectory.times(3) = 30.0;
goal.trajectory.times(4) = 45.0;
```

**Consigli**:
- Tempi più lunghi → movimento più lento e fluido
- Tempi più corti → movimento più veloce (rischio di errori maggiori)

### 2. Guadagno di Feedback
In `ComputeTaskReferences.m`:
```matlab
gain_obj = 0.8;  % <-- Modifica questo valore
```

**Consigli**:
- Guadagno alto → correzione rapida ma possibili oscillazioni
- Guadagno basso → correzione lenta ma più stabile

---

## Vantaggi del Trajectory Follower

1. **Timing Deterministico**: Sappiamo esattamente quando raggiungere ogni waypoint
2. **Velocità Controllata**: La velocità è esplicitamente pianificata e non dipende solo dall'errore
3. **Movimento Fluido**: L'interpolazione quintica garantisce continuità in accelerazione
4. **Migliore Tracking**: Il feedforward anticipa il movimento, riducendo l'errore
5. **Ripetibilità**: Lo stesso timing produce sempre lo stesso movimento
6. **Coordinazione**: Più facile coordinare con altri sistemi grazie al timing esplicito

---

## Come Testare

1. **Esegui la simulazione**: `main.m`
2. **Verifica il timing**: I waypoint dovrebbero essere raggiunti ai tempi specificati
3. **Controlla l'errore di tracking**: Dovrebbe essere minore rispetto al path follower
4. **Analizza i plot**: Confronta le traiettorie desiderate vs effettive

---

## Possibili Estensioni Future

1. **Traiettorie B-spline**: Per controllo locale e maggiore flessibilità
2. **Ottimizzazione del tempo**: Calcolo automatico dei tempi ottimali
3. **Vincoli di velocità/accelerazione**: Limitare dinamicamente i riferimenti
4. **Controllo predittivo (MPC)**: Anticipare vincoli e ostacoli
5. **Traiettorie adattive**: Modificare online in base a perturbazioni

---

## File Obsoleti (Non più utilizzati)

- `GetLosPoint.m`: Logica LOS non più necessaria (ma mantenuto per compatibilità)
- `prova_GetLosPoint.m`: Script di test per LOS

---

## Autore
Sistema modificato da Path Follower a Trajectory Follower
Data: Ottobre 2025
