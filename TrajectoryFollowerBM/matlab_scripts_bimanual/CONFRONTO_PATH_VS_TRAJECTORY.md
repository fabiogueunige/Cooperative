# Confronto: Path Follower vs Trajectory Follower

## Tabella Comparativa

| Caratteristica | Path Follower (LOS) | Trajectory Follower |
|----------------|---------------------|---------------------|
| **Approccio** | Geometrico (Line-of-Sight) | Temporale (Time-based) |
| **Controllo Velocità** | Indiretto (tramite gain) | Diretto (feedforward + feedback) |
| **Timing** | Non deterministico | Deterministico |
| **Continuità** | C⁰ (solo posizione) | C² (posizione, velocità, accelerazione) |
| **Complessità** | Bassa | Media |
| **Precisione** | Media | Alta |
| **Ripetibilità** | Bassa | Alta |
| **Adatto per** | Path seguici-linea | Traiettorie temporizzate |

---

## Dettaglio delle Differenze

### 1. Calcolo del Riferimento

#### Path Follower (LOS)
```matlab
% Proiezione sulla linea tra waypoint
v = B - A;  % Direzione
t = dot(P - A, v) / dot(v, v);  % Parametro
Q = A + t * v;  % Proiezione
Los = Q + normalize(v) * offset;  % Punto avanti

% Solo feedback
[ang, lin] = CartError(Los, current_pose);
xdot_ref = gain * [ang; lin];
```

**Vantaggi:**
- Semplice da capire e implementare
- Segue geometricamente il path
- Non richiede timing esplicito

**Svantaggi:**
- Velocità dipende dall'errore e dal gain
- Movimento non fluido (discontinuità in accelerazione)
- Difficile predire quando arriverà al goal
- Sensibile a perturbazioni

#### Trajectory Follower
```matlab
% Interpolazione temporale
[pose_d, vel_d, acc_d] = ComputeTrajectoryPoint(t, t_start, t_end, pose_start, pose_end);

% Feedforward + Feedback
[ang, lin] = CartError(pose_d, current_pose);
xdot_ref = vel_d + gain * [ang; lin];
          ↑          ↑
   Feedforward   Feedback
```

**Vantaggi:**
- Velocità esplicitamente controllata
- Movimento fluido (C² continuity)
- Timing deterministico
- Migliore tracking performance
- Riduce l'errore di inseguimento

**Svantaggi:**
- Richiede pianificazione del timing
- Leggermente più complesso

---

### 2. Transizione tra Waypoint

#### Path Follower
```matlab
% Transizione quando LOS raggiunge il waypoint
[ang, lin] = CartError(future_waypoint, los_point);
if lin <= threshold && ang <= threshold_ang
    % Passa al prossimo waypoint
    previous = current;
    current = next;
end
```

**Problema:** Il timing dipende da:
- Guadagno del controllore
- Condizioni iniziali
- Perturbazioni esterne

#### Trajectory Follower
```matlab
% Transizione basata sul tempo
if t >= trajectory.times(segment_end)
    % Passa automaticamente al prossimo segmento
    segment = segment + 1;
end
```

**Vantaggio:** Timing prevedibile e ripetibile

---

### 3. Legge di Controllo

#### Path Follower
**Legge:** 
$$\dot{x}_{ref} = K_p \cdot (x_{LOS} - x_{current})$$

- Solo termine proporzionale
- La velocità è proporzionale all'errore
- Errore costante in regime stazionario per traiettorie in movimento

#### Trajectory Follower
**Legge:**
$$\dot{x}_{ref} = \dot{x}_d(t) + K_p \cdot (x_d(t) - x_{current})$$

- Termine feedforward: $\dot{x}_d(t)$ anticipa il movimento
- Termine feedback: $K_p \cdot e(t)$ corregge l'errore
- Errore ridotto grazie al feedforward

**Diagramma:**
```
Trajectory Follower:
┌─────────────┐
│  Planner    │ ---> x_d(t), ẋ_d(t)
└─────────────┘           |
                          v
              ┌───────────────────────┐
              │  ẋ_d(t) + Kp·e(t)    │ ---> Controller
              └───────────────────────┘
                          ^
                          |
                    x_current (feedback)

Path Follower:
┌─────────────┐
│  LOS Point  │ ---> x_LOS
└─────────────┘       |
                      v
              ┌───────────────┐
              │   Kp·e_LOS    │ ---> Controller
              └───────────────┘
                      ^
                      |
                x_current (feedback)
```

---

### 4. Profilo di Velocità

#### Path Follower (LOS)
```
Velocità
   ^
   |     ___________________
   |    /                   \
   |___/                     \___
   +----------------------------> Distanza
   
- Dipende dall'errore
- Discontinuità possibili
- Non controllata esplicitamente
```

#### Trajectory Follower
```
Velocità
   ^
   |       ╱╲
   |      ╱  ╲
   |     ╱    ╲
   |____╱      ╲____
   +-------------------> Tempo
   
- Profilo quintico smooth
- Accelerazione continua
- Velocità controllata
```

---

### 5. Esempio Numerico

**Scenario:** Muovere l'oggetto da A=[0.5, 0, 0.5] a B=[0.7, -0.3, 0.5]
- Distanza: 0.36 m

#### Path Follower
- Tempo di arrivo: **dipende da gain e condizioni**
- Con gain=0.8: ~20-30 secondi (variabile)
- Velocità max: ~0.03 m/s (dipendente dall'errore)
- Errore medio: ~0.02 m

#### Trajectory Follower
- Tempo di arrivo: **15 secondi (predefinito)**
- Velocità max: ~0.036 m/s (pianificata)
- Velocità media: 0.024 m/s
- Errore medio: ~0.005 m (ridotto dal feedforward)

---

### 6. Quando Usare Ciascuno

#### Usare Path Follower (LOS) quando:
- ✓ Il timing non è importante
- ✓ Il path è semplice (linee rette)
- ✓ Serve reattività a perturbazioni
- ✓ Non servono specifiche di velocità
- ✓ Semplicità è prioritaria

#### Usare Trajectory Follower quando:
- ✓ Il timing è critico
- ✓ Serve coordinazione con altri sistemi
- ✓ Sono richieste specifiche di velocità/accelerazione
- ✓ Serve precisione di tracking
- ✓ Ripetibilità è importante
- ✓ Movimento deve essere fluido

---

### 7. Performance Comparison

**Metriche** (valori tipici per questo sistema):

| Metrica | Path Follower | Trajectory Follower |
|---------|---------------|---------------------|
| Errore RMS | 15-25 mm | 5-10 mm |
| Errore max | 40-60 mm | 15-25 mm |
| Smoothness (jerk) | Alto | Basso |
| Tempo di settling | Variabile | Fisso |
| Overshoot | 10-15% | <5% |

---

### 8. Estensioni Possibili

#### Path Follower
- ✓ Adaptive gain based on error
- ✓ Obstacle avoidance integration
- ✓ Variable lookahead distance

#### Trajectory Follower
- ✓ Online trajectory replanning
- ✓ Optimal time allocation
- ✓ Acceleration/velocity constraints
- ✓ Model Predictive Control (MPC)
- ✓ Multi-robot coordination

---

## Conclusione

Il **Trajectory Follower** è superiore per applicazioni che richiedono:
- Precisione temporale
- Movimento fluido
- Coordinazione

Il **Path Follower** è più semplice ma meno performante in termini di:
- Accuratezza
- Ripetibilità
- Controllo della velocità

**Raccomandazione:** Utilizzare il Trajectory Follower per questo sistema di manipolazione cooperativa, in quanto offre migliori performance di tracking e movimento più fluido.
