# 📖 INDICE DOCUMENTAZIONE - Trajectory Follower

## 🚀 INIZIA QUI

### Per iniziare velocemente:
1. Leggi: **RIEPILOGO_TRASFORMAZIONE.md**
2. Esegui: `test_trajectory_follower.m`
3. Prova: `main.m`

---

## 📁 STRUTTURA DOCUMENTAZIONE

### 1️⃣ Panoramica e Setup
- **RIEPILOGO_TRASFORMAZIONE.md** ⭐ START HERE
  - Cosa è cambiato
  - Come utilizzare il nuovo sistema
  - Checklist di verifica

### 2️⃣ Documentazione Tecnica
- **TRAJECTORY_FOLLOWER_README.md**
  - Spiegazione dettagliata del sistema
  - Equazioni matematiche
  - Descrizione funzioni create/modificate
  - Parametri di configurazione

### 3️⃣ Confronto e Analisi
- **CONFRONTO_PATH_VS_TRAJECTORY.md**
  - Tabella comparativa
  - Differenze tecniche
  - Performance comparison
  - Quando usare ciascun approccio

### 4️⃣ Guide Pratiche
- **ESEMPI_TRAIETTORIA.m**
  - 10 esempi di personalizzazione
  - Modificare timing e velocità
  - Aggiungere waypoint
  - Traiettorie speciali (cerchio, pausa, ecc.)

### 5️⃣ Testing
- **test_trajectory_follower.m**
  - Test automatici delle funzioni
  - Verifica continuità
  - Visualizzazione profili
  - 5 test completi

---

## 🔧 FILE CODICE

### Nuove Funzioni
```
ComputeTrajectoryPoint.m      ← Interpolazione quintica
GetCurrentTrajectorySegment.m ← Gestione segmenti
```

### File Modificati
```
InitRobot.m                   ← Struttura trajectory
ComputeTaskReferences.m       ← Feedforward + feedback
UpdateMissionPhase.m          ← Gestione timing
main.m                        ← Debug aggiornati
```

### File Originali (non modificati)
```
ComputeJacobians.m
ComputeActivationFunctions.m
UpdateTransforms.m
ActionTransition.m
... (tutti gli altri)
```

---

## 📚 GUIDA LETTURA CONSIGLIATA

### Per Utenti Base
1. **RIEPILOGO_TRASFORMAZIONE.md** - Capire cosa è cambiato
2. **ESEMPI_TRAIETTORIA.m** - Come personalizzare
3. Esegui `main.m` - Prova la simulazione

### Per Sviluppatori
1. **TRAJECTORY_FOLLOWER_README.md** - Documentazione completa
2. **ComputeTrajectoryPoint.m** - Capire l'interpolazione
3. **CONFRONTO_PATH_VS_TRAJECTORY.md** - Analisi tecnica
4. **test_trajectory_follower.m** - Esegui i test

### Per Ricercatori
1. **CONFRONTO_PATH_VS_TRAJECTORY.md** - Performance metrics
2. **TRAJECTORY_FOLLOWER_README.md** - Equazioni matematiche
3. Codice sorgente - Implementazione dettagliata

---

## 🎯 TASK COMUNI

### Voglio modificare la velocità del movimento
→ Leggi: **ESEMPI_TRAIETTORIA.m** - Esempio 1
→ Modifica: `InitRobot.m` → `goal.trajectory.times`

### Voglio aggiungere waypoint
→ Leggi: **ESEMPI_TRAIETTORIA.m** - Esempio 2
→ Modifica: `InitRobot.m` → `goal.trajectory.n_waypoints`

### Voglio capire come funziona l'interpolazione
→ Leggi: **TRAJECTORY_FOLLOWER_README.md** - Sezione Equazioni
→ Codice: `ComputeTrajectoryPoint.m`

### Voglio testare il sistema
→ Esegui: `test_trajectory_follower.m`
→ Verifica: I 5 test devono passare ✓

### Voglio confrontare con il vecchio sistema
→ Leggi: **CONFRONTO_PATH_VS_TRAJECTORY.md**
→ Tabella: Metriche di performance

### Voglio regolare il controllo
→ Leggi: **TRAJECTORY_FOLLOWER_README.md** - Sezione Parametri
→ Modifica: `ComputeTaskReferences.m` → `gain_obj`

### Voglio creare una traiettoria speciale
→ Leggi: **ESEMPI_TRAIETTORIA.m** - Esempi 3-10
→ Esempi: Cerchio, pausa, velocità variabile

---

## 📊 QUICK REFERENCE

### Parametri Principali

| Parametro | File | Linea | Descrizione |
|-----------|------|-------|-------------|
| `goal.trajectory.times` | InitRobot.m | ~105 | Timing waypoint |
| `gain_obj` | ComputeTaskReferences.m | ~9 | Guadagno feedback |
| `goal.trajectory.n_waypoints` | InitRobot.m | ~96 | Numero waypoint |

### Funzioni Chiave

| Funzione | Input | Output | Scopo |
|----------|-------|--------|-------|
| `ComputeTrajectoryPoint` | t, segment | pose, vel, acc | Interpolazione |
| `GetCurrentTrajectorySegment` | t, trajectory | segment_info | Trova segmento |

---

## 🆘 HELP

### Errori Comuni

**"Undefined function GetCurrentTrajectorySegment"**
→ Assicurati che il file sia nella directory corretta

**"Index exceeds array bounds"**
→ Verifica `goal.trajectory.n_waypoints` sia corretto

**"Il robot non si muove"**
→ Controlla che `mission.phase == 2`

**"Movimento troppo veloce"**
→ Aumenta i valori in `goal.trajectory.times`

---

## 📞 SUPPORTO

Per problemi o domande:
1. Controlla: **RIEPILOGO_TRASFORMAZIONE.md** - Sezione Troubleshooting
2. Verifica: `test_trajectory_follower.m` - Tutti i test passano?
3. Leggi: **TRAJECTORY_FOLLOWER_README.md** - FAQ

---

## 🔄 VERSIONI

- **v1.0** (Ottobre 2025) - Implementazione iniziale trajectory follower
  - Interpolazione polinomiale quintica
  - SLERP per orientamento
  - Feedforward + feedback control

---

## 📝 CHANGELOG

### Da Path Follower a Trajectory Follower

**Aggiunte:**
- ✅ Interpolazione temporale
- ✅ Feedforward control
- ✅ Timing deterministico
- ✅ Continuità C²

**Rimosse:**
- ❌ Logica Line-of-Sight (LOS)
- ❌ Dipendenza da GetLosPoint (in fase 2)

**Mantenute:**
- ✓ iCAT framework
- ✓ Task prioritization
- ✓ Activation functions
- ✓ Cooperative manipulation

---

## 🎓 RISORSE AGGIUNTIVE

### Teoria
- Interpolazione polinomiale quintica
- SLERP (Spherical Linear Interpolation)
- Feedforward control
- iCAT (intermediate Control Actions Task)

### Paper di Riferimento
- Trajectory planning for robotics
- Quintic splines for smooth motion
- Cooperative manipulation

---

**Buon lavoro con il Trajectory Follower! 🚀**
