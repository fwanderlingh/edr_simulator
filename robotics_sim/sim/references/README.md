# Soluzioni di riferimento per il docente

Ogni file contiene i metodi risolti dell'esercizio corrispondente. Le firme dei
metodi e lo stato fornito sono gli stessi delle classi dello studente. La modalità
`reference` del simulatore usa queste implementazioni anche per le prove pratiche.

## Lezione 4

| Esercizio | Soluzione | Metodi da implementare |
| --- | --- | --- |
| [lesson04/lidar.py](../../exercises/lesson04/lidar.py) | [lidar.py](lidar.py), `ReferenceLidar` | `directions(yaw)` |
| [lesson04/rangefinder.py](../../exercises/lesson04/rangefinder.py) | [rangefinder.py](rangefinder.py), `ReferenceRangeStop` | `command(pose, goal, distance, hit, max_range)` |
| [lesson04/avoidance.py](../../exercises/lesson04/avoidance.py) | [avoidance.py](avoidance.py), `ReferenceSectors` e `ReferenceObstacleSupervisor` | `summarize(...)` e `command(pose, goal, sectors)` |
| [lesson04/estimation.py](../../exercises/lesson04/estimation.py), attività aggiuntiva | [estimation.py](estimation.py), `ReferenceOdometry` e `ReferenceInertial` | `update(...)` |

I metodi contengono gli algoritmi richiesti agli studenti, con gli import e gli
eventuali metodi ausiliari definiti nello stesso file. Acquisizione dei sensori,
ray casting e applicazione dei comandi alle ruote rimangono nel codice fornito
dell'esercizio. Il controllo nominale riusa la Lezione 3.

### Corrispondenza con la notazione degli appunti

In `lidar.py`, `yaw` è l'angolo theta degli appunti, `R_w_b` è la rotazione dalla
base al mondo, `u_s` è la direzione del raggio nel sensore e `u_w` la stessa
direzione nel mondo. Il sensore è allineato alla base, quindi `R_b_s` è l'identità.
Il prodotto `R_w_b @ u_s` implementa esplicitamente il cambio di riferimento.
La soluzione costruisce le righe con un ciclo e restituisce `np.array(directions)`:
il risultato è un array NumPy con una riga per raggio, come nel sensore di base.

I punti sono indicati con `P` negli appunti, e con `P_bar` si indicano qui le
loro coordinate omogenee, con ultima componente 1. Il metodo `directions` deve
invece restituire direzioni: in coordinate omogenee avrebbero ultima componente
0, che annulla la traslazione. La terza componente delle triplette restituite
dall'API è la coordinata fisica z, nulla per i raggi orizzontali; non è una
coordinata omogenea.

In `rangefinder.py`, `rho` è l'errore di distanza dal goal, `self.eps_pos` la
tolleranza di posizione e `self.enter`/`self.exit` le soglie dell'isteresi.
Le transizioni seguono la FSM degli appunti: GO, STOP e DONE, con priorità al
raggiungimento dell'obiettivo. Le misure sono assunte sempre valide e disponibili;
non sono previsti controlli sull'età del dato.

`sensors.py` conserva gli import precedenti per compatibilità; le implementazioni
da consultare e modificare sono nei file elencati nella tabella.
