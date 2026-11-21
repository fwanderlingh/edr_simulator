# Soluzioni di riferimento per il docente

Ogni file contiene i metodi risolti dell'esercizio corrispondente. Le firme dei
metodi e lo stato fornito sono gli stessi delle classi dello studente. La modalità
`reference` del simulatore usa queste implementazioni anche per le prove pratiche.

## Lezione 4

| Esercizio | Soluzione | Metodi da implementare |
| --- | --- | --- |
| [lesson04/rangefinder.py](../../exercises/lesson04/rangefinder.py) | [rangefinder.py](rangefinder.py), `ReferenceRangeStop` | `command(pose, goal, distance, hit, max_range)` |
| [lesson04/avoidance.py](../../exercises/lesson04/avoidance.py) | [avoidance.py](avoidance.py), `ReferenceSectors` e `ReferenceObstacleSupervisor` | `summarize(...)` e `command(pose, goal, sectors)` |
| [lesson04/estimation.py](../../exercises/lesson04/estimation.py), attività aggiuntiva | [estimation.py](estimation.py), `ReferenceOdometry` e `ReferenceInertial` | `update(...)` |

I metodi contengono gli algoritmi richiesti agli studenti, con gli import e gli
eventuali metodi ausiliari definiti nello stesso file. Acquisizione dei sensori,
ray casting e applicazione dei comandi alle ruote rimangono nel codice fornito
dell'esercizio. Il controllo nominale riusa la Lezione 3.

La Parte II di EDR.04 usa il LiDAR già fornito: prima si interpreta la scansione,
poi si completa `StudentSectors.summarize` in `sectors` e infine
`StudentObstacleSupervisor.command` in `avoid`, riusando il controllore di EDR.03 sia per il goal intermedio sia per quello finale.
L'attività **Scan obstacles with 2D lidar** è una prova di osservazione.

### Geometria interna del sensore

In `lidar.py`, `yaw` è l'orientamento della base, `R_w_b` è la rotazione dalla
base al mondo, `u_s` è la direzione del raggio nel sensore e `u_w` la stessa
direzione nel mondo. Il sensore è allineato alla base, quindi `R_b_s` è l'identità.
Il prodotto `R_w_b @ u_s` implementa il cambio di riferimento nel sensore fornito.
Il vecchio esercizio `lesson04/lidar.py` rimane disponibile per esperimenti
programmatici, ma non fa parte del percorso della Parte II né dei selettori desktop.
La soluzione costruisce le righe con un ciclo e restituisce `np.array(directions)`:
il risultato è un array NumPy con una riga per raggio, come nel sensore di base.

I punti sono indicati con `P` negli appunti, e con `P_bar` si indicano qui le
loro coordinate omogenee, con ultima componente 1. Il metodo `directions` deve
invece restituire direzioni: in coordinate omogenee avrebbero ultima componente
0, che annulla la traslazione. La terza componente delle triplette restituite
dall'API è la coordinata fisica z, nulla per i raggi orizzontali; non è una
coordinata omogenea.

### Arresto e supervisione

In `rangefinder.py`, `rho` è l'errore di distanza dal goal, `self.eps_pos` la
tolleranza di posizione, `self.eps_angle` la tolleranza di orientamento e
`self.enter`/`self.exit` le soglie dell'isteresi. Il goal è sempre `(x, y, yaw)`;
si usa `gx, gy, gt = goal`, come nel controllore di EDR.03. DONE richiede entrambe
le tolleranze e il controllo nominale conserva l'allineamento finale ALIGN.
Le transizioni seguono la FSM degli appunti: GO, STOP e DONE, con priorità al
raggiungimento dell'obiettivo. Le misure sono assunte sempre valide e disponibili;
non sono previsti controlli sull'età del dato.

In `avoidance.py`, il supervisore usa GO, STOP, DETOUR e DONE. All'uscita da STOP
memorizza una posa intermedia in `detour_goal`; `goal_command` riusa la stessa
istanza del controllore di EDR.03 per raggiungerla e poi ripristinare il goal
finale. `goal_controller_done` è l'esito del controllore per la posa attiva e
non va confuso con DONE del supervisore, riservato all'obiettivo finale.
Durante DETOUR, `advance_allowed` conserva l'isteresi del fronte e impedisce
il comando di avanzamento quando manca il margine, permettendo la rotazione
iniziale verso il lato già verificato. Non ci sono guadagni o velocità aggiuntivi
per l'aggiramento: il supervisore restituisce il comando di EDR.03 oppure zero.

`sensors.py` conserva gli import precedenti per compatibilità; le implementazioni
da consultare e modificare sono nei file elencati nella tabella.
