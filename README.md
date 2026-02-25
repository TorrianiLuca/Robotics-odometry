## Odometria e Tracking GPS di un Veicolo Autonomo

Questo progetto, sviluppato per il corso di **Robotics**, implementa un sistema basato su ROS per la stima della traiettoria di un veicolo autonomo. Il sistema integra i dati degli encoder delle ruote e le misurazioni GPS per calcolare e visualizzare la posizione del veicolo sul circuito di Monza. L'obiettivo principale è l'elaborazione dei dati grezzi del veicolo (velocità e angolo di sterzata) e dei segnali GPS a doppia antenna per ricostruire il percorso del veicolo.

### Caratteristiche Principali:
* **Odometria Differential Drive:** Integrazione della velocità del veicolo ($km/h$) e dell'angolo del volante (gradi) per stimare posizione $(x, y)$ e orientamento $(\theta)$.
* **Trasformazione di Coordinate:** Implementazione degli algoritmi di conversione da ECEF a ENU (East-North-Up).
* **Elaborazione GPS:** Traduzione dei dati grezzi di Latitudine/Longitudine in un sistema di riferimento cartesiano locale per il tracciamento in tempo reale.
* **Visualizzazione:** Setup personalizzato su RViz per l'analisi comparativa tra Odometria e GPS.
