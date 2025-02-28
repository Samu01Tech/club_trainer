# Golf Trainer

![logo](assets/logo.png)

## Introduzione

Il presente documento descrive in dettaglio il funzionamento di un simulatore di golf sviluppato su piattaforma Arduino. Il sistema è progettato per analizzare il movimento dello swing utilizzando un sensore IMU BNO055, fornire feedback multisensoriale: visivo, tramite LED e interfaccia grafica, sonoro, attraverso il dispositivo collegato, e tattile, grazie a un motore a vibrazione. L'obiettivo principale è creare un'esperienza interattiva che permetta ai giocatori di migliorare la propria tecnica attraverso l'utilizzo di più stimoli sensoriali e ottenere un riscontro immediato delle loro prestazioni.

## Panoramica del Sistema

Il simulatore di golf utilizza una serie di componenti hardware che lavorano insieme per raccogliere e interpretare i dati relativi allo swing del giocatore. Il cuore del sistema è rappresentato dal sensore di movimento BNO055, il quale, è in grado di rilevare posizione, velocità e accelerazione per tutti e tre gli assi. Pertanto, è possibile monitorare l'angolazione e la velocità del movimento. Questi dati vengono elaborati in tempo reale e utilizzati per determinare la qualità del colpo eseguito.

Per garantire un'interazione completa con l'utente, il simulatore integra un sistema di feedback visivo composto da una serie di LED posizionati strategicamente. Questi indicatori luminosi segnalano lo stato del colpo e l'orientamento del movimento, fornendo al giocatore informazioni utili per correggere eventuali errori. Inoltre, il sistema incorpora un motore a vibrazione che emette segnali tattili in risposta a specifiche condizioni di gioco, come un colpo ben eseguito o un errore di posizione.

## Architettura Hardware e Componenti

L'implementazione del sistema si basa su due componenti connesse: un prototipo di mazza da golf e un server. 

### Prototipo

Il prototipo di mazza da golf funge da punto di contatto tra l'utente e il sistema: sono presenti tutti i sistemi di acquisizione dei dati dall'ambiente, come sensori e pulsanti, e alcuni sistemi di interazione con l'utente, come feedback sul movimento. Inoltre, a bordo del prototipo è installato il microcontrollore che gestisce le componenti sul prototipo e inoltra le informazioni al sistema.

*foto mazza*

#### Hardware

The hardware installed on this protype can be divided into three classes: sensors, actuators and microcontrollers.

##### Sensors

Sensors are devices that generally transform physical quantities into electrical ones.

###### Adafruit BNO055

The Adafruit BNO055 is a 9-DOF (Degrees of Freedom) absolute orientation sensor that integrates a triaxial 14 bits accelerometer, a triaxial 16 bits gyroscope, and magnetometer, all runnig with an onboard ARM Cortex-M0 microcontroller. Unlike raw IMUs, which require complex sensor fusion algorithms to calculate orientation, the BNO055 directly outputs absolute orientation data. The design is a system in package (SiP), where a small LGA housing (3.8 x 5.2 mm) contains all the components. 

**Power Supply**
The BNO055 is powered by a 5V DC supply, which is regulated internally to the required voltage for its sensors and processing unit. This internal voltage regulator allows the sensor to be compatible with both 5V and 3.3V logic systems, making it easy to integrate with microcontrollers such as Arduino (5V logic) and Raspberry Pi (3.3V logic) without requiring external level shifting.

**Communication Protocols**

The BNO055 can communicate with other devices using I²C or UART. In this project, I²C is used, which enables bidirectional communication through two dedicated lines:

SDA (Serial Data Line) – Transmits data between the sensor and the microcontroller.
SCL (Serial Clock Line) – Synchronizes data transmission.

In an I²C setup, the microcontroller (master) sends requests for specific sensor data such as acceleration, angular velocity, orientation, or temperature. The BNO055 (slave) processes the request and sends the required information back to the master.

Additionally, the master can store calibration values in its EEPROM (Electrically Erasable Programmable Read-Only Memory) to avoid recalibrating the sensor on every power cycle. Alternatively, the system can perform recalibration at startup to ensure optimal accuracy. In our project, the magnetometer is recalibrated each time the system boots. The calibration data, along with other sensor outputs, can be accessed using manufacturer-provided libraries, such as the Adafruit BNO055 library for Arduino.

*foto BNO055*

###### Ribbon sensor (linear position sensor)

The SoftPot linear position sensor is a flexible, thin, and low-profile membrane potentiometer that provides variable resistance based on touch or mechanical contact along its length. It is commonly used in applications requiring linear position tracking, such as robotics, touch-sensitive controls, industrial automation, and musical instruments. Its shape can be linear or circular.

**Working Principle & Analog Output**

The SoftPot linear position sensor operates as a variable resistor, functioning like a voltage divider when properly connected. It requires three connections: power (VCC), ground (GND), and output (wiper).

**Power Supply**

The sensor can be powered using 3.3V or 5V DC, depending on the logic level of the microcontroller or circuit it is connected to. The choice of supply voltage directly affects the sensor's output range:

+ If powered with 5V, the output voltage will range from 0V (at one end of the sensor) to 5V (at the other end).
+ If powered with 3.3V, the output will vary between 0V and 3.3V, making it suitable for microcontrollers like Raspberry Pi or ESP32, which operate at 3.3V logic.

Most microcontrollers (like the Teensy used in this project) use 10-bit ADCs, meaning they divide the input voltage range into 1024 discrete steps (0–1023).

+ With a 5V supply, each ADC step corresponds to ≈4.88mV (5V / 1024).
+ With a 3.3V supply, each step corresponds to ≈3.22mV (3.3V / 1024).

In other words, a higher VCC results in larger voltage steps, potentially improving accuracy when mapped to a percentage-based position. For this reason, the sensor is supplied by 5V.

**Working principle**
The SoftPot sensor functions as a linear potentiometer, meaning its resistance changes depending on the position of the applied force along its surface.

When no force is applied, the wiper floats and may give unpredictable readings.
When a touch or mechanical pressure is applied, the wiper makes contact with the resistive track, forming a voltage divider circuit.
The voltage at the wiper (middle pin) is proportional to the touch position along the length of the sensor:
+ 0V (GND side) → Start of the strip
+ VCC (5V or 3.3V) → End of the strip
+ Intermediate values correspond to positions in between.

**Additional resistor**
The sensor does not produce an output unless it is touched, meaning that when no pressure is applied, the wiper floats and may give random, unstable readings. To prevent this, a pull-down resistor (typically 10kΩ) is connected between the wiper and the ground. If the resistor is connected to the VCC pin (pull-up configuration), the output would be high when it is not touched.


- **Sensore IMU BNO055**: Un accelerometro e giroscopio  che misura l'orientamento del colpo in tre dimensioni.
- **EEPROM**: Utilizzata per memorizzare dati persistenti relativi alla calibrazione e ai parametri di utilizzo.
- **Sensore di effetto Hall**: Impiegato per il rilevamento del passaggio della mazza nel punto *virtuale* di contatto con la palla da golf.
- **Sensore a nastro (ribbon sensor)**: Definisce la posizione della mano durante l'esecuzione dello swing, garantendo una corretta impugnatura della mazza.
- **Sistema di feedback mani**: Questo sistema aiuta il giocatore a impugnare correttamente la mazza. Una serie di LED disposti verticalmente guida il posizionamento delle mani: se queste sono troppo alte, si illumineranno i LED rossi superiori; se sono troppo basse, si accenderanno i LED rossi inferiori. Se la posizione delle mani è corretta, i LED verdi confermeranno il posizionamento.
- **Sistema di feedback swing**: Questo sistema aiuta a interpretare la correttezza dello swing. Una serie di LED disposti orizzontalmente segnala l'orientamento del movimento: se il colpo devia troppo a destra, si accenderanno i LED rossi sulla destra; se invece è troppo inclinato a sinistra, si illumineranno i LED rossi sulla sinistra. Se lo swing è corretto e ben allineato, i LED verdi centrali si illumineranno per confermare l'esecuzione ideale del colpo.
- **Motore a vibrazione**: Implementato per fornire un feedback aptico che aiuta il giocatore a percepire il momento in cui la palla viene colpita. Questo consente di modificare lo swing in modo appropriato, ad esempio anticipando l'estensione del braccio o correggendo il movimento per ottenere un colpo più efficace.
- **Pure Data (PD)**: Software utilizzato per elaborare e generare effetti sonori in tempo reale sulla base dei dati ricevuti da Arduino.
- **Processing**: Software utilizzato per la visualizzazione grafica del colpo e dell'orientamento della mazza in tempo reale, nonchè ottenere uno storico dei colpi effettuati.

## Integrazione con Pure Data

Pure Data (PD) è un ambiente di programmazione grafica per l'elaborazione audio. Nel contesto del simulatore di golf, Pure Data riceve i dati da Arduino attraverso una connessione seriale e genera suoni sincronizzati con il movimento dello swing.

### Flusso di lavoro di Pure Data

1. **Ricezione dei dati da Arduino**: Il codice PD legge i dati inviati via seriale e li interpreta per determinare il momento dell'impatto e la qualità del colpo.
2. **Generazione degli effetti sonori**: In base ai dati ricevuti, il sistema riproduce suoni diversi, tra cui:
   - Un suono più intenso per un colpo potente
   - Un suono più ovattato per un colpo debole
   - Un effetto acustico direzionale che varia in base all’angolazione dello swing
3. **Regolazione dei parametri audio**: Il sistema consente di personalizzare gli effetti sonori regolando il volume, il tono e la riverberazione in base alle prestazioni del giocatore.

## Integrazione con Processing

Processing è un ambiente di programmazione visuale utilizzato per la rappresentazione grafica dei dati ricevuti da Arduino. Nel contesto del simulatore di golf, il codice Processing legge i dati dallo swing e fornisce una visualizzazione in tempo reale dell'orientamento della mazza e della qualità del colpo.

### Flusso di lavoro di Processing

1. **Connessione con Arduino**: Processing stabilisce una connessione seriale per ricevere i dati dei sensori.
2. **Visualizzazione del movimento**: Il sistema genera un'animazione che mostra l'angolazione e la traiettoria dello swing.
3. **Feedback visivo avanzato**: Il programma può includere indicatori di posizione, angoli e vettori per segnalare eventuali errori nello swing e suggerire correzioni.

## Logica di Funzionamento

Il codice implementa una serie di routine per acquisire e interpretare i dati provenienti dai sensori e fornire un riscontro immediato al giocatore. La sequenza operativa è la seguente:

1. **Inizializzazione del sistema**: Durante la fase di avvio, il microcontrollore configura i sensori e verifica la loro corretta calibrazione. Se necessario, i dati di calibrazione vengono caricati dalla memoria EEPROM.
2. **Monitoraggio del movimento**: Il sensore IMU acquisisce costantemente dati riguardanti l'orientamento e la velocità della mazza. Questi valori vengono confrontati con soglie predefinite per determinare la qualità del colpo.
3. **Attivazione dei feedback visivi e tattili**: In base ai dati acquisiti, il sistema rappresenta delle informazioni nell'interfaccia connessasia di tipo visivo che audio. Inoltre,si attivano dei feedback visivi sulla mazza.

## Conclusioni

Il simulatore di golf basato su Arduino rappresenta una soluzione innovativa per l’allenamento e il miglioramento della tecnica di gioco. Grazie alla combinazione di sensori avanzati e sistemi di feedback multisensoriali, l’utente può affinare il proprio swing con un supporto costante e immediato. 
