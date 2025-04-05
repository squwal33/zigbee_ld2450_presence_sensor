# LD2450 Zigbee Presence Sensor for Home Assistant

## English

### Introduction

This project implements a Zigbee presence sensor using the LD2450 mmWave radar sensor and ESP32-C6 microcontroller. The sensor can detect human presence, movement, and position of up to 3 people in a room. It integrates with Home Assistant using Zigbee2MQTT, providing presence detection for your home automation scenarios.

The system also includes a BH1750 light sensor to measure ambient light levels.

---

### Required Hardware

- ESP32-C6 development board
- LD2450 mmWave radar sensor
- BH1750 light sensor (optional)
- Jumper wires
- USB cable for programming and power
- 5V power supply (optional for standalone operation)

![ESP32-C6](/img/esp32-c6_devkit.png)

---

### Wiring Instructions

#### LD2450 Sensor Connection
- Connect LD2450 TX to ESP32-C6 GPIO4 (UART_RX)
- Connect LD2450 RX to ESP32-C6 GPIO5 (UART_TX)
- Connect LD2450 VCC to 5V
- Connect LD2450 GND to GND

**Important**: Pay attention to the orientation of the LD2450 sensor when mounting it:

![LD2450 Orientation](/img/ld2450_orientation.jpg)

#### BH1750 Light Sensor Connection (Optional)
- Connect BH1750 SDA to ESP32-C6 GPIO6
- Connect BH1750 SCL to ESP32-C6 GPIO7
- Connect BH1750 VCC to 5V
- Connect BH1750 GND to GND

---

### Software Setup

#### Prerequisites
- ESP-IDF (Espressif IoT Development Framework)
- Visual Studio Code with ESP-IDF extension (recommended)

#### Installation and Configuration

1. Clone this repository:
   ```
   git clone https://github.com/squwal33/zigbee_ld2450_presence_sensor.git
   ```

2. Open the project in Visual Studio Code with the ESP-IDF extension installed.

3. Configure, build, and flash the project using the "Build, Flash and Monitor" button in the ESP-IDF extension.


---

### Home Assistant Integration

Copy the file zigbee2mqtt_config/squwalinc_esp32c6.js to your Zigbee2MQTT configuration folder at: /homeassistant/zigbee2mqtt/external_converters/squwalinc_esp32c6.js

Restart the Zigbee2MQTT add-on in Home Assistant.
Put your Zigbee coordinator in pairing mode using the button in Zigbee2MQTT.

Once paired with your Zigbee network, the device will automatically be discovered by Home Assistant through Zigbee2MQTT. It will provide the following entities:

- Presence detection (binary sensor)
- Target position (X, Y coordinates)
- Movement speed
- Light level (if BH1750 is connected)
- A switch to turn on the development board LED

![Z2M Entities](/img/z2m_presence_entities.jpg)


---

### Home Assistant Display
A sample card is provided in the lovelace_card/plotly_location.yaml file.
You need to rename the sensor name in the file according to your configuration.

This allows you to visualize the position of targets in your space like this:
![Card Image](/img/location.png)

This example card requires the following cards:
- Vertical Stack In Card
- Plotly Graph Card

---

### Troubleshooting

- If the device doesn't appear in Home Assistant, ensure your Zigbee coordinator is in pairing mode.
- Restart your ESP32-C6.
- Check the serial monitor for sensor debugging information.
- Verify the wiring connections between the ESP32-C6 and the sensors.
- Check the Zigbee2MQTT logs to confirm that the external converter is properly loaded.

## Additional Resources

- [Detailed Project Build Log](https://blog.squwal.com) - Visit my blog for a detailed explanation of this project's development process, challenges, and solutions.

---

## License

This project is licensed under the GNU General Public License v3.0 - see the LICENSE file for details.

---

## Français

### Introduction

Ce projet implémente un capteur de présence Zigbee utilisant le capteur radar mmWave LD2450 et un microcontrôleur ESP32-C6. Le capteur peut détecter la présence humaine, les mouvements et la position dans une pièce de 3 personnes. Il s'intègre à Home Assistant à l'aide de Zigbee2MQTT, fournissant une détection de présence pour vos scénarios de domotique.

Le système inclut également un capteur de luminosité BH1750 pour mesurer les niveaux de lumière ambiante.

---

### Matériel Nécessaire

- Carte de développement ESP32-C6
- Capteur radar mmWave LD2450
- Capteur de lumière BH1750 (optionnel)
- Fils de raccordement (jumpers)
- Câble USB pour la programmation et l'alimentation
- Alimentation 5V (optionnelle pour un fonctionnement autonome)

![ESP32-C6](/img/esp32-c6_devkit.png)

---

### Instructions de Branchement

#### Connexion du Capteur LD2450
- Connectez LD2450 TX à ESP32-C6 GPIO4 (UART_RX)
- Connectez LD2450 RX à ESP32-C6 GPIO5 (UART_TX)
- Connectez LD2450 VCC à 5V
- Connectez LD2450 GND à GND

**Important**: Faites attention à l'orientation du capteur LD2450 lors de son installation :

![Orientation LD2450](/img/ld2450_orientation.jpg)

#### Connexion du Capteur de Lumière BH1750 (Optionnel)
- Connectez BH1750 SDA à ESP32-C6 GPIO6
- Connectez BH1750 SCL à ESP32-C6 GPIO7
- Connectez BH1750 VCC à 5V
- Connectez BH1750 GND à GND

---

### Configuration Logicielle

#### Prérequis
- ESP-IDF (Espressif IoT Development Framework)
- Visual Studio Code avec l'extension ESP-IDF (recommandé)

#### Installation et Configuration

1. Clonez ce dépôt:
   ```
   git clone https://github.com/squwal33/zigbee_ld2450_presence_sensor.git
   ```

2. Ouvrez le projet dans Visual Studio Code avec l'extension ESP-IDF installée.

3. Configurez, compilez et flashez le projet en utilisant le bouton "Build, Flash and Monitor" dans l'extension ESP-IDF.


---

### Intégration avec Home Assistant

Copiez le fichier zigbee2mqtt_config/squwalinc_esp32c6.js dans le dossier de configuration de votre instance de Zigbee2MQTT à l'emplacement suivant : /homeassistant/zigbee2mqtt/external_converters/squwalinc_esp32c6.js
Redémarrer le module complémentaire Zigbee2MQTT dans home assistant.
Passez votre coordinateur zigbee en mode appairage à l'aide du bouton dans Zigbee2MQTT.
Une fois associé au réseau Zigbee, l'appareil devrait automatiquement remonter à Home Assistant par l'intermédiaire de Zigbee2MQTT les entités suivantes:

- Détection de présence (capteur binaire)
- Position de la cible (coordonnées X, Y)
- Vitesse de déplacement
- Luminosité (si BH1750 connecté)
- Un interrupteur pour allumer la LED de la carte de développement

![Entités Z2M](/img/z2m_presence_entities.jpg)


---

### Affichage dans Home Assistant
Une carte d'exemple est fournie dans le fichier lovelace_card/plotly_location.yaml
Vous devez renommer le nom du sensor dans le fichier en fonction de votre configuration.

Cela permet de visualiser la position des cibles dans votre espace de cette manière :
![Image de la carte](/img/location.png)

Cette carte d'exemple nécessite les cartes suivantes :
- Vertical Stack In Card
- Plotly Graph Card

---

### Dépannage

- Si l'appareil n'apparaît pas dans Home Assistant, assurez-vous que votre coordinateur Zigbee est en mode d'appairage.
- Redémarrez votre ESP32-C6.
- Vérifiez le moniteur série pour les informations de débogage du capteur.
- Vérifiez les connexions de câblage entre l'ESP32-C6 et les capteurs.
- Vérifiez les logs de Zigbee2MQTT pour confirmer que le convertisseur externe est bien chargé

---

## Ressources Additionnelles

- [Journal détaillé du projet](https://blog.squwal.com) - Visitez mon blog pour une explication détaillée du processus de développement de ce projet, des défis rencontrés et des solutions apportées.

---

## Licence

Ce projet est sous licence GNU General Public License v3.0 - consultez le fichier LICENSE pour plus de détails.
