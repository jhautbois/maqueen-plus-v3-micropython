# Sources des bibliothèques - Maqueen Plus V3 Project

Ce document liste toutes les sources utilisées pour créer les bibliothèques MicroPython de ce projet.

## 📚 Bibliothèques officielles DFRobot

### 1. Capteur Laser SEN0628 ✅ COMPLET

**Bibliothèque Arduino officielle :**
- **Repo** : https://github.com/DFRobot/DFRobot_MatrixLidar
- **Fichier analysé** : `DFRobot_MatrixLidar.cpp`
- **État** : ✅ Protocole I2C entièrement analysé et porté en MicroPython

**Protocole découvert :**
- Adresse I2C par défaut : `0x33` (configurable 0x30-0x32)
- Communication par **paquets de commandes** (pas de registres)
- Format envoi : `0x55 | argsNumH | argsNumL | cmd | args...`
- Format réception : `status | cmd | lenL | lenH | data...`
- Données : 16-bit little-endian

**Commandes implémentées :**
- `0x01` SET_MODE - Configure 4x4 ou 8x8
- `0x02` ALL_DATA - Lire matrice complète
- `0x03` FIXED_POINT - Lire point (x, y)

**Fichier créé :** `src/laser_matrix.py`

---

### 2. Robot Maqueen Plus V2/V3 ⚠️ PARTIELLEMENT ANALYSÉ

**Bibliothèque MakeCode officielle :**
- **Repo** : https://github.com/DFRobot/pxt-DFRobot_MaqueenPlus_v20
- **Fichier analysé** : `maqueenPlusV2.ts` (TypeScript pour MakeCode)
- **État** : ⚠️ Protocole I2C STM8 partiellement analysé

**Protocole I2C STM8 découvert :**

| Composant | Registre | Format | État |
|-----------|----------|--------|------|
| **Moteurs** | | | |
| Left Motor | 0x00 | 3 bytes: [reg, dir, speed] | ✅ Implémenté |
| Right Motor | 0x02 | 3 bytes: [reg, dir, speed] | ✅ Implémenté |
| **Encodeurs** | | | |
| Left Encoder | 0x04 | 2 bytes? | ⚠️ À tester |
| Right Encoder | 0x06 | 2 bytes? | ⚠️ À tester |
| **RGB LEDs (headlights)** | | | |
| Left LED | 0x0B | 2 bytes: [reg, on/off] | ✅ Implémenté |
| Right LED | 0x0C | 2 bytes: [reg, on/off] | ✅ Implémenté |
| **Capteurs ligne** | | | |
| Line State | 0x1D | 1 byte (bitfield) | ⚠️ Non implémenté |
| Line R2 | 0x1E | 2 bytes (little-endian) | ✅ Implémenté |
| Line R1 | 0x20 | 2 bytes (little-endian) | ✅ Implémenté |
| Line M | 0x22 | 2 bytes (little-endian) | ✅ Implémenté |
| Line L1 | 0x24 | 2 bytes (little-endian) | ✅ Implémenté |
| Line L2 | 0x26 | 2 bytes (little-endian) | ✅ Implémenté |
| **Système** | | | |
| Version Count | 0x32 | 1 byte | ✅ Implémenté |
| Version Data | 0x33 | 1 byte | ⚠️ À tester |
| System Reset | 0x49 | 1 byte (write 1) | ⚠️ Non implémenté |

**Non trouvé dans le protocole MakeCode :**
- Capteurs de lumière (light sensors) - registres inconnus
- Servo control - registres inconnus
- IR receiver - géré côté micro:bit ?

**Fichier créé :** `src/maqueen_plus_v3.py`

**Notes importantes :**
- Le protocole MakeCode est pour V2, **peut avoir des différences avec V3**
- Certaines fonctionnalités nécessitent tests hardware pour validation
- Les capteurs de lumière utilisent des registres supposés (0x20, 0x22) - **à vérifier**

---

## 🔍 Bibliothèques communautaires (référence)

Ces bibliothèques ont servi de **référence** mais n'ont pas été analysées en détail :

### MicroPython Maqueen Plus V2
- **Repo** : https://github.com/jdonwells/micropython-MaqueenPlusV2
- **Usage** : Référence pour patterns I2C généraux
- **État** : Non analysé en détail

### MicroPython Maqueen Plus (ancienne version)
- **Repo** : https://github.com/almasy/micropython-maqueen-plus
- **Usage** : Référence pour structure de code
- **État** : Non analysé en détail

---

## 📖 Documentation officielle

### Wiki DFRobot Maqueen Plus V3
- **URL** : https://wiki.dfrobot.com/SKU_MBT0050_Maqueen_Plus_V3
- **Contenu** : Spécifications hardware, assemblage, exemples MakeCode
- **Limite** : Pas de documentation MicroPython officielle

### Wiki DFRobot SEN0628 Laser Sensor
- **URL** : https://wiki.dfrobot.com/SKU_SEN0628_Matrix%20Laser%20Ranging%20Sensor
- **Contenu** : Spécifications capteur, API Arduino
- **Usage** : Référence pour paramètres capteur (FOV, portée, etc.)

### micro:bit MicroPython Documentation
- **URL** : https://microbit-micropython.readthedocs.io/
- **Usage** : API micro:bit (i2c, pins, display, etc.)
- **État** : Documentation officielle complète

---

## ✅ Statut d'implémentation

### Fonctionnalités testées avec hardware ✅
*Aucune pour l'instant - nécessite tests sur robot réel*

### Fonctionnalités implémentées basées sur protocole officiel ✅
- ✅ Capteur laser : Lecture matrice 8x8 / 4x4
- ✅ Capteur laser : Zones gauche/centre/droite
- ✅ Capteur laser : Changement de mode
- ✅ Moteurs : Contrôle direction + vitesse
- ✅ Capteurs ligne : Lecture ADC (analog)
- ✅ Capteurs ligne : Lecture digitale (threshold)
- ✅ LEDs headlights : Contrôle RGB basique

### Fonctionnalités à vérifier ⚠️
- ⚠️ Encodeurs : Lecture implémentée mais format à valider
- ⚠️ Capteurs lumière : Registres supposés, à vérifier
- ⚠️ LEDs headlights : Protocole RGB complet (actuellement on/off)
- ⚠️ Version STM8 : Format de réponse à valider

### Fonctionnalités non implémentées ❌
- ❌ Servo control
- ❌ System reset (0x49)
- ❌ Line sensor state bitfield (0x1D)
- ❌ PID control (si supporté par STM8)
- ❌ IR receiver (probablement géré côté micro:bit)

---

## 🔬 Méthode d'analyse des protocoles

### Pour le capteur laser (DFRobot_MatrixLidar)

1. **WebFetch** du fichier C++ source
2. Extraction des commandes et formats de paquets
3. Analyse du protocole send/receive
4. Port en MicroPython avec gestion I2C micro:bit
5. Création de tests unitaires

### Pour le Maqueen STM8 (pxt-DFRobot_MaqueenPlus_v20)

1. **WebFetch** du fichier TypeScript source
2. Extraction de la carte des registres I2C
3. Identification des formats de données (byte order, taille)
4. Port des fonctions essentielles en MicroPython
5. Documentation des zones inconnues

---

## 📝 Notes pour futurs développeurs

### Si vous devez débugger/améliorer les drivers :

1. **Toujours commencer par scanner I2C** : `make scan`
2. **Tester registre par registre** avec REPL
3. **Comparer avec bibliothèque Arduino/MakeCode** pour valider
4. **Documenter les découvertes** dans ce fichier

### Ressources pour reverse engineering :

**Outils :**
- REPL micro:bit : `make repl`
- I2C scanner : `make scan`
- Test laser : `make test-laser`

**Approche :**
```python
# Dans REPL micro:bit
from microbit import i2c

# Tester un registre inconnu
i2c.write(0x10, bytes([0xXX]))  # Écrire registre
data = i2c.read(0x10, 2)         # Lire réponse
print([hex(b) for b in data])    # Afficher
```

**Bibliothèques à analyser si besoin :**
- Arduino : Code C++ lisible, protocole I2C clair
- MakeCode : TypeScript, bonne documentation inline
- Python/RPi : Si existant, plus facile à porter

---

## 🎯 Recommandations

### Pour utilisateurs du projet :

1. **Commencer avec** : `make scan` pour identifier composants
2. **Tester laser** : `make test-laser` avant usage
3. **Fonctions sûres** : Moteurs, capteur laser, capteurs ligne
4. **Fonctions à vérifier** : Encodeurs, capteurs lumière

### Pour contributeurs :

1. **Ajouter tests hardware** quand disponible
2. **Compléter protocole STM8** (encodeurs, lumière)
3. **Valider avec vraie V3** (actuellement basé sur V2)
4. **Documenter différences V2/V3** si trouvées

---

## 📚 Résumé

| Composant | Source | Qualité | Testé |
|-----------|--------|---------|-------|
| Laser ToF | ✅ Officielle Arduino | 95% | ❌ Non |
| Moteurs | ✅ Officielle MakeCode | 90% | ❌ Non |
| Capteurs ligne | ✅ Officielle MakeCode | 85% | ❌ Non |
| LEDs RGB | ✅ Officielle MakeCode | 70% | ❌ Non |
| Encodeurs | ⚠️ Supposé | 50% | ❌ Non |
| Capteurs lumière | ⚠️ Supposé | 30% | ❌ Non |

**Légende :**
- ✅ Protocole officiel analysé
- ⚠️ Implémentation supposée/incomplète
- ❌ Non testé sur hardware réel

---

**Dernière mise à jour** : 2024 (création du projet)
**Version bibliothèques** : v1.0 (initiale)
