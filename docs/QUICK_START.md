# Maqueen Plus V3 - Guide de démarrage rapide

## 🚀 Installation

```bash
# 1. Installer les outils Python
pip install uflash microfs

# 2. Installer terminal série (optionnel)
sudo apt-get install screen picocom

# 3. Vérifier l'installation
cd /home/jm/SynologyDrive/Léandre/Projets/micro_bit
make check
```

## 🔌 Setup physique : USB + Robot en même temps

**Question importante :** Oui, vous pouvez brancher le micro:bit en USB **ET** l'avoir connecté au Maqueen simultanément !

### Configuration de développement standard

```
[PC] ←──USB──→ [micro:bit V2] ←──inséré──→ [Maqueen Plus V3]
                     ↓                            ↓
               REPL + Flash              [Batterie ON]
                     ↓                            ↓
                [I2C + GPIO via edge connector]
                     ↓
        ┌────────────┼────────────┐
        ↓            ↓            ↓
   [STM8 0x10]  [Laser 0x33]  [LEDs]
```

### Ce qui fonctionne simultanément

**Via USB :**
- ✅ Flash du code (`make flash`)
- ✅ Upload de fichiers (`make upload`)
- ✅ REPL série en direct (`make repl`)
- ✅ Alimentation du micro:bit

**Via le robot :**
- ✅ Communication I2C (STM8 + capteur laser)
- ✅ Contrôle moteurs
- ✅ Lecture capteurs (ligne, lumière, encodeurs)
- ✅ LEDs RGB (headlights + underglow)
- ✅ Alimentation depuis batterie robot

### Workflow typique

```bash
# 1. Brancher micro:bit en USB
# 2. Insérer micro:bit dans le Maqueen
# 3. Allumer la batterie du Maqueen
# 4. Le robot est prêt !

# Flash et debug en live
make flash upload
make repl

# Dans le REPL, vous voyez les print() en temps réel
# Le robot peut bouger pendant le debug !
```

### ⚠️ Sécurité pendant les tests

**Le robot peut se déplacer même avec USB branché !**

**Solutions :**
1. **Surélever le robot** (cale, livres) pour tester moteurs sans déplacement
2. **Cable USB long** (2-3m) pour laisser le robot au sol
3. **Emergency stop** : Toujours implémenter Button B pour arrêt d'urgence

```python
# Exemple de sécurité dans le code
from microbit import button_b

while True:
    if button_b.was_pressed():
        robot.stop()
        print("EMERGENCY STOP!")
        break
```

### Alimentation

- **USB** : Alimente uniquement le micro:bit (3.3V)
- **Batterie Maqueen** : Alimente moteurs, capteurs, LEDs ET peut alimenter le micro:bit
- **Les deux ensemble** : Pas de problème, protection interne

**Pour développement :** USB + Batterie ON = configuration idéale

## 🔍 Étape 1 : Scanner les composants I2C

**TRÈS IMPORTANT** : Commencez toujours par scanner le bus I2C pour identifier les composants.

```bash
make scan
make repl
```

**Attendu :**
- `0x10` - STM8 motor controller (obligatoire)
- `0x33` - Capteur laser SEN0628 (ou 0x30-0x32 selon DIP switches)

**Si le capteur laser n'apparaît pas :**
1. Vérifier les connexions physiques
2. Vérifier l'alimentation (3.3-5V)
3. Vérifier les DIP switches (configuration adresse)
4. Power cycle complet (débrancher batterie)

## 🎯 Étape 2 : Tester le capteur laser

```bash
make test-laser
make repl
```

Le script effectue automatiquement :
- ✅ Scan I2C et détection du capteur
- ✅ Test lecture point unique
- ✅ Test matrice complète 8x8
- ✅ Test zones (gauche/centre/droite)
- ✅ Test performance (30 échantillons à 15Hz)
- ✅ Test mode 4x4 (optionnel, 60Hz)
- ✅ Diagnostic et recommandations

**Résultat attendu :**
```
✓ Sensor is working PERFECTLY!
  - 100% read success rate
  - 15.0 Hz actual rate (target: 15Hz)
```

**Si le test échoue :**
Consultez `LASER_SENSOR_INTEGRATION_PLAN.md` pour le dépannage détaillé.

## 🤖 Étape 3 : Déployer le programme d'évitement d'obstacles

```bash
make all
```

Cette commande :
1. Flash `main.py` (programme d'évitement d'obstacles)
2. Upload `maqueen_plus_v3.py` (bibliothèque robot)
3. Upload `laser_matrix.py` (driver laser)

**Utilisation :**
- **Button A** : Start/Stop
- **Button B** : Emergency stop
- **LEDs underglow** : Vert = OK, Rouge = obstacle
- **Display 5x5** : Flèches indiquant la direction

## 📚 Autres programmes

### Suiveur de ligne
```bash
make line-follower
```
**Avant utilisation :**
1. Calibrer les capteurs avec bouton "Calc-Key" sur le robot
2. Ajuster `LINE_THRESHOLD` dans le code selon votre surface
3. Placer le robot sur une ligne noire (fond blanc)

### Suiveur de lumière
```bash
make light-seeker
```
**Utilisation :**
- Pointer une lampe torche vers le robot
- Le robot suit la source de lumière

### Contrôle IR (template)
```bash
make remote-control
```
**Note :** Nécessite configuration des codes IR de votre télécommande.

## 🔧 Commandes utiles

```bash
# Développement
make all          # Flash + upload (déploiement complet)
make flash        # Flash main.py seulement
make upload       # Upload bibliothèques seulement
make ls           # Lister les fichiers sur micro:bit
make clean        # Nettoyer les bibliothèques

# Debugging
make repl         # Ouvrir REPL série (Ctrl-A K pour quitter)
make scan         # Scanner I2C
make test-laser   # Test complet du capteur laser
make test         # Self-test du robot

# Aide
make help         # Afficher toutes les commandes
make check        # Vérifier installation des outils
```

## 📝 Structure du code

### Bibliothèque principale (`maqueen_plus_v3.py`)

```python
from maqueen_plus_v3 import MaqueenPlusV3

robot = MaqueenPlusV3()

# Moteurs
robot.drive(150)           # Avancer
robot.turn(-100)           # Tourner gauche
robot.motors(100, 150)     # Contrôle indépendant
robot.stop()

# Capteur laser
zones = robot.laser_zones()  # {'left': mm, 'center': mm, 'right': mm}
matrix = robot.laser_matrix()  # Liste de 64 distances
distance = robot.laser_distance()  # Distance centrale

# Capteurs ligne
line = robot.line_sensors()     # [L2, L1, M, R1, R2] analog
digital = robot.line_digital()  # [L2, L1, M, R1, R2] digital

# Capteurs lumière
left, right = robot.light_sensors()

# LEDs
robot.headlights('red', 'green')
robot.underglow(0, 255, 0, 0)  # LED 0 = rouge
robot.underglow('all', 0, 255, 0)  # Toutes = vert
robot.underglow_off()

# Encodeurs
left, right = robot.read_encoders()
```

### Driver laser (`laser_matrix.py`)

```python
from laser_matrix import LaserMatrix

# Initialisation (essaie 0x33 par défaut)
sensor = LaserMatrix()

# Ou spécifier l'adresse
sensor = LaserMatrix(0x30)

# Configuration
sensor.set_mode(LaserMatrix.MODE_8x8)  # 64 points, 15Hz
sensor.set_mode(LaserMatrix.MODE_4x4)  # 16 points, 60Hz

# Lecture
zones = sensor.read_zones()  # Simplifié (L/C/R)
matrix = sensor.read_matrix()  # Matrice complète
dist = sensor.read_point(3, 3)  # Point spécifique
```

## ⚙️ Protocole I2C du capteur laser

Le SEN0628 utilise un **protocole à paquets** (pas de registres classiques).

**Format envoi :**
```
0x55 | argsNumH | argsNumL | cmd | args...
```

**Format réception :**
```
status | cmd | lenL | lenH | data...
```

**Commandes :**
- `0x01` - SET_MODE (4x4 ou 8x8)
- `0x02` - ALL_DATA (lire matrice)
- `0x03` - FIXED_POINT (lire point x,y)

**Adresses I2C :**
- `0x33` - Par défaut
- `0x30`, `0x31`, `0x32` - Alternatives (DIP switches)

## 🐛 Dépannage

### Le capteur laser n'est pas détecté

1. **Scanner I2C :**
   ```bash
   make scan
   make repl
   ```
   Chercher 0x33 (ou 0x30-0x32)

2. **Vérifier DIP switches :** Configuration d'adresse sur le capteur

3. **Power cycle :** Débrancher complètement, rebrancher

4. **Connexions physiques :** Vérifier câble I2C (PH2.0-4P)

### Erreurs de lecture du capteur

1. **Test diagnostic :**
   ```bash
   make test-laser
   make repl
   ```

2. **Symptômes courants :**
   - `Read error` répétés → Problème I2C (câble, pull-ups)
   - Valeurs toujours à 4000 → Capteur non initialisé correctement
   - Timeout → Bus I2C saturé ou capteur bloqué

3. **Solutions :**
   - Réduire vitesse lecture (augmenter `sleep()`)
   - Passer en mode 4x4 (plus rapide)
   - Vérifier alimentationdu capteur

### Le robot ne bouge pas

1. **Vérifier batterie :** Niveau de charge suffisant ?

2. **Test moteurs :**
   ```bash
   make test
   make repl
   ```

3. **Vérifier STM8 :** Doit apparaître à 0x10 dans le scan I2C

### Mémoire insuffisante (MemoryError)

MicroPython sur micro:bit V2 a ~100KB de RAM.

**Solutions :**
1. Supprimer fonctions inutilisées
2. Utiliser mode 4x4 au lieu de 8x8 (moins de données)
3. Réduire fréquence de lecture
4. Ne pas stocker l'historique des mesures

## 📖 Ressources

### Documentation
- **README.md** - Documentation complète du projet
- **CLAUDE.md** - Contexte technique pour AI
- **LASER_SENSOR_INTEGRATION_PLAN.md** - Intégration capteur laser (historique)

### Liens externes
- **Maqueen Plus V3 Wiki :** https://wiki.dfrobot.com/SKU_MBT0050_Maqueen_Plus_V3
- **SEN0628 Laser Sensor :** https://wiki.dfrobot.com/SKU_SEN0628_Matrix%20Laser%20Ranging%20Sensor
- **Bibliothèque Arduino :** https://github.com/DFRobot/DFRobot_MatrixLidar
- **micro:bit MicroPython :** https://microbit-micropython.readthedocs.io/

## 🎓 Workflow de développement

### Modifier un programme

1. **Éditer dans Zed :**
   ```bash
   zed src/main.py
   ```

2. **Déployer :**
   ```bash
   make flash upload
   ```

3. **Débugger :**
   ```bash
   make repl
   ```

### Créer un nouveau programme

1. **Créer fichier :**
   ```bash
   zed src/mon_programme.py
   ```

2. **Importer bibliothèque :**
   ```python
   from maqueen_plus_v3 import MaqueenPlusV3
   robot = MaqueenPlusV3()
   # ... votre code
   ```

3. **Tester :**
   ```bash
   uflash src/mon_programme.py
   ufs put src/maqueen_plus_v3.py
   ufs put src/laser_matrix.py
   make repl
   ```

### Contraintes MicroPython

**À éviter :**
- ❌ `import time` → Utiliser `from microbit import sleep`
- ❌ `machine.I2C()` → Utiliser `microbit.i2c`
- ❌ Bibliothèques standard (`json`, `struct`, etc.) → Non disponibles
- ❌ Fichiers volumineux → Splitter en modules

**Disponible :**
- ✅ `microbit` - Hardware principal
- ✅ `neopixel` - WS2812 LEDs
- ✅ `radio` - Communication micro:bit
- ✅ `music` - Son/buzzer
- ✅ `speech` - Synthèse vocale

## 🎉 C'est parti !

Vous êtes maintenant prêt à programmer votre Maqueen Plus V3 !

**Commande magique pour tout tester :**
```bash
make check && make scan && make test-laser && make all
```

Cette commande :
1. ✅ Vérifie l'installation des outils
2. ✅ Scanne les composants I2C
3. ✅ Teste le capteur laser
4. ✅ Déploie le programme d'évitement d'obstacles

**Bon codage ! 🤖**
