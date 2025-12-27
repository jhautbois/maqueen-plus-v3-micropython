# Plan d'intégration du capteur SEN0628 Matrix Laser Ranging Sensor

## ✅ MISE À JOUR : Driver complété !

**Le driver `laser_matrix.py` a été complètement réécrit basé sur le protocole officiel DFRobot.**

### Protocole découvert

Le capteur SEN0628 utilise un **protocole à base de paquets de commandes** (pas de registres I2C classiques).

**Format de paquet envoyé :**
```
HEAD | argsNumH | argsNumL | cmd | args[0..n]
0x55 | (len>>8) | (len&0xFF) | 0x01-0x08 | arguments
```

**Format de paquet reçu :**
```
status | cmd | lenL | lenH | buf[0..n]
0x53/0x63 | echo_cmd | (len&0xFF) | (len>>8) | données
```

**Commandes principales :**
- `0x01` - SET_MODE : Configure 4x4 ou 8x8
- `0x02` - ALL_DATA : Lire toute la matrice (64 ou 16 points)
- `0x03` - FIXED_POINT : Lire un point spécifique (x, y)

**Adresse I2C par défaut : 0x33** (configurable à 0x30, 0x31, 0x32)

### Test rapide

```bash
# 1. Scanner I2C
make scan
make repl

# 2. Tester le capteur laser
uflash src/test_laser.py
ufs put src/laser_matrix.py
make repl
```

Le script `test_laser.py` effectue un diagnostic complet automatique.

---

## Informations du capteur (depuis le wiki DFRobot)

### Spécifications techniques
- **Modèle**: SEN0628 Matrix Laser Ranging Sensor
- **Adresses I2C**: 0x30, 0x31, 0x32, 0x33 (sélection via DIP switches)
- **Résolution**: 8x8 (64 points) ou 4x4 (16 points)
- **Portée**: 20-4000 mm
- **Vitesse**: 15-60 Hz (configurable)
- **Format données**: 16-bit unsigned integers (millimètres)
- **Voltage**: 3.3V-5V compatible
- **Interface**: PH2.0-4P (Gravity)

### Points importants
1. **Changement d'adresse**: Déconnecter l'alimentation après chaque changement d'adresse via DIP switches
2. **Valeurs invalides**: Le firmware V1.3 retourne 4000 pour les valeurs invalides
3. **Modes disponibles**: 4x4 (plus rapide, 60Hz) et 8x8 (plus précis, 15Hz)

## Étapes de test et intégration

### Phase 1: Identification du capteur (PRIORITAIRE)

```bash
# 1. Scanner le bus I2C pour trouver le capteur
make scan
make repl
```

**Vérifications**:
- [ ] Le capteur apparaît à l'adresse 0x30 (ou 0x31-0x33 selon DIP switches)
- [ ] Le STM8 apparaît à 0x10
- [ ] Aucune collision d'adresse I2C

**Si le capteur n'apparaît pas**:
1. Vérifier les connexions physiques (PH2.0-4P)
2. Vérifier l'alimentation du capteur (3.3-5V)
3. Vérifier les DIP switches (configuration d'adresse)
4. Power cycle complet (débrancher batterie)

### Phase 2: Protocole I2C - Tests manuels

Créer un script de test minimal pour explorer le protocole:

```python
# test_laser_protocol.py
from microbit import i2c, display, sleep, Image

LASER_ADDR = 0x30  # Ajuster selon votre configuration

def test_read_raw():
    """Tester la lecture brute de données"""
    display.show(Image.HEART)

    # Test 1: Lire les premiers registres (identification?)
    print("Test 1: Reading first registers...")
    for reg in range(0x00, 0x10):
        try:
            i2c.write(LASER_ADDR, bytes([reg]))
            data = i2c.read(LASER_ADDR, 4)
            print(f"Reg 0x{reg:02X}: {[hex(b) for b in data]}")
        except:
            print(f"Reg 0x{reg:02X}: ERROR")
        sleep(50)

    # Test 2: Lire un bloc de données (distances?)
    print("\nTest 2: Reading data block...")
    try:
        # Essayer de lire 128 bytes (64 x 16-bit)
        data = i2c.read(LASER_ADDR, 128)
        print(f"Got {len(data)} bytes")

        # Interpréter comme distances 16-bit
        for i in range(0, min(16, len(data)), 2):
            dist = (data[i] << 8) | data[i+1]
            print(f"Point {i//2}: {dist} mm")
    except Exception as e:
        print(f"ERROR: {e}")

    display.show(Image.HAPPY)

test_read_raw()
```

**Actions**:
- [ ] Flasher le script de test: `uflash test_laser_protocol.py`
- [ ] Observer les résultats dans le REPL
- [ ] Noter quels registres retournent des données valides
- [ ] Identifier le format des données

### Phase 3: Reverse engineering depuis MakeCode

Si les tests manuels ne suffisent pas, analyser la bibliothèque MakeCode officielle:

**Bibliothèques de référence**:
1. https://github.com/DFRobot/pxt-DFRobot_MaqueenPlus_v20
2. Chercher les fonctions liées au laser:
   - `getLaserData()`
   - `setLaserMode()`
   - Commandes I2C utilisées

**Recherche**:
```bash
# Cloner la bibliothèque MakeCode
git clone https://github.com/DFRobot/pxt-DFRobot_MaqueenPlus_v20.git
cd pxt-DFRobot_MaqueenPlus_v20

# Chercher les références I2C au laser
grep -r "0x30" .
grep -r "laser" -i .
grep -r "matrix" -i .
```

**Extraire**:
- [ ] Adresses de registres utilisées
- [ ] Séquence d'initialisation
- [ ] Format de commande pour lecture
- [ ] Format de réponse (big-endian vs little-endian)
- [ ] Commandes de configuration (mode 4x4 vs 8x8)

### Phase 4: Bibliothèque VL53L8CX

Le SEN0628 utilise probablement le chip VL53L8CX de STMicroelectronics.

**Références Arduino**:
- https://github.com/DFRobot/DFRobot_VL53L8CX

**Fonctions clés à porter en MicroPython**:
```cpp
// À adapter depuis Arduino vers MicroPython
uint8_t begin();                              // Initialisation
uint8_t setRangingMode(eMatrix_t matrix);     // 4x4 ou 8x8
uint8_t getAllData(void *buf);                 // Lire matrice complète
uint16_t getFixedPointData(uint8_t x, uint8_t y);  // Lire un point
```

**Points d'attention**:
- Le VL53L8CX nécessite souvent un firmware upload via I2C
- Vérifier si le STM8 gère l'initialisation du capteur
- Le capteur peut avoir un délai de boot (attendre ~100ms après power-on)

### Phase 5: Mise à jour de laser_matrix.py

Basé sur les découvertes des phases précédentes:

```python
class LaserMatrix:
    """Driver pour SEN0628 (VL53L8CX)"""

    DEFAULT_ADDR = 0x30

    # Registres découverts lors des tests
    REG_INIT = 0x??           # À déterminer
    REG_MODE = 0x??           # Configuration 4x4/8x8
    REG_STATUS = 0x??         # Status de mesure
    REG_DATA_START = 0x??     # Début des données

    def __init__(self, addr=DEFAULT_ADDR):
        self.addr = addr
        self._check_connection()
        self._init_sensor()

    def _init_sensor(self):
        """Initialisation basée sur protocole découvert"""
        # TODO: Implémenter séquence d'init
        # 1. Vérifier si capteur est prêt
        # 2. Configurer mode 8x8
        # 3. Démarrer mesures continues
        pass

    def read_matrix(self):
        """Lire matrice 8x8"""
        # TODO: Implémenter lecture selon protocole
        # 1. Vérifier status (mesure prête?)
        # 2. Lire 128 bytes (64 x 16-bit)
        # 3. Parser en liste de distances
        # 4. Gérer valeurs invalides (4000)
        pass
```

**Checklist de mise à jour**:
- [ ] Implémenter séquence d'initialisation correcte
- [ ] Ajouter vérification de status avant lecture
- [ ] Corriger format de données (byte order)
- [ ] Implémenter mode 4x4 (optionnel, pour 60Hz)
- [ ] Gérer timeout de lecture
- [ ] Valider avec hardware réel

### Phase 6: Tests avec hardware

**Tests unitaires**:
```python
def test_laser_matrix():
    """Test du driver laser mis à jour"""
    from laser_matrix import LaserMatrix

    print("Test 1: Initialisation...")
    sensor = LaserMatrix()
    print("✓ Capteur initialisé")

    print("\nTest 2: Lecture simple...")
    dist = sensor.read_distance_simple()
    print(f"Distance centrale: {dist} mm")
    assert 20 <= dist <= 4000, "Distance invalide"

    print("\nTest 3: Matrice complète...")
    matrix = sensor.read_matrix()
    assert len(matrix) == 64, "Matrice incomplète"
    print(f"✓ 64 points lus")
    print(f"  Min: {min(matrix)} mm")
    print(f"  Max: {max(matrix)} mm")

    print("\nTest 4: Zones (gauche/centre/droite)...")
    zones = sensor.read_zones()
    print(f"  Gauche: {zones['left']} mm")
    print(f"  Centre: {zones['center']} mm")
    print(f"  Droite: {zones['right']} mm")

    print("\n✓ Tous les tests réussis!")
```

**Tests d'intégration**:
- [ ] Capteur lit correctement les distances
- [ ] Évitement d'obstacles fonctionne
- [ ] Pas de freeze / timeout I2C
- [ ] Performance acceptable (15 Hz minimum)

### Phase 7: Optimisation (optionnel)

**Si le driver fonctionne mais est lent**:

1. **Mode 4x4**: Passer en résolution réduite pour 60Hz
   ```python
   sensor.set_resolution(LaserMatrix.RES_4x4)
   ```

2. **Lecture partielle**: Ne lire que les zones nécessaires
   ```python
   # Au lieu de lire 64 points, lire seulement 3 zones
   # Calculer indices des points clés
   ```

3. **Cache**: Éviter lectures répétées
   ```python
   self._last_read = 0
   self._cache = None

   def read_matrix(self):
       now = running_time()
       if now - self._last_read < 50:  # Cache 50ms
           return self._cache
       # ...
   ```

## Ressources supplémentaires

### Documentation officielle
- **SEN0628 Wiki**: https://wiki.dfrobot.com/SKU_SEN0628_Matrix%20Laser%20Ranging%20Sensor
- **VL53L8CX Datasheet**: Disponible sur le site STMicroelectronics
- **Maqueen Plus V3 Wiki**: https://wiki.dfrobot.com/SKU_MBT0050_Maqueen_Plus_V3

### Bibliothèques de référence
- **Arduino (DFRobot)**: https://github.com/DFRobot/DFRobot_VL53L8CX
- **MakeCode (DFRobot)**: https://github.com/DFRobot/pxt-DFRobot_MaqueenPlus_v20
- **VL53L8CX Arduino (ST)**: https://github.com/stm32duino/VL53L8CX

### Support
- **Forum DFRobot**: https://forum.dfrobot.com/
- **Rechercher**: "Maqueen Plus V3 MicroPython laser"

## Prochaines étapes recommandées

1. **MAINTENANT**: Exécuter `make scan` pour confirmer l'adresse du capteur
2. **ENSUITE**: Créer et tester `test_laser_protocol.py`
3. **PUIS**: Analyser la bibliothèque MakeCode si nécessaire
4. **ENFIN**: Mettre à jour `laser_matrix.py` avec le protocole correct

## Notes

- Le driver actuel dans `laser_matrix.py` est un **template générique**
- Il fonctionnera peut-être tel quel, mais **probablement pas**
- Les registres et le protocole devront être ajustés selon le hardware réel
- Le STM8 pourrait **gérer le capteur** et exposer les données via ses propres registres
  - Dans ce cas, lire depuis STM8 au lieu de communiquer directement avec le capteur
  - Vérifier dans la doc Maqueen si le laser est "pré-intégré" via STM8

**Important**: Si le STM8 gère le capteur, le protocole I2C sera complètement différent !
Il faudra alors lire les données laser via les registres STM8, pas directement du capteur.

---

**Bon courage pour l'intégration ! N'hésitez pas à ajuster ce plan selon vos découvertes.**
