"""Distance Map - Carte polaire 360 degres avec memoire"""
from microbit import running_time

# Configuration
NUM_SECTORS = 16       # 22.5 degres par secteur
MAX_AGE = 15           # Expire apres ~1.5s (15 ticks x 100ms)
MAX_DIST = 4000        # Distance max en mm
LIDAR_FOV_SECTORS = 3  # LIDAR couvre 3 secteurs (60 degres)

class DistanceMap:
    """Carte polaire 360 degres: 16 secteurs, 0=avant, 4=droite, 8=arriere, 12=gauche"""

    def __init__(self):
        # Stockage: (age << 12) | distance
        # distance: 0-4000 (12 bits)
        # age: 0-15 (4 bits)
        self._data = [0] * NUM_SECTORS
        self._last_update = 0
        self._heading = 0  # Rotation accumulee (degres)

    def _pack(self, dist, age):
        """Pack distance et age dans un entier"""
        d = min(MAX_DIST, max(0, dist))
        a = min(MAX_AGE, max(0, age))
        return (a << 12) | d

    def _unpack(self, val):
        """Unpack entier en (distance, age)"""
        return (val & 0xFFF, val >> 12)

    def get_distance(self, sector):
        """Obtenir distance d'un secteur (0=invalide/expire)"""
        if sector < 0 or sector >= NUM_SECTORS:
            return 0
        d, a = self._unpack(self._data[sector])
        if a >= MAX_AGE:
            return 0  # Donnee expiree
        return d

    def get_age(self, sector):
        """Obtenir age d'un secteur"""
        if sector < 0 or sector >= NUM_SECTORS:
            return MAX_AGE
        _, a = self._unpack(self._data[sector])
        return a

    def set_sector(self, sector, dist):
        """Mettre a jour un secteur (age=0)"""
        if sector < 0 or sector >= NUM_SECTORS:
            return
        self._data[sector] = self._pack(dist, 0)

    def update_from_lidar(self, lidar_cols):
        """Mettre a jour depuis les 8 colonnes LIDAR (secteurs 15, 0, 1)"""
        if len(lidar_cols) != 8:
            return

        # Mapper les 8 colonnes aux 3 secteurs avant
        # Colonnes 0-1 -> secteur 15 (gauche)
        # Colonnes 2-5 -> secteur 0 (centre)
        # Colonnes 6-7 -> secteur 1 (droite)

        # Secteur 15 (avant-gauche): colonnes 0-2
        d15 = self._median([lidar_cols[0], lidar_cols[1], lidar_cols[2]])
        self.set_sector(15, d15)

        # Secteur 0 (avant): colonnes 2-5
        d0 = self._median([lidar_cols[2], lidar_cols[3], lidar_cols[4], lidar_cols[5]])
        self.set_sector(0, d0)

        # Secteur 1 (avant-droit): colonnes 5-7
        d1 = self._median([lidar_cols[5], lidar_cols[6], lidar_cols[7]])
        self.set_sector(1, d1)

    def _median(self, vals):
        """Calculer mediane (plus robuste que min)"""
        s = sorted(vals)
        n = len(s)
        if n == 0:
            return MAX_DIST
        return s[n // 2]

    def age_all(self):
        """Vieillir toutes les donnees d'un tick"""
        for i in range(NUM_SECTORS):
            d, a = self._unpack(self._data[i])
            if a < MAX_AGE:
                self._data[i] = self._pack(d, a + 1)

    def rotate(self, degrees):
        """Tourner la carte quand le robot tourne. degrees: positif=droite"""
        self._heading += degrees

        # Calculer nombre de secteurs a decaler
        sector_angle = 360 // NUM_SECTORS  # 22.5 degres
        shift = int(self._heading / sector_angle)

        if shift == 0:
            return

        # Mise a jour du reste
        self._heading -= shift * sector_angle

        # Rotation de la carte (inverse de la rotation robot)
        # Robot tourne a droite -> carte tourne a gauche
        if shift > 0:
            # Decaler vers la gauche (indices croissants)
            self._data = self._data[shift:] + self._data[:shift]
        else:
            # Decaler vers la droite (indices decroissants)
            shift = -shift
            self._data = self._data[-shift:] + self._data[:-shift]

        # Invalider les secteurs qui sortent du champ de vision
        # (ils seront remplis par de nouvelles lectures LIDAR)

    def get_front_sectors(self):
        """Obtenir les 8 secteurs avant (12-3, couvrant 180 degres)"""
        # Secteurs 12, 13, 14, 15, 0, 1, 2, 3
        indices = [12, 13, 14, 15, 0, 1, 2, 3]
        return [self.get_distance(i) for i in indices]

    def get_all_valid(self):
        """Obtenir tous les secteurs avec donnees valides"""
        result = []
        for i in range(NUM_SECTORS):
            d = self.get_distance(i)
            if d > 0:
                result.append((i, d, self.get_age(i)))
        return result

    def find_best_direction(self):
        """Trouver meilleure direction. Retourne (sector_index, distance)"""
        best_idx = 0
        best_dist = 0
        best_score = 0

        for i in range(NUM_SECTORS):
            d = self.get_distance(i)
            if d == 0:
                continue

            age = self.get_age(i)
            # Score = distance * fraicheur (age faible = meilleur)
            freshness = (MAX_AGE - age) / MAX_AGE
            score = d * freshness

            # Bonus pour secteurs avec voisins libres
            left = self.get_distance((i - 1) % NUM_SECTORS)
            right = self.get_distance((i + 1) % NUM_SECTORS)
            if left > 0 and right > 0:
                neighbor_avg = (left + right) // 2
                score += neighbor_avg * 0.3

            if score > best_score:
                best_score = score
                best_idx = i
                best_dist = d

        return (best_idx, best_dist)

    def sector_to_angle(self, sector):
        """Convertir index secteur en angle (0=avant, positif=droite)"""
        # Secteur 0 = 0 degres (avant)
        # Secteur 4 = 90 degres (droite)
        # Secteur 8 = 180 degres (arriere)
        # Secteur 12 = -90 degres (gauche)
        angle = sector * (360 // NUM_SECTORS)
        if angle > 180:
            angle -= 360
        return angle

    def angle_to_sector(self, angle):
        """Convertir angle en index secteur"""
        # Normaliser angle a 0-360
        while angle < 0:
            angle += 360
        while angle >= 360:
            angle -= 360
        return int(angle / (360 / NUM_SECTORS)) % NUM_SECTORS
