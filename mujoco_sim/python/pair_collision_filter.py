"""Model-local exact pair filtering using two otherwise unused collision bits.

No global mjcb_contactfilter callback (which would couple independent runners).
All original third-party compatibility decisions are preserved, including geoms
with contype=0 or conaffinity=0. Explicit MJCF <pair> bypasses bit filtering and
is rejected for the managed pair rather than silently providing no protection.
"""
import numpy as np


class PairCollisionFilter:
    def __init__(self, model, geom1, geom2):
        self.model, self.geoms = model, (geom1, geom2)
        if any({int(a), int(b)} == set(self.geoms)
               for a, b in zip(model.pair_geom1, model.pair_geom2)):
            raise ValueError('Explicit connector <pair> bypasses collision masks')
        self.original_type = model.geom_contype.copy()
        self.original_affinity = model.geom_conaffinity.copy()
        occupied = int(np.bitwise_or.reduce(self.original_type | self.original_affinity))
        available = [1 << bit for bit in range(30) if not occupied & (1 << bit)]
        if len(available) < 2:
            raise ValueError('Two unused collision bits required for connector pair')
        b1, b2 = available[:2]
        self.bits = b1, b2
        self.types = self.original_type.copy()
        self.affinities = self.original_affinity.copy()
        for pad, bit in zip(self.geoms, self.bits):
            for other in range(model.ngeom):
                if other in self.geoms:
                    continue
                compatible = ((int(self.original_type[pad]) & int(self.original_affinity[other])) |
                              (int(self.original_type[other]) & int(self.original_affinity[pad])))
                if compatible:
                    self.affinities[other] |= bit
            self.types[pad] = bit
        self.original_pair_active = bool(
            (int(self.original_type[geom1]) & int(self.original_affinity[geom2])) |
            (int(self.original_type[geom2]) & int(self.original_affinity[geom1])))
        self.set_active(True)

    def set_active(self, active):
        self.model.geom_contype[:] = self.types
        self.model.geom_conaffinity[:] = self.affinities
        g1, g2 = self.geoms
        b1, b2 = self.bits
        self.active = bool(active)
        enabled = self.active and self.original_pair_active
        self.model.geom_conaffinity[g1] = b2 if enabled else 0
        self.model.geom_conaffinity[g2] = b1 if enabled else 0
