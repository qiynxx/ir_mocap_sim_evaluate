"""
IR image generator for the simulation platform.

Generates realistic 8-bit grayscale IR camera images with LED blobs,
background noise, clutter, and distance-based intensity falloff.

Extracted and adapted from ir_mocap_sim_evaluate/mocap_sim.py.
"""

import time
import numpy as np
import cv2

# Reference resolution for the base LED radius / glow constants
_REF_WIDTH = 1920

IR_LED_INTENSITY = 255
IR_LED_RADIUS_BASE = 2       # at 1920 width (2-3 pixels)
IR_LED_GLOW_RADIUS_BASE = 4  # at 1920 width
IR_BACKGROUND_NOISE_MEAN = 15
IR_BACKGROUND_NOISE_STD = 8
IR_SALT_PEPPER_PROB = 0.001
IR_GAUSSIAN_BLUR_SIZE = 3

# Perspective size model: blob radius ∝ 1/d (linear), intensity constant.
# At IR_REF_DISTANCE the blob is drawn at full base radius.
IR_REF_DISTANCE = 0.5        # reference distance (meters) where blob = base radius


class IRImageGenerator:
    """
    Generates realistic IR camera images with:
    - LED blobs with intensity falloff and glow
    - Background noise (Gaussian + salt/pepper)
    - Random reflections and clutter objects
    """

    def __init__(self, width, height, led_radius=None, glow_radius=None):
        self.width = width
        self.height = height

        scale = width / _REF_WIDTH
        self.led_radius = led_radius if led_radius is not None else max(2, int(IR_LED_RADIUS_BASE * scale))
        self.glow_radius = glow_radius if glow_radius is not None else max(4, int(IR_LED_GLOW_RADIUS_BASE * scale))

        self.clutter_objects = []
        self.clutter_update_interval = 1.0  # seconds
        self._last_clutter_update = 0.0

        # Pre-allocated buffers
        self._buf = np.empty((height, width), dtype=np.float32)
        self._sp_buf = np.empty((height, width), dtype=np.float32)

        # Background noise cache: regenerated at ~15 Hz instead of every frame.
        # IR sensor thermal noise is temporally correlated; 15 Hz is imperceptible.
        self.rng = np.random.default_rng()
        self._f64_buf = np.empty((height, width), dtype=np.float64)
        self._bg_cache = np.empty((height, width), dtype=np.float32)
        self._bg_last_update = 0.0
        self.bg_update_interval = 1.0 / 15.0  # seconds (~15 Hz)

    def update_led_params(self, led_radius, glow_radius):
        self.led_radius = led_radius
        self.glow_radius = glow_radius

    def _update_clutter(self, noise_active):
        now = time.monotonic()
        if now - self._last_clutter_update < self.clutter_update_interval:
            return
        self._last_clutter_update = now
        self.clutter_objects = []

        if not noise_active:
            return

        for _ in range(np.random.randint(2, 5)):
            self.clutter_objects.append({
                'type': 'reflection',
                'pos': (np.random.randint(0, self.width), np.random.randint(0, self.height)),
                'intensity': np.random.randint(80, 180),
                'size': np.random.randint(2, 6),
            })

        for _ in range(np.random.randint(1, 3)):
            self.clutter_objects.append({
                'type': 'rect',
                'pos': (np.random.randint(0, max(1, self.width - 50)),
                        np.random.randint(0, max(1, self.height - 50))),
                'size': (np.random.randint(20, 80), np.random.randint(20, 60)),
                'intensity': np.random.randint(30, 80),
            })

        for _ in range(np.random.randint(0, 2)):
            self.clutter_objects.append({
                'type': 'line',
                'start': (np.random.randint(0, self.width), np.random.randint(0, self.height)),
                'end': (np.random.randint(0, self.width), np.random.randint(0, self.height)),
                'intensity': np.random.randint(60, 120),
            })

    def generate_image(self, led_positions_2d, noise_active=False):
        """
        Generate a realistic IR camera image.

        Args:
            led_positions_2d: list of (u, v, distance) or None entries.
            noise_active: whether to add clutter and salt-pepper noise.

        Returns:
            uint8 grayscale ndarray of shape (height, width).
        """
        # --- Background layer: regenerate at ~15 Hz, reuse otherwise ---
        now = time.monotonic()
        if now - self._bg_last_update >= self.bg_update_interval:
            self.rng.standard_normal(out=self._f64_buf)
            self._f64_buf *= IR_BACKGROUND_NOISE_STD
            self._f64_buf += IR_BACKGROUND_NOISE_MEAN
            np.clip(self._f64_buf, 0, 255, out=self._f64_buf)
            np.copyto(self._bg_cache, self._f64_buf, casting='unsafe')

            self._update_clutter(noise_active)
            if noise_active:
                for obj in self.clutter_objects:
                    if obj['type'] == 'reflection':
                        cv2.circle(self._bg_cache, obj['pos'], obj['size'], obj['intensity'], -1)
                    elif obj['type'] == 'rect':
                        x, y = obj['pos']
                        w, h = obj['size']
                        self._bg_cache[y:y+h, x:x+w] = np.clip(
                            self._bg_cache[y:y+h, x:x+w] + obj['intensity'], 0, 255)
                    elif obj['type'] == 'line':
                        cv2.line(self._bg_cache, obj['start'], obj['end'], obj['intensity'], 1)

            self._bg_last_update = now

        # --- Foreground layer: copy background, draw LEDs every frame ---
        np.copyto(self._buf, self._bg_cache)
        img = self._buf

        for led_data in led_positions_2d:
            if led_data is None:
                continue
            u, v, distance = led_data

            if u < 0 or u >= self.width or v < 0 or v >= self.height:
                continue

            # Fixed LED size regardless of distance
            core_radius = self.led_radius
            glow_radius = self.glow_radius

            # IR LED intensity stays constant regardless of distance
            intensity = int(IR_LED_INTENSITY * (0.85 + np.random.rand() * 0.15))

            for r in range(glow_radius, core_radius, -1):
                alpha = (glow_radius - r) / max(1, glow_radius - core_radius) * 0.5
                glow_intensity = int(intensity * alpha)
                cv2.circle(img, (int(u), int(v)), r, glow_intensity, 1)

            cv2.circle(img, (int(u), int(v)), core_radius, intensity, -1)
            bright_core = min(255, int(intensity * 1.2))
            cv2.circle(img, (int(u), int(v)), max(1, core_radius // 2), bright_core, -1)

        if noise_active:
            self._sp_buf[:] = np.random.random(size=(self.height, self.width))
            img[self._sp_buf < IR_SALT_PEPPER_PROB] = 255
            img[self._sp_buf > 1 - IR_SALT_PEPPER_PROB] = 0

        img = np.clip(img, 0, 255).astype(np.uint8)
        img = cv2.GaussianBlur(img, (IR_GAUSSIAN_BLUR_SIZE, IR_GAUSSIAN_BLUR_SIZE), 0)
        return img
