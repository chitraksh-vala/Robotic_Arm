"""
Object Detection Module for Robotic Arm Sorting System (standalone version)

This module uses OpenCV to detect and classify objects based on:
- Color (red, orange, yellow, lime, green, teal, cyan, blue, indigo, purple,
  magenta, pink, white, gray, black)
- Shape (circle, square, triangle, rectangle)

Run it directly to see live detections from a webcam in a window, with each
object's length, width and position in centimetres on your workspace board:

    python object_detector_standalone.py                 # first run: click the 4 board corners
    python object_detector_standalone.py --recalibrate   # redo the corner clicks
    python object_detector_standalone.py --object-height 4

Keys: c = recalibrate, q / Esc = quit. With --ik-preview, select an object
with 1-9 and press p to check its target on the IK server (preview only).

Noise and lighting: each frame is white-balanced and brightness-flattened
against the board, thresholds adapt to the measured board, and contours must
be solid, thick enough, off the board border and a plausible size
(--min-object-cm / --max-object-cm) for --confirm-frames frames. Dark, dull objects
are named by hue ("dark green"). Pale objects brighter than the board are
only detected with --white-objects; if they look pure white, lower --exposure. A saved reference image warns when the board has moved
since calibration (press c).

Requires: pip install opencv-python numpy
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
import sys
import time
import urllib.request
from typing import List, Tuple, Dict, Optional

import cv2
import numpy as np


class ObjectDetector:
    """Detects and classifies objects using computer vision"""

    def __init__(self):
        # OpenCV HSV hue is 0..179. These hue bands cover common saturated
        # object colors; separate bands avoid labeling one contour twice.
        self.color_ranges = {
            'red': [(0, 8), (172, 179)],
            'orange': [(9, 19)],
            'yellow': [(20, 34)],
            'lime': [(35, 44)],
            'green': [(45, 79)],
            'teal': [(80, 89)],
            'cyan': [(90, 99)],
            'blue': [(100, 119)],
            'indigo': [(120, 134)],
            'purple': [(135, 149)],
            'magenta': [(150, 161)],
            'pink': [(162, 171)],
        }
        # On a white workspace, white/gray masks mostly detect lighting
        # gradients in the board itself, so the only neutral class is black,
        # thresholded relative to the board in _normalize_lighting().
        self.lighting = {}
        self.normalized_bgr = None

        self.min_area = 500  # Minimum contour area to consider
        self.max_area = 50000  # Maximum contour area to consider
        # Shape-quality filters that reject tape strips, shadows and glare
        # fragments while keeping solid pickable objects:
        self.min_solidity = 0.75     # contour area / convex hull area
        self.min_thickness_px = 14   # shortest side of the rotated box (full res)
        self.max_aspect = 6.0        # long side / short side of the rotated box
        # A contour that runs along the workspace boundary is almost always
        # the tape or something outside the board leaking in.
        self.max_border_fraction = 0.25
        # Objects in other colours may sit against the tape (e.g. a cube in a
        # corner has two sides on the border); thin tape strips are still
        # rejected by the thickness/solidity checks.
        self.max_border_fraction_objects = 0.6
        self.tape_colors = {'red', 'orange', 'pink', 'magenta'}
        self.tape_inset_px = 10
        # Detect objects clearly brighter than the board (pale or
        # overexposed boxes). Disable if strong glare on the board is
        # mistaken for objects.
        self.detect_white = False
        # The glossy board's bright stripes leave thin bright bands; a white
        # object must be at least ~2 cm across (full-resolution pixels).
        self.min_white_thickness_px = 36
        # Lab tint (|a,b| of the averaged colour) above which a dark object is
        # named by its hue ("dark green") instead of black/gray.
        self.min_dark_tint = 2.5
        # Shadow removal: board-coloured pixels between this and 0.95 x the
        # nearby board brightness, attached to a dark object's outline. Black
        # objects measure ~0.28 x, the shadows here 0.5-0.8 x.
        self.shadow_min_ratio = 0.4
        self.shadow_max_tint = 2.5
        # Diagnostics: per-frame count of contours each filter rejected.
        self.rejected = {}

    def detect_objects(self, frame: np.ndarray,
                       roi_mask: Optional[np.ndarray] = None) -> List[Dict]:
        """
        Detect objects in the frame

        Args:
            frame: BGR image from camera

        Returns:
            List of detected objects with properties
        """
        objects = []
        self.rejected = {}
        # Restrict processing to the calibrated board, then segment a smaller
        # image. Contours are mapped back to full-frame coordinates below.
        offset_x = offset_y = 0
        if roi_mask is not None:
            offset_x, offset_y, roi_w, roi_h = cv2.boundingRect(roi_mask)
            frame = frame[offset_y:offset_y + roi_h,
                          offset_x:offset_x + roi_w]
            roi_mask = roi_mask[offset_y:offset_y + roi_h,
                                offset_x:offset_x + roi_w]
        detect_scale = 0.4
        frame = cv2.resize(frame, None, fx=detect_scale, fy=detect_scale,
                           interpolation=cv2.INTER_AREA)
        roi_interior = None
        if roi_mask is not None:
            roi_mask = cv2.resize(roi_mask, (frame.shape[1], frame.shape[0]),
                                  interpolation=cv2.INTER_NEAREST)
            # Pixels more than a few px inside the ROI; contour points outside
            # this band lie on the workspace boundary.
            roi_interior = cv2.erode(roi_mask, np.ones((7, 7), np.uint8))
            # Red/orange/pink masks share the tape's colour, so they stay a
            # further tape_inset_px inside the calibrated corners.
            inset = max(1, int(round(self.tape_inset_px * detect_scale)))
            roi_tape_safe = cv2.erode(roi_mask, cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (2 * inset + 1, 2 * inset + 1)))

        # Normalise lighting against the white board (flatten brightness,
        # remove colour cast), then threshold relative to the board itself.
        hsv, light = self._normalize_lighting(frame, roi_mask)
        self.lighting = light
        s_min, v_min = light['s_min'], light['v_min']
        # Smooth sensor texture before thresholding so a printed/textured
        # circle forms one solid contour instead of many small patches.
        hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

        # Require clear color on most hues so slight color casts on the white
        # workspace do not become false cyan/blue objects.
        color_masks = {}
        for color_name, hue_ranges in self.color_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for hue_low, hue_high in hue_ranges:
                band = cv2.inRange(hsv, (hue_low, s_min, v_min), (hue_high, 255, 255))
                mask = cv2.bitwise_or(mask, band)
            color_masks[color_name] = mask
        # Anything clearly darker than the board (black, or dull/dark colours
        # like a dark green cube with too little saturation for the hue masks).
        # Relative to the board, so dim rooms do not make the whole board dark;
        # the board's own stripes stay above ~0.78 x board brightness.
        # Patterned objects (e.g. a cube with a pale criss-cross on a dark top)
        # break into fragments per pixel; averaged over ~1.3 cm they are still
        # clearly darker than the board, so the whole face is kept.
        smoothed_v = cv2.blur(hsv[..., 2], (9, 9))
        dark = (hsv[..., 2] <= light['dark_v']) | (smoothed_v <= light['dark_v'])
        # Strongly coloured dark regions (the red tape, a saturated object)
        # belong to the hue masks and their tape rules, not to this one.
        # Average the colour before taking saturation: per-pixel saturation
        # of dim, amplified pixels is mostly noise.
        blurred_bgr = cv2.blur(self.normalized_bgr, (9, 9))
        dark &= cv2.cvtColor(blurred_bgr, cv2.COLOR_BGR2HSV)[..., 1] < s_min
        color_masks['dark'] = dark.astype(np.uint8) * 255
        # Bright, low-saturation regions: pale or overexposed objects whose
        # colour the camera has lost. Board glare stays below light_v.
        if self.detect_white:
            color_masks['white'] = cv2.inRange(hsv, (0, 0, light['light_v']),
                                               (179, s_min, 255))

        # Where coloured objects are: a dark blob that is all shadow and
        # touches one of them is that object's cast shadow, not an object.
        coloured_union = np.zeros(hsv.shape[:2], np.uint8)
        for color_name, mask in color_masks.items():
            if color_name != 'dark':
                coloured_union |= mask
        coloured_union = cv2.morphologyEx(coloured_union, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        # Remove tiny speckles, then close small gaps inside textured objects.
        # The ellipse open removes 1-2 px sensor noise and thin glare edges
        # that a 3x3 square kernel let through.
        open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        close_kernel = np.ones((9, 9), np.uint8)
        for color_name, mask in color_masks.items():
            # A small median and morphological cleanup suppress isolated
            # speckles without applying several full-frame filters per color.
            mask = cv2.medianBlur(mask, 3)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
            if roi_mask is not None:
                tape_coloured = color_name in self.tape_colors
                mask = cv2.bitwise_and(mask, roi_tape_safe if tape_coloured else roi_mask)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                if color_name == 'dark':
                    # Textured or unevenly lit objects come out as fragments;
                    # grow them to the object's real outline first, then cut
                    # away the cast shadow that the darkness test also caught.
                    contour = self._refine_outline(contour, roi_mask)
                    contour = self._strip_shadow(contour, roi_mask, coloured_union)
                    if contour is None:
                        self.rejected['shadow'] = self.rejected.get('shadow', 0) + 1
                        continue
                reason = self._reject_reason(
                    contour, detect_scale, roi_interior,
                    self.min_white_thickness_px if color_name == 'white'
                    else self.min_thickness_px,
                    self.max_border_fraction if color_name in self.tape_colors
                    else self.max_border_fraction_objects)
                if reason:
                    self.rejected[reason] = self.rejected.get(reason, 0) + 1
                    continue
                # Label colour from the lighting-normalised pixels.
                color = self._estimate_color(hsv, contour, color_name, light)
                # Remove pixel-scale stair steps from the outline while
                # retaining the measured object boundary and its corners.
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    contour = cv2.approxPolyDP(contour, 0.018 * perimeter, True)
                contour = np.round(contour.astype(np.float32) / detect_scale).astype(np.int32)
                contour[:, 0, 0] += offset_x
                contour[:, 0, 1] += offset_y
                area = cv2.contourArea(contour)

                if self.min_area < area < self.max_area:
                    # Get object properties
                    obj = self._analyze_contour(contour, color)
                    if obj:
                        objects.append(obj)

        # Several hue/neutral masks can describe the same physical object.
        # Keep the largest contour and discard detections mostly contained in
        # its box, which prevents one mottled circle becoming several objects.
        objects.sort(key=lambda obj: obj['area'], reverse=True)
        unique = []
        for obj in objects:
            x, y, w, h = obj['bbox']
            duplicate = False
            for kept in unique:
                kx, ky, kw, kh = kept['bbox']
                ix = max(0, min(x + w, kx + kw) - max(x, kx))
                iy = max(0, min(y + h, ky + kh) - max(y, ky))
                intersection = ix * iy
                smaller_box = min(w * h, kw * kh)
                if smaller_box and intersection / smaller_box >= 0.65:
                    duplicate = True
                    break
            if not duplicate:
                unique.append(obj)
        return unique

    def _reject_reason(self, contour, detect_scale: float,
                       roi_interior: Optional[np.ndarray],
                       min_thickness_px: float,
                       max_border_fraction: float) -> Optional[str]:
        """Return why a raw (detection-scale) contour is noise, or None.

        Checks run on the unsimplified outline: approxPolyDP can turn a thin
        L-shaped tape strip into a solid-looking triangle.
        """
        area = cv2.contourArea(contour)
        min_area_scaled = self.min_area * detect_scale * detect_scale
        if area <= min_area_scaled:
            return "small"
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        if hull_area <= 0 or area / hull_area < self.min_solidity:
            return "not solid"
        (_, _), (rw, rh), _ = cv2.minAreaRect(contour)
        short_side, long_side = min(rw, rh), max(rw, rh)
        if short_side / detect_scale < min_thickness_px:
            return "thin"
        if long_side / max(short_side, 1e-6) > self.max_aspect:
            return "elongated"
        if roi_interior is not None:
            pts = contour.reshape(-1, 2)
            xs = np.clip(pts[:, 0], 0, roi_interior.shape[1] - 1)
            ys = np.clip(pts[:, 1], 0, roi_interior.shape[0] - 1)
            on_border = float(np.mean(roi_interior[ys, xs] == 0))
            if on_border > max_border_fraction:
                return "on border"
        return None

    def _normalize_lighting(self, frame: np.ndarray,
                            roi_mask: Optional[np.ndarray]) -> Tuple[np.ndarray, Dict]:
        """Return (HSV image, thresholds) normalised against the white board.

        1. Flatten brightness: estimate the board's light field (a large
           closing removes darker objects, a blur smooths it) and divide it
           out, so spotlights, vignetting and dim rooms all map the board to
           about the same brightness.
        2. White balance: scale channels so the median board pixel is
           neutral, which removes warm/cool casts that shift hues.
        3. Thresholds are then set relative to the measured board.
        """
        valid = (roi_mask > 0) if roi_mask is not None else np.ones(frame.shape[:2], bool)
        if not valid.any():
            valid = np.ones(frame.shape[:2], bool)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        board_gray = float(np.median(gray[valid]))
        # Outside the board (table, tape) must not pull the light field down.
        gray_filled = gray.copy()
        gray_filled[~valid] = int(board_gray)
        # A wide median ignores objects both darker and brighter than the
        # board (a closing would fold large bright boxes into the light
        # field and normalise them away). Run it on a 1/4 image for speed.
        small = cv2.resize(gray_filled, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
        k = max(5, int(0.6 * min(small.shape[:2])) | 1)
        light = cv2.medianBlur(small, k)
        light = cv2.resize(light, (frame.shape[1], frame.shape[0]),
                           interpolation=cv2.INTER_LINEAR).astype(np.float32)
        light = cv2.GaussianBlur(light, (0, 0), max(3.0, k))
        # Cap the gain so near-black regions are not amplified into noise.
        gain = np.clip(200.0 / np.maximum(light, 1.0), 0.6, 4.0)
        img = frame.astype(np.float32) * gain[..., None]
        board_bgr = np.median(img[valid], axis=0)
        img *= (float(board_bgr.mean()) / np.maximum(board_bgr, 1.0))[None, None, :]
        img = np.clip(img, 0, 255).astype(np.uint8)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        self.normalized_bgr = img

        board_s = float(np.percentile(hsv[..., 1][valid], 60))
        board_v = float(np.median(hsv[..., 2][valid]))
        thresholds = {
            # Colour must stand clearly above the board's own residual tint.
            's_min': int(np.clip(board_s + 30, 45, 90)),
            'v_min': int(np.clip(0.2 * board_v, 25, 60)),
            'muted_s_min': int(np.clip(board_s + 12, 15, 60)),
            # Black = much darker than the board. Overexposure lifts black to
            # ~40% of the board.
            'black_v': int(np.clip(0.45 * board_v, 40, 105)),
            # Darker-than-board objects. Measured on this board: its darkest
            # stripes stay above ~0.78 x board, a dull dark-green cube on the
            # bright patch is ~0.71 x board.
            'dark_v': int(np.clip(0.75 * board_v, 60, 190)),
            # Objects clearly brighter than the board (pale or overexposed
            # boxes whose colour the camera has lost).
            'light_v': int(np.clip(board_v + 45, 235, 250)),
            'board_brightness': board_gray,
            'board_v': board_v,
            # Share of board pixels the camera has clipped to white.
            'overexposed': float(np.mean(frame.max(axis=2)[valid] >= 250)),
        }
        return hsv, thresholds

    def _refine_outline(self, contour, roi_mask: Optional[np.ndarray]):
        """Grow a fragmentary blob to the object's outline with a watershed.

        A patterned top face (pale lines on a dark face) or one half in a
        sunlit patch is only partly darker than the board. Seeded with the
        blob's core as object and a ring well outside it as board, the
        watershed assigns the band in between to whichever side it connects
        to across weaker edges, so the boundary settles on the object's real
        edge. Falls back to the input if the result looks implausible.
        """
        img = self.normalized_bgr
        x, y, w, h = cv2.boundingRect(contour)
        grow = max(6, int(0.35 * max(w, h)))
        x0, y0 = max(0, x - grow - 2), max(0, y - grow - 2)
        x1 = min(img.shape[1], x + w + grow + 2)
        y1 = min(img.shape[0], y + h + grow + 2)
        crop = cv2.GaussianBlur(img[y0:y1, x0:x1], (3, 3), 0)
        blob = np.zeros(crop.shape[:2], np.uint8)
        cv2.drawContours(blob, [contour - np.array([x0, y0])], -1, 255, cv2.FILLED)
        sure_fg = cv2.erode(blob, np.ones((5, 5), np.uint8))
        if not sure_fg.any():
            sure_fg = blob
        reach = cv2.dilate(blob, cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * grow + 1, 2 * grow + 1)))
        markers = np.zeros(crop.shape[:2], np.int32)
        markers[reach == 0] = 1
        if roi_mask is not None:
            markers[roi_mask[y0:y1, x0:x1] == 0] = 1
        markers[sure_fg > 0] = 2
        cv2.watershed(crop, markers)
        region = (markers == 2).astype(np.uint8) * 255
        # Bridge the 1 px watershed lines inside textured faces.
        region = cv2.morphologyEx(region, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        found, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not found:
            return contour
        refined = max(found, key=cv2.contourArea)
        old_area, new_area = cv2.contourArea(contour), cv2.contourArea(refined)
        hull_area = cv2.contourArea(cv2.convexHull(refined))
        # Accept growth only if it stays compact: a leak into the board shows
        # up as a large jump in area or a ragged, non-solid outline.
        if not (0.8 * old_area <= new_area <= 2.2 * max(old_area, 1.0)):
            return contour
        if hull_area <= 0 or new_area / hull_area < 0.85:
            return contour
        return refined + np.array([x0, y0])

    def _strip_shadow(self, contour, roi_mask: Optional[np.ndarray],
                      coloured: Optional[np.ndarray] = None):
        """Remove the cast shadow attached to a dark object's outline.

        A shadow is the board under less light: darker than the board around
        it (here 0.4-0.95 x) but the same colour, while the object differs in
        colour (a dark green cube: Lab tint ~6 vs the board) or is darker
        still (black). Pixels like that are removed when they connect to the
        outline, so pale pattern lines inside an object's face are kept.
        If most of the blob is shadow-like: it is dropped (None) when it
        touches a coloured object (that object's shadow), otherwise kept as
        is (a genuinely grey object).
        """
        img = self.normalized_bgr
        x, y, w, h = cv2.boundingRect(contour)
        pad = 6
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(img.shape[1], x + w + pad), min(img.shape[0], y + h + pad)
        crop = cv2.GaussianBlur(img[y0:y1, x0:x1], (5, 5), 0)
        region = np.zeros(crop.shape[:2], np.uint8)
        cv2.drawContours(region, [contour - np.array([x0, y0])], -1, 255, cv2.FILLED)
        ring = cv2.dilate(region, np.ones((9, 9), np.uint8)) & ~cv2.dilate(region, np.ones((3, 3), np.uint8))
        if roi_mask is not None:
            ring &= roi_mask[y0:y1, x0:x1]
        if not ring.any():
            return contour
        lab = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB).astype(np.float32)
        value = crop.max(axis=2).astype(np.float32)
        board_v = float(np.median(value[ring > 0]))
        board_ab = np.median(lab[ring > 0][:, 1:], axis=0)
        ratio = value / max(board_v, 1.0)
        tint_diff = np.hypot(lab[..., 1] - board_ab[0], lab[..., 2] - board_ab[1])
        shadow_like = ((ratio >= self.shadow_min_ratio) & (ratio <= 0.95)
                       & (tint_diff <= self.shadow_max_tint) & (region > 0)).astype(np.uint8)
        edge = region & ~cv2.erode(region, np.ones((3, 3), np.uint8))
        shadow = shadow_like & (edge > 0).astype(np.uint8)
        for _ in range(40):   # grow inward from the outline through shadow only
            grown = cv2.dilate(shadow, np.ones((3, 3), np.uint8)) & shadow_like
            if np.array_equal(grown, shadow):
                break
            shadow = grown
        keep = ((region > 0) & (shadow == 0)).astype(np.uint8) * 255
        # Drop thin shadow leftovers along the edge.
        keep = cv2.morphologyEx(keep, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        found, _ = cv2.findContours(keep, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = max(found, key=cv2.contourArea) if found else None
        if best is None or cv2.contourArea(best) < 0.6 * cv2.contourArea(contour):
            # Mostly shadow-like: a coloured object's shadow if it touches one.
            if coloured is not None:
                near = cv2.dilate(region, np.ones((7, 7), np.uint8))
                if np.any((near > 0) & (coloured[y0:y1, x0:x1] > 0)):
                    return None
            return contour   # a grey object, not a shadow
        return best + np.array([x0, y0])

    def _estimate_color(self, hsv: np.ndarray, contour, fallback: str,
                        light: Dict) -> str:
        """Choose one color label from the normalised pixels inside a contour
        (both in detection-scale coordinates)."""
        x, y, w, h = cv2.boundingRect(contour)
        inside = np.zeros((h, w), dtype=np.uint8)
        shifted = contour.copy()
        shifted[:, 0, 0] -= x
        shifted[:, 0, 1] -= y
        cv2.drawContours(inside, [shifted], -1, 255, cv2.FILLED)
        pixels = self.normalized_bgr[y:y + h, x:x + w][inside > 0]
        if pixels.size == 0:
            return fallback
        # Average the colour first, then convert once. Per-pixel hue and
        # saturation of dark, amplified pixels are mostly sensor noise.
        mean_bgr = np.median(pixels, axis=0).astype(np.uint8).reshape(1, 1, 3)
        hue, saturation, value = cv2.cvtColor(mean_bgr, cv2.COLOR_BGR2HSV)[0, 0].astype(float)
        if saturation >= light['s_min']:
            for color_name, ranges in self.color_ranges.items():
                if any(low <= hue <= high for low, high in ranges):
                    return color_name
        if fallback == 'white':
            return 'white'
        if fallback != 'dark':
            return fallback
        # Dark, dull objects. HSV saturation is unstable for dark colours, so
        # judge the tint in Lab: measured here, black objects stay within
        # |ab| <= 2.2 even under warm/cool casts, while a dull dark-green cube
        # is 2.8-6 on the green side.
        lab = cv2.cvtColor(mean_bgr, cv2.COLOR_BGR2LAB)[0, 0].astype(float)
        tint = float(np.hypot(lab[1] - 128.0, lab[2] - 128.0))
        if tint >= self.min_dark_tint:
            return "dark " + self._coarse_hue(hue)
        return 'black' if value <= light['black_v'] else 'gray'

    @staticmethod
    def _coarse_hue(hue: float) -> str:
        """Broad hue name for dull colours, where fine bands are unreliable."""
        if hue < 20 or hue >= 165:
            return 'red'
        if hue < 35:
            return 'yellow'
        if hue < 105:  # dull teal/cyan reads as green
            return 'green'
        if hue < 135:
            return 'blue'
        return 'purple'

    def _analyze_contour(self, contour, color: str) -> Dict:
        """Analyze a contour to determine shape and position"""

        # Calculate moments for centroid
        M = cv2.moments(contour)
        if M['m00'] == 0:
            return None

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        shape = self._classify_shape(contour)

        # Get bounding box
        x, y, w, h = cv2.boundingRect(contour)

        return {
            'color': color,
            'shape': shape,
            'centroid': (cx, cy),
            'bbox': (x, y, w, h),
            'area': cv2.contourArea(contour),
            'contour': contour
        }

    def _classify_shape(self, contour) -> str:
        """Classify by how the outline fills its rotated bounding box.

        A square/rectangle fills ~90-100% of its minimum-area box, a circle
        pi/4 = 78.5% with high circularity, a triangle ~50%. Vertex counts
        alone misread a square with slightly rounded corners as a circle,
        because a square's circularity is already 0.785.
        """
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        (_, _), (rw, rh), _ = cv2.minAreaRect(contour)
        if area <= 0 or perimeter <= 0 or rw <= 0 or rh <= 0:
            return 'unknown'
        fill = area / (rw * rh)
        aspect = max(rw, rh) / min(rw, rh)
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        hull = cv2.convexHull(contour)
        corners = len(cv2.approxPolyDP(hull, 0.04 * cv2.arcLength(hull, True), True))
        if fill >= 0.86:
            return 'square' if aspect <= 1.2 else 'rectangle'
        if circularity >= 0.80 and aspect <= 1.25:
            return 'circle'
        if corners == 3 and 0.4 <= fill <= 0.65:
            return 'triangle'
        if fill >= 0.80:
            return 'square' if aspect <= 1.2 else 'rectangle'
        return 'unknown'

    def draw_detections(self, frame: np.ndarray, objects: List[Dict]) -> np.ndarray:
        """Draw detected objects on the frame"""

        output = frame.copy()

        for obj in objects:
            # Draw contour
            cv2.drawContours(output, [obj['contour']], -1, (0, 255, 0), 2)

            # Draw centroid
            cx, cy = obj['centroid']
            cv2.circle(output, (cx, cy), 5, (0, 0, 255), -1)

            # Add label
            x, y, w, h = obj['bbox']
            label = f"{obj['color']} {obj['shape']}"
            cv2.putText(output, label, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return output

    def get_pixel_to_world_transform(self, camera_height: float,
                                     focal_length: float) -> np.ndarray:
        """
        Calculate transformation matrix from pixel coordinates to world coordinates

        Args:
            camera_height: Height of camera above work surface (mm)
            focal_length: Camera focal length in pixels

        Returns:
            Transformation matrix
        """
        # Simplified perspective transform
        # In a real system, this would use camera calibration
        scale = camera_height / focal_length
        return np.array([[scale, 0, 0],
                        [0, scale, 0],
                        [0, 0, 1]])


class ObjectTracker:
    """Confirm detections that persist across frames and give them stable IDs.

    Single-frame blips (a hand passing, glare, a shadow edge) never reach the
    confirm count. Confirmed objects survive a few missed frames so the list
    does not flicker, and their centroids are smoothed, which also steadies
    the pick target sent to the arm. Tracks match by position only; the shown
    colour and shape are a majority vote over recent frames, because a dull
    object's label can flip (black/gray/dark green) as the light changes.
    """

    def __init__(self, confirm_frames: int = 4, max_missed: int = 3,
                 match_px: float = 40.0, smoothing: float = 0.35, vote_frames: int = 8):
        self.confirm_frames = confirm_frames
        self.max_missed = max_missed
        self.match_px = match_px
        self.smoothing = smoothing  # weight of the newest centroid
        self.vote_frames = vote_frames
        self.tracks: List[Dict] = []
        self.next_id = 1

    def update(self, detections: List[Dict]) -> List[Dict]:
        pairs = []
        for ti, track in enumerate(self.tracks):
            tx, ty = track['centroid_f']
            for di, det in enumerate(detections):
                dx, dy = det['centroid']
                distance = float(np.hypot(dx - tx, dy - ty))
                if distance <= self.match_px:
                    pairs.append((distance, ti, di))
        matched_tracks, matched_dets = set(), set()
        for _, ti, di in sorted(pairs):
            if ti in matched_tracks or di in matched_dets:
                continue
            matched_tracks.add(ti)
            matched_dets.add(di)
            track, det = self.tracks[ti], detections[di]
            a = self.smoothing
            tx, ty = track['centroid_f']
            track['centroid_f'] = (a * det['centroid'][0] + (1 - a) * tx,
                                   a * det['centroid'][1] + (1 - a) * ty)
            track['obj'] = det
            track['labels'] = (track['labels'] + [(det['color'], det['shape'])])[-self.vote_frames:]
            track['hits'] += 1
            track['missed'] = 0
        for ti, track in enumerate(self.tracks):
            if ti not in matched_tracks:
                track['missed'] += 1
        self.tracks = [t for t in self.tracks if t['missed'] <= self.max_missed]
        for di, det in enumerate(detections):
            if di not in matched_dets:
                self.tracks.append({'id': self.next_id, 'obj': det, 'hits': 1, 'missed': 0,
                                    'centroid_f': tuple(float(v) for v in det['centroid']),
                                    'labels': [(det['color'], det['shape'])]})
                self.next_id += 1

        confirmed = []
        for track in sorted(self.tracks, key=lambda t: t['id']):
            if track['hits'] < self.confirm_frames:
                continue
            obj = dict(track['obj'])
            colors = [c for c, _ in track['labels']]
            shapes = [sh for _, sh in track['labels']]
            color = max(set(colors), key=colors.count)
            neutral = [c for c in colors if c in ('black', 'gray')]
            # A near-black object's faint tint is noise-limited: its hue jumps
            # between frames (dark red, dark green, ...). Only trust a "dark
            # <hue>" label that wins clearly; otherwise it is black/gray.
            if (color.startswith('dark ') and colors.count(color) < 0.6 * len(colors)
                    and neutral):
                color = max(set(neutral), key=neutral.count)
            obj['color'] = color
            obj['shape'] = max(set(shapes), key=shapes.count)
            obj['track_id'] = track['id']
            obj['centroid'] = (int(round(track['centroid_f'][0])),
                               int(round(track['centroid_f'][1])))
            obj['centroid_f'] = track['centroid_f']
            obj['stale'] = track['missed'] > 0
            confirmed.append(obj)
        return confirmed

    def reset(self) -> None:
        self.tracks = []


# ---------------------------------------------------------------------------
# Standalone runner: live detection with real-world (cm) measurements
#
# Pixels are converted to centimetres with a homography computed from the four
# corners of the taped board, whose real size is known. This replaces the
# camera-height / focal-length estimate: it needs neither, and it also corrects
# for a camera that is not perfectly square to the table.
# ---------------------------------------------------------------------------

WINDOW_NAME = "Object Detector"
PANEL_WIDTH = 450
LINE_H = 22
FONT = cv2.FONT_HERSHEY_SIMPLEX
# Default data files live at the repository root (sim2hw/ etc.), whatever
# folder the detector is started from, so every session reads and writes the
# same calibration and taught corrections.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_CALIB_FILE = "workspace_calibration.json"
DEFAULT_HEIGHT_CALIB_FILE = "workspace_height_calibration.json"
DEFAULT_IK_MAPPING_FILE = "sim2hw/cv_ik_mapping.json"
DEFAULT_IK_CORRECTIONS_FILE = "sim2hw/cv_pick_corrections.json"


# ---- calibration helpers ---------------------------------------------------

def save_calibration(path: str, frame_size: Tuple[int, int],
                     board_cm: Tuple[float, float], corners_px: np.ndarray) -> None:
    data = {"frame_size": list(frame_size), "board_cm": list(board_cm),
            "corners_px": np.asarray(corners_px).tolist()}
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


def calibration_reference_path(calib_path: str) -> str:
    """Image saved next to the corner calibration for drift checks."""
    stem = calib_path[:-5] if calib_path.lower().endswith(".json") else calib_path
    return stem + "_reference.png"


def _drift_edges(frame: np.ndarray, corners: np.ndarray) -> Tuple[np.ndarray, Tuple[int, int]]:
    """Edge image of the board area (plus its tape margin) at half scale."""
    x, y, w, h = cv2.boundingRect(corners.astype(np.int32))
    pad = 60
    x0, y0 = max(0, x - pad), max(0, y - pad)
    x1, y1 = min(frame.shape[1], x + w + pad), min(frame.shape[0], y + h + pad)
    gray = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    gray = cv2.GaussianBlur(gray, (5, 5), 0).astype(np.float32)
    edges = cv2.magnitude(cv2.Sobel(gray, cv2.CV_32F, 1, 0), cv2.Sobel(gray, cv2.CV_32F, 0, 1))
    return edges, (x0, y0)


def calibration_drift_px(reference: np.ndarray, frame: np.ndarray,
                         corners: np.ndarray) -> Optional[float]:
    """How far (pixels) the board appears to have moved since calibration,
    from phase correlation of edge images; None if no reliable estimate."""
    if reference is None or reference.shape != frame.shape:
        return None
    ref_edges, _ = _drift_edges(reference, corners)
    cur_edges, _ = _drift_edges(frame, corners)
    window = cv2.createHanningWindow(ref_edges.shape[::-1], cv2.CV_32F)
    (dx, dy), response = cv2.phaseCorrelate(ref_edges, cur_edges, window)
    if response < 0.05:
        return None
    return float(np.hypot(dx, dy) * 2.0)  # back to full-resolution pixels


DRIFT_WARN_PX = 8.0


def load_calibration(path: str, frame_size: Tuple[int, int],
                     board_cm: Tuple[float, float]) -> Optional[np.ndarray]:
    """Return saved corners if the file matches this resolution and board size."""
    try:
        with open(path) as f:
            data = json.load(f)
        if (tuple(data["frame_size"]) == tuple(frame_size)
                and np.allclose(data["board_cm"], board_cm)):
            return np.array(data["corners_px"], np.float32)
    except (OSError, ValueError, KeyError):
        pass
    return None


def save_height_calibration(path: str, frame_size: Tuple[int, int],
                            object_height_cm: float,
                            reference_cm: Tuple[float, float],
                            image_points_px: np.ndarray,
                            board_points_cm: np.ndarray) -> None:
    data = {
        "frame_size": list(frame_size),
        "object_height_cm": float(object_height_cm),
        "reference_cm": list(reference_cm),
        "placements": 4,
        "image_points_px": np.asarray(image_points_px).tolist(),
        "board_points_cm": np.asarray(board_points_cm).tolist(),
    }
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


def load_height_calibration(path: str, frame_size: Tuple[int, int],
                            object_height_cm: float,
                            reference_cm: Tuple[float, float]
                            ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Load four-placement points only when camera, height, and size match."""
    try:
        with open(path) as f:
            data = json.load(f)
        if (tuple(data["frame_size"]) == tuple(frame_size)
                and np.isclose(data["object_height_cm"], object_height_cm)
                and np.allclose(data["reference_cm"], reference_cm)):
            image_points = np.asarray(data["image_points_px"], np.float32)
            board_points = np.asarray(data["board_points_cm"], np.float32)
            if image_points.shape == (16, 2) and board_points.shape == (16, 2):
                return image_points, board_points
    except (OSError, ValueError, KeyError):
        pass
    return None


def build_homography(corners_px: np.ndarray, board_cm: Tuple[float, float]):
    """Homography from image pixels to board centimetres (and its inverse).
    Corner 1 is the board origin, corner 1->2 is the x axis (board length)."""
    L, W = board_cm
    dst = np.array([[0, 0], [L, 0], [L, W], [0, W]], np.float32)
    H = cv2.getPerspectiveTransform(np.asarray(corners_px, np.float32), dst)
    return H, np.linalg.inv(H)


def align_height_calibration(image_points_px: np.ndarray, board_points_cm: np.ndarray,
                             corners_px: np.ndarray, board_cm: Tuple[float, float]):
    """Express the height-plane points in the corner calibration's frame.

    The corner calibration defines the board frame: the first clicked corner
    is (0, 0). A height calibration made with a different corner as
    "Corner 1" (or before the corners were re-clicked in another order) is
    rotated or mirrored relative to it, which sends the arm to the opposite
    side of the board. The board-plane homography already sees where each
    reference square really was (to within the 4 cm parallax), so pick the
    relabelling that agrees with it.

    Returns (board_points_cm in the corner frame, what was changed, mean
    disagreement in cm).
    """
    L, W = board_cm
    H, _ = build_homography(corners_px, board_cm)
    seen = transform_points(H, image_points_px)
    pts = np.asarray(board_points_cm, np.float64)
    candidates = {
        "unchanged": pts,
        "rotated 180 degrees": np.column_stack([L - pts[:, 0], W - pts[:, 1]]),
        "mirrored along X": np.column_stack([L - pts[:, 0], pts[:, 1]]),
        "mirrored along Y": np.column_stack([pts[:, 0], W - pts[:, 1]]),
    }
    errors = {name: float(np.mean(np.linalg.norm(seen - c, axis=1)))
              for name, c in candidates.items()}
    best = min(errors, key=errors.get)
    return candidates[best].astype(np.float32), best, errors[best]


def fit_height_plane_homography(image_points_px: np.ndarray,
                                board_points_cm: np.ndarray):
    """Fit an image-to-board homography from four square placements."""
    H, inliers = cv2.findHomography(
        np.asarray(image_points_px, np.float32),
        np.asarray(board_points_cm, np.float32),
        method=cv2.RANSAC,
        ransacReprojThreshold=1.0,
    )
    if H is None or inliers is None or int(inliers.sum()) < 8:
        raise ValueError("Could not fit the 4 cm plane from the four square placements")
    return H, np.linalg.inv(H)


def transform_points(H: np.ndarray, pts) -> np.ndarray:
    pts = np.asarray(pts, np.float32).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, H).reshape(-1, 2)


def describe_calibration(corners: np.ndarray,
                         board_cm: Tuple[float, float]) -> Tuple[str, str]:
    """Report the cm-per-pixel along each board side and flag inconsistencies.
    Returns (full console text, short line for the side panel)."""
    L, W = board_cm
    side = lambda a, b: float(np.linalg.norm(corners[a] - corners[b]))
    s_long = L / ((side(0, 1) + side(3, 2)) / 2)
    s_short = W / ((side(1, 2) + side(0, 3)) / 2)
    diff = abs(s_long - s_short) / ((s_long + s_short) / 2) * 100
    text = (f"Calibrated: {s_long:.4f} cm/px along the {L:g} cm side, "
            f"{s_short:.4f} cm/px along the {W:g} cm side")
    if diff > 5:
        text += (f"\nWARNING: the two scales differ by {diff:.1f}%. Re-click the "
                 f"corners (press c) or check the board dimensions.")
    return text, f"Scale: {s_long:.4f} / {s_short:.4f} cm/px"


def pick_corners(frame: np.ndarray, board_cm: Tuple[float, float]) -> Optional[np.ndarray]:
    """Show a still frame; the user clicks the 4 board corners in order.
    Returns a 4x2 float32 array of pixel coordinates, or None if cancelled."""
    L, W = board_cm
    prompts = [
        f"Click corner 1 (start of a {L:g} cm side)",
        f"Click corner 2 (end of that {L:g} cm side)",
        f"Click corner 3 (end of the {W:g} cm side)",
        "Click corner 4 (the last corner)",
    ]
    fh, fw = frame.shape[:2]
    points: List[Tuple[int, int]] = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4 and x < fw and y < fh:
            points.append((x, y))

    cv2.setMouseCallback(WINDOW_NAME, on_mouse)
    message = ""
    result = None
    while True:
        canvas = frame.copy()
        for i, p in enumerate(points, start=1):
            cv2.circle(canvas, p, 5, (0, 0, 255), -1)
            cv2.putText(canvas, str(i), (p[0] + 8, p[1] - 8), FONT, 0.7,
                        (0, 0, 255), 2, cv2.LINE_AA)
        if len(points) > 1:
            cv2.polylines(canvas, [np.array(points, np.int32).reshape(-1, 1, 2)],
                          len(points) == 4, (0, 255, 255), 1)

        lines = ["CALIBRATION", prompts[len(points)] if len(points) < 4
                 else "Enter = accept", "u = undo   r = reset   Esc = cancel", ""]
        if message:
            lines.append(message)
        show_canvas(add_info_panel(canvas, lines))

        key = cv2.waitKey(20) & 0xFF
        if key == 27:
            break
        elif key == ord('u') and points:
            points.pop()
            message = ""
        elif key == ord('r'):
            points.clear()
            message = ""
        elif key in (13, 10, 32) and len(points) == 4:
            pts = np.array(points, np.float32)
            if cv2.isContourConvex(pts.reshape(-1, 1, 2)):
                result = pts
                break
            message = "Corners are not in order. Press r and click around the board."

    cv2.setMouseCallback(WINDOW_NAME, lambda *args: None)
    return result


def pick_height_reference_corners(
        frame: np.ndarray, reference_cm: Tuple[float, float],
        object_height_cm: float,
        square_center_cm: Tuple[float, float]) -> Optional[np.ndarray]:
    """Click the corners of the known reference's top face."""
    L, W = reference_cm
    cx, cy = square_center_cm
    hx, hy = L / 2.0, W / 2.0
    prompts = [
        f"Click board corner ({cx-hx:g}, {cy-hy:g}) cm",
        f"Click board corner ({cx+hx:g}, {cy-hy:g}) cm",
        f"Click board corner ({cx+hx:g}, {cy+hy:g}) cm",
        f"Click board corner ({cx-hx:g}, {cy+hy:g}) cm",
    ]
    fh, fw = frame.shape[:2]
    points: List[Tuple[int, int]] = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4 and x < fw and y < fh:
            points.append((x, y))

    cv2.setMouseCallback(WINDOW_NAME, on_mouse)
    result = None
    message = "Keep square edges aligned with board X/Y; click in the shown order."
    while True:
        canvas = frame.copy()
        for i, point in enumerate(points, start=1):
            cv2.circle(canvas, point, 5, (0, 0, 255), -1)
            cv2.putText(canvas, str(i), (point[0] + 8, point[1] - 8), FONT,
                        0.7, (0, 0, 255), 2)
        if len(points) > 1:
            cv2.polylines(canvas, [np.array(points, np.int32).reshape(-1, 1, 2)],
                          len(points) == 4, (0, 255, 255), 1)
        lines = [f"{object_height_cm:g} cm HEIGHT REFERENCE",
                 prompts[len(points)] if len(points) < 4 else "Enter = accept",
                 "u = undo   r = reset   Esc = cancel", message]
        show_canvas(add_info_panel(canvas, lines))

        key = cv2.waitKey(20) & 0xFF
        if key == 27:
            break
        if key == ord('u') and points:
            points.pop()
        elif key == ord('r'):
            points.clear()
        elif key in (13, 10, 32) and len(points) == 4:
            pts = np.array(points, np.float32)
            if cv2.isContourConvex(pts.reshape(-1, 1, 2)):
                result = pts
                break
            message = "Corners must go around the top face in order; press r to retry."

    cv2.setMouseCallback(WINDOW_NAME, lambda *args: None)
    return result


def capture_height_reference_frame(cap: cv2.VideoCapture, center_cm, reference_cm,
                                   placement_name: str, placement_index: int, total: int,
                                   object_height_cm: float,
                                   board_to_px: Optional[np.ndarray] = None
                                   ) -> Optional[np.ndarray]:
    """Show live video until the user confirms a reference-square placement.

    With board_to_px, the target square and the (0, 0) corner are drawn so it
    is clear which physical corner each placement means.
    """
    cx, cy = center_cm
    hx, hy = reference_cm[0] / 2.0, reference_cm[1] / 2.0
    guide = origin = None
    if board_to_px is not None:
        square = [(cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)]
        guide = transform_points(board_to_px, square).astype(np.int32).reshape(-1, 1, 2)
        origin = tuple(int(v) for v in transform_points(board_to_px, [(0.0, 0.0)])[0])
    prompt_lines = [
        f"Place square snug at board {placement_name} (yellow outline)",
        "Corner 1 is the corner you clicked first: (0, 0).",
        f"Center: ({cx:g}, {cy:g}) cm; edges flush with board edges",
        f"Top face height = {object_height_cm:g} cm",
        "Press Enter/Space when still; Esc cancels.",
    ]
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            continue
        view = frame.copy()
        if guide is not None:
            cv2.polylines(view, [guide], True, (0, 255, 255), 2)
            cv2.circle(view, origin, 7, (0, 0, 255), -1)
            cv2.putText(view, "(0,0)", (origin[0] + 10, origin[1] + 20), FONT, 0.6,
                        (0, 0, 255), 2, cv2.LINE_AA)
        show_canvas(add_info_panel(
            view, [f"HEIGHT REFERENCE {placement_index}/{total}"] + prompt_lines
        ))
        key = cv2.waitKey(20) & 0xFF
        if key == 27:
            return None
        if key in (10, 13, 32):
            return frame.copy()


# ---- measurement helpers ---------------------------------------------------

# General inset of the detection area; tape-coloured masks use --roi-inset.
WORKSPACE_INSET_PX = 3


def make_workspace_mask(frame_shape, corners: np.ndarray, inset_px: int) -> np.ndarray:
    """Filled board polygon, shrunk by inset_px so tape on the calibrated
    edge (and anything leaning in from outside) is never segmented."""
    mask = np.zeros(frame_shape[:2], dtype=np.uint8)
    cv2.fillConvexPoly(mask, corners.astype(np.int32), 255)
    if inset_px > 0:
        size = 2 * inset_px + 1
        mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size)))
    return mask


def find_candidates(detector: "ObjectDetector", frame: np.ndarray,
                    workspace_mask: np.ndarray, corners: np.ndarray,
                    H_measure: np.ndarray, edge_margin: float,
                    min_object_cm: float, max_object_cm: float) -> List[Dict]:
    """One frame's detections whose centre is well inside the board and whose
    real-world size is plausible. Each carries its cm measurement."""
    quad = corners.reshape(-1, 1, 2).astype(np.float32)
    candidates = []
    for o in detector.detect_objects(frame, workspace_mask):
        if cv2.pointPolygonTest(quad, (float(o['centroid'][0]),
                                       float(o['centroid'][1])), True) < edge_margin:
            detector.rejected['near edge'] = detector.rejected.get('near edge', 0) + 1
            continue
        m = measure_object_cm(o['contour'], H_measure)
        if not (min_object_cm <= m['width_cm'] and m['length_cm'] <= max_object_cm):
            detector.rejected['size'] = detector.rejected.get('size', 0) + 1
            continue
        o['measure'] = m
        candidates.append(o)
    return candidates


def measure_object_cm(contour, H: np.ndarray) -> Dict:
    """Map the contour onto the board plane (cm) and fit the minimum-area
    rotated rectangle there. Length = long side, width = short side."""
    pts_cm = transform_points(H, contour)
    rect = cv2.minAreaRect(pts_cm)
    w, h = rect[1]
    return {'length_cm': max(w, h), 'width_cm': min(w, h), 'rect_cm': rect}


# ---- side panel UI -----------------------------------------------------------

PANEL_BG = (28, 26, 24)
TEXT_COLOR = (235, 235, 235)
DIM_COLOR = (150, 150, 150)
HEADER_COLOR = (90, 200, 255)
OK_COLOR = (110, 220, 110)
WARN_COLOR = (60, 200, 255)
ERR_COLOR = (90, 90, 255)
SELECT_BG = (90, 60, 30)
FONT_SCALE = 0.5
PAD = 12
# Panel-relative button boxes: (x0, y0, x1, y1). Shared by drawing and clicks.
BUTTON_TOP, BUTTON_BOTTOM = 10, 54
BUTTONS = {
    "stop": ((PAD, BUTTON_TOP, PANEL_WIDTH // 2 - 5, BUTTON_BOTTOM),
             (40, 40, 200), "KILL / STOP  [s]"),
    "home": ((PANEL_WIDTH // 2 + 5, BUTTON_TOP, PANEL_WIDTH - PAD, BUTTON_BOTTOM),
             (50, 120, 50), "HOME  [h]"),
}
SWATCH_BGR = {
    'red': (40, 40, 220), 'orange': (0, 140, 255), 'yellow': (0, 220, 240),
    'lime': (0, 255, 170), 'green': (60, 180, 60), 'teal': (150, 160, 0),
    'cyan': (230, 220, 0), 'blue': (220, 90, 30), 'indigo': (130, 0, 75),
    'purple': (160, 40, 140), 'magenta': (200, 0, 200), 'pink': (180, 105, 255),
    'black': (40, 40, 40), 'white': (235, 235, 235), 'gray': (128, 128, 128),
    'dark red': (30, 30, 140), 'dark yellow': (20, 120, 140), 'dark green': (40, 100, 40),
    'dark blue': (120, 50, 20), 'dark purple': (100, 30, 90),
}


def _text_width(text: str, scale: float = FONT_SCALE) -> int:
    return cv2.getTextSize(text, FONT, scale, 1)[0][0]


def wrap_text_px(text: str, max_width: int, scale: float = FONT_SCALE) -> List[str]:
    """Wrap text by its rendered pixel width, breaking over-long words."""
    words = str(text).split()
    if not words:
        return [""]
    lines, current = [], ""
    for word in words:
        while _text_width(word, scale) > max_width:
            # Hard-break a long token (paths, tuples) at the widest prefix.
            cut = len(word)
            while cut > 1 and _text_width((current + " " if current else "") + word[:cut],
                                          scale) > max_width:
                cut -= 1
            if current and cut < 4:
                lines.append(current)
                current = ""
                continue
            lines.append((current + " " if current else "") + word[:cut])
            current, word = "", word[cut:]
        candidate = f"{current} {word}" if current else word
        if _text_width(candidate, scale) <= max_width:
            current = candidate
        else:
            lines.append(current)
            current = word
    if current:
        lines.append(current)
    return lines


def status_color(text: str):
    lowered = str(text).lower()
    if any(word in lowered for word in ("fail", "cannot", "error", "rejected", "exceeds")):
        return ERR_COLOR
    if any(word in lowered for word in ("running", "checking", "wait", "...", "requested")):
        return WARN_COLOR
    return OK_COLOR


def render_panel(frame: np.ndarray, sections: List[Dict],
                 show_buttons: bool = False) -> np.ndarray:
    """Return frame + a side panel built from prioritised sections.

    Each section is {"title": str|None, "rows": [row...], "priority": int}
    where a row is a str or a dict {text, color?, highlight?, swatch?}.
    When everything does not fit, the lowest-priority sections are dropped
    first, then the last section is cut with an ellipsis.
    """
    h = frame.shape[0]
    panel = np.full((h, PANEL_WIDTH, 3), PANEL_BG, np.uint8)
    top = BUTTON_BOTTOM + 14 if show_buttons else PAD
    usable = h - top - PAD

    def layout(section):
        lines = []
        if section.get("title"):
            lines.append(("header", section["title"], HEADER_COLOR, False, None))
        for row in section["rows"]:
            row = {"text": row} if isinstance(row, str) else row
            indent = 22 if row.get("swatch") is not None else 0
            wrapped = wrap_text_px(row["text"], PANEL_WIDTH - 2 * PAD - indent)
            for k, text in enumerate(wrapped):
                lines.append(("row", text, row.get("color", TEXT_COLOR),
                              row.get("highlight", False),
                              row.get("swatch") if k == 0 else None, indent))
        return lines

    laid_out = [(s, layout(s)) for s in sections]
    gap = 8

    def total_height(items):
        return sum(len(lines) * LINE_H + gap for _, lines in items)

    while len(laid_out) > 1 and total_height(laid_out) > usable:
        drop = min(laid_out, key=lambda item: item[0].get("priority", 0))
        laid_out.remove(drop)

    y = top
    flat = []
    for _, lines in laid_out:
        flat.extend(lines)
        flat.append(None)  # gap between sections
    for line in flat:
        if line is None:
            y += gap
            continue
        if y + LINE_H > h - PAD:
            cv2.putText(panel, "...", (PAD, y + 15), FONT, FONT_SCALE, DIM_COLOR, 1,
                        cv2.LINE_AA)
            break
        if line[0] == "header":
            cv2.putText(panel, line[1].upper(), (PAD, y + 16), FONT, 0.45,
                        line[2], 1, cv2.LINE_AA)
            cv2.line(panel, (PAD, y + LINE_H - 2), (PANEL_WIDTH - PAD, y + LINE_H - 2),
                     (70, 70, 70), 1)
        else:
            _, text, color, highlight, swatch, indent = line
            if highlight:
                cv2.rectangle(panel, (PAD - 6, y + 1), (PANEL_WIDTH - PAD + 6, y + LINE_H - 1),
                              SELECT_BG, -1)
            if swatch is not None:
                cv2.rectangle(panel, (PAD, y + 5), (PAD + 13, y + 18), swatch, -1)
                cv2.rectangle(panel, (PAD, y + 5), (PAD + 13, y + 18), (200, 200, 200), 1)
            cv2.putText(panel, text, (PAD + indent, y + 16), FONT, FONT_SCALE, color, 1,
                        cv2.LINE_AA)
        y += LINE_H

    if show_buttons:
        for (x0, y0, x1, y1), color, label in BUTTONS.values():
            cv2.rectangle(panel, (x0, y0), (x1, y1), color, -1)
            cv2.rectangle(panel, (x0, y0), (x1, y1), (210, 210, 210), 1)
            tw = _text_width(label, 0.52)
            cv2.putText(panel, label, ((x0 + x1 - tw) // 2, (y0 + y1) // 2 + 6), FONT, 0.52,
                        (255, 255, 255), 1, cv2.LINE_AA)
    return np.hstack([frame, panel])


def panel_button_at(x: int, y: int, image_width: int) -> Optional[str]:
    """Name of the side-panel button under canvas point (x, y), if any."""
    px = x - image_width
    for name, ((x0, y0, x1, y1), _, _) in BUTTONS.items():
        if x0 <= px <= x1 and y0 <= y <= y1:
            return name
    return None


def add_info_panel(frame: np.ndarray, lines: List[str], top_margin: int = 0) -> np.ndarray:
    """Simple panel for calibration screens: first line is the title."""
    lines = [str(line) for line in lines]
    rows = [{"text": line,
             "color": ERR_COLOR if status_color(line) == ERR_COLOR else TEXT_COLOR}
            for line in lines[1:] if line]
    return render_panel(frame, [{"title": lines[0] if lines else None,
                                 "rows": rows, "priority": 1}])


_window_fit_for = None


def show_canvas(canvas: np.ndarray) -> None:
    """imshow, scaling the window to fit the screen whenever the canvas size
    changes. Mouse callbacks still receive canvas (unscaled) coordinates."""
    global _window_fit_for
    size = (canvas.shape[1], canvas.shape[0])
    if size != _window_fit_for:
        _window_fit_for = size
        screen_w, screen_h = 1600, 900
        try:
            import ctypes
            user32 = ctypes.windll.user32
            screen_w, screen_h = user32.GetSystemMetrics(0), user32.GetSystemMetrics(1)
        except (AttributeError, OSError):
            pass
        # Leave room for the title bar and taskbar.
        scale = min(1.0, 0.96 * screen_w / size[0], 0.86 * screen_h / size[1])
        cv2.resizeWindow(WINDOW_NAME, int(size[0] * scale), int(size[1] * scale))
        cv2.moveWindow(WINDOW_NAME, 8, 8)
    cv2.imshow(WINDOW_NAME, canvas)


def load_ik_mapping(path: str) -> Dict:
    """Fit a board-to-IK-world affine plane transform from saved point pairs."""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    points_path = data.get("robot_points_file", "sim2hw/cv_robot_points.json")
    if not os.path.isabs(points_path) and not os.path.exists(points_path):
        points_path = os.path.join(REPO_ROOT, points_path)
    with open(points_path, encoding="utf-8") as f:
        point_data = json.load(f)
    points = point_data.get("points", {})
    if isinstance(points, dict):
        points = list(points.values())
    if not isinstance(points, list) or len(points) < 3:
        raise ValueError("record at least three board points in the IK server first")
    board_xy, robot_xyz = [], []
    for point in points:
        xy = point.get("board_xy_cm")
        xyz = point.get("robot_xyz_m")
        if (isinstance(xy, list) and len(xy) == 2 and isinstance(xyz, list) and len(xyz) == 3
                and all(np.isfinite(float(v)) for v in xy + xyz)):
            board_xy.append([float(v) for v in xy])
            robot_xyz.append([float(v) for v in xyz])
    if len(board_xy) < 3:
        raise ValueError("saved CV points need at least three valid coordinate pairs")
    design = np.column_stack([np.asarray(board_xy, np.float64), np.ones(len(board_xy))])
    if np.linalg.matrix_rank(design) < 3:
        raise ValueError("saved CV board points must not all lie on one line")
    coefficients, _, _, _ = np.linalg.lstsq(design, np.asarray(robot_xyz), rcond=None)
    residuals = design @ coefficients - np.asarray(robot_xyz)
    rms_error = float(np.sqrt(np.mean(np.sum(residuals ** 2, axis=1))))
    contact_offset = float(data.get("tool_contact_offset_m", 0.0))
    if not np.isfinite(contact_offset):
        raise ValueError("tool_contact_offset_m must be finite")
    return {"board_to_robot": coefficients,
            "point_count": len(board_xy), "point_rms_error_m": rms_error,
            "tool_contact_offset_m": contact_offset}


def base_board_to_robot_xyz(board_xy_cm, tool_height_cm: float, mapping: Dict) -> np.ndarray:
    """Board cm -> IK metres from the recorded calibration points only."""
    x, y = (float(value) for value in board_xy_cm)
    xyz = np.array([x, y, 1.0]) @ mapping["board_to_robot"]
    xyz[2] += tool_height_cm / 100.0 + mapping["tool_contact_offset_m"]
    return xyz


# Taught corrections blend within roughly this distance of where they were
# taught; far from every correction their weighted mean applies.
# 1.5 cm keeps each taught spot within ~1 mm of its own correction even when
# spots are a few cm apart, while still blending smoothly between them.
CORRECTION_RADIUS_CM = 1.5

# ---- learned board -> robot model ------------------------------------------
#
# With 3+ taught spots the targets come from a model shaped like the real
# error sources instead of patching the recorded corner map:
#   * the board is a rigid, true-size rectangle somewhere in the robot frame
#     (its position/rotation is learned, so a moved board is relearned), and
#   * the arm has small pan-gain and reach (offset/scale/curvature) errors
#     around its base, which is how joint calibration errors show up.
# On the taught spots this predicted held-out spots ~2-5x better than a
# linear map. A local layer then makes each taught spot replay exactly.
LEARN_MIN_SPOTS = 3
# Consistency is judged with a stiff model: the board as a rigid, true-size
# rectangle (3 parameters). The flexible arm model could bend to join spots
# from two different setups. A spot further than this from the rigid fit was
# taught before the board, camera or arm moved (sessions here differed by
# ~5 cm) and is ignored; the arm's own distortion stays well inside it.
LEARN_INLIER_TOL_M = 0.025
# Weak prior keeping the arm-error terms small; tuned by leave-one-out.
LEARN_PRIOR = 0.03
# Weak prior pulling the board's rotation toward the one implied by the
# recorded corner points, so a few spots along one line cannot swing it.
LEARN_POSE_PRIOR = 0.05
# Spots where the object overhung the board edge: its visible centre is
# biased, so they count less.
OVERHANG_WEIGHT = 0.3
# Spots taught in the same sitting as the newest one describe the current
# setup; earlier sessions (separated by a gap longer than this) count less
# when deciding which spots belong together.
SESSION_GAP_S = 3600.0
OLDER_SESSION_WEIGHT = 0.25


def _session_factors(records: List[Dict], order: List[int]) -> np.ndarray:
    """1 for spots in the newest teaching session, less for older ones."""
    from datetime import datetime
    def when(r):
        try:
            return datetime.strptime(r.get("saved_at", ""), "%Y-%m-%dT%H:%M:%S%z").timestamp()
        except ValueError:
            return None
    times = [when(records[i]) for i in order]   # newest first
    factors, factor, previous = [], 1.0, None
    for t in times:
        if t is not None and previous is not None and previous - t > SESSION_GAP_S:
            factor = OLDER_SESSION_WEIGHT
        factors.append(factor)
        if t is not None:
            previous = t
    return np.array(factors)


def _rot2(theta: float, mirror: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, s], [-s, c]]) @ np.diag([1.0, mirror])


def _arm_model_xy(params, board_m: np.ndarray) -> np.ndarray:
    theta, ox, oy, pan_gain, reach0, reach1, reach2, mirror = params
    real = board_m @ _rot2(theta, mirror) + np.array([ox, oy])
    radius = np.hypot(real[:, 0], real[:, 1])
    angle = np.arctan2(real[:, 1], real[:, 0]) * (1.0 + pan_gain)
    radius = radius + reach0 + reach1 * radius + reach2 * radius * radius
    return np.column_stack([radius * np.cos(angle), radius * np.sin(angle)])


def _fit_arm_model(board_m: np.ndarray, target_xy: np.ndarray, weights: np.ndarray,
                   pose_prior=None):
    """Weighted Gauss-Newton fit of the rigid-board + arm-error model.

    pose_prior = (theta, mirror) from the recorded corner map, or None.
    """
    sw = np.sqrt(weights)
    cb = (weights[:, None] * board_m).sum(0) / weights.sum()
    ct = (weights[:, None] * target_xy).sum(0) / weights.sum()
    U, _, Vt = np.linalg.svd(((board_m - cb) * weights[:, None]).T @ (target_xy - ct))
    mirror = float(np.sign(np.linalg.det(U @ Vt))) or 1.0
    rot = U @ np.diag([1.0, mirror]) @ Vt
    theta = float(np.arctan2(rot[0, 1], rot[0, 0]))
    theta_ref = None
    if pose_prior is not None and pose_prior[1] == mirror:
        theta_ref = pose_prior[0]
    origin = ct - cb @ _rot2(theta, mirror)
    params = np.array([theta, origin[0], origin[1], 0.0, 0.0, 0.0, 0.0])

    def residuals(p):
        full = np.append(p, mirror)
        res = ((_arm_model_xy(full, board_m) - target_xy) * sw[:, None]).ravel()
        prior = LEARN_PRIOR * np.array([p[3], p[4] * 10.0, p[5], p[6] * 3.0])
        if theta_ref is not None:
            turn = (p[0] - theta_ref + np.pi) % (2 * np.pi) - np.pi
            prior = np.append(prior, LEARN_POSE_PRIOR * turn)
        return np.concatenate([res, prior])

    for _ in range(40):
        r0 = residuals(params)
        J = np.empty((len(r0), len(params)))
        for k in range(len(params)):
            step = np.zeros_like(params)
            step[k] = 1e-6
            J[:, k] = (residuals(params + step) - r0) / 1e-6
        delta = np.linalg.lstsq(J, -r0, rcond=None)[0]
        params = params + delta
        if np.linalg.norm(delta) < 1e-9:
            break
    return np.append(params, mirror)


def _fit_z_plane(board_m: np.ndarray, z: np.ndarray, weights: np.ndarray) -> np.ndarray:
    X = np.column_stack([board_m, np.ones(len(board_m))]) * np.sqrt(weights)[:, None]
    coef, *_ = np.linalg.lstsq(X, z * np.sqrt(weights), rcond=None)
    return coef


def _spot_weight(record: Dict, board_cm) -> float:
    if record.get("overhang"):
        return OVERHANG_WEIGHT
    if board_cm is not None and "overhang" not in record:
        # Older records: a centre within 1.5 cm of (or past) an edge means
        # the object hung over the tape.
        x, y = record["board_xy_cm"]
        L, W = board_cm
        if min(x, y, L - x, W - y) < 1.5:
            return OVERHANG_WEIGHT
    return 1.0


def corner_map_pose(mapping: Dict):
    """(theta, mirror) of the recorded corner map, as a weak learning prior."""
    A = np.asarray(mapping["board_to_robot"])[:2, :2]
    U, _, Vt = np.linalg.svd(A)
    mirror = float(np.sign(np.linalg.det(U @ Vt))) or 1.0
    rot = U @ np.diag([1.0, mirror]) @ Vt
    return float(np.arctan2(rot[0, 1], rot[0, 0])), mirror


def learn_board_to_robot(records: List[Dict], board_cm=None, pose_prior=None) -> Optional[Dict]:
    """Fit the learned model to taught spots of one gripper side.

    Finds the largest set of spots consistent with the NEWEST one (a tiny
    RANSAC over triples containing it). The newest spot describes the current
    setup; spots from before the board, camera or arm moved disagree with it
    and are ignored, however many there are. Returns None if fewer than
    LEARN_MIN_SPOTS spots agree.
    """
    if len(records) < LEARN_MIN_SPOTS:
        return None
    order = sorted(range(len(records)), key=lambda i: records[i].get("saved_at", ""),
                   reverse=True)
    board = np.array([records[i]["board_xy_cm"] for i in order], float) / 100.0
    target = np.array([records[i]["robot_xyz_m"] for i in order], float)
    # Normalise heights so spots taught at different grasp heights combine.
    z0 = target[:, 2] - np.array([records[i]["tool_height_cm"] for i in order]) / 100.0
    weights = np.array([_spot_weight(records[i], board_cm) for i in order])
    session = _session_factors(records, order)
    n = len(order)

    def consensus(subset):
        """Residual of every spot under a rigid board fitted to subset."""
        w = weights[subset]
        cb = (w[:, None] * board[subset]).sum(0) / w.sum()
        ct = (w[:, None] * target[subset, :2]).sum(0) / w.sum()
        U, _, Vt = np.linalg.svd(((board[subset] - cb) * w[:, None]).T @ (target[subset, :2] - ct))
        mirror = float(np.sign(np.linalg.det(U @ Vt))) or 1.0
        rot = U @ np.diag([1.0, mirror]) @ Vt
        predicted = (board - cb) @ rot + ct
        return None, np.linalg.norm(predicted - target[:, :2], axis=1)

    best = None
    recent = list(range(min(n, 10)))
    for a in recent[:1]:          # the newest spot is always in the set
        for b in recent:
            for c in recent:
                if not a < b < c:
                    continue
                tri = board[[a, b, c]] * 100.0
                u, v = tri[1] - tri[0], tri[2] - tri[0]
                area = abs(u[0] * v[1] - u[1] * v[0]) / 2.0
                if area < 10.0:     # cm^2: nearly collinear spots fix nothing
                    continue
                _, err = consensus([a, b, c])
                inliers = np.where(err < LEARN_INLIER_TOL_M)[0]
                if 0 not in inliers:
                    continue
                # Most consistent weight (current session counting most),
                # then the more recent set.
                score = (round(float((weights * session)[inliers].sum()), 3),
                         -int(inliers.sum()))
                if best is None or score > best[0]:
                    best = (score, inliers)
    if best is None or len(best[1]) < LEARN_MIN_SPOTS:
        return None
    inliers = best[1]
    for _ in range(3):   # refit on the consensus set until it is stable
        _, err = consensus(list(inliers))
        refined = np.where(err < LEARN_INLIER_TOL_M)[0]
        if (len(refined) < LEARN_MIN_SPOTS or 0 not in refined
                or set(refined) == set(inliers)):
            break
        inliers = refined
    # Only now fit the flexible arm model, to the consistent spots.
    params = _fit_arm_model(board[inliers], target[inliers, :2], weights[inliers], pose_prior)
    zcoef = _fit_z_plane(board[inliers], z0[inliers], weights[inliers])

    # Typical prediction error at a new spot (leave-one-out over inliers).
    loo = []
    if len(inliers) >= 4:
        for k in inliers:
            rest = [i for i in inliers if i != k]
            p = _fit_arm_model(board[rest], target[rest, :2], weights[rest], pose_prior)
            loo.append(float(np.linalg.norm(_arm_model_xy(p, board[k:k + 1])[0]
                                            - target[k, :2])))
    model = {
        "params": params,
        "zcoef": zcoef,
        "inliers": [order[i] for i in inliers],
        "ignored": [order[i] for i in range(n) if i not in set(inliers)],
        "loo_mm": float(np.median(loo) * 1000.0) if loo else None,
        "loo_each_mm": [v * 1000.0 for v in loo],
        "spots_cm": board[inliers] * 100.0,
    }
    # Local layer: what the smooth model still misses at each taught spot.
    predicted = np.column_stack([_arm_model_xy(params, board[inliers]),
                                 np.column_stack([board[inliers], np.ones(len(inliers))]) @ zcoef])
    model["residuals"] = np.column_stack([target[inliers, :2], z0[inliers]]) - predicted
    return model


def _blend_loo_mm(corrections) -> Optional[float]:
    """Median leave-one-out XY error of the corner map + blended corrections."""
    if len(corrections) < 4:
        return None
    where = np.array([c[0] for c in corrections])
    deltas = np.array([c[1] for c in corrections])
    errs = []
    for k in range(len(corrections)):
        rest = np.arange(len(corrections)) != k
        w = 1.0 / (np.sum((where[rest] - where[k]) ** 2, axis=1) + CORRECTION_RADIUS_CM ** 2)
        guess = (w[:, None] * deltas[rest]).sum(0) / w.sum()
        errs.append(float(np.linalg.norm(guess[:2] - deltas[k][:2])))
    return float(np.median(errs) * 1000.0)


def learned_xyz(model: Dict, board_xy_cm, tool_height_cm: float) -> np.ndarray:
    b = np.asarray(board_xy_cm, float).reshape(1, 2)
    xy = _arm_model_xy(model["params"], b / 100.0)[0]
    z = float(np.array([b[0, 0] / 100.0, b[0, 1] / 100.0, 1.0]) @ model["zcoef"])
    xyz = np.array([xy[0], xy[1], z + tool_height_cm / 100.0])
    d2 = np.sum((model["spots_cm"] - b) ** 2, axis=1)
    w = 1.0 / (d2 + CORRECTION_RADIUS_CM ** 2)
    # Fade the local layer out away from taught spots so it never adds a
    # far-away spot's quirk to the smooth model.
    fade = float(np.max(CORRECTION_RADIUS_CM ** 2 * w))
    return xyz + fade * (w[:, None] * model["residuals"]).sum(0) / w.sum()


def learned_model_for(mapping: Dict) -> Optional[Dict]:
    return (mapping.get("learned") or {}).get(mapping.get("jaw_side"))


# The IK server refuses targets closer than 8 cm to the base axis (its
# MIN_TARGET_RADIUS); closer than that the approach cannot be planned. Its
# model predicts gripper/shoulder contact below ~12 cm, but the real arm
# reaches there, so that is only a preflight warning.
ARM_MIN_RADIUS_M = 0.085


def suggest_teach_spot(spots_cm, board_cm, margin_cm: float = 3.0, predict=None):
    """Reachable board point farthest from every taught spot (teach next).

    predict(board_xy_cm) -> robot xyz excludes points the arm cannot reach.
    """
    L, W = board_cm
    xs, ys = np.meshgrid(np.arange(margin_cm, L - margin_cm + 1e-6, 1.0),
                         np.arange(margin_cm, W - margin_cm + 1e-6, 1.0))
    grid = np.column_stack([xs.ravel(), ys.ravel()])
    if predict is not None:
        reach = np.array([np.hypot(*predict(b)[:2]) for b in grid])
        if np.any(reach >= ARM_MIN_RADIUS_M):
            grid = grid[reach >= ARM_MIN_RADIUS_M]
    if spots_cm is None or len(spots_cm) == 0:
        return tuple(grid[len(grid) // 2]), float("inf")
    gaps = np.min(np.linalg.norm(grid[:, None, :] - np.asarray(spots_cm)[None, :, :], axis=2), axis=1)
    k = int(np.argmax(gaps))
    return tuple(grid[k]), float(gaps[k])


def correction_at(board_xy_cm, mapping: Dict) -> np.ndarray:
    """Inverse-distance blend of taught pick corrections at a board point."""
    corrections = mapping.get("corrections") or []
    if not corrections:
        return np.zeros(3)
    # A correction taught with the motor jaw on one side is wrong for the
    # other side (the grip point moves), so only use matching ones.
    side = mapping.get("jaw_side")
    active = (mapping.get("active_corrections") or {}).get(side)
    if active is not None:
        corrections = active
    else:
        corrections = [c for c in corrections if c[2] is None or c[2] == side]
    if not corrections:
        return np.zeros(3)
    where = np.array([c[0] for c in corrections])
    deltas = np.array([c[1] for c in corrections])
    d2 = np.sum((where - np.asarray(board_xy_cm, float)) ** 2, axis=1)
    weights = 1.0 / (d2 + CORRECTION_RADIUS_CM ** 2)
    return (weights[:, None] * deltas).sum(axis=0) / weights.sum()


def board_to_robot_xyz(board_xy_cm, tool_height_cm: float, mapping: Dict):
    """Map board-local cm to IK coordinates at a tool height above the board.

    With 3+ consistent taught spots for the current gripper side, the learned
    model gives the target. Otherwise the recorded corner points give a
    linear base map, shifted by the blended taught corrections.
    """
    model = learned_model_for(mapping)
    if model is not None:
        xyz = learned_xyz(model, board_xy_cm, tool_height_cm)
    else:
        xyz = base_board_to_robot_xyz(board_xy_cm, tool_height_cm, mapping)
        xyz = xyz + correction_at(board_xy_cm, mapping)
    # "Nudge everything" offset (m cycles to ALL): shifts every pick and drop.
    xyz = xyz + global_offset(mapping)
    return tuple(float(value) for value in xyz)


def global_offset(mapping: Dict) -> np.ndarray:
    return np.asarray(mapping.get("global_offset", np.zeros(3)), float)


def load_pick_corrections(path: str, mapping: Dict) -> None:
    """Attach taught corrections from path to mapping (missing file = none)."""
    records = []
    mapping["global_offset"] = np.zeros(3)
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        records = data.get("corrections", [])
        offset = [float(v) for v in data.get("global_offset_m", [0.0, 0.0, 0.0])][:3]
        if len(offset) == 3 and all(np.isfinite(offset)):
            mapping["global_offset"] = np.array(offset)
    except FileNotFoundError:
        pass
    except (OSError, ValueError, AttributeError) as exc:
        print(f"Ignoring unreadable IK corrections file {path!r}: {exc}", file=sys.stderr)
    valid, corrections = [], []
    for record in records:
        try:
            board_xy = [float(v) for v in record["board_xy_cm"]][:2]
            robot_xyz = np.array([float(v) for v in record["robot_xyz_m"]][:3])
            height = float(record["tool_height_cm"])
        except (KeyError, TypeError, ValueError):
            continue
        if len(board_xy) != 2 or robot_xyz.shape != (3,):
            continue
        # Stored as absolute taught targets, so they stay valid if the base
        # calibration points are re-recorded.
        delta = robot_xyz - base_board_to_robot_xyz(board_xy, height, mapping)
        valid.append(record)
        corrections.append((np.array(board_xy), delta, record.get("jaw_side")))
    mapping["correction_records"] = valid
    mapping["corrections"] = corrections
    # One learned model per gripper side (records without a side count for both).
    learned, active, stale = {}, {}, {}
    mapping["unused_models"] = {}
    for side in ("base", "away", "auto"):
        idx = [i for i, r in enumerate(valid) if r.get("jaw_side") in (None, side)]
        side_records = [valid[i] for i in idx]
        model = learn_board_to_robot(side_records, mapping.get("board_cm"),
                                     corner_map_pose(mapping))
        if model is not None:
            model["records"] = side_records
            stale[side] = [side_records[i] for i in model["ignored"]]
            consistent = [corrections[idx[i]] for i in model["inliers"]]
            # Use the learned model only if it predicts held-out spots better
            # than blending the same consistent corrections over the corner map.
            blend = _blend_loo_mm(consistent)
            model["blend_loo_mm"] = blend
            if model["loo_mm"] is None or blend is None or model["loo_mm"] <= blend:
                learned[side] = model
            else:
                active[side] = consistent
                mapping.setdefault("unused_models", {})[side] = model
        elif side_records:
            # Too few spots for the model: blend only the corrections that
            # agree with the newest one (within 2 cm), never a moved setup's.
            newest = max(range(len(idx)), key=lambda k: side_records[k].get("saved_at", ""))
            ref = corrections[idx[newest]][1]
            keep = [k for k in range(len(idx))
                    if np.linalg.norm(corrections[idx[k]][1][:2] - ref[:2]) < 0.02]
            active[side] = [corrections[idx[k]] for k in keep]
            stale[side] = [side_records[k] for k in range(len(idx)) if k not in keep]
    mapping["learned"] = learned
    mapping["active_corrections"] = active
    mapping["stale_records"] = stale
    teach = {}
    if mapping.get("board_cm"):
        for side in ("base", "away", "auto"):
            model = learned.get(side) or mapping["unused_models"].get(side)
            if model is not None:
                spots = model["spots_cm"]
            else:
                spots = np.array([c[0] for c in active.get(side, [])]).reshape(-1, 2)
            previous = mapping.get("jaw_side")
            mapping["jaw_side"] = side   # predict with this side's model

            def predict(b):
                return board_to_robot_xyz(b, 1.0, mapping)
            teach[side] = suggest_teach_spot(spots, mapping["board_cm"], predict=predict)
            mapping["jaw_side"] = previous
    mapping["teach_next"] = teach


def learning_summary(mapping: Dict) -> Tuple[str, bool]:
    """One-line state of the learned pick model for the current side."""
    side = mapping.get("jaw_side")
    model = learned_model_for(mapping)
    ignored = len((mapping.get("stale_records") or {}).get(side) or [])
    tail = f"; {ignored} old spot{'s' if ignored != 1 else ''} ignored (Shift+G deletes)" if ignored else ""
    if model is not None:
        accuracy = (f", typical miss ~{model['loo_mm']:.0f} mm" if model["loo_mm"] is not None
                    else ", add a 4th spot to estimate accuracy")
        return f"Learned arm model from {len(model['inliers'])} spots{accuracy}{tail}", True
    active = (mapping.get("active_corrections") or {}).get(side)
    if active:
        unused = (mapping.get("unused_models") or {}).get(side)
        how = (f" (blend beat the arm model: ~{unused['blend_loo_mm']:.0f} vs "
               f"{unused['loo_mm']:.0f} mm)" if unused and unused.get("blend_loo_mm") else
               f"; teach {max(0, LEARN_MIN_SPOTS - len(active))} more to learn the arm model"
               if len(active) < LEARN_MIN_SPOTS else "")
        return f"Using {len(active)} consistent corrections{how}{tail}", True
    return "No taught spots for this gripper side yet (nudge, then g)", False


def _write_corrections(path: str, records: List[Dict], offset=None) -> None:
    data = {"units": {"board_xy": "cm", "robot_xyz": "m"}, "corrections": records}
    if offset is not None and np.any(np.asarray(offset) != 0.0):
        data["global_offset_m"] = [float(v) for v in offset]
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)


def set_global_offset(path: str, mapping: Dict, offset) -> None:
    """Save the "nudge everything" offset; it applies to every target at once."""
    mapping["global_offset"] = np.asarray(offset, float).copy()
    _write_corrections(path, mapping.get("correction_records", []), mapping["global_offset"])


def save_pick_correction(path: str, mapping: Dict, kind: str, board_xy_cm,
                         robot_xyz_m, tool_height_cm: float, overhang: bool = False) -> None:
    """Store a taught target; re-teaching within 1.5 cm replaces the old one."""
    board_xy = [float(v) for v in board_xy_cm]
    side = mapping.get("jaw_side")
    records = [r for r in mapping.get("correction_records", [])
               if r.get("jaw_side") != side
               or np.hypot(r["board_xy_cm"][0] - board_xy[0],
                           r["board_xy_cm"][1] - board_xy[1]) > 1.5]
    # Stored without the "nudge everything" offset, which is added back to
    # every target; otherwise changing that offset later would count twice.
    robot_xyz_m = np.asarray(robot_xyz_m, float) - global_offset(mapping)
    records.append({
        "kind": kind,
        "jaw_side": side,
        "board_xy_cm": board_xy,
        "robot_xyz_m": [float(v) for v in robot_xyz_m],
        "tool_height_cm": float(tool_height_cm),
        "overhang": bool(overhang),
        "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    })
    _write_corrections(path, records, global_offset(mapping))
    load_pick_corrections(path, mapping)


def delete_ignored_corrections(path: str, mapping: Dict) -> int:
    """Remove the spots ignored as inconsistent for the current side."""
    ignored = (mapping.get("stale_records") or {}).get(mapping.get("jaw_side")) or []
    if not ignored:
        return 0
    drop = {id(r) for r in ignored}
    records = [r for r in mapping.get("correction_records", []) if id(r) not in drop]
    removed = len(mapping.get("correction_records", [])) - len(records)
    _write_corrections(path, records, global_offset(mapping))
    load_pick_corrections(path, mapping)
    return removed


def preview_ik_target(server_url: str, xyz, timeout: float = 60.0,
                      orientation: str = "downwards", duration: float = 5.0,
                      jaw_side: str = "base") -> Dict:
    body = json.dumps({"x": xyz[0], "y": xyz[1], "z": xyz[2],
                       "orientation": orientation, "duration": duration,
                       "fixed_jaw_side": jaw_side}).encode("utf-8")
    request = urllib.request.Request(server_url.rstrip("/") + "/coordinate_preview",
                                     data=body, headers={"Content-Type": "application/json"},
                                     method="POST")
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        # urllib's default message only says "HTTP Error 400" and hides the
        # IK server's useful reach-check / validation reason in the body.
        detail = exc.read().decode("utf-8", errors="replace").strip()
        raise RuntimeError(
            f"IK preview rejected target ({xyz[0]:.4f}, {xyz[1]:.4f}, "
            f"{xyz[2]:.4f}) m, orientation={orientation}: "
            f"{detail or exc.reason}"
        ) from exc


def preview_pick_and_drop(server_url: str, pick_xyz, drop_xyz,
                          duration: float = 5.0, jaw_side: str = "base") -> Dict:
    """Preflight both pick and adjusted drop targets with the tool pointing down."""
    return {
        "pick": preview_ik_target(server_url, pick_xyz, orientation="downwards",
                                   duration=duration, jaw_side=jaw_side),
        "drop": preview_ik_target(server_url, drop_xyz, orientation="downwards",
                                   duration=duration, jaw_side=jaw_side),
    }


def corner2_drop_xy(board_cm, object_length_cm: float, object_width_cm: float):
    """Choose a drop center just inside Corner 2, accounting for object size.

    Corner 2 is (board X max, board Y min). Inset by the object's half-diagonal
    plus clearance so the full object stays on the board regardless of yaw.
    """
    board_x, board_y = board_cm
    inset = float(np.hypot(object_length_cm / 2.0, object_width_cm / 2.0) + 0.5)
    if inset * 2.0 >= min(board_x, board_y):
        raise ValueError("Object is too large to place safely inside the board corner")
    return float(board_x - inset), float(inset)


def _ik_post(server_url: str, path: str, body: Dict, timeout: float = 60.0) -> Dict:
    payload = json.dumps(body).encode("utf-8")
    request = urllib.request.Request(server_url.rstrip("/") + path, data=payload,
                                     headers={"Content-Type": "application/json"},
                                     method="POST")
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"IK {path} rejected request: {detail or exc.reason}") from exc


def _ik_get(server_url: str, path: str, timeout: float = 5.0) -> Dict:
    with urllib.request.urlopen(server_url.rstrip("/") + path, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def stop_arm(server_url: str) -> str:
    """Stop motion and wait until the server has finished cancelling its pipeline."""
    result = _ik_post(server_url, "/stop", {}, timeout=5.0)
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        state = _ik_get(server_url, "/state")
        if not state.get("pipeline_running"):
            return result.get("msg", "Arm stopped")
        time.sleep(0.1)
    raise TimeoutError("Timed out waiting for the arm pipeline to stop")


def home_arm(server_url: str, timeout: float = 20.0) -> str:
    """Stop an active pipeline before starting the saved home-pose motion."""
    state = _ik_get(server_url, "/state")
    if state.get("pipeline_running"):
        _ik_post(server_url, "/stop", {}, timeout=5.0)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = _ik_get(server_url, "/state")
            log = state.get("pipeline_log", [])
            stopped = any("Pipeline failed: Stopped by kill switch" in item for item in log)
            completed = any("Pipeline complete" in item for item in log)
            if not state.get("pipeline_running") and (stopped or completed):
                break
            time.sleep(0.1)
        else:
            raise TimeoutError("Timed out waiting for the active motion to stop before homing")
    _ik_post(server_url, "/home", {}, timeout=5.0)
    deadline = time.monotonic() + timeout + 10.0
    while time.monotonic() < deadline:
        state = _ik_get(server_url, "/state")
        if not state.get("pipeline_running"):
            failure = next((line for line in state.get("pipeline_log", [])
                            if "Pipeline failed" in line), None)
            if failure:
                raise RuntimeError(failure)
            if any("Pipeline complete" in line for line in state.get("pipeline_log", [])):
                return "Home pose reached."
        time.sleep(0.15)
    raise TimeoutError("Timed out waiting for the arm to reach home")


def _wait_for_pipeline(server_url: str, timeout: float = 150.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with urllib.request.urlopen(server_url.rstrip("/") + "/state", timeout=10.0) as response:
            state = json.loads(response.read().decode("utf-8"))
        if not state.get("pipeline_running", False):
            log = state.get("pipeline_log", [])
            if any("STOP requested" in line or
                   "Pipeline failed: Stopped by kill switch" in line for line in log):
                raise RuntimeError("Stopped by kill switch; pick/place sequence cancelled")
            failure = next((line for line in log if "Pipeline failed" in line), None)
            if failure:
                raise RuntimeError(failure)
            return
        time.sleep(0.5)
    raise TimeoutError("Timed out waiting for the arm motion pipeline")


def execute_pick_and_place(server_url: str, pick_xyz, place_xyz,
                           duration: float = 5.0, jaw_side: str = "base") -> str:
    """Pick one object, then place it at the size-adjusted Corner 2 target."""
    # Check both locations before gripping so an unreachable destination cannot
    # leave the arm holding an object without a valid drop path.
    preview_ik_target(server_url, pick_xyz, orientation="downwards", duration=duration,
                      jaw_side=jaw_side)
    preview_ik_target(server_url, place_xyz, orientation="downwards", duration=duration,
                      jaw_side=jaw_side)
    pick = {"x": pick_xyz[0], "y": pick_xyz[1], "z": pick_xyz[2],
            "orientation": "downwards", "fixed_jaw_side": jaw_side,
            "duration": duration,
            "pre_gripper": "open", "post_gripper": "close", "retreat": True}
    result = _ik_post(server_url, "/coordinate_action", pick)
    if not result.get("ok", False):
        raise RuntimeError(result.get("error", "Pick command was rejected"))
    _wait_for_pipeline(server_url)

    place = {"x": place_xyz[0], "y": place_xyz[1], "z": place_xyz[2],
             "orientation": "downwards", "fixed_jaw_side": jaw_side,
             "duration": duration,
             "pre_gripper": "hold", "post_gripper": "open", "retreat": True}
    result = _ik_post(server_url, "/coordinate_action", place)
    if not result.get("ok", False):
        raise RuntimeError(result.get("error", "Place command was rejected"))
    _wait_for_pipeline(server_url)
    return "Pick and place completed at Corner 2."


def lock_camera(cap: cv2.VideoCapture) -> None:
    """Try to disable autofocus and auto white balance so colors stay stable.
    Not every driver honors this; auto-exposure is backend-specific, so it is
    left alone here."""
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)


def set_manual_exposure(cap: cv2.VideoCapture, exposure: float) -> None:
    """Switch off auto-exposure and set a fixed exposure. Overexposed pale
    objects lose their colour entirely, which no software step can recover."""
    # DirectShow uses 0.25 for manual mode; other backends use 1 (or 0).
    for manual_value in (0.25, 1.0, 0.0):
        if cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, manual_value):
            break
    if not cap.set(cv2.CAP_PROP_EXPOSURE, exposure):
        print(f"Camera driver ignored exposure {exposure:g}.", file=sys.stderr)
    else:
        print(f"Exposure set to {cap.get(cv2.CAP_PROP_EXPOSURE):g} (auto-exposure off).")


def main() -> int:
    parser = argparse.ArgumentParser(description="Live color/shape detection window")
    parser.add_argument("--camera", type=int, default=1, help="camera index (default: 1)")
    parser.add_argument("--backend", choices=("dshow", "msmf", "any"), default="dshow",
                        help="capture backend (default: dshow; use msmf or any if needed)")
    parser.add_argument("--width", type=int, default=1280, help="capture width")
    parser.add_argument("--height", type=int, default=720, help="capture height")
    parser.add_argument("--lock", action="store_true",
                        help="try to disable autofocus and auto white balance")
    parser.add_argument("--board-length", type=float, default=21.3,
                        help="board X side of the calibrated workspace in cm (default 21.3)")
    parser.add_argument("--board-width", type=float, default=28.3,
                        help="board Y side of the calibrated workspace in cm (default 28.3)")
    parser.add_argument("--object-height", type=float, default=4.0,
                        help="object height above the board in cm (default: 4.0)")
    parser.add_argument("--grasp-height", type=float, default=1.0,
                        help="gripper target height above the board in cm (default: 1.0; lower-side grasp)")
    parser.add_argument("--move-duration", type=float, default=5.0,
                        help="seconds per arm motion (default: 5; lower is faster)")
    parser.add_argument("--feedback-step-xy-mm", type=float, default=5.0,
                        help="drop-target feedback step for X/Y in mm (default: 5)")
    parser.add_argument("--feedback-step-z-mm", type=float, default=2.0,
                        help="drop-target feedback step for Z in mm (default: 2)")
    parser.add_argument("--feedback-fine-step-mm", type=float, default=1.0,
                        help="nudge step with Shift held (L/J/I/K/U/O), in mm (default: 1)")
    parser.add_argument("--height-reference-length", type=float, default=6.0,
                        help="reference square side length in cm (default: 6)")
    parser.add_argument("--height-reference-width", type=float, default=6.0,
                        help="reference square side width in cm (default: 6)")
    parser.add_argument("--height-calib-file", default=DEFAULT_HEIGHT_CALIB_FILE,
                        help="file for the object-height plane calibration")
    parser.add_argument("--ik-preview", action="store_true",
                        help="enable preview-only targets using the IK server (press 1-9, then p)")
    parser.add_argument("--ik-pick-place", action="store_true",
                        help="enable hardware pick/place; select an object and press a to run")
    parser.add_argument("--ik-corrections", default=DEFAULT_IK_CORRECTIONS_FILE,
                        help="taught pick/drop corrections (press g to add one)")
    parser.add_argument("--clear-ik-corrections", action="store_true",
                        help="start without the saved taught corrections")
    parser.add_argument("--learning-mode", choices=("on", "off"), default="on",
                        help="start with learning mode on (g teaches spots, teach-here marker) "
                             "or off; toggle live with t. Learned spots are used either way.")
    parser.add_argument("--gripper-motor-side", choices=("base", "away", "auto"),
                        default="base",
                        help="which side of the object the gripper's motor (fixed) jaw "
                             "goes: toward the robot base, away from it, or auto "
                             "(may flip between nearby targets). Toggle live with w.")
    parser.add_argument("--ik-mapping", default=DEFAULT_IK_MAPPING_FILE,
                        help="JSON board-to-robot transform used by --ik-preview")
    parser.add_argument("--ik-url", default="http://127.0.0.1:7842",
                        help="IK server base URL")
    parser.add_argument("--calib-file", default=DEFAULT_CALIB_FILE,
                        help="where the corner calibration is saved/loaded")
    parser.add_argument("--recalibrate", action="store_true",
                        help="ignore any saved calibration and click the corners again")
    parser.add_argument("--recalibrate-height", action="store_true",
                        help="redo the known reference top-face calibration")
    parser.add_argument("--edge-margin", type=float, default=30.0,
                        help="ignore detections whose centre is closer than this many "
                             "pixels to the board edge")
    parser.add_argument("--roi-inset", type=int, default=10,
                        help="keep red/orange/pink detections this many pixels inside the "
                             "calibrated corners so the red border tape is never "
                             "segmented; other colours may touch the tape (default: 10)")
    parser.add_argument("--min-object-cm", type=float, default=1.0,
                        help="ignore objects narrower than this on the board (default: 1)")
    parser.add_argument("--max-object-cm", type=float, default=15.0,
                        help="ignore objects longer than this on the board, e.g. the "
                             "workspace border or a hand (default: 15)")
    parser.add_argument("--white-objects", action="store_true",
                        help="also detect white/pale objects brighter than the board "
                             "(off by default: bright patches and glare on the board "
                             "look the same)")
    parser.add_argument("--no-white-objects", action="store_true",
                        help=argparse.SUPPRESS)  # old flag; white is now off by default
    parser.add_argument("--exposure", type=float, default=None,
                        help="manual camera exposure (DirectShow: about -4 bright .. -9 dark). "
                             "Lower it if pale objects look pure white")
    parser.add_argument("--confirm-frames", type=int, default=4,
                        help="frames an object must persist before it is listed "
                             "(default: 4; raise to suppress more flicker)")
    args = parser.parse_args()
    for option in ("calib_file", "height_calib_file", "ik_mapping", "ik_corrections"):
        value = getattr(args, option)
        if value == parser.get_default(option) and not os.path.isabs(value):
            setattr(args, option, os.path.join(REPO_ROOT, value))
    if args.roi_inset < 0 or args.confirm_frames < 1:
        parser.error("--roi-inset must be >= 0 and --confirm-frames >= 1")
    if not (0 <= args.min_object_cm < args.max_object_cm):
        parser.error("--min-object-cm must be >= 0 and below --max-object-cm")

    if not np.isfinite(args.object_height) or args.object_height < 0:
        parser.error("--object-height must be a finite, non-negative value in cm")
    if (not np.isfinite(args.grasp_height) or args.grasp_height < 0
            or args.grasp_height > args.object_height):
        parser.error("--grasp-height must be finite and between 0 and --object-height")
    if not np.isfinite(args.move_duration) or not (0.2 <= args.move_duration <= 10.0):
        parser.error("--move-duration must be between 0.2 and 10 seconds")
    if (not np.isfinite(args.feedback_step_xy_mm) or args.feedback_step_xy_mm <= 0
            or not np.isfinite(args.feedback_step_z_mm) or args.feedback_step_z_mm <= 0
            or not np.isfinite(args.feedback_fine_step_mm) or args.feedback_fine_step_mm <= 0):
        parser.error("feedback step sizes must be finite and greater than zero")
    if (not np.isfinite(args.height_reference_length)
            or not np.isfinite(args.height_reference_width)
            or args.height_reference_length <= 0 or args.height_reference_width <= 0):
        parser.error("height reference dimensions must be finite and greater than zero")
    ik_mapping = None
    if args.ik_preview or args.ik_pick_place:
        try:
            ik_mapping = load_ik_mapping(args.ik_mapping)
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            parser.error(f"cannot load IK mapping {args.ik_mapping!r}: {exc}")
        print(f"Loaded {ik_mapping['point_count']} CV-to-IK points; "
              f"fit RMS error {ik_mapping['point_rms_error_m'] * 1000:.1f} mm")
        if args.clear_ik_corrections:
            ik_mapping["corrections"], ik_mapping["correction_records"] = [], []
        else:
            ik_mapping["board_cm"] = (args.board_length, args.board_width)
            load_pick_corrections(args.ik_corrections, ik_mapping)
        print(f"Loaded {len(ik_mapping['corrections'])} taught pick/drop corrections "
              f"from {args.ik_corrections}")
        ik_mapping["jaw_side"] = args.gripper_motor_side

    # DirectShow is the backend used by cv_testing.py for this external camera.
    # OpenCV's default backend on Windows (often MSMF) can open the device but
    # return black frames, or take a long time to become ready.
    backends = {
        "dshow": [cv2.CAP_DSHOW],
        "msmf": [cv2.CAP_MSMF],
        "any": [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY],
    }[args.backend]
    cap = None
    for backend in backends:
        candidate = cv2.VideoCapture(args.camera, backend)
        if candidate.isOpened():
            cap = candidate
            break
        candidate.release()

    if cap is None:
        print(f"Could not open camera {args.camera} with backend {args.backend}. "
              "Try --backend any or a different --camera index.", file=sys.stderr)
        return 1

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if args.lock:
        lock_camera(cap)
    if args.exposure is not None:
        set_manual_exposure(cap, args.exposure)

    board_cm = (args.board_length, args.board_width)
    detector = ObjectDetector()
    detector.detect_white = args.white_objects and not args.no_white_objects
    detector.tape_inset_px = args.roi_inset
    # Resizable window: show_canvas() scales it to fit the screen, so the side
    # panel is never cut off on smaller or high-DPI-scaled displays.
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)

    try:
        # Discard startup frames while auto-exposure settles, but keep checking
        # each read instead of assuming the final one succeeded.
        frame = None
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            ok, candidate = cap.read()
            if ok and candidate is not None and candidate.size:
                frame = candidate
                # A tiny mean usually indicates a black frame from a backend
                # that opened the camera without delivering its video stream.
                if float(np.mean(frame)) > 2.0:
                    break
            time.sleep(0.03)
        if frame is None or float(np.mean(frame)) <= 2.0:
            print("Camera opened, but no usable image arrived (frames are black "
                  "or unavailable). Try --backend msmf or a different --camera "
                  "index, and check "
                  "that another app is not using the camera.", file=sys.stderr)
            return 1
        frame_size = (frame.shape[1], frame.shape[0])

        corners = None if args.recalibrate else load_calibration(
            args.calib_file, frame_size, board_cm)
        if corners is None:
            corners = pick_corners(frame, board_cm)
            if corners is None:
                print("Calibration cancelled.", file=sys.stderr)
                return 1
            save_calibration(args.calib_file, frame_size, board_cm, corners)
            cv2.imwrite(calibration_reference_path(args.calib_file), frame)

        def sync_height_calibration_frame(image_points, board_points, corners):
            """Keep the height plane in the corner frame (first click = 0,0)."""
            board_points, change, err_cm = align_height_calibration(
                image_points, board_points, corners, board_cm)
            if change != "unchanged":
                print(f"NOTE: the {args.object_height:g} cm height calibration was {change} "
                      "relative to your corner calibration (first clicked corner = (0, 0)). "
                      "Relabelled it to match and saved.", file=sys.stderr)
                save_height_calibration(args.height_calib_file, frame_size, args.object_height,
                                        (args.height_reference_length,
                                         args.height_reference_width),
                                        image_points, board_points)
            if err_cm > 4.0:
                print(f"WARNING: the height calibration disagrees with the corners by "
                      f"{err_cm:.1f} cm on average; redo it with --recalibrate-height.",
                      file=sys.stderr)
            return board_points

        # A saved board image lets us notice when the camera or board has
        # moved since the corners were clicked (e.g. on a new day).
        calibration_reference = cv2.imread(calibration_reference_path(args.calib_file))
        drift_px = calibration_drift_px(calibration_reference, frame, corners)
        if drift_px is not None and drift_px > DRIFT_WARN_PX:
            print(f"WARNING: the board looks shifted ~{drift_px:.0f} px since calibration. "
                  "Press c to recalibrate.", file=sys.stderr)
        last_drift_check = time.monotonic()

        H, H_inv = build_homography(corners, board_cm)
        # The board homography is valid at z=0. For objects with a nonzero,
        # shared height, use a second homography calibrated from a known
        # 6x6 cm reference square at four known board positions at that height.
        H_measure, H_measure_inv = H, H_inv
        if args.object_height > 0:
            reference_cm = (args.height_reference_length,
                            args.height_reference_width)
            height_calibration = None if args.recalibrate_height else load_height_calibration(
                args.height_calib_file, frame_size, args.object_height, reference_cm)
            if height_calibration is None:
                print(f"Calibrate the {args.object_height:g} cm plane with the "
                      f"{reference_cm[0]:g} x {reference_cm[1]:g} cm square at four positions.")
                hx, hy = reference_cm[0] / 2.0, reference_cm[1] / 2.0
                placement_centers = [
                    (hx, hy),
                    (args.board_length - hx, hy),
                    (args.board_length - hx, args.board_width - hy),
                    (hx, args.board_width - hy),
                ]
                placement_names = ["Corner 1", "Corner 2", "Corner 3", "Corner 4"]
                image_points = []
                board_points = []
                for placement_index, center_cm in enumerate(placement_centers, start=1):
                    placement_frame = capture_height_reference_frame(
                        cap, center_cm, reference_cm, placement_names[placement_index - 1], placement_index,
                        len(placement_centers), args.object_height, board_to_px=H_inv)
                    if placement_frame is None:
                        print("Height calibration cancelled.", file=sys.stderr)
                        return 1
                    square_corners = pick_height_reference_corners(
                        placement_frame, reference_cm, args.object_height, center_cm)
                    if square_corners is None:
                        print("Height calibration cancelled.", file=sys.stderr)
                        return 1
                    image_points.extend(square_corners.tolist())
                    cx, cy = center_cm
                    board_points.extend([
                        [cx - hx, cy - hy], [cx + hx, cy - hy],
                        [cx + hx, cy + hy], [cx - hx, cy + hy],
                    ])
                image_points = np.asarray(image_points, np.float32)
                board_points = np.asarray(board_points, np.float32)
                save_height_calibration(args.height_calib_file, frame_size,
                                        args.object_height, reference_cm,
                                        image_points, board_points)
            else:
                image_points, board_points = height_calibration
            board_points = sync_height_calibration_frame(image_points, board_points, corners)
            try:
                H_measure, H_measure_inv = fit_height_plane_homography(
                    image_points, board_points)
            except ValueError as exc:
                print(f"Height calibration failed: {exc}", file=sys.stderr)
                return 1

        calib_text, scale_line = describe_calibration(corners, board_cm)
        print(calib_text)

        fps = 0.0
        last = time.time()
        # Selection follows the tracked object, not its list position, so it
        # stays on the same object when others appear or disappear.
        selected_id = None
        tracker = ObjectTracker(confirm_frames=args.confirm_frames)
        ik_status = ""
        preview_executor = ThreadPoolExecutor(max_workers=1)
        preview_future = None
        preview_task = None
        control_executor = ThreadPoolExecutor(max_workers=2)
        control_futures = []
        latest_control_id = 0
        pending_control = {"action": None}

        def on_detector_mouse(event, x, y, flags, param):
            if event != cv2.EVENT_LBUTTONUP:
                return
            # Buttons are in the right-hand side panel, aligned with the top.
            button = panel_button_at(x, y, frame.shape[1])
            if button is not None:
                pending_control["action"] = button

        if args.ik_pick_place:
            cv2.setMouseCallback(WINDOW_NAME, on_detector_mouse)
        # Session feedback can tune the next pick or drop target in robot XYZ.
        pick_feedback_m = np.zeros(3, dtype=np.float64)
        drop_feedback_m = np.zeros(3, dtype=np.float64)
        nudge_target = "pick"
        jaw_side = args.gripper_motor_side
        learning = args.learning_mode == "on"
        # Board points used by the last preview/cycle, for saving corrections.
        last_targets = {"pick": None, "drop": None}
        workspace_mask = make_workspace_mask(frame.shape, corners, WORKSPACE_INSET_PX)

        def selected_obj():
            return next((o for o in objects if o['track_id'] == selected_id), None)

        objects = []
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read a frame from the camera.", file=sys.stderr)
                break

            # Keep the camera/UI responsive while the IK server performs its
            # comparatively expensive reachability solve.
            if preview_future is not None and preview_future.done():
                try:
                    result = preview_future.result()
                    if preview_task == "pick_place":
                        ik_status = result
                    elif preview_task == "preview_pair":
                        ik_status = "Pick and Corner 2 drop reachable (downwards)."
                        notes = [f"{name}: {item['label']}"
                                 for name in ("pick", "drop")
                                 for item in result.get(name, {}).get("preflight", [])
                                 if item.get("label", "").startswith(("WARNING", "Gripper side FLIPPED"))]
                        if notes:
                            ik_status += " " + " | ".join(notes)
                    else:
                        errors = [f"{item.get('label', 'target')}: {item.get('err_mm', '?')} mm"
                                  for item in result.get('preflight', [])]
                        ik_status = ("Reachable preview: " + "; ".join(errors))[:100]
                    print(ik_status)
                except Exception as exc:
                    task_label = "Pick/place" if preview_task == "pick_place" else "IK preview"
                    error_detail = str(exc)
                    # Keep the full IK reason in the terminal. The status pane
                    # is narrow, and truncating this at 70 chars hid the
                    # reach-check details needed to diagnose rejected targets.
                    ik_status = f"{task_label} failed: {error_detail}"
                    print(ik_status, file=sys.stderr)
                preview_future = None

            remaining_controls = []
            for control_id, action, future in control_futures:
                if future.done():
                    try:
                        message = future.result()
                        print(f"Arm {action}: {message}")
                        if control_id == latest_control_id:
                            ik_status = message
                    except Exception as exc:
                        print(f"Arm {action} failed: {exc}", file=sys.stderr)
                        if control_id == latest_control_id:
                            ik_status = f"{action.upper()} failed: {exc}"
                else:
                    remaining_controls.append((control_id, action, future))
            control_futures = remaining_controls

            # Keep only objects whose centre is well inside the board, with a
            # plausible real-world size, then require them to persist across
            # frames before they are listed or can be picked.
            if time.monotonic() - last_drift_check > 10.0:
                last_drift_check = time.monotonic()
                drift_px = calibration_drift_px(calibration_reference, frame, corners)
            candidates = find_candidates(detector, frame, workspace_mask, corners, H_measure,
                                         args.edge_margin, args.min_object_cm,
                                         args.max_object_cm)
            objects = tracker.update(candidates)
            selected_index = next((i for i, o in enumerate(objects)
                                   if o['track_id'] == selected_id), None)

            output = frame.copy()
            cv2.polylines(output, [corners.astype(np.int32).reshape(-1, 1, 2)],
                          True, (255, 255, 0), 1)

            object_rows = []
            for i, obj in enumerate(objects, start=1):
                m = obj['measure']
                # The tracker's smoothed centroid steadies the pick target.
                bx, by = transform_points(H_measure, [obj['centroid_f']])[0]
                obj['length_cm'] = m['length_cm']
                obj['width_cm'] = m['width_cm']
                obj['board_xy_cm'] = (float(bx), float(by))
                obj['height_cm'] = args.object_height
                if args.ik_preview or args.ik_pick_place:
                    obj['robot_xyz_m'] = board_to_robot_xyz((bx, by), args.grasp_height,
                                                            ik_mapping)

                is_selected = (i - 1) == selected_index
                too_close = ('robot_xyz_m' in obj and
                             np.hypot(*obj['robot_xyz_m'][:2]) < ARM_MIN_RADIUS_M)
                outline = (0, 255, 255) if is_selected else (255, 0, 255)
                if too_close:
                    outline = (0, 0, 255)
                # Draw one shape outline: an ellipse for circles, otherwise
                # the measured rectangle. Avoid stacking multiple box styles.
                if obj['shape'] == 'circle' and len(obj['contour']) >= 5:
                    cv2.ellipse(output, cv2.fitEllipse(obj['contour']), outline,
                                3 if is_selected else 2)
                else:
                    box_px = transform_points(H_measure_inv, cv2.boxPoints(m['rect_cm']))
                    cv2.polylines(output, [box_px.astype(np.int32).reshape(-1, 1, 2)],
                                  True, outline, 3 if is_selected else 2)
                cx, cy = obj['centroid']
                cv2.circle(output, (cx, cy), 5, (0, 0, 255), -1)
                x, y, _, h_px = obj['bbox']
                label = f"{i}. {obj['color']} {obj['shape']}"
                cv2.putText(output, label, (x, max(18, y - 10)), FONT, 0.6, (0, 0, 0), 4,
                            cv2.LINE_AA)
                cv2.putText(output, label, (x, max(18, y - 10)), FONT, 0.6, (255, 255, 255), 1,
                            cv2.LINE_AA)
                cv2.putText(output, f"{m['length_cm']:.1f}x{m['width_cm']:.1f}cm",
                            (x, y + h_px + 20), FONT, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

                row = {
                    "text": f"{i}. {obj['color']} {obj['shape']}  "
                            f"{m['length_cm']:.1f}x{m['width_cm']:.1f} cm  "
                            f"@({bx:.1f}, {by:.1f})",
                    "swatch": SWATCH_BGR.get(obj['color'], (128, 128, 128)),
                    "highlight": is_selected,
                }
                if too_close:
                    reach_cm = np.hypot(*obj['robot_xyz_m'][:2]) * 100.0
                    row["text"] += (f"  - too close to the arm ({reach_cm:.1f} cm < "
                                    f"{ARM_MIN_RADIUS_M * 100:.1f} cm), move it further out")
                    row["color"] = ERR_COLOR
                object_rows.append(row)
            if not object_rows:
                object_rows.append({"text": "None - place an object inside the board",
                                    "color": DIM_COLOR})

            # Smoothed FPS
            now = time.time()
            inst = 1.0 / max(now - last, 1e-6)
            fps = inst if fps == 0.0 else 0.9 * fps + 0.1 * inst
            last = now

            light = detector.lighting
            warning_rows = []
            status_rows = [
                f"Objects: {len(objects)}    FPS: {fps:.1f}",
                scale_line,
                f"Object h {args.object_height:g} cm  |  grasp {args.grasp_height:g} cm above board",
            ]
            if light:
                brightness = light['board_brightness']
                hint = "  (dim - add light)" if brightness < 70 else ""
                status_rows.append({"text": f"Board light {brightness:.0f}/255, "
                                            f"colour threshold S>={light['s_min']}{hint}",
                                    "color": WARN_COLOR if hint else DIM_COLOR})
                if light['overexposed'] > 0.05:
                    warning_rows.append({
                        "text": f"Overexposed: {light['overexposed']:.0%} of board is pure "
                                "white, pale colours are lost. Lower --exposure (e.g. -7).",
                        "color": WARN_COLOR})
            if detector.rejected:
                noise = ", ".join(f"{k} {v}" for k, v in sorted(detector.rejected.items()))
                status_rows.append({"text": f"Noise filtered: {noise}", "color": DIM_COLOR})
            if calibration_reference is None:
                status_rows.append({"text": "No calibration reference image: press c once "
                                            "to enable the moved-board check.",
                                    "color": DIM_COLOR})
            elif drift_px is not None and drift_px > DRIFT_WARN_PX:
                warning_rows.insert(0, {"text": f"Board/camera moved ~{drift_px:.0f} px since "
                                                "calibration - press c to recalibrate.",
                                        "color": ERR_COLOR})
            if ik_mapping is not None:
                status_rows.append(f"IK fit: {ik_mapping['point_count']} pts, "
                                   f"RMS {ik_mapping['point_rms_error_m'] * 1000:.1f} mm, "
                                   f"{len(ik_mapping.get('corrections', []))} taught corrections")

            sections = [
                {"title": "Objects", "rows": object_rows, "priority": 9},
                {"title": "Status", "rows": status_rows, "priority": 6},
            ]
            if warning_rows:
                sections.insert(0, {"title": "Warning", "rows": warning_rows, "priority": 10})
            if args.ik_preview or args.ik_pick_place:
                sections.insert(1 + bool(warning_rows), {"title": "Arm / IK",
                                    "rows": [{"text": ik_status or "Ready",
                                              "color": status_color(ik_status or "Ready")}],
                                    "priority": 8})
            if args.ik_pick_place:
                side_text = {"base": "toward robot base", "away": "away from robot base",
                             "auto": "auto (may flip)"}[jaw_side]
                summary, has_data = learning_summary(ik_mapping)
                correction_row = {"text": summary, "color": OK_COLOR if has_data else DIM_COLOR}
                teach_next = (ik_mapping.get("teach_next") or {}).get(jaw_side)
                if learning and teach_next is not None and teach_next[1] > 4.0:
                    (tx, ty), gap = teach_next
                    sections_teach = {"text": f"Teach next near board ({tx:.0f}, {ty:.0f}) cm "
                                              f"(yellow +): {gap:.0f} cm from any taught spot",
                                      "color": WARN_COLOR}
                    mx, my = (int(v) for v in transform_points(H_inv, [(tx, ty)])[0])
                    cv2.drawMarker(output, (mx, my), (0, 255, 255), cv2.MARKER_CROSS, 28, 2)
                    cv2.putText(output, "teach here", (mx + 12, my - 10), FONT, 0.5,
                                (0, 255, 255), 1, cv2.LINE_AA)
                else:
                    sections_teach = None
                mode_row = ({"text": "Learning mode: ON  (t turns off)  - g saves / confirms a spot",
                             "color": OK_COLOR} if learning else
                            {"text": "Learning mode: OFF  (t turns on)  - using learned spots",
                             "color": DIM_COLOR})
                sections.append({"title": f"Nudge ({nudge_target} active, m: pick/drop/all)", "rows": [
                    mode_row,
                    f"Gripper motor jaw: {side_text}  (w toggles)",
                    correction_row,
                ] + ([sections_teach] if sections_teach else []) + [
                    "Pick (mm): "
                    f"X {pick_feedback_m[0] * 1000:+.0f}  "
                    f"Y {pick_feedback_m[1] * 1000:+.0f}  "
                    f"Z {pick_feedback_m[2] * 1000:+.0f}",
                    "Drop (mm): "
                    f"X {drop_feedback_m[0] * 1000:+.0f}  "
                    f"Y {drop_feedback_m[1] * 1000:+.0f}  "
                    f"Z {drop_feedback_m[2] * 1000:+.0f}",
                    {"text": "All targets (mm, saved): "
                             f"X {global_offset(ik_mapping)[0] * 1000:+.0f}  "
                             f"Y {global_offset(ik_mapping)[1] * 1000:+.0f}  "
                             f"Z {global_offset(ik_mapping)[2] * 1000:+.0f}",
                     "color": WARN_COLOR if np.any(global_offset(ik_mapping)) else DIM_COLOR},
                ], "priority": 4})
            key_rows = []
            if args.ik_preview or args.ik_pick_place:
                key_rows.append("1-9 select   p preview" +
                                ("   a pick/place" if args.ik_pick_place else ""))
            if args.ik_pick_place:
                key_rows.append("l/j +-X   i/k +-Y   u/o +-Z   0 reset nudge")
                key_rows.append("m cycles PICK / DROP / ALL (ALL shifts every target, saved)")
                key_rows.append(f"Shift + l/j/i/k/u/o = fine {args.feedback_fine_step_mm:g} mm steps")
                key_rows.append("t learning mode on/off")
                if learning:
                    key_rows.append("g save spot (no nudge = confirm it)   Shift+G delete ignored")
                key_rows.append("w switch gripper side")
            key_rows.append("c recalibrate   q / Esc quit")
            sections.append({"title": "Keys", "rows": [{"text": r, "color": DIM_COLOR}
                                                        for r in key_rows],
                             "priority": 1})

            panel = render_panel(output, sections, show_buttons=args.ik_pick_place)
            show_canvas(panel)

            key = cv2.waitKey(1) & 0xFF
            requested_control = pending_control["action"]
            pending_control["action"] = None
            if key == ord('s'):
                requested_control = "stop"
            elif key == ord('h'):
                requested_control = "home"
            if args.ik_pick_place and requested_control in ("stop", "home"):
                if control_futures:
                    ik_status = "Wait for the current STOP/HOME command to finish."
                else:
                    latest_control_id += 1
                    if requested_control == "stop":
                        control_future = control_executor.submit(stop_arm, args.ik_url)
                        ik_status = "KILL requested; waiting for motion to stop..."
                    else:
                        control_future = control_executor.submit(home_arm, args.ik_url)
                        ik_status = "Stopping current motion, then moving to home..."
                    control_futures.append((latest_control_id, requested_control, control_future))
            if key in (ord('q'), 27):  # q or Esc
                break
            if (args.ik_preview or args.ik_pick_place) and ord('1') <= key <= ord('9'):
                index = key - ord('1')
                if index < len(objects):
                    selected_id = objects[index]['track_id']
                    ik_status = f"Selected object {index + 1}"
                else:
                    ik_status = f"No object {index + 1} visible."
            elif (args.ik_preview or args.ik_pick_place) and key == ord('p'):
                if control_futures:
                    ik_status = "Wait for STOP/HOME to finish before previewing."
                elif selected_obj() is None:
                    ik_status = "Select a visible object first (keys 1-9)."
                elif preview_future is not None:
                    ik_status = "IK preview is still running; wait for its result."
                else:
                    obj = selected_obj()
                    target_xyz = tuple(obj['robot_xyz_m'][i] + pick_feedback_m[i]
                                        for i in range(3))
                    if args.ik_pick_place:
                        try:
                            drop_xy = corner2_drop_xy(
                                board_cm, obj['length_cm'], obj['width_cm'])
                            drop_xyz = board_to_robot_xyz(
                                drop_xy, args.grasp_height, ik_mapping)
                            drop_xyz = tuple(drop_xyz[i] + drop_feedback_m[i]
                                             for i in range(3))
                            last_targets["pick"] = obj['board_xy_cm']
                            last_targets["drop"] = drop_xy
                            preview_future = preview_executor.submit(
                                preview_pick_and_drop, args.ik_url,
                                target_xyz, drop_xyz, args.move_duration, jaw_side)
                            preview_task = "preview_pair"
                            ik_status = "Checking pick and drop targets (downwards)..."
                            print(f"Checking downward pick={target_xyz} m and "
                                  f"drop={drop_xyz} m")
                        except (KeyError, ValueError) as exc:
                            ik_status = f"Cannot preview pick/place: {str(exc)[:70]}"
                    else:
                        preview_future = preview_executor.submit(
                            preview_ik_target, args.ik_url, target_xyz,
                            orientation="downwards", duration=args.move_duration,
                            jaw_side=jaw_side)
                        preview_task = "preview"
                        ik_status = "IK preview running (downwards)..."
                        print(f"Checking downward IK target {tuple(round(v, 4) for v in target_xyz)} m")
            elif args.ik_pick_place and key == ord('a'):
                if control_futures:
                    ik_status = "Wait for STOP/HOME to finish before another cycle."
                elif selected_obj() is None:
                    ik_status = "Select a visible object first (keys 1-9)."
                elif preview_future is not None:
                    ik_status = "IK task is still running; wait for its result."
                else:
                    obj = selected_obj()
                    try:
                        drop_xy = corner2_drop_xy(
                            board_cm, obj['length_cm'], obj['width_cm'])
                        drop_xyz = board_to_robot_xyz(drop_xy, args.grasp_height,
                                                      ik_mapping)
                        pick_xyz = obj['robot_xyz_m']
                        pick_xyz = tuple(pick_xyz[i] + pick_feedback_m[i]
                                         for i in range(3))
                        drop_xyz = tuple(drop_xyz[i] + drop_feedback_m[i]
                                         for i in range(3))
                        last_targets["pick"] = obj['board_xy_cm']
                        last_targets["drop"] = drop_xy
                        preview_future = preview_executor.submit(
                            execute_pick_and_place, args.ik_url, pick_xyz, drop_xyz,
                            args.move_duration, jaw_side)
                        preview_task = "pick_place"
                        ik_status = (f"Pick/place running; Corner 2 drop at "
                                     f"({drop_xy[0]:.1f}, {drop_xy[1]:.1f}) cm...")
                        print(f"PICK/PLACE: pick={tuple(round(v, 4) for v in pick_xyz)} m; "
                              f"Corner 2 drop={drop_xy} cm -> "
                              f"{tuple(round(v, 4) for v in drop_xyz)} m")
                    except (KeyError, ValueError) as exc:
                        ik_status = f"Cannot start pick/place: {str(exc)[:70]}"
            elif args.ik_pick_place and key in (ord('g'), ord('G')) and not learning:
                ik_status = "Learning mode is off; press t to turn it on before teaching."
            elif args.ik_pick_place and key == ord('g') and nudge_target == "all":
                ik_status = ("The shift for every target is saved automatically; press m for "
                             "PICK or DROP to teach a single spot.")
            elif args.ik_pick_place and key == ord('g'):
                feedback_m = pick_feedback_m if nudge_target == "pick" else drop_feedback_m
                board_xy = last_targets[nudge_target]
                if board_xy is None:
                    ik_status = (f"Run a preview (p) or cycle (a) first, then nudge the "
                                 f"{nudge_target} onto the object and press g.")
                else:
                    # A zero nudge is valid: it confirms the current target
                    # was already right, which the learner keeps as a spot.
                    taught = np.array(board_to_robot_xyz(board_xy, args.grasp_height,
                                                         ik_mapping)) + feedback_m
                    # An object hanging over the board edge has a biased
                    # visible centre; the learner gives such spots less weight.
                    overhang = False
                    current = selected_obj()
                    if (nudge_target == "pick" and current is not None
                            and np.hypot(current['board_xy_cm'][0] - board_xy[0],
                                         current['board_xy_cm'][1] - board_xy[1]) < 2.0):
                        box = cv2.boxPoints(current['measure']['rect_cm'])
                        overhang = bool(np.any(box < -0.3) or np.any(box[:, 0] > board_cm[0] + 0.3)
                                        or np.any(box[:, 1] > board_cm[1] + 0.3))
                    try:
                        save_pick_correction(args.ik_corrections, ik_mapping, nudge_target,
                                             board_xy, taught, args.grasp_height, overhang)
                        offset_mm = ", ".join(f"{'XYZ'[i]}{feedback_m[i] * 1000:+.0f}"
                                              for i in range(3))
                        summary, _ = learning_summary(ik_mapping)
                        what = ("confirmed as correct (no nudge)" if not feedback_m.any()
                                else f"spot ({offset_mm} mm)")
                        ik_status = (f"Saved {nudge_target} {what} at "
                                     f"({board_xy[0]:.1f}, {board_xy[1]:.1f}) cm"
                                     f"{' [overhangs edge: counts less]' if overhang else ''}. "
                                     f"{summary}.")
                        print(ik_status)
                        feedback_m[:] = 0.0  # now part of the mapping
                    except OSError as exc:
                        ik_status = f"Could not save correction: {exc}"
            elif args.ik_pick_place and key == ord('G'):
                try:
                    removed = delete_ignored_corrections(args.ik_corrections, ik_mapping)
                    ik_status = (f"Deleted {removed} ignored old spot{'s' if removed != 1 else ''}."
                                 if removed else "No ignored spots to delete for this side.")
                except OSError as exc:
                    ik_status = f"Could not update corrections: {exc}"
            elif args.ik_pick_place and key == ord('t'):
                learning = not learning
                ik_status = ("Learning mode ON: nudge a pick onto the object, then g saves it "
                             "(g with no nudge confirms a spot that was already right)."
                             if learning else
                             "Learning mode OFF: learned spots are still used; g/Shift+G disabled.")
            elif args.ik_pick_place and key == ord('w'):
                jaw_side = "away" if jaw_side == "base" else "base"
                ik_mapping["jaw_side"] = jaw_side
                ik_status = (f"Gripper motor jaw now {'toward' if jaw_side == 'base' else 'away from'} "
                             "the robot base; preview (p) before running.")
            elif args.ik_pick_place and key == ord('m'):
                nudge_target = {"pick": "drop", "drop": "all", "all": "pick"}[nudge_target]
                ik_status = (f"Nudge mode: {nudge_target.upper()}; "
                             + ("keys shift EVERY pick and drop target and are saved."
                                if nudge_target == "all" else
                                "feedback keys apply to the next preview or cycle."))
            elif args.ik_pick_place and key in (
                    ord('l'), ord('j'), ord('i'), ord('k'), ord('u'), ord('o'), ord('0'),
                    ord('L'), ord('J'), ord('I'), ord('K'), ord('U'), ord('O')):
                if nudge_target == "all":
                    feedback_m = global_offset(ik_mapping).copy()
                else:
                    feedback_m = pick_feedback_m if nudge_target == "pick" else drop_feedback_m
                if key == ord('0'):
                    feedback_m[:] = 0.0
                    ik_status = (f"{nudge_target.capitalize()} feedback reset; "
                                 "applies to the next preview or cycle.")
                    if nudge_target == "all":
                        set_global_offset(args.ik_corrections, ik_mapping, feedback_m)
                        ik_status = "Shift for every target reset to zero (saved)."
                else:
                    # Shift (upper case) gives fine steps for the last few mm.
                    fine = chr(key).isupper()
                    axis, sign = {
                        ord('l'): (0, 1), ord('j'): (0, -1),
                        ord('i'): (1, 1), ord('k'): (1, -1),
                        ord('u'): (2, 1), ord('o'): (2, -1),
                    }[ord(chr(key).lower())]
                    if fine:
                        step_mm = args.feedback_fine_step_mm
                    else:
                        step_mm = (args.feedback_step_z_mm if axis == 2
                                   else args.feedback_step_xy_mm)
                    feedback_m[axis] += sign * step_mm / 1000.0
                    direction = ("+" if sign > 0 else "-") + "XYZ"[axis]
                    ik_status = (f"{nudge_target.capitalize()} nudged {direction} "
                                 f"{step_mm:g} mm; applies to the next preview or cycle.")
                    if nudge_target == "all":
                        try:
                            set_global_offset(args.ik_corrections, ik_mapping, feedback_m)
                            total = ", ".join(f"{'XYZ'[i]} {feedback_m[i] * 1000:+.0f}"
                                              for i in range(3))
                            ik_status = (f"Every target shifted {direction} {step_mm:g} mm "
                                         f"(total {total} mm, saved).")
                        except OSError as exc:
                            ik_status = f"Could not save the shift: {exc}"
            if key == ord('c'):
                new_corners = pick_corners(frame, board_cm)
                if new_corners is not None:
                    corners = new_corners
                    save_calibration(args.calib_file, frame_size, board_cm, corners)
                    cv2.imwrite(calibration_reference_path(args.calib_file), frame)
                    calibration_reference = frame.copy()
                    drift_px = 0.0
                    H, H_inv = build_homography(corners, board_cm)
                    if args.object_height > 0:
                        # The new first click may be a different corner.
                        board_points = sync_height_calibration_frame(
                            image_points, board_points, corners)
                        H_measure, H_measure_inv = fit_height_plane_homography(
                            image_points, board_points)
                    else:
                        H_measure, H_measure_inv = H, H_inv
                    workspace_mask = make_workspace_mask(frame.shape, corners, WORKSPACE_INSET_PX)
                    tracker.reset()
                    selected_id = None
                    calib_text, scale_line = describe_calibration(corners, board_cm)
                    print(calib_text)
                last = time.time()
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if 'preview_executor' in locals():
            preview_executor.shutdown(wait=False)
        if 'control_executor' in locals():
            control_executor.shutdown(wait=False)

    return 0


if __name__ == "__main__":
    sys.exit(main())
