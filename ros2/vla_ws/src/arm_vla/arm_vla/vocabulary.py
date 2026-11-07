"""What the detector can be asked for.

Kept in its own module so the commander can resolve a label without importing
the vision node, which would drag in cv2, cv_bridge and message_filters just to
read a dictionary.

Hue in OpenCV is 0..179, not 0..359. Red straddles the wrap point, so it needs
two bands. Saturation/value floors keep grey and shadow out; raise them if the
detector latches onto the table, lower them if a dull object goes unseen.
"""

COLOR_BANDS = {
    "red":    [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (179, 255, 255))],
    "orange": [((11, 120, 70), (22, 255, 255))],
    "yellow": [((23, 100, 80), (33, 255, 255))],
    "green":  [((36, 80, 60), (86, 255, 255))],
    "cyan":   [((87, 80, 60), (99, 255, 255))],
    # Widened 2026-09-02 and made the default target. Blue is the safe choice
    # on this desk: the surface has no blue in it, so the table cannot leak
    # into the mask the way it did with red. H 95-130 covers cyan-ish through
    # violet-ish blue; S floor 100 keeps out grey and shadow.
    "blue":   [((95, 100, 50), (130, 255, 255))],
    "purple": [((129, 70, 50), (158, 255, 255))],
}

STOP_WORDS = {
    "touch", "the", "a", "an", "go", "to", "reach", "point", "at",
    "please", "move", "poke", "press", "and", "then", "over", "on",
}
