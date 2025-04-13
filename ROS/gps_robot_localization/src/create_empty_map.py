#!/usr/bin/env python3

import numpy as np
from PIL import Image
import os

def create_empty_map(width=2000, height=2000, path="empty_map.pgm"):
    """Create an empty PGM map file.
    
    Args:
        width: Width of the map in pixels
        height: Height of the map in pixels
        path: Output file path
    """
    # Create a blank map (255 = free space)
    empty_map = np.ones((height, width), dtype=np.uint8) * 255
    
    # Save as PGM
    img = Image.fromarray(empty_map)
    img.save(path)
    
    print(f"Created empty map at {path} ({width}x{height})")

if __name__ == "__main__":
    # Create a directory for maps if it doesn't exist
    maps_dir = os.path.expanduser("~/gmu-rtx/ROS/losi_launch/maps")
    os.makedirs(maps_dir, exist_ok=True)
    
    # Create the empty map
    create_empty_map(path=os.path.join(maps_dir, "empty_map.pgm"))