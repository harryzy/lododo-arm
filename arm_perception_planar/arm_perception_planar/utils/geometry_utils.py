#!/usr/bin/env python3
"""
Geometry utility functions for shape detection and analysis
"""

import cv2 # type: ignore
import numpy as np
from typing import Tuple, List, Dict, Optional


def calculate_aspect_ratio(bbox: Tuple[int, int, int, int]) -> float:
    """
    Calculate aspect ratio of bounding box
    
    Args:
        bbox: (x, y, w, h) bounding box
        
    Returns:
        Aspect ratio (w/h)
    """
    _, _, w, h = bbox
    if h == 0:
        return 0.0
    return float(w) / float(h)


def calculate_solidity(contour: np.ndarray) -> float:
    """
    Calculate contour solidity (convexity)
    
    Args:
        contour: OpenCV contour
        
    Returns:
        Solidity = contour_area / convex_hull_area (0-1)
    """
    area = cv2.contourArea(contour)
    if area == 0:
        return 0.0
    
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    
    if hull_area == 0:
        return 0.0
    
    return area / hull_area


def approximate_polygon(contour: np.ndarray, epsilon_factor: float = 0.04) -> np.ndarray:
    """
    Polygon approximation to simplify contour
    
    Args:
        contour: OpenCV contour
        epsilon_factor: Approximation accuracy factor
        
    Returns:
        Simplified contour vertices
    """
    perimeter = cv2.arcLength(contour, True)
    epsilon = epsilon_factor * perimeter
    approx = cv2.approxPolyDP(contour, epsilon, True)
    return approx


def is_cube_like(area: float, aspect_ratio: float, solidity: float, 
                 vertices: int, min_area: float, max_area: float,
                 min_aspect: float, max_aspect: float,
                 min_solidity: float, expected_vertices: int,
                 vertex_tolerance: int) -> bool:
    """
    Determine if contour is cube-like
    
    Args:
        area: Contour area
        aspect_ratio: Width/height ratio
        solidity: Convexity
        vertices: Number of vertices
        min_area, max_area: Area range
        min_aspect, max_aspect: Aspect ratio range
        min_solidity: Minimum solidity
        expected_vertices: Expected vertex count
        vertex_tolerance: Vertex count tolerance
        
    Returns:
        Whether it is cube-like
    """
    # Condition 1: Area range
    area_ok = min_area <= area <= max_area
    
    # Condition 2: Aspect ratio close to 1 (square)
    aspect_ok = min_aspect <= aspect_ratio <= max_aspect
    
    # Condition 3: High solidity (regular shape)
    solidity_ok = solidity >= min_solidity
    
    # Condition 4: Vertex count close to 4
    vertices_ok = abs(vertices - expected_vertices) <= vertex_tolerance
    
    return area_ok and aspect_ok and solidity_ok and vertices_ok


def calculate_cube_confidence(area: float, aspect_ratio: float, 
                              solidity: float, vertices: int,
                              min_area: float, max_area: float) -> float:
    """
    Calculate detection confidence (0-1)
    
    Based on weighted scoring of multiple features:
    - Aspect ratio deviation: closer to 1 scores higher
    - Solidity: higher scores better
    - Vertex count: exactly 4 scores highest
    - Area: within reasonable range
    
    Args:
        area: Contour area
        aspect_ratio: Width/height ratio
        solidity: Convexity
        vertices: Number of vertices
        min_area, max_area: Area range
        
    Returns:
        Confidence (0-1)
    """
    # Aspect ratio score (1.0 is perfect)
    aspect_score = 1.0 - min(abs(aspect_ratio - 1.0), 0.4) / 0.4  # More tolerant
    aspect_score = max(0.0, min(1.0, aspect_score))
    
    # Solidity score (most important for color segmentation)
    solidity_score = (solidity - 0.85) / 0.15  # Normalize to [0,1]
    solidity_score = max(0.0, min(1.0, solidity_score))
    
    # Vertex score (very important for cube detection)
    vertex_error = abs(vertices - 4)
    vertex_score = max(0.0, 1.0 - vertex_error * 0.25)
    
    # Area score (less important, just filter extremes)
    ideal_area = (min_area + max_area) / 2
    area_range = max_area - min_area
    area_deviation = abs(area - ideal_area) / area_range
    area_score = max(0.0, 1.0 - area_deviation)
    
    # Weighted average (prioritize solidity and vertices for color-based detection)
    confidence = (
        0.25 * aspect_score +      # Reduced from 0.35
        0.35 * solidity_score +    # Increased from 0.25
        0.30 * vertex_score +      # Increased from 0.25
        0.10 * area_score          # Reduced from 0.15
    )
    
    return confidence


def get_contour_center(contour: np.ndarray) -> Tuple[float, float]:
    """
    Calculate contour center point
    
    Args:
        contour: OpenCV contour
        
    Returns:
        (cx, cy) center coordinates
    """
    M = cv2.moments(contour)
    if M['m00'] == 0:
        # Use bounding box center
        x, y, w, h = cv2.boundingRect(contour)
        return (x + w/2, y + h/2)
    
    cx = M['m10'] / M['m00']
    cy = M['m01'] / M['m00']
    return (cx, cy)


def filter_contours_by_hierarchy(contours: List[np.ndarray], 
                                 hierarchy: np.ndarray) -> List[np.ndarray]:
    """
    Filter contours by hierarchy, keep only outer contours
    
    Args:
        contours: List of contours
        hierarchy: Contour hierarchy relationships
        
    Returns:
        Filtered contour list
    """
    if hierarchy is None or len(hierarchy) == 0:
        return contours
    
    # hierarchy[i][3] == -1 means no parent contour, i.e., outermost layer
    filtered = []
    for i, contour in enumerate(contours):
        if hierarchy[0][i][3] == -1:  # No parent contour
            filtered.append(contour)
    
    return filtered


def segment_color_regions(image: np.ndarray, 
                         n_colors: int = 8,
                         min_saturation: int = 30,
                         min_region_pixels: int = 500,
                         max_region_pixels: int = 50000,
                         morph_size: int = 3,
                         logger = None) -> list:
    """
    Automatically segment image into different color regions using K-means clustering
    
    Args:
        image: Input BGR image
        n_colors: Number of dominant colors to extract (K-means clusters)
        min_saturation: Minimum saturation to filter out gray/white areas
        min_region_pixels: Minimum pixels for a region to be considered valid
        max_region_pixels: Maximum pixels to avoid including entire background
        morph_size: Morphology kernel size for cleanup
        logger: Optional logger for debug output
        
    Returns:
        List of binary masks, each representing one color region
    """
    if len(image.shape) != 3:
        return []
    
    # Convert to HSV for better color clustering
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, w = hsv.shape[:2]
    
    # Reshape for K-means (pixels as samples, HSV as features)
    pixels = hsv.reshape(-1, 3).astype(np.float32)
    
    # Apply K-means clustering to find dominant colors
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.2)
    _, labels, centers = cv2.kmeans(pixels, n_colors, None, criteria, 10, cv2.KMEANS_PP_CENTERS)
    
    # Reshape labels back to image shape
    labels = labels.reshape(h, w)
    
    # Create mask for each color cluster
    color_masks = []
    for i in range(n_colors):
        # Create binary mask for this cluster
        cluster_mask = (labels == i).astype(np.uint8) * 255
        
        # Filter out low saturation areas (background/table usually has low saturation)
        saturation = hsv[:, :, 1]
        sat_mask = (saturation > min_saturation).astype(np.uint8) * 255
        cluster_mask = cv2.bitwise_and(cluster_mask, sat_mask)
        
        # Count pixels before morphology
        pixel_count = np.sum(cluster_mask > 0)
        
        # Skip if mask is too small (likely noise)
        if pixel_count < min_region_pixels:
            continue
        
        # Apply moderate morphological operations to clean up while preserving shape
        # Use smaller kernel to avoid merging cube with background
        kernel_size = min(7, morph_size + 2)  # Limit maximum kernel size
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        
        # Close to fill small holes and connect nearby pixels
        cluster_mask = cv2.morphologyEx(cluster_mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        
        # Open to remove small noise
        kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        cluster_mask = cv2.morphologyEx(cluster_mask, cv2.MORPH_OPEN, kernel_small, iterations=1)
        
        # Check size again after morphology
        pixel_count_after = np.sum(cluster_mask > 0)
        
        # Reject if region grew too much (likely merged with background)
        growth_ratio = pixel_count_after / max(pixel_count, 1)
        if growth_ratio > 3.0:  # Grew more than 3x, likely merged with background
            if logger:
                logger.debug(f'Color region {i}: REJECTED - grew too much ({pixel_count} -> {pixel_count_after}, ratio={growth_ratio:.2f})')
            continue
        
        # Still check minimum size
        if pixel_count_after < min_region_pixels:
            continue
        
        # Also reject if region is too large (likely entire background)
        if pixel_count_after > max_region_pixels:
            if logger:
                logger.debug(f'Color region {i}: REJECTED - too large ({pixel_count_after} pixels > {max_region_pixels})')
            continue
        
        if logger:
            logger.debug(f'Color region {i}: {pixel_count} -> {pixel_count_after} pixels after morphology (ratio={growth_ratio:.2f})')
        
        color_masks.append(cluster_mask)
    
    if logger:
        logger.debug(f'Segmentation: {n_colors} clusters -> {len(color_masks)} valid color regions')
    
    return color_masks


def detect_edges(image: np.ndarray, 
                blur_size: int = 5,
                canny_low: int = 50,
                canny_high: int = 150,
                morph_size: int = 3,
                use_color_seg: bool = True,
                n_colors: int = 8,
                min_saturation: int = 30,
                min_region_pixels: int = 500,
                max_region_pixels: int = 50000,
                logger = None) -> np.ndarray:
    """
    Edge detection pipeline with optional automatic color segmentation
    
    Args:
        image: Input image (BGR or grayscale)
        blur_size: Gaussian blur kernel size
        canny_low: Canny low threshold
        canny_high: Canny high threshold
        morph_size: Morphology kernel size
        use_color_seg: Use automatic color segmentation for better cube detection
        n_colors: Number of color clusters for K-means segmentation
        min_saturation: Minimum saturation to filter background (0-255)
        min_region_pixels: Minimum pixels for valid color region
        max_region_pixels: Maximum pixels to avoid entire background
        logger: Optional logger for debug output
        
    Returns:
        Edge image (combined from all color regions if use_color_seg=True)
    """
    if use_color_seg and len(image.shape) == 3:
        # Automatic color segmentation: detect all distinct color regions
        if logger:
            logger.info(f'Color segmentation: n_colors={n_colors}, min_saturation={min_saturation}, min_region_pixels={min_region_pixels}, max_region_pixels={max_region_pixels}')
        
        color_masks = segment_color_regions(image,
                                           n_colors=n_colors,
                                           min_saturation=min_saturation,
                                           min_region_pixels=min_region_pixels,
                                           max_region_pixels=max_region_pixels,
                                           morph_size=morph_size,
                                           logger=logger)

        
        # Combine all color masks
        if len(color_masks) > 0:
            if logger:
                logger.info(f'Color segmentation successful: found {len(color_masks)} valid color regions')
            combined_mask = color_masks[0].copy()
            for mask in color_masks[1:]:
                combined_mask = cv2.bitwise_or(combined_mask, mask)
            return combined_mask
        else:
            # Fallback to traditional edge detection if no color regions found
            if logger:
                logger.warning('Color segmentation failed: no valid color regions found, falling back to traditional edge detection')
            pass
    
    # Traditional edge detection (fallback or when use_color_seg=False)
    # Convert to grayscale
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image.copy()
    
    # Gaussian blur
    blurred = cv2.GaussianBlur(gray, (blur_size, blur_size), 0)
    
    # Canny edge detection
    edges = cv2.Canny(blurred, canny_low, canny_high)
    
    # Morphological closing to connect edges
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (morph_size, morph_size))
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    
    return closed



def find_cube_contours(image: np.ndarray,
                      edge_params: Dict,
                      geom_params: Dict,
                      color_seg_params: Dict = None,
                      debug: bool = False,
                      logger = None) -> Tuple[List[Dict], np.ndarray]:
    """
    Find cube contours in image
    
    Args:
        image: Input image
        edge_params: Edge detection parameter dictionary
        geom_params: Geometric feature parameter dictionary
        color_seg_params: Color segmentation parameter dictionary (optional)
        debug: Enable debug logging and visualization
        logger: Optional logger for debug output
        
    Returns:
        Tuple of (detection_list, debug_image):
        - detection_list: List of detections, each containing:
            - contour: Contour
            - bbox: Bounding box (x, y, w, h)
            - center: Center point (cx, cy)
            - area: Area
            - confidence: Confidence
        - debug_image: Color image showing segmentation process (BGR)
    """
    # Get color segmentation parameters
    if color_seg_params is None:
        color_seg_params = {}
    
    use_color_seg = edge_params.get('use_color_segmentation', True)
    n_colors = color_seg_params.get('n_colors', 8)
    min_saturation = color_seg_params.get('min_saturation', 30)
    min_region_pixels = color_seg_params.get('min_region_pixels', 500)
    max_region_pixels = color_seg_params.get('max_region_pixels', 50000)
    morph_size = edge_params.get('morph_kernel_size', 3)
    
    # Create debug image (will show color segmentation process)
    debug_image = image.copy() if debug else None
    
    # Edge detection (with automatic color segmentation if enabled)
    if use_color_seg and len(image.shape) == 3 and debug:
        # Get color masks for visualization
        color_masks = segment_color_regions(image, 
                                           n_colors=n_colors,
                                           min_saturation=min_saturation,
                                           min_region_pixels=min_region_pixels,
                                           max_region_pixels=max_region_pixels,
                                           morph_size=morph_size,
                                           logger=logger)
        
        # Draw each color region with different color on debug image
        colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
            (255, 0, 255), (0, 255, 255), (128, 128, 0), (128, 0, 128)
        ]
        
        # Create overlay to show color regions
        overlay = image.copy()
        for idx, mask in enumerate(color_masks):
            color = colors[idx % len(colors)]
            # Find contours of this color region
            region_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # Draw contours with thick line
            cv2.drawContours(overlay, region_contours, -1, color, 3)
            # Fill region with semi-transparent color
            colored = np.zeros_like(image)
            colored[mask > 0] = color
            overlay = cv2.addWeighted(overlay, 0.85, colored, 0.15, 0)
        
        debug_image = overlay
        
        # Combine masks for edge detection
        if len(color_masks) > 0:
            edges = color_masks[0].copy()
            for mask in color_masks[1:]:
                edges = cv2.bitwise_or(edges, mask)
        else:
            # Fallback to traditional edge detection
            edges = detect_edges(image, 
                               blur_size=edge_params.get('gaussian_blur_size', 5),
                               canny_low=edge_params.get('canny_threshold1', 50),
                               canny_high=edge_params.get('canny_threshold2', 150),
                               morph_size=morph_size,
                               use_color_seg=False,
                               logger=logger)
    else:
        # Normal edge detection without debug visualization
        edges = detect_edges(
            image,
            blur_size=edge_params.get('gaussian_blur_size', 5),
            canny_low=edge_params.get('canny_threshold1', 50),
            canny_high=edge_params.get('canny_threshold2', 150),
            morph_size=morph_size,
            use_color_seg=use_color_seg,
            n_colors=n_colors,
            min_saturation=min_saturation,
            min_region_pixels=min_region_pixels,
            max_region_pixels=max_region_pixels,
            logger=logger
        )
    
    # Contour extraction
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                          cv2.CHAIN_APPROX_SIMPLE)
    
    if logger:
        logger.info(f'Edge detection completed: found {len(contours)} total contours')
        if len(contours) == 0:
            logger.warning('No contours found in edge image - check edge detection parameters')
            # Log edge image statistics for debugging
            edge_pixels = np.sum(edges > 0)
            total_pixels = edges.shape[0] * edges.shape[1]
            logger.info(f'Edge image: {edge_pixels}/{total_pixels} pixels are edges ({edge_pixels/total_pixels*100:.1f}%)')
    
    results = []
    filtered_count = 0
    
    for i, contour in enumerate(contours):
        # Calculate geometric features
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        if logger:
            logger.info(f'Processing contour {i}: area={area:.0f}, perimeter={perimeter:.1f}')
        
        if area < geom_params['min_area']:
            if logger:  # Show ALL contours rejected by area
                logger.info(f'Contour {i}: area={area:.0f} REJECTED - too small (min: {geom_params["min_area"]})')
            continue
        elif area > geom_params['max_area']:
            if logger:  # Show ALL contours rejected by area
                logger.info(f'Contour {i}: area={area:.0f} REJECTED - too large (max: {geom_params["max_area"]})')
            continue
        
        # Bounding box
        bbox = cv2.boundingRect(contour)
        x, y, w, h = bbox
        
        # Aspect ratio
        aspect_ratio = calculate_aspect_ratio(bbox)
        
        # Solidity
        solidity = calculate_solidity(contour)
        
        # Polygon approximation
        approx = approximate_polygon(contour)
        vertices = len(approx)
        
        if logger:
            logger.debug(f'Contour {i}: area={area:.0f}, aspect={aspect_ratio:.2f}, solidity={solidity:.2f}, vertices={vertices}')
        
        # Check if cube-like
        if is_cube_like(
            area, aspect_ratio, solidity, vertices,
            geom_params['min_area'], geom_params['max_area'],
            geom_params['min_aspect_ratio'], geom_params['max_aspect_ratio'],
            geom_params['min_solidity'], geom_params['expected_vertices'],
            geom_params['vertex_tolerance']
        ):
            # Calculate confidence
            confidence = calculate_cube_confidence(
                area, aspect_ratio, solidity, vertices,
                geom_params['min_area'], geom_params['max_area']
            )
            
            # Center point
            center = get_contour_center(contour)
            
            if logger:
                logger.info(f'Contour {i}: ACCEPTED as cube with confidence={confidence:.2f}')
            
            results.append({
                'contour': contour,
                'bbox': bbox,
                'center': center,
                'area': area,
                'confidence': confidence,
                'aspect_ratio': aspect_ratio,
                'solidity': solidity,
                'vertices': vertices
            })
        else:
            if logger:
                filtered_count += 1
                # Show why it was rejected
                checks = []
                if not (geom_params['min_area'] <= area <= geom_params['max_area']):
                    checks.append(f"area")
                if not (geom_params['min_aspect_ratio'] <= aspect_ratio <= geom_params['max_aspect_ratio']):
                    checks.append(f"aspect_ratio")
                if not (solidity >= geom_params['min_solidity']):
                    checks.append(f"solidity")
                if not (abs(vertices - geom_params['expected_vertices']) <= geom_params['vertex_tolerance']):
                    checks.append(f"vertices")
                logger.debug(f'Contour {i}: REJECTED - failed checks: {", ".join(checks)}')
    
    if logger:
        logger.info(f'Contour filtering completed: {len(contours)} total, {len(results)} accepted as cubes, {filtered_count} rejected')
        if len(results) == 0 and len(contours) > 0:
            # 分析面积分布，提供智能建议
            areas = []
            for contour in contours:
                area = cv2.contourArea(contour)
                areas.append(area)
            
            if areas:
                min_area = min(areas)
                max_area = max(areas)
                avg_area = sum(areas) / len(areas)
                logger.warning(f'No cubes detected - all contours were filtered out.')
                logger.info(f'Area statistics: min={min_area:.0f}, max={max_area:.0f}, avg={avg_area:.0f}')
                
                # 提供智能建议
                if max_area < geom_params['min_area']:
                    logger.info(f'SUGGESTION: All contours are too small. Reduce min_area from {geom_params["min_area"]} to around {max_area*0.8:.0f}')
                elif min_area > geom_params['max_area']:
                    logger.info(f'SUGGESTION: All contours are too large. Increase max_area from {geom_params["max_area"]} to around {min_area*1.2:.0f}')
                else:
                    logger.info(f'SUGGESTION: Try adjusting area range to include values around {avg_area:.0f}')
                    logger.info(f'Consider range: {min_area*0.8:.0f}-{max_area*1.2:.0f}')
    
    # Draw accepted contours on debug image
    if debug and debug_image is not None:
        for detection in results:
            contour = detection['contour']
            bbox = detection['bbox']
            x, y, w, h = bbox
            
            # Draw green rectangle for accepted detections
            cv2.rectangle(debug_image, (x, y), (x+w, y+h), (0, 255, 0), 3)
            
            # Draw contour
            cv2.drawContours(debug_image, [contour], -1, (0, 255, 0), 2)
            
            # Add label
            label = f"Cube {detection['confidence']:.2f}"
            cv2.putText(debug_image, label, (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # Sort by confidence
    results.sort(key=lambda x: x['confidence'], reverse=True)
    
    return results, debug_image
