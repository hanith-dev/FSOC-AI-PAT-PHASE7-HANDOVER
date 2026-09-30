import time
import logging
from typing import Dict, Any, Tuple, List, Optional
import numpy as np
import cv2

from app.detection.localization import get_localizer, BaseLocalizer, LocalizationResult

logger = logging.getLogger("FSOC-Detector")

# Try to import ultralytics for YOLO support, but fail gracefully if pytorch/ultralytics is missing
try:
    from ultralytics import YOLO
    HAS_ULTRALYTICS = True
except ImportError:
    HAS_ULTRALYTICS = False
    logger.warning("ultralytics package not found. YOLO detector will not be available locally.")

class Detector:
    def detect(self, img: np.ndarray) -> Dict[str, Any]:
        """
        Processes image frame and returns detection dictionary.
        """
        raise NotImplementedError

class OpenCVDetector(Detector):
    def __init__(
        self,
        hsv_lower: List[int] = [35, 50, 50],
        hsv_upper: List[int] = [95, 255, 255],
        localization_mode: str = "baseline",
        localizer: Optional[BaseLocalizer] = None
    ):
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)
        self.localization_mode = localization_mode
        self.localizer = localizer if localizer is not None else get_localizer(localization_mode)
        logger.info(f"OpenCV Detector initialized with HSV Lower: {hsv_lower}, Upper: {hsv_upper}, Localization: {self.localization_mode}")

    def set_localization_mode(self, mode: str):
        self.localization_mode = mode
        self.localizer = get_localizer(mode)
        logger.info(f"OpenCV Detector localization mode switched to: {mode}")

    def detect(self, img: np.ndarray) -> Dict[str, Any]:
        start_time = time.time()
        
        # Default empty result
        result = {
            "target_id": "BEACON-01",
            "confidence": 0.0,
            "bbox": [0, 0, 0, 0],
            "center": [0.0, 0.0],
            "detected": False,
            "candidates_count": 0,
            "inference_time_ms": 0.0
        }
        
        if img is None or img.size == 0:
            return result
            
        h, w, c = img.shape
        
        # 1. Color space conversion (BGR to HSV)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # 2. Thresholding
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        
        # 3. Morphological filtering
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        # Remove small speckles (noise)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        # Close gaps in shapes
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        
        # 4. Contour detection
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Filter out tiny or huge contours
            if area < 10 or area > (w * h * 0.5):
                continue
                
            # Compute circularity: 4 * pi * area / perimeter^2
            perimeter = cv2.arcLength(cnt, True)
            circularity = 0.0
            if perimeter > 0:
                circularity = (4.0 * np.pi * area) / (perimeter * perimeter)
                
            # Bright beacons should look circular (glowing sphere)
            if circularity > 0.4:
                # Bounding box
                x, y, box_w, box_h = cv2.boundingRect(cnt)
                bbox = [x, y, x + box_w, y + box_h]

                # Decoupled Precision Localization
                if self.localizer is not None:
                    loc_res = self.localizer.localize(img, bbox)
                    cx = loc_res.x
                    cy = loc_res.y
                    loc_info = loc_res.to_dict()
                else:
                    cx = x + box_w / 2.0
                    cy = y + box_h / 2.0
                    loc_info = None

                candidates.append({
                    "bbox": bbox,
                    "center": [cx, cy],
                    "area": area,
                    "confidence": min(1.0, float(circularity)), # Use circularity as confidence proxy
                    "localization": loc_info
                })
                
        result["candidates_count"] = len(candidates)
        result["candidates"] = candidates
        
        # 5. Extract target (largest area candidate)
        if candidates:
            candidates.sort(key=lambda x: x["area"], reverse=True)
            best = candidates[0]
            result["detected"] = True
            result["confidence"] = best["confidence"]
            result["bbox"] = best["bbox"]
            result["center"] = best["center"]
            
        result["inference_time_ms"] = (time.time() - start_time) * 1000.0
        return result

class YOLODetector(Detector):
    def __init__(self, model_path: str = "models/beacon_detector.pt"):
        self.model_path = model_path
        self.model = None
        
        if not HAS_ULTRALYTICS:
            raise ImportError("Cannot instantiate YOLODetector because ultralytics is not installed.")
            
        try:
            # Try to load model
            self.model = YOLO(model_path)
            logger.info(f"YOLO Detector initialized with model: {model_path}")
        except Exception as e:
            logger.error(f"Failed to load YOLO model: {e}")
            raise FileNotFoundError(f"YOLO weights not found or invalid: {model_path}")

    def detect(self, img: np.ndarray) -> Dict[str, Any]:
        start_time = time.time()
        
        result = {
            "target_id": "BEACON-01",
            "confidence": 0.0,
            "bbox": [0, 0, 0, 0],
            "center": [0.0, 0.0],
            "detected": False,
            "candidates_count": 0,
            "inference_time_ms": 0.0
        }
        
        if img is None or img.size == 0 or self.model is None:
            return result
            
        # Run inference
        try:
            # verbose=False reduces console clutter
            predictions = self.model(img, verbose=False)[0]
            boxes = predictions.boxes
            
            result["candidates_count"] = len(boxes)
            
            if len(boxes) > 0:
                candidates = []
                for box in boxes:
                    conf = float(box.conf.cpu().numpy()[0])
                    xyxy = box.xyxy.cpu().numpy()[0]
                    candidates.append({
                        "bbox": [int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])],
                        "center": [
                            float((xyxy[0] + xyxy[2]) / 2.0),
                            float((xyxy[1] + xyxy[3]) / 2.0)
                        ],
                        "confidence": conf
                    })
                result["candidates"] = candidates
                
                # Find detection with highest confidence
                best_box_idx = int(np.argmax(boxes.conf.cpu().numpy()))
                best_box = boxes[best_box_idx]
                
                conf = float(best_box.conf.cpu().numpy()[0])
                xyxy = best_box.xyxy.cpu().numpy()[0]
                
                result["detected"] = True
                result["confidence"] = conf
                result["bbox"] = [int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])]
                result["center"] = [
                    float((xyxy[0] + xyxy[2]) / 2.0),
                    float((xyxy[1] + xyxy[3]) / 2.0)
                ]
            else:
                result["candidates"] = []
        except Exception as e:
            logger.error(f"YOLO inference error: {e}")
            result["candidates"] = []
            
        result["inference_time_ms"] = (time.time() - start_time) * 1000.0
        return result

class DetectorInterface:
    def __init__(
        self,
        mode: str = "opencv",
        yolo_path: str = "models/beacon_detector.pt",
        hsv_lower: List[int] = [35, 50, 50],
        hsv_upper: List[int] = [95, 255, 255],
        localization_mode: str = "baseline"
    ):
        self.mode = mode
        self.yolo_path = yolo_path
        self.hsv_lower = hsv_lower
        self.hsv_upper = hsv_upper
        self.localization_mode = localization_mode
        
        self.opencv_detector = OpenCVDetector(hsv_lower, hsv_upper, localization_mode=localization_mode)
        self.yolo_detector = None
        self.fallback_warning_active = False
        
        self._initialize_yolo()

    def set_localization_mode(self, mode: str):
        self.localization_mode = mode
        self.opencv_detector.set_localization_mode(mode)

    def _initialize_yolo(self) -> bool:
        if not HAS_ULTRALYTICS:
            return False
        try:
            self.yolo_detector = YOLODetector(self.yolo_path)
            return True
        except Exception as e:
            logger.warning(f"YOLO initialization skipped/failed. Falling back to OpenCV. Details: {e}")
            self.yolo_detector = None
            return False

    def set_mode(self, mode: str):
        if mode not in ["opencv", "yolo", "hybrid"]:
            logger.error(f"Invalid detector mode: {mode}")
            return
        self.mode = mode
        logger.info(f"Detector mode switched to: {mode}")

    def detect(self, img: np.ndarray) -> Tuple[Dict[str, Any], str]:
        """
        Executes active detector and handles fallbacks.
        Returns a tuple of (detection_result, active_detector_name)
        """
        active_run_mode = self.mode
        
        if self.mode == "yolo":
            if self.yolo_detector is not None:
                try:
                    return self.yolo_detector.detect(img), "yolo"
                except Exception as e:
                    logger.warning(f"YOLO detect failed: {e}. Falling back to OpenCV.")
                    self.fallback_warning_active = True
                    active_run_mode = "opencv"
            else:
                self.fallback_warning_active = True
                active_run_mode = "opencv"
                
        if active_run_mode == "hybrid":
            if self.yolo_detector is not None:
                try:
                    res = self.yolo_detector.detect(img)
                    if res["detected"] and res["confidence"] > 0.5:
                        return res, "yolo"
                except Exception as e:
                    logger.warning(f"Hybrid YOLO pass failed: {e}. Trying OpenCV.")
            active_run_mode = "opencv"
            
        # Default OpenCV pass
        return self.opencv_detector.detect(img), "opencv"
