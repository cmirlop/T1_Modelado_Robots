import sys
import json
from pathlib import Path
from typing import Dict, Any, Tuple
import cv2 as cv
import numpy as np


def ensure_odd(n: int) -> int:
    return n if n % 2 == 1 else max(1, n + 1)


def default_config() -> Dict[str, Any]:
    return {
        "active_space": "HSV",  # HSV | LAB | CMYK | YCrCb
        "HSV": {"H": {"min": 100, "max": 140}, "S": {"min": 120, "max": 255}, "V": {"min": 70, "max": 255}},
        "LAB": {"L": {"min": 0, "max": 255}, "A": {"min": 100, "max": 180}, "B": {"min": 120, "max": 200}},
        "YCrCb": {"Y": {"min": 0, "max": 255}, "Cr": {"min": 0, "max": 255}, "Cb": {"min": 0, "max": 255}},
        "CMYK": {
            "C": {"min": 0, "max": 255},
            "M": {"min": 0, "max": 255},
            "Y": {"min": 0, "max": 255},
            "K": {"min": 0, "max": 255},
        },
        "morphology": {"kernel_idx": 2, "open_iterations": 1, "close_iterations": 1},
        "blur": {"ksize_idx": 0, "sigma": 0},
        "lsd": {
            "enabled": 0,       # 0=off, 1=on
            "input": 1,         # 0=Frame, 1=Mask
            "refine": 1,        # 0..2
            "scale": 0.8,       # 0.1..2.0
            "sigma_scale": 0.6, # 0.1..1.5
            "quant": 2.0,       # 0.1..20.0
            "ang_th": 22.5,     # degrees
            "log_eps": 0.0,     # typical 0
            "density_th": 0.7,  # 0..1
            "n_bins": 1024,     # 32..4096
        },
    }


def load_config(path: Path) -> Dict[str, Any]:
    cfg = default_config()
    if path.exists():
        try:
            with path.open("r", encoding="utf-8") as f:
                incoming = json.load(f)
            # shallow merge
            for k, v in incoming.items():
                if isinstance(v, dict) and k in cfg:
                    cfg[k].update(v) if isinstance(cfg[k], dict) else None
                else:
                    cfg[k] = v
        except Exception as e:
            print(f"Warning: could not load calibration JSON: {e}")
    return cfg


def save_config(path: Path, cfg: Dict[str, Any]) -> None:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
        print(f"Saved calibration to {path}")
    except Exception as e:
        print(f"Error saving calibration: {e}")


def open_camera(camera_index: int) -> cv.VideoCapture:
    cap = cv.VideoCapture(camera_index, cv.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv.VideoCapture(camera_index)
    return cap


def _noop(_: int) -> None:
    return


def create_controls_panel(panel: str, cfg: Dict[str, Any]) -> None:
    cv.namedWindow(panel, cv.WINDOW_NORMAL)
    cv.resizeWindow(panel, 360, 300)
    active_map = {"HSV": 0, "LAB": 1, "CMYK": 2, "YCrCb": 3}
    cv.createTrackbar("Active (0=HSV 1=LAB 2=CMYK 3=YCrCb)", panel, active_map.get(cfg["active_space"], 0), 3, _noop)
    cv.createTrackbar("Save (toggle 1)", panel, 0, 1, _noop)
    cv.createTrackbar("Reload (toggle 1)", panel, 0, 1, _noop)
    cv.createTrackbar("Morph Kernel idx (0-15)", panel, int(cfg["morphology"]["kernel_idx"]), 15, _noop)
    cv.createTrackbar("Open iterations", panel, int(cfg["morphology"]["open_iterations"]), 10, _noop)
    cv.createTrackbar("Close iterations", panel, int(cfg["morphology"]["close_iterations"]), 10, _noop)
    cv.createTrackbar("Blur k idx (0-15)", panel, int(cfg["blur"]["ksize_idx"]), 15, _noop)
    cv.createTrackbar("Blur sigma (0-100)", panel, int(cfg["blur"]["sigma"]), 100, _noop)


def create_hsv_panel(panel: str, cfg: Dict[str, Any]) -> None:
    cv.namedWindow(panel, cv.WINDOW_NORMAL)
    cv.resizeWindow(panel, 360, 200)
    hsv = cfg["HSV"]
    cv.createTrackbar("HSV H min", panel, int(hsv["H"]["min"]), 179, _noop)
    cv.createTrackbar("HSV H max", panel, int(hsv["H"]["max"]), 179, _noop)
    cv.createTrackbar("HSV S min", panel, int(hsv["S"]["min"]), 255, _noop)
    cv.createTrackbar("HSV S max", panel, int(hsv["S"]["max"]), 255, _noop)
    cv.createTrackbar("HSV V min", panel, int(hsv["V"]["min"]), 255, _noop)
    cv.createTrackbar("HSV V max", panel, int(hsv["V"]["max"]), 255, _noop)


def create_lab_panel(panel: str, cfg: Dict[str, Any]) -> None:
    cv.namedWindow(panel, cv.WINDOW_NORMAL)
    cv.resizeWindow(panel, 360, 200)
    lab = cfg["LAB"]
    cv.createTrackbar("LAB L min", panel, int(lab["L"]["min"]), 255, _noop)
    cv.createTrackbar("LAB L max", panel, int(lab["L"]["max"]), 255, _noop)
    cv.createTrackbar("LAB A min", panel, int(lab["A"]["min"]), 255, _noop)
    cv.createTrackbar("LAB A max", panel, int(lab["A"]["max"]), 255, _noop)
    cv.createTrackbar("LAB B min", panel, int(lab["B"]["min"]), 255, _noop)
    cv.createTrackbar("LAB B max", panel, int(lab["B"]["max"]), 255, _noop)


def create_cmyk_panel(panel: str, cfg: Dict[str, Any]) -> None:
    cv.namedWindow(panel, cv.WINDOW_NORMAL)
    cv.resizeWindow(panel, 360, 260)
    cmyk = cfg["CMYK"]
    cv.createTrackbar("CMYK C min", panel, int(cmyk["C"]["min"]), 255, _noop)
    cv.createTrackbar("CMYK C max", panel, int(cmyk["C"]["max"]), 255, _noop)
    cv.createTrackbar("CMYK M min", panel, int(cmyk["M"]["min"]), 255, _noop)
    cv.createTrackbar("CMYK M max", panel, int(cmyk["M"]["max"]), 255, _noop)
    cv.createTrackbar("CMYK Y min", panel, int(cmyk["Y"]["min"]), 255, _noop)
    cv.createTrackbar("CMYK Y max", panel, int(cmyk["Y"]["max"]), 255, _noop)
    cv.createTrackbar("CMYK K min", panel, int(cmyk["K"]["min"]), 255, _noop)
    cv.createTrackbar("CMYK K max", panel, int(cmyk["K"]["max"]), 255, _noop)


def create_ycrcb_panel(panel: str, cfg: Dict[str, Any]) -> None:
    cv.namedWindow(panel, cv.WINDOW_NORMAL)
    cv.resizeWindow(panel, 360, 200)
    ycrcb = cfg.get("YCrCb", {"Y": {"min": 0, "max": 255}, "Cr": {"min": 0, "max": 255}, "Cb": {"min": 0, "max": 255}})
    cv.createTrackbar("YCrCb Y min", panel, int(ycrcb["Y"]["min"]), 255, _noop)
    cv.createTrackbar("YCrCb Y max", panel, int(ycrcb["Y"]["max"]), 255, _noop)
    cv.createTrackbar("YCrCb Cr min", panel, int(ycrcb["Cr"]["min"]), 255, _noop)
    cv.createTrackbar("YCrCb Cr max", panel, int(ycrcb["Cr"]["max"]), 255, _noop)
    cv.createTrackbar("YCrCb Cb min", panel, int(ycrcb["Cb"]["min"]), 255, _noop)
    cv.createTrackbar("YCrCb Cb max", panel, int(ycrcb["Cb"]["max"]), 255, _noop)


def create_lsd_panel(panel: str, cfg: Dict[str, Any]) -> None:
    cv.namedWindow(panel, cv.WINDOW_NORMAL)
    cv.resizeWindow(panel, 420, 360)
    lsd = cfg["lsd"]
    cv.createTrackbar("LSD Show (toggle 1)", panel, int(lsd.get("enabled", 0)), 1, _noop)
    cv.createTrackbar("LSD Input (0=Frame 1=Mask)", panel, int(lsd.get("input", 1)), 1, _noop)
    cv.createTrackbar("LSD Refine (0-2)", panel, int(lsd.get("refine", 1)), 2, _noop)
    cv.createTrackbar("LSD Scale x100 (10-200)", panel, int(round(lsd.get("scale", 0.8) * 100)), 200, _noop)
    cv.createTrackbar("LSD Sigma x100 (10-150)", panel, int(round(lsd.get("sigma_scale", 0.6) * 100)), 150, _noop)
    cv.createTrackbar("LSD Quant x10 (10-200)", panel, int(round(lsd.get("quant", 2.0) * 10)), 200, _noop)
    cv.createTrackbar("LSD AngTh deg (5-90)", panel, int(round(lsd.get("ang_th", 22.5))), 90, _noop)
    cv.createTrackbar("LSD LogEps (0-10)", panel, int(round(lsd.get("log_eps", 0.0))), 10, _noop)
    cv.createTrackbar("LSD Density x100 (0-100)", panel, int(round(lsd.get("density_th", 0.7) * 100)), 100, _noop)
    cv.createTrackbar("LSD n_bins (32-4096)", panel, int(lsd.get("n_bins", 1024)), 4096, _noop)


def read_lsd_from_panel(panel: str, cfg: Dict[str, Any]) -> Dict[str, Any]:
    def clamp(v: int, lo: int, hi: int) -> int:
        return max(lo, min(hi, v))

    s100 = clamp(cv.getTrackbarPos("LSD Scale x100 (10-200)", panel), 10, 200)
    sg100 = clamp(cv.getTrackbarPos("LSD Sigma x100 (10-150)", panel), 10, 150)
    q10 = clamp(cv.getTrackbarPos("LSD Quant x10 (10-200)", panel), 10, 200)
    ang = clamp(cv.getTrackbarPos("LSD AngTh deg (5-90)", panel), 5, 90)
    loge = clamp(cv.getTrackbarPos("LSD LogEps (0-10)", panel), 0, 10)
    dens100 = clamp(cv.getTrackbarPos("LSD Density x100 (0-100)", panel), 0, 100)
    n_bins = clamp(cv.getTrackbarPos("LSD n_bins (32-4096)", panel), 32, 4096)

    cfg["lsd"]["enabled"] = int(cv.getTrackbarPos("LSD Show (toggle 1)", panel))
    cfg["lsd"]["input"] = int(cv.getTrackbarPos("LSD Input (0=Frame 1=Mask)", panel))
    cfg["lsd"]["refine"] = int(cv.getTrackbarPos("LSD Refine (0-2)", panel))
    cfg["lsd"]["scale"] = float(s100) / 100.0
    cfg["lsd"]["sigma_scale"] = float(sg100) / 100.0
    cfg["lsd"]["quant"] = float(q10) / 10.0
    cfg["lsd"]["ang_th"] = float(ang)
    cfg["lsd"]["log_eps"] = float(loge)
    cfg["lsd"]["density_th"] = float(dens100) / 100.0
    cfg["lsd"]["n_bins"] = int(n_bins)
    return cfg


def apply_lsd_to_panel(panel: str, cfg: Dict[str, Any]) -> None:
    lsd = cfg.get("lsd", {})
    cv.setTrackbarPos("LSD Show (toggle 1)", panel, int(lsd.get("enabled", 0)))
    cv.setTrackbarPos("LSD Input (0=Frame 1=Mask)", panel, int(lsd.get("input", 1)))
    cv.setTrackbarPos("LSD Refine (0-2)", panel, int(lsd.get("refine", 1)))
    cv.setTrackbarPos("LSD Scale x100 (10-200)", panel, int(round(lsd.get("scale", 0.8) * 100)))
    cv.setTrackbarPos("LSD Sigma x100 (10-150)", panel, int(round(lsd.get("sigma_scale", 0.6) * 100)))
    cv.setTrackbarPos("LSD Quant x10 (10-200)", panel, int(round(lsd.get("quant", 2.0) * 10)))
    cv.setTrackbarPos("LSD AngTh deg (5-90)", panel, int(round(lsd.get("ang_th", 22.5))))
    cv.setTrackbarPos("LSD LogEps (0-10)", panel, int(round(lsd.get("log_eps", 0.0))))
    cv.setTrackbarPos("LSD Density x100 (0-100)", panel, int(round(lsd.get("density_th", 0.7) * 100)))
    cv.setTrackbarPos("LSD n_bins (32-4096)", panel, int(lsd.get("n_bins", 1024)))


def detect_lsd_overlay(frame_bgr: np.ndarray, mask: np.ndarray, cfg: Dict[str, Any]) -> np.ndarray:
    params = cfg.get("lsd", {})
    refine = int(params.get("refine", 1))
    scale = float(params.get("scale", 0.8))
    sigma_scale = float(params.get("sigma_scale", 0.6))
    quant = float(params.get("quant", 2.0))
    ang_th = float(params.get("ang_th", 22.5))
    log_eps = float(params.get("log_eps", 0.0))
    density_th = float(params.get("density_th", 0.7))
    n_bins = int(params.get("n_bins", 1024))

    # Choose input
    use_mask = int(params.get("input", 1)) == 1
    if use_mask:
        gray = mask
        overlay = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
    else:
        gray = cv.cvtColor(frame_bgr, cv.COLOR_BGR2GRAY)
        overlay = frame_bgr.copy()

    try:
        lsd = cv.createLineSegmentDetector(refine, scale, sigma_scale, quant, ang_th, log_eps, density_th, n_bins)
    except Exception as e:
        cv.putText(overlay, f"LSD unavailable: {e}", (10, 25), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv.LINE_AA)
        return overlay

    lines = None
    try:
        result = lsd.detect(gray)
        lines = result[0] if isinstance(result, (list, tuple)) and len(result) > 0 else None
    except Exception as e:
        cv.putText(overlay, f"LSD error: {e}", (10, 25), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv.LINE_AA)
        return overlay

    if lines is not None:
        overlay = lsd.drawSegments(overlay, lines)
    return overlay


def read_config_from_panels(controls_panel: str, hsv_panel: str, lab_panel: str, cmyk_panel: str, ycrcb_panel: str, lsd_panel: str, cfg: Dict[str, Any]) -> Dict[str, Any]:
    idx_map = {0: "HSV", 1: "LAB", 2: "CMYK", 3: "YCrCb"}
    active_idx = cv.getTrackbarPos("Active (0=HSV 1=LAB 2=CMYK 3=YCrCb)", controls_panel)
    cfg["active_space"] = idx_map.get(active_idx, "HSV")

    cfg["morphology"]["kernel_idx"] = cv.getTrackbarPos("Morph Kernel idx (0-15)", controls_panel)
    cfg["morphology"]["open_iterations"] = cv.getTrackbarPos("Open iterations", controls_panel)
    cfg["morphology"]["close_iterations"] = cv.getTrackbarPos("Close iterations", controls_panel)
    cfg["blur"]["ksize_idx"] = cv.getTrackbarPos("Blur k idx (0-15)", controls_panel)
    cfg["blur"]["sigma"] = cv.getTrackbarPos("Blur sigma (0-100)", controls_panel)

    def read_range(win: str, prefix: str, keys: Tuple[str, ...]) -> Dict[str, Dict[str, int]]:
        out: Dict[str, Dict[str, int]] = {}
        for k in keys:
            mn = cv.getTrackbarPos(f"{prefix} {k} min", win)
            mx = cv.getTrackbarPos(f"{prefix} {k} max", win)
            if mn > mx:
                mn, mx = mx, mn
            out[k] = {"min": int(mn), "max": int(mx)}
        return out

    if cfg["active_space"] == "HSV":
        cfg["HSV"] = read_range(hsv_panel, "HSV", ("H", "S", "V"))
    elif cfg["active_space"] == "LAB":
        cfg["LAB"] = read_range(lab_panel, "LAB", ("L", "A", "B"))
    elif cfg["active_space"] == "CMYK":
        cfg["CMYK"] = read_range(cmyk_panel, "CMYK", ("C", "M", "Y", "K"))
    elif cfg["active_space"] == "YCrCb":
        cfg["YCrCb"] = read_range(ycrcb_panel, "YCrCb", ("Y", "Cr", "Cb"))

    cfg = read_lsd_from_panel(lsd_panel, cfg)
    return cfg


def apply_config_to_panels(controls_panel: str, hsv_panel: str, lab_panel: str, cmyk_panel: str, ycrcb_panel: str, lsd_panel: str, cfg: Dict[str, Any]) -> None:
    space_to_idx = {"HSV": 0, "LAB": 1, "CMYK": 2, "YCrCb": 3}
    cv.setTrackbarPos("Active (0=HSV 1=LAB 2=CMYK 3=YCrCb)", controls_panel, space_to_idx.get(cfg["active_space"], 0))
    cv.setTrackbarPos("Morph Kernel idx (0-15)", controls_panel, int(cfg["morphology"]["kernel_idx"]))
    cv.setTrackbarPos("Open iterations", controls_panel, int(cfg["morphology"]["open_iterations"]))
    cv.setTrackbarPos("Close iterations", controls_panel, int(cfg["morphology"]["close_iterations"]))
    cv.setTrackbarPos("Blur k idx (0-15)", controls_panel, int(cfg["blur"]["ksize_idx"]))
    cv.setTrackbarPos("Blur sigma (0-100)", controls_panel, int(cfg["blur"]["sigma"]))

    def set_range(win: str, prefix: str, d: Dict[str, Dict[str, int]]) -> None:
        for ch in d:
            cv.setTrackbarPos(f"{prefix} {ch} min", win, int(d[ch]["min"]))
            cv.setTrackbarPos(f"{prefix} {ch} max", win, int(d[ch]["max"]))

    if cfg["active_space"] == "HSV":
        set_range(hsv_panel, "HSV", cfg["HSV"])
    elif cfg["active_space"] == "LAB":
        set_range(lab_panel, "LAB", cfg["LAB"])
    elif cfg["active_space"] == "CMYK":
        set_range(cmyk_panel, "CMYK", cfg["CMYK"])
    elif cfg["active_space"] == "YCrCb":
        set_range(ycrcb_panel, "YCrCb", cfg.get("YCrCb", {"Y": {"min": 0, "max": 255}, "Cr": {"min": 0, "max": 255}, "Cb": {"min": 0, "max": 255}}))

    apply_lsd_to_panel(lsd_panel, cfg)


def bgr_to_cmyk(img_bgr: np.ndarray) -> np.ndarray:
    rgb = img_bgr[:, :, ::-1].astype(np.float32) / 255.0
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    k = 1.0 - np.maximum(np.maximum(r, g), b)
    denom = 1.0 - k
    denom_safe = np.where(denom == 0, 1.0, denom)
    c = (1.0 - r - k) / denom_safe
    m = (1.0 - g - k) / denom_safe
    y = (1.0 - b - k) / denom_safe
    c = np.where(denom == 0, 0.0, c)
    m = np.where(denom == 0, 0.0, m)
    y = np.where(denom == 0, 0.0, y)
    cmyk = np.stack([c, m, y, k], axis=-1)
    cmyk_u8 = np.clip(cmyk * 255.0, 0, 255).astype(np.uint8)
    return cmyk_u8


def mask_from_space(frame_bgr: np.ndarray, cfg: Dict[str, Any]) -> np.ndarray:
    space = cfg["active_space"]
    if space == "HSV":
        hsv = cv.cvtColor(frame_bgr, cv.COLOR_BGR2HSV)
        low = np.array([cfg["HSV"]["H"]["min"], cfg["HSV"]["S"]["min"], cfg["HSV"]["V"]["min"]], dtype=np.uint8)
        up = np.array([cfg["HSV"]["H"]["max"], cfg["HSV"]["S"]["max"], cfg["HSV"]["V"]["max"]], dtype=np.uint8)
        mask = cv.inRange(hsv, low, up)
        return mask
    elif space == "LAB":
        lab = cv.cvtColor(frame_bgr, cv.COLOR_BGR2LAB)
        low = np.array([cfg["LAB"]["L"]["min"], cfg["LAB"]["A"]["min"], cfg["LAB"]["B"]["min"]], dtype=np.uint8)
        up = np.array([cfg["LAB"]["L"]["max"], cfg["LAB"]["A"]["max"], cfg["LAB"]["B"]["max"]], dtype=np.uint8)
        mask = cv.inRange(lab, low, up)
        return mask
    elif space == "YCrCb":
        ycrcb = cv.cvtColor(frame_bgr, cv.COLOR_BGR2YCrCb)
        low = np.array([cfg["YCrCb"]["Y"]["min"], cfg["YCrCb"]["Cr"]["min"], cfg["YCrCb"]["Cb"]["min"]], dtype=np.uint8)
        up = np.array([cfg["YCrCb"]["Y"]["max"], cfg["YCrCb"]["Cr"]["max"], cfg["YCrCb"]["Cb"]["max"]], dtype=np.uint8)
        mask = cv.inRange(ycrcb, low, up)
        return mask
    else:
        cmyk = bgr_to_cmyk(frame_bgr)
        C = cmyk[:, :, 0]
        M = cmyk[:, :, 1]
        Y = cmyk[:, :, 2]
        K = cmyk[:, :, 3]
        cfgc = cfg["CMYK"]
        mC = (C >= cfgc["C"]["min"]) & (C <= cfgc["C"]["max"])
        mM = (M >= cfgc["M"]["min"]) & (M <= cfgc["M"]["max"])
        mY = (Y >= cfgc["Y"]["min"]) & (Y <= cfgc["Y"]["max"])
        mK = (K >= cfgc["K"]["min"]) & (K <= cfgc["K"]["max"])
        mask = (mC & mM & mY & mK).astype(np.uint8) * 255
        return mask


def postprocess_mask(mask: np.ndarray, cfg: Dict[str, Any]) -> np.ndarray:
    k_idx = int(cfg["morphology"]["kernel_idx"])
    ksize = ensure_odd(2 * k_idx + 1)
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (ksize, ksize))
    open_it = int(cfg["morphology"]["open_iterations"])
    close_it = int(cfg["morphology"]["close_iterations"])

    if open_it > 0:
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=open_it)
    if close_it > 0:
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel, iterations=close_it)

    b_idx = int(cfg["blur"]["ksize_idx"])
    bksize = ensure_odd(2 * b_idx + 1)
    sigma = float(cfg["blur"]["sigma"])
    if bksize > 1:
        mask = cv.GaussianBlur(mask, (bksize, bksize), sigmaX=sigma)
        _, mask = cv.threshold(mask, 127, 255, cv.THRESH_BINARY)
    return mask


def main() -> None:
    # Args: [camera_index] [calibration_json_path]
    camera_index = 0
    json_path = Path("calibration.json")
    if len(sys.argv) >= 2:
        try:
            camera_index = int(sys.argv[1])
        except ValueError:
            print("First argument must be camera index (int).")
            return
    if len(sys.argv) >= 3:
        json_path = Path(sys.argv[2])

    cfg = load_config(json_path)

    cap = open_camera(camera_index)
    if not cap.isOpened():
        print(f"Could not open camera index {camera_index}")
        return

    controls = "Controls"
    hsv_panel = "HSV Panel"
    lab_panel = "LAB Panel"
    cmyk_panel = "CMYK Panel"
    ycrcb_panel = "YCrCb Panel"
    lsd_panel = "LSD Panel"

    create_controls_panel(controls, cfg)
    create_lsd_panel(lsd_panel, cfg)

    def ensure_active_panel(target_space: str, current_space: str) -> None:
        if target_space == current_space:
            return
        # Destroy old
        old_panel = None
        if current_space == "HSV": old_panel = hsv_panel
        elif current_space == "LAB": old_panel = lab_panel
        elif current_space == "CMYK": old_panel = cmyk_panel
        elif current_space == "YCrCb": old_panel = ycrcb_panel
        
        if old_panel:
            try:
                cv.destroyWindow(old_panel)
            except Exception:
                pass
        
        # Create new
        if target_space == "HSV": create_hsv_panel(hsv_panel, cfg)
        elif target_space == "LAB": create_lab_panel(lab_panel, cfg)
        elif target_space == "CMYK": create_cmyk_panel(cmyk_panel, cfg)
        elif target_space == "YCrCb": create_ycrcb_panel(ycrcb_panel, cfg)
    
    # Initial creation
    current_space = cfg["active_space"]
    if current_space == "HSV": create_hsv_panel(hsv_panel, cfg)
    elif current_space == "LAB": create_lab_panel(lab_panel, cfg)
    elif current_space == "CMYK": create_cmyk_panel(cmyk_panel, cfg)
    elif current_space == "YCrCb": create_ycrcb_panel(ycrcb_panel, cfg)
    
    apply_config_to_panels(controls, hsv_panel, lab_panel, cmyk_panel, ycrcb_panel, lsd_panel, cfg)
    apply_lsd_to_panel(lsd_panel, cfg)

    cv.namedWindow("Frame", cv.WINDOW_NORMAL)
    cv.namedWindow("Mask", cv.WINDOW_NORMAL)
    cv.namedWindow("LSD", cv.WINDOW_NORMAL)

    idx_map = {0: "HSV", 1: "LAB", 2: "CMYK", 3: "YCrCb"}

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Camera feed lost.")
            break

        # Check if active space changed via Controls panel
        active_idx = cv.getTrackbarPos("Active (0=HSV 1=LAB 2=CMYK 3=YCrCb)", controls)
        target_space = idx_map.get(active_idx, "HSV")
        
        if target_space != cfg["active_space"]:
            ensure_active_panel(target_space, cfg["active_space"])
            cfg["active_space"] = target_space
        
        cfg = read_config_from_panels(controls, hsv_panel, lab_panel, cmyk_panel, ycrcb_panel, lsd_panel, cfg)
        mask = mask_from_space(frame, cfg)
        mask = postprocess_mask(mask, cfg)

        cv.putText(frame, f"Active: {cfg['active_space']}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv.imshow("Frame", frame)
        cv.imshow("Mask", mask)

        if cfg.get("lsd", {}).get("enabled", 0) == 1:
            overlay = detect_lsd_overlay(frame, mask, cfg)
            cv.imshow("LSD", overlay)

        if cv.getTrackbarPos("Save (toggle 1)", controls) == 1:
            save_config(json_path, cfg)
            cv.setTrackbarPos("Save (toggle 1)", controls, 0)
        if cv.getTrackbarPos("Reload (toggle 1)", controls) == 1:
            old_space = cfg["active_space"]
            cfg = load_config(json_path)
            new_space = cfg["active_space"]
            ensure_active_panel(new_space, old_space)
            apply_config_to_panels(controls, hsv_panel, lab_panel, cmyk_panel, ycrcb_panel, lsd_panel, cfg)
            cv.setTrackbarPos("Reload (toggle 1)", controls, 0)

        key = cv.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        elif key == ord("s"):
            save_config(json_path, cfg)
        elif key == ord("l"):
            old_space = cfg["active_space"]
            cfg = load_config(json_path)
            new_space = cfg["active_space"]
            ensure_active_panel(new_space, old_space)
            apply_config_to_panels(controls, hsv_panel, lab_panel, cmyk_panel, ycrcb_panel, lsd_panel, cfg)

    cap.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()

