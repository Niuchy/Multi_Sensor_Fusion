"""Exact, per-device RF aircraft-model admission; no geometry or DDS I/O."""
import math
import os
from pathlib import Path

import yaml


DEFAULTS = dict(enabled=True, drone_models=[], min_confidence=.5,
                confirm_reports=3, max_gap_ms=5000, max_age_ms=12000)


def model_key(value):
    return str(value or "").strip(" \t\r\n\0").casefold()


class RFClassification:
    def __init__(self, root, config_path=None):
        self.path = Path(config_path or os.environ.get(
            "SB_RF_CONFIG", str(Path(root) / "configs/rf_devices.yaml"))).resolve()
        # An absent default is safe (empty allowlist); explicit bad paths fail fast.
        explicit = config_path is not None or "SB_RF_CONFIG" in os.environ
        if explicit and not self.path.is_file():
            raise ValueError(f"RF classification config missing: {self.path}")
        data = yaml.safe_load(self.path.read_text(encoding="utf-8")) if self.path.is_file() else {}
        data = data or {}
        if not isinstance(data, dict):
            raise ValueError("RF configuration must be a YAML mapping")
        self.defaults = {**DEFAULTS, **data.get("defaults", {})}
        self.devices = {str(k): v for k, v in data.get("devices", {}).items()}
        for device in [None, *self.devices]:
            p = self.profile(device)
            if not isinstance(p["enabled"], bool):
                raise ValueError("RF enabled must be YAML true/false")
            models = p["drone_models"]
            if not isinstance(models, list) or any(
                    not isinstance(m, str) or not model_key(m) or "*" in m for m in models):
                raise ValueError("RF drone_models must be an explicit list of full model names")
            confidence = p["min_confidence"]
            if (isinstance(confidence, bool) or not isinstance(confidence, (int, float))
                    or not math.isfinite(confidence) or not 0 < confidence <= 1):
                raise ValueError("RF min_confidence must be within (0, 1]")
            for key, low, high in (("confirm_reports", 2, 20), ("max_gap_ms", 100, 60000),
                                   ("max_age_ms", 100, 60000)):
                if type(p[key]) is not int or not low <= p[key] <= high:
                    raise ValueError(f"Invalid RF {key}")
            if p["max_age_ms"] < p["max_gap_ms"]:
                raise ValueError("RF max_age_ms must be >= max_gap_ms")

    def profile(self, device_id):
        return {**self.defaults, **self.devices.get(str(device_id), {})}

    def classify(self, obj):
        p = self.profile(obj.device_id)
        model = str(obj.brand_model or "").strip(" \t\r\n\0")
        key = model_key(model)
        try:
            confidence = float(obj.confidence) / 100.0
        except (TypeError, ValueError, AttributeError):
            confidence = None
        if confidence is not None and (not math.isfinite(confidence) or not 0 <= confidence <= 1):
            confidence = None
        if not p["enabled"]:
            reason = "disabled"
        elif not key:
            reason = "missing_model"
        elif "control" in key or "遥控" in key:
            reason = "controller_not_aircraft"
        elif len(model.encode("utf-8")) >= 32:
            reason = "model_may_be_truncated"
        elif key not in {model_key(m) for m in p["drone_models"]}:
            reason = "model_not_allowlisted"
        elif confidence is None or confidence < p["min_confidence"]:
            reason = "invalid_or_low_confidence"
        else:
            reason = "explicit_aircraft_model"
        valid = reason == "explicit_aircraft_model"
        return dict(valid=valid, cls="drone" if valid else "unknown", reason=reason,
                    model=model, confidence=confidence, device_id=int(obj.device_id),
                    source_track_id=int(obj.target_id),
                    timestamp_ms=getattr(obj, "timestamp_ms", None),
                    confirm_reports=p["confirm_reports"], max_gap_ms=p["max_gap_ms"],
                    max_age_ms=p["max_age_ms"])
