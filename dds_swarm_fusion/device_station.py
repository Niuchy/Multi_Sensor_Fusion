"""Device installation coordinates, never target or remote-controller GPS.

The explicit field map follows dev_proto.h. RadarTarget and ESMTarget currently
do not carry receiver installation GPS; no guessing from their target fields.
"""
import math


DEVICE_GPS_FIELDS = {
    "eo": ("ptz_lng_deg", "ptz_lat_deg", "ptz_alt_m"),
    "acoustic": ("device_lng_deg", "device_lat_deg", "device_alt_m"),
}
SOURCE_LABELS = {"reported_lla": "报文设备自身 GPS",
                 "configured_lla": "config 经纬高",
                 "configured_enu": "config ENU 换算经纬高",
                 "unknown": "站址未知"}
REASON_LABELS = {"valid_device_gps": "报文设备 GPS 有效",
                 "protocol_has_no_device_gps": "当前协议无设备自身 GPS 字段",
                 "device_gps_missing": "报文缺少设备自身 GPS",
                 "device_gps_invalid": "报文设备 GPS 无效（零坐标、非有限数或越界）"}


def vector3(value):
    try:
        if len(value) != 3 or any(isinstance(x, bool) for x in value):
            return None
        result = [float(x) for x in value]
        return result if all(math.isfinite(x) for x in result) else None
    except (TypeError, ValueError, OverflowError):
        return None


def valid_lla(value):
    p = vector3(value)
    return (p is not None and -180 <= p[0] <= 180 and -90 <= p[1] <= 90
            and (p[0] != 0 or p[1] != 0))


def enu_to_lla(converter, position):
    """Inverse of this project's small-area converter, not a GPS measurement."""
    if abs(converter.meters_per_deg_lng) < 1e-6:
        raise ValueError("Cannot convert configured ENU station at a polar origin")
    x, y, z = position
    return [converter.lng+x/converter.meters_per_deg_lng,
            converter.lat+y/converter.METERS_PER_DEG_LAT, converter.alt+z]


def select_station(converter, kind, obj, *, config_enu=None, config_lla=None,
                   config_label=None):
    """Select GPS from THIS packet first, otherwise the immutable configuration.

    No previous packet's station is retained as a silent fallback. The returned
    metadata travels with the same station used by input/tracking/fusion.
    """
    fields = DEVICE_GPS_FIELDS.get(kind)
    reported = [getattr(obj, k, None) for k in fields] if fields else None
    reason = ("protocol_has_no_device_gps" if fields is None else
              "device_gps_missing" if any(x is None for x in reported) else
              "device_gps_invalid")
    out = dict(station_source="unknown", station_known=False, station_lla=None,
               station_enu=None, station_config=None, station_reason=reason,
               station_report_fields=list(fields or ()),
               station_reference_lla=[converter.lng, converter.lat, converter.alt])
    if valid_lla(reported):
        lla = vector3(reported)
        out.update(station_source="reported_lla", station_known=True,
                   station_lla=lla, station_enu=list(converter.to_enu(*lla)),
                   station_reason="valid_device_gps")
        return out
    if config_lla is not None:
        if not valid_lla(config_lla):
            raise ValueError(f"Invalid configured device GPS: {config_label}")
        lla = vector3(config_lla)
        out.update(station_source="configured_lla", station_known=True,
                   station_lla=lla, station_enu=list(converter.to_enu(*lla)),
                   station_config=config_label)
    elif config_enu is not None:
        enu = vector3(config_enu)
        if enu is None:
            raise ValueError(f"Invalid configured device ENU: {config_label}")
        lla = enu_to_lla(converter, enu)
        if not all(math.isfinite(x) for x in lla) or not (-180 <= lla[0] <= 180 and -90 <= lla[1] <= 90):
            raise ValueError(f"Configured ENU converts to invalid GPS: {config_label}")
        out.update(station_source="configured_enu", station_known=True,
                   station_lla=lla, station_enu=enu, station_config=config_label)
    return out
