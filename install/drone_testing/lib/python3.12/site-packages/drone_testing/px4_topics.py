"""Version-agnostic subscription to PX4 uXRCE-DDS topics.

PX4 publishes each uORB topic under whatever name the flight controller's
``dds_topics.yaml`` gives it. On message-versioned builds that name carries a
``_vN`` suffix (``/fmu/out/vehicle_local_position_v1``); on others it does not
(``/fmu/out/vehicle_local_position``). The two are decided per topic, so a
single firmware can -- and this airframe's currently does -- publish
``vehicle_status_v1`` next to an unversioned ``vehicle_local_position``.

Hard-coding either spelling fails *silently*: rclpy accepts a subscription to
a topic nobody publishes, so the callback simply never fires and the node sits
waiting for a position that is right there on the bus under another name.

``subscribe_versioned`` subscribes to both spellings and lets whichever one
exists deliver. The callbacks in this package all just store the latest
message, so the duplicate subscription costs one unused endpoint and nothing
else. It keeps working across a reflash, which is the point.
"""


def versioned_names(base, max_version=4):
    """Every plausible /fmu/out spelling of ``base``, unversioned first."""
    return [f'/fmu/out/{base}'] + [
        f'/fmu/out/{base}_v{n}' for n in range(1, max_version + 1)
    ]


def subscribe_versioned(node, msg_type, base, callback, qos, max_version=1):
    """Subscribe to /fmu/out/<base> and /fmu/out/<base>_vN alike.

    Returns the list of subscriptions; keep a reference to it or rclpy will
    garbage-collect them. ``max_version`` defaults to 1 because that is the
    only suffix PX4 has actually shipped -- raise it if that changes.
    """
    return [
        node.create_subscription(msg_type, name, callback, qos_profile=qos)
        for name in versioned_names(base, max_version)
    ]
