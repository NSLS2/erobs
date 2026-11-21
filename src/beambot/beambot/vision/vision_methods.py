"""Marker, sample-ROI and spincoater detection adapters for vision tasks."""


def detect_marker(ctx):
    """Return a cached or detected marker pose in base_link, or None."""
    vision = ctx.vision
    goal = ctx.goal

    cached = vision.get_cached_pose(goal.tag_id)
    if cached is not None:
        pos = cached.pose.position
        vision.logger.info(
            f"Using cached pose for tag {goal.tag_id}: "
            f"[{pos.x * 1000:.2f}, {pos.y * 1000:.2f}, {pos.z * 1000:.2f}] mm"
        )
        return cached

    if ctx.scan_positions is not None:
        return vision.detect_tag_multiposition(
            tag_id=goal.tag_id,
            scan_positions=ctx.scan_positions,
            timeout=goal.timeout,
            settle_time=vision._settle_time,
        )

    return vision.detect_and_transform_tag(goal.tag_id, goal.timeout)


def detect_sample_roi(ctx):
    """Return a sample pickup pose in base_link from a marker-defined ROI, or None."""
    vision = ctx.vision
    goal = ctx.goal
    strategy = goal.strategy or "farthest_edge"
    # Preserve zero as a valid inset.
    edge_inset_mm = goal.edge_inset_mm
    vision.logger.info(
        f"Using sample_roi detection (tag {goal.tag_id}, "
        f"strategy={strategy}, inset={edge_inset_mm}mm)"
    )
    return vision.detect_and_transform_sample_roi(
        tag_id=goal.tag_id,
        strategy=strategy,
        edge_inset_mm=edge_inset_mm,
        timeout=goal.timeout,
    )


def _capture_2d_for_spincoater(ctx):
    """Capture a flash-lit Zivid BGR image with a 15-second timeout."""
    from beambot.vision.camera.zivid import capture_2d

    ctx.vision.logger.info("spincoater: capturing 2D image...")
    return capture_2d(ctx.vision.rclpy_node, timeout=15.0)


def detect_spincoater_pocket(ctx):
    """Return pocket geometry and angle as a dictionary, or None."""
    from beambot.vision.perception import detect_spincoater_pocket as _detect

    image = _capture_2d_for_spincoater(ctx)
    if image is None:
        ctx.vision.logger.error("spincoater_pocket: 2D capture failed")
        return None
    detection = _detect(image)
    if detection is not None:
        ctx.vision.logger.info(
            f"pocket detected — angle_mod90={detection['angle_mod90']:.1f}°, "
            f"aspect={detection['aspect']:.2f}, solidity={detection['solidity']:.2f}"
        )
    return detection


def detect_spincoater_sample(ctx):
    """Return sample geometry, angle and confidence as a dictionary, or None."""
    from beambot.vision.perception import detect_spincoater_sample as _detect

    image = _capture_2d_for_spincoater(ctx)
    if image is None:
        ctx.vision.logger.error("spincoater_sample: 2D capture failed")
        return None
    detection = _detect(image)
    if detection is not None:
        ctx.vision.logger.info(
            f"sample detected — angle_mod90={detection['angle_mod90']:.1f}°, "
            f"confidence={detection['confidence']:.2f}, center={detection['center_px']}"
        )
    return detection


VISION_METHODS = {
    "marker": detect_marker,
    "sample_roi": detect_sample_roi,
    "spincoater_pocket": detect_spincoater_pocket,
    "spincoater_sample": detect_spincoater_sample,
}


def get_vision_method(name: str):
    """Return a vision_method or raise KeyError listing the available names."""
    try:
        return VISION_METHODS[name]
    except KeyError:
        raise KeyError(
            f"unknown vision_method '{name}'. Registered: {sorted(VISION_METHODS)}"
        ) from None
