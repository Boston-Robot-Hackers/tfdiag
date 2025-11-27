# TF Publisher Detection Implementation Notes

## Overview

The tfdiag tool provides two implementations for detecting which nodes publish which TF frames:

1. **Original Implementation** ([tf_lister.py](tfdiag/tf_lister.py)) - Topic-based approximation
2. **Accurate Implementation** ([tf_lister_accurate.py](tfdiag/tf_lister_accurate.py)) - Enhanced detection using endpoint information

## Usage

```bash
# Use original implementation (default)
ros2 run tfdiag tfdiag list

# Use accurate implementation
ros2 run tfdiag tfdiag list --accurate
```

## Implementation Comparison

### Original Implementation (tf_lister.py)

**How it works:**
1. Subscribes to `/tf` and `/tf_static` topics
2. Tracks which frames appear on which topic
3. Queries all nodes publishing to each topic
4. Associates ALL nodes on a topic with ALL frames on that topic

**Limitations:**
- Cannot distinguish which specific node publishes which specific frame
- If multiple nodes publish to `/tf`, all frames are attributed to all nodes
- Results in an approximation when multiple publishers exist

**Example:**
```
If nodes A, B, C all publish to /tf:
- Frame "base_link" appears on /tf → attributed to A, B, C
- Frame "camera" appears on /tf → attributed to A, B, C
- Frame "lidar" appears on /tf → attributed to A, B, C

Even if A only publishes base_link, B only publishes camera, etc.
```

### Accurate Implementation (tf_lister_accurate.py)

**How it works:**
1. Subscribes to `/tf` and `/tf_static` topics
2. Tracks which frames appear on which topic
3. Uses `endpoint_gid` (Global ID) information from publisher metadata
4. Builds a GID-to-node mapping for more precise attribution

**Improvements:**
- Attempts to use endpoint GID information when available
- Better handling of publisher metadata
- More structured approach to correlation

**Current Status:**
This implementation provides the same results as the original for now, but has the infrastructure to leverage GID-based tracking when message_info becomes available in the callback. This is a framework for future enhancement.

## Technical Details

### Why GID-based tracking is difficult in rclpy

ROS2's rclpy library doesn't directly expose message_info (which contains the publisher GID) in regular subscription callbacks. To get true per-message publisher identification would require:

1. Using `raw=True` subscriptions (not fully supported in rclpy)
2. Using middleware-specific APIs (DDS introspection)
3. Creating custom subscription implementations

### Future Enhancements

To achieve true per-frame publisher detection, potential approaches include:

1. **Use ros2cli introspection tools** - Parse output from `ros2 topic info /tf --verbose`
2. **DDS introspection** - Query DDS layer directly for participant information
3. **Custom middleware hooks** - Implement custom QoS event handlers
4. **TF2 graph analysis** - Analyze the TF tree structure and timing to infer publishers

## Recommendation

For now, both implementations provide similar results. The `--accurate` flag is provided for:
1. Future enhancements to leverage GID-based tracking
2. Easy A/B comparison when testing new detection methods
3. A clean separation allowing easy rollback if needed

## Rollback

To remove the accurate implementation:
1. Delete [tfdiag/tf_lister_accurate.py](tfdiag/tf_lister_accurate.py)
2. Remove the import and flag from [tfdiag/main.py](tfdiag/main.py)
3. Revert the `list()` function to its original form

## References

- [ROS2 rclpy Subscription API](https://docs.ros.org/en/rolling/p/rclpy/api/topics.html)
- [DDS Security and QoS](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html)
- [TF2 Design](https://docs.ros.org/en/rolling/Concepts/About-Tf2.html)
