# Remote Segmentation Setup

This document describes how to use the remote segmentation setup for the Summit XL robot. The remote setup allows you to run the heavy LangSAM model on a separate machine (e.g., a GPU server) while keeping the ROS node lightweight.

## Overview

The remote segmentation system consists of two components:

1. **LangSAM Server** ([server_ros.py](~/rap/lang-segment-anything/lang_sam/server_ros.py)): A LitServe-based HTTP server that runs the LangSAM model and provides segmentation via REST API
2. **Remote Segmentation Node** ([segmentation_node_remote.py](icclab_summit_xl/scripts/segmentation_node_remote.py)): A ROS 2 node that sends images to the server and publishes segmentation results

## Architecture

```
┌─────────────────────────┐          ┌──────────────────────────┐
│  ROS 2 Environment      │          │  LangSAM Server          │
│                         │  HTTP    │  (GPU Machine)           │
│  ┌───────────────────┐  │  POST    │  ┌────────────────────┐  │
│  │ Remote Seg Node   │──┼─────────>│  │  server_ros.py     │  │
│  │                   │  │ /predict │  │                    │  │
│  │ - RGB-D inputs    │  │          │  │  - LangSAM model   │  │
│  │ - Text prompts    │  │<─────────┤  │  - JSON endpoint   │  │
│  │ - Mask output     │  │   JSON   │  └────────────────────┘  │
│  │ - Point cloud     │  │          │                          │
│  └───────────────────┘  │          │  Port: 8001              │
└─────────────────────────┘          └──────────────────────────┘
```

## Setup

### 1. Start the LangSAM Server

On the machine with GPU (or locally for testing):

```bash
cd ~/rap/lang-segment-anything
python3 -m lang_sam.server_ros
```

**Important:** You must run the server using `python3 -m lang_sam.server_ros` from the root directory. Running it directly as `python3 lang_sam/server_ros.py` will cause module import errors.

The server will start on port 8001 by default and print:
```
Starting LangSAM ROS Server on port 8001...
Endpoints:
  - POST http://localhost:8001/predict (output_format=image for PNG)
  - POST http://localhost:8001/predict (output_format=json for JSON)
```

**Note:** The original server (with Gradio UI) can still be started with:
```bash
cd ~/rap/lang-segment-anything
python3 app.py
```
It runs on port 8000 and both servers can run simultaneously.

### 2. Build the ROS Package

If you just added the remote segmentation node:

```bash
cd ~/colcon_ws
colcon build --packages-select icclab_summit_xl
source install/setup.bash
```

### 3. Launch the Remote Segmentation Node

```bash
ros2 launch icclab_summit_xl segmentation_remote.launch.py
```

Or with custom server URL:

```bash
ros2 launch icclab_summit_xl segmentation_remote.launch.py server_url:=http://192.168.1.100:8001
```

## Configuration

### Launch Parameters

The remote segmentation node supports the following parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `server_url` | `http://localhost:8001` | URL of the LangSAM server |
| `server_timeout` | `30.0` | Request timeout in seconds |
| `sam_type` | `sam2.1_hiera_small` | SAM model type |
| `box_threshold` | `0.3` | Bounding box detection threshold |
| `text_threshold` | `0.25` | Text/phrase detection threshold |
| `rgb_topic` | `/arm_camera/color/image_raw` | RGB camera topic |
| `depth_topic` | `/arm_camera/depth/image_raw` | Depth camera topic |
| `camera_info_topic` | `/arm_camera/color/camera_info` | Camera info topic |
| `voxel_size` | `0.002` | Voxel downsampling size (meters) |
| `remove_outliers` | `True` | Enable statistical outlier removal |

### Server Configuration

Edit [server_ros.py](~/rap/lang-segment-anything/lang_sam/server_ros.py) to change:
- `PORT`: Server port (default: 8001)
- Initial SAM model type in `setup()` method

## Usage

### Publishing Segmentation Requests

The remote segmentation node subscribes to text prompts on `/segment_text`:

```bash
# Segment all cups in the image
ros2 topic pub --once /segment_text std_msgs/msg/String "data: 'cup'"

# Segment multiple objects
ros2 topic pub --once /segment_text std_msgs/msg/String "data: 'apple. orange. banana.'"
```

### Receiving Results

The node publishes:

1. **Segmentation Mask** (`/segmentation_mask`): Binary mask as `sensor_msgs/Image` (mono8)
2. **Point Cloud** (`/segmented_pointcloud`): Filtered 3D point cloud as `sensor_msgs/PointCloud2`
3. **Status** (`/segmentation_status`): Status messages as `std_msgs/String`

Example subscriber:

```bash
# View mask
ros2 topic echo /segmentation_mask

# View point cloud in RViz
rviz2
# Add PointCloud2 display, topic: /segmented_pointcloud
```

## API Reference

### Server Endpoint: POST /predict

**Request (multipart/form-data):**
- `image`: Image file (PNG, JPEG, etc.)
- `text_prompt`: Text description of objects to segment
- `sam_type`: (optional) SAM model type
- `box_threshold`: (optional) Box threshold (0.0-1.0)
- `text_threshold`: (optional) Text threshold (0.0-1.0)
- `output_format`: `"json"` for raw data, `"image"` for PNG

**Response (JSON when output_format=json):**
```json
{
  "masks": [
    {
      "data": "base64-encoded-mask-data",
      "shape": [height, width],
      "dtype": "uint8"
    }
  ],
  "boxes": [[x1, y1, x2, y2], ...],
  "scores": [0.95, 0.87, ...],
  "labels": ["cup", "cup", ...],
  "image_shape": [height, width, 3]
}
```

## Troubleshooting

### Server Connection Issues

**Problem:** `ERROR: Connection failed`

**Solutions:**
- Verify server is running: `curl http://localhost:8001/predict`
- Check firewall settings if using remote server
- Ensure correct URL in launch parameters

### Timeout Errors

**Problem:** `ERROR: Server timeout`

**Solutions:**
- Increase `server_timeout` parameter (first inference is slow due to model loading)
- Check server logs for errors
- Verify GPU availability on server

### No Objects Found

**Problem:** `NO_OBJECTS_FOUND` status

**Solutions:**
- Adjust `box_threshold` (lower = more detections)
- Adjust `text_threshold` (lower = less strict text matching)
- Improve text prompt specificity
- Check image quality and lighting

### Empty Point Cloud

**Problem:** Point cloud has 0 points after filtering

**Solutions:**
- Check depth image alignment with RGB
- Verify camera intrinsics are being received
- Disable outlier removal: `remove_outliers:=False`
- Increase `voxel_size` for less aggressive downsampling

## Comparison: Local vs Remote

| Aspect | Local Segmentation | Remote Segmentation |
|--------|-------------------|---------------------|
| Model Location | On ROS machine | On separate server |
| Dependencies | Full LangSAM install | Only requests + numpy |
| GPU Required | Yes (or slow CPU) | No (on ROS machine) |
| Network | Not needed | Required (LAN/WAN) |
| Latency | Lower | Higher (network overhead) |
| Scalability | Single machine | Multiple robots → one server |
| Setup Complexity | Simple | Moderate |

## Performance Notes

- **First request**: Slow (~10-30s) due to model initialization on server
- **Subsequent requests**: Faster (~1-5s depending on image size and network)
- **Network bandwidth**: ~1-5 MB per request (depends on image resolution)
- **Recommended network**: Gigabit LAN for best performance

## Development

### Testing the Server Locally

You can test the server using curl:

```bash
# Test with an image
curl -X POST http://localhost:8001/predict \
  -F "image=@test_image.jpg" \
  -F "text_prompt=cup" \
  -F "output_format=json" \
  -F "box_threshold=0.3" \
  -F "text_threshold=0.25"
```

### Modifying the API

To add new features to the server:

1. Edit [server_ros.py](~/rap/lang-segment-anything/lang_sam/server_ros.py)
2. Modify the `decode_request()`, `predict()`, or `encode_response()` methods
3. Restart the server
4. Update the client in [segmentation_node_remote.py](icclab_summit_xl/scripts/segmentation_node_remote.py)

## See Also

- Original segmentation node: [segmentation_node.py](icclab_summit_xl/scripts/segmentation_node.py)
- Original LangSAM server: [~/rap/lang-segment-anything/lang_sam/server.py](~/rap/lang-segment-anything/lang_sam/server.py)
- LangSAM documentation: [~/rap/lang-segment-anything/README.md](~/rap/lang-segment-anything/README.md)
