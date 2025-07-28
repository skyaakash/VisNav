# VisNav

This repository contains various navigation and image-processing utilities.

## Creating a Video from Images

The `create_video.py` script can be used to generate an MP4 video from a
sequence of still images contained in a directory.

### Usage

```bash
python create_video.py <image_directory> <output_video.mp4> [--fps FPS]
```

- `<image_directory>` should point to a folder containing image files.
- `<output_video.mp4>` is the path to the resulting video file.
- `--fps` (optional) sets the frames per second of the generated video.

Supported image formats are PNG, JPG, JPEG, and BMP.
