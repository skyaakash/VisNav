import os
import cv2


def images_to_video(image_dir, output_path, fps=30):
    """Create a video file from a directory of images.

    Parameters
    ----------
    image_dir : str
        Path to directory containing images. Images are read in sorted order.
    output_path : str
        Path to the output video file.
    fps : int, optional
        Frames per second of the output video, by default 30.
    """
    images = sorted([
        os.path.join(image_dir, img)
        for img in os.listdir(image_dir)
        if img.lower().endswith((".png", ".jpg", ".jpeg", ".bmp"))
    ])
    if not images:
        raise ValueError(f"No images found in {image_dir}")

    first_frame = cv2.imread(images[0])
    height, width, _ = first_frame.shape
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    for img_path in images:
        frame = cv2.imread(img_path)
        if frame is None:
            raise ValueError(f"Failed to read {img_path}")
        writer.write(frame)
    writer.release()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Generate a video from images")
    parser.add_argument("image_dir", help="Directory with input images")
    parser.add_argument("output", help="Path to output video file")
    parser.add_argument("--fps", type=int, default=30, help="Frames per second")
    args = parser.parse_args()

    images_to_video(args.image_dir, args.output, fps=args.fps)
