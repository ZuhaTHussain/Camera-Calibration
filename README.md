# Camera Calibration Pipeline – Setup & Usage

## 1. Installation

Install Python 3.8+ and the required package:

```bash
pip install opencv-python numpy
```

## How to run the scripts
### Intrinsic calibration
```bash
python intrinsic_calibration.py \
    --image_dir <path_to_calibration_images> \
    --square_size <square_size_in_meters> \
    --cols <number_of_inner_corners_per_row> \
    --rows <number_of_inner_corners_per_column> \
```
Optional: 
1. add --show to preview detected corner
2.  --output <filename.npz> (default: calibration_intrinsics.npz)

#### Example:
--square_size 0.025 for 25 mm squares, --cols 10 --rows 7 for an 11×8 board.

### Exttrinsic calibration
```bash
python extrinsic_calibration.py \
    --image <path_to_road_board_image> \
    --intrinsics <intrinsics_file.npz> \
    --square_size <square_size_in_meters> \
    --cols <number_of_inner_corners_per_row> \
    --rows <number_of_inner_corners_per_column> \
```
Optional: 
1. add --show to see the board with world axes overlaid
2.  --output <filename.npz> (default: calibration_intrinsics.npz)

#### Example:
--square_size 0.04 for 40 mm squares, --cols 6 --rows 4 for a 7×5 board.
