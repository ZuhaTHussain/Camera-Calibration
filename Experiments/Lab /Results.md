# Camera Calibration Results

---

# Intrinsic Calibration

**Board:** 6×4 inner corners, square size = 40 mm  
**Accepted Frames:** 25 / 25 images  

---

## INTRINSIC CALIBRATION RESULTS

**RMS reprojection error:** 0.5177 px  
**Image size:** 1080 × 1920 px  

### Intrinsic Matrix (K)

fx = 880.78 px  
fy = 883.61 px  
cx = 540.83 px  
cy = 962.82 px  

K =
    [  880.78      0.00    540.83 ]
    [    0.00    883.61    962.82 ]
    [    0.00      0.00      1.00 ]

---

### Distortion Coefficients

k1 =  0.031441  
k2 = -0.098710  
p1 =  0.002845  
p2 = -0.001201  
k3 =  0.082448  

---

### Per-image Reprojection Errors

frame_00003_GX010391_000030.jpg  0.1079 px  
frame_00025_GX010391_000250.jpg  0.1128 px  
frame_00053_GX010391_000530.jpg  0.1136 px  
frame_00055_GX010391_000550.jpg  0.1228 px  
frame_00057_GX010391_000570.jpg  0.1128 px  
frame_00084_GX010391_000260.jpg  0.0906 px  
frame_00085_GX010391_000270.jpg  0.0974 px  
frame_00086_GX010391_000280.jpg  0.1131 px  
frame_00095_GX010391_000370.jpg  0.1226 px  
frame_00102_GX010391_000440.jpg  0.0716 px  
frame_00127_GX010392_000110.jpg  0.1274 px  
frame_00155_GX010392_000390.jpg  0.0900 px  
frame_00159_GX010392_000430.jpg  0.0910 px  
frame_00160_GX010392_000440.jpg  0.0850 px  
frame_00171_GX010392_000550.jpg  0.0992 px  
frame_00178_GX010392_000620.jpg  0.0920 px  
frame_00194_GX010392_000780.jpg  0.1141 px  
frame_00197_GX010392_000810.jpg  0.1300 px  
frame_00239_GX010392_000270.jpg  0.1539 px  
frame_00254_GX010392_000420.jpg  0.0871 px  
frame_00276_GX010392_000640.jpg  0.1089 px  
frame_00288_GX010392_000760.jpg  0.1060 px  
frame_00305_GX010392_000930.jpg  0.0819 px  
frame_00306_GX010392_000940.jpg  0.0869 px  
frame_00307_GX010392_000950.jpg  0.0825 px  

Best:  0.0716 px  
Worst: 0.1539 px  
Mean:  0.1040 px  

---

# Extrinsic Calibration

Loaded intrinsics from Stage 1  
fx=880.78  fy=883.61  cx=540.83  cy=962.82  

Image size: 1080 × 1920 px  
Detected 6×4 = 24 corners ✓  

---

## EXTRINSIC CALIBRATION RESULTS

### Rotation Matrix (R)

[  0.119700    0.992665    0.016949 ]  
[  0.048481   -0.022896    0.998562 ]  
[  0.991626   -0.118706   -0.050866 ]  

---

### Translation Vector (t)

[-0.032861, 0.626360, 0.791637]

---

### Camera Centre (World Coordinates)

X = -0.8114 m  
Y =  0.1409 m  
Z = -0.5846 m  

Camera height above road: 0.5846 m  

---

## Ground Plane Homography (Pixels → Ground)

[  810.639794   1023.345383    504.271745 ]  
[ 1260.164660   -169.930193   1661.949285 ]  
[    1.252626     -0.149950      1.000000 ]  

---

## Inverse Homography (Ground → Pixels)

[ -0.000056   0.000770  -1.251590 ]  
[ -0.000576  -0.000125   0.498675 ]  
[ -0.000017  -0.000983   1.000000 ]  

---

## Homography Reprojection Check

RMS error: 0.3547 px  
Max error: 0.6487 px  
Min error: 0.0684 px  

✓ Homography looks good  

---

## Local Scale

829.26 px/m  
(1 pixel ≈ 1.21 mm at this location)
