# Camera Calibration Pipeline

The repository here implements a two-stage camera calibration pipeline
for metric measurements (in meters) from the image of a road scene.

The two main parts of the pipeline are:

1.  Intrinsic calibration (estimates the camera's internal parameters)

2.  Extrinsic calibration + Homography (estimates the camera's pose and
    maps the image pixels from image coordinate frame to world
    coordinate frame)

We used OpenCV for implementation and Zhang's method (and optionally
Perspective‑n‑Point (PnP) for non‑planar points) are used.

## Intrinsic Calibration

The goal is to estimate the intrinsic parameters of the camera which
include focal lengths and principal point and the distortion coefficients
(though the basic model here assumes no distortion; radial and tangential
distortion can be added later). This shows us how the camera forms an image.

$$K = \begin{bmatrix}
f_x & 0 & O_x \\
0 & f_y & O_y \\
0 & 0 & 1
\end{bmatrix}$$

K here is the internal camera matrix or intrinsics matrix where

a.  $(f_x,f_y)$ are the focal lengths in pixels in x and y direction. The
    camera only has one effective focal length but in order to
    accommodate for non‑equal pixel densities in x and y direction, we
    have $f_x$ and $f_y$.

b.  $(O_x,O_y)$ is the principal point

$(f_x, f_y, O_x, O_y)$ are the internal parameters of the camera.

The full camera projection model is

$$x = K[R|t] X_w$$

where,

a.  $X_w = (X, Y, Z)$ in the world coordinate frame

b.  Here, $x = (u,v)$ in pixels in the image coordinate frame

c.  $K$ is the intrinsic matrix

d.  $R$ is the rotation matrix, and $t$ is the translation vector; both are
    the extrinsic parameters

The extrinsic parameters are the position and orientation of the camera in
the world coordinate frame.

$$X_c = R X_w + t$$

Where $X_c$ is the position of a point in camera coordinate frame and $X_w$
is the position of point in world coordinate frame.

The reason we are recovering these parameters here is only to build
constraints on $K$.

To get a linear model, we will convert to homogeneous coordinates. This
also helps in scaling and normalization.

We take multiple images of an object of known geometry; a checkerboard to
identify the corresponding points between the 3D scene and the image
points to get the parameters that best project the scene. This is done
using Zhang's method.

To estimate $K$, we use multiple images of an object of known geometry; in
this case a checkerboard. Through this, we identify correspondences
between the 3D scenes and the image points. For each corresponding point
$i$ in scene and image, we form equations to map them.

Known 3D points $X_{ij}$ are mapped to detected image points $x_{ij}$.

Our assumption here is that the points are on a plane, $Z=0$. We have
reduced the 3D point to a 2D plane.

Homography is a $3 \times 3$ matrix that maps one plane to another plane. So we
can map the 2D image plane to the 2D world plane through a homography matrix.

So, $x \sim H X_w$

$X_w = (X, Y, 1)$ is the real world plane and $x = (u, v, 1)$ is the image plane and
$H$ is the Homography matrix.

To get the Homography matrix, we know that

Image plane $x \sim K[R|t] X_w$

As $Z=0$,

$x \sim K [r_1 \; r_2 \; t] \begin{bmatrix} X \\ Y \\ 1 \end{bmatrix}$

We can represent the Homography matrix as

$$H = K [r_1 \; r_2 \; t]$$ which we can say is [intrinsics] and [pose of the
plane in camera]

Thus, $x \sim H X_w$

This intuitively means that a 3D plane on a flat surface maps to an
image point using a single matrix $H$.

But we get one matrix for every image, and we want shared parameters
across all images. Each homography provides constraints on the intrinsic
matrix $K$, rather than directly giving it.

$$H_i = K \cdot (\text{pose}_i)$$

So, the pose changes per image and $K$ stays constant. This gives us
constraints on $K$.

Each $H_i$ contains $\begin{bmatrix} h_1 & h_2 & h_3 \end{bmatrix}$

Theoretically, $h_1 = K r_1$, $h_2 = K r_2$ and $h_3 = K r_3$ (but note that
$r_3$ is not directly used because $Z=0$).

So, $K^{-1} h_1 = r_1$ and so on.

And as the rotation matrix $R$ is orthonormal, its vectors must be
orthonormal to each other, which means that $r_1 \cdot r_2 = 0$ and
$\|r_1\| = \|r_2\| = 1$.

Using this, we get two constraints for each homography:

$$(K^{-1} h_1) \cdot (K^{-1} h_2) = 0$$
$$\|K^{-1} h_1\|^2 = \|K^{-1} h_2\|^2$$

These will be the constraint equations for $K$. With multiple images we build a
system and we can solve for $K$.

Now, moving to the projection matrix, we know that

Mapping world to camera coordinates: $X_c = R X_w + t$

And mapping camera coordinates to image frame: $x = K X_c$

Where $x = (u,v)$ and in homogeneous coordinates,

We have $u = \frac{x}{z}$ and $v = \frac{y}{z}$ and our projection
function becomes

$$P = K(R X_w + t)$$ which gives us a full mapping from 3D to 2D.

Now, connecting everything,

As $Z=0$, our projection collapses into a single matrix: $P = H X_w$

Now, how do we get these matrices?

For each calibration image, we have 3D planar points $X_w = (X, Y, 1)$ and
corresponding image points $x = (u, v, 1)$.

We will use Direct Linear Transform (DLT) to solve for the equations; each
point gives us two equations so with more than or equal to 4 points, we can
solve for $H$. $H$ has 8 degrees of freedom (9 entries up to scale).

$$\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \begin{bmatrix}
h_{11} & h_{12} & h_{13} \\
h_{21} & h_{22} & h_{23} \\
h_{31} & h_{32} & h_{33}
\end{bmatrix} \cdot \begin{bmatrix} X \\ Y \\ 1 \end{bmatrix}$$

$$u = \frac{h_1^T X}{h_3^T X} \quad \text{and} \quad v = \frac{h_2^T X}{h_3^T X}$$

And we get two equations

$$h_1^T X - u\,h_3^T X = 0$$
$$h_2^T X - v\,h_3^T X = 0$$

Expanded:

$$X \cdot h_{11} + Y \cdot h_{12} + h_{13} - u\,(X \cdot h_{31} + Y \cdot h_{32} + h_{33}) = 0$$
$$X \cdot h_{21} + Y \cdot h_{22} + h_{23} - v\,(X \cdot h_{31} + Y \cdot h_{32} + h_{33}) = 0$$

In matrix form, it is $A h = 0$ where $h$ is a vector of 9 unknowns and each
point corresponds to 2 rows in the matrix. Using singular value decomposition
and at least 4 known points (8 equations), we can solve for the 9 unknowns
up to scale. The eigenvector corresponding to the smallest singular value
gives us $h$, and reshaping it will give us $H$, a $3 \times 3$ matrix.

Now that we have $H$, we solve for $K$ using the two constraint equations above.

After we get $K$, we solve for $R$ and $t$. But note: $H$ is only known up to
scale, so we must first normalise.

Let $\widetilde{H}_i$ be the homography obtained from DLT. Then compute:

$$\begin{bmatrix} \widetilde{r}_1 & \widetilde{r}_2 & \widetilde{t} \end{bmatrix} = K^{-1} \widetilde{H}_i$$

Find the scale factor $\lambda = \frac{1}{\|\widetilde{r}_1\|}$ (or use the
average of $\|\widetilde{r}_1\|$ and $\|\widetilde{r}_2\|$). Then:

$$r_1 = \lambda \widetilde{r}_1, \quad r_2 = \lambda \widetilde{r}_2, \quad t = \lambda \widetilde{t}$$

To get the third axis, $r_3 = r_1 \times r_2$.

The full rotation matrix is $R = [r_1 \; r_2 \; r_3]$. In practice, we may need
to enforce orthonormality via SVD, but for most purposes this works.

The translation vector is $t_i = \lambda K^{-1} h_3$.

So, for each image we get $R_i$ and $t_i$, and we have completed the pipeline
to get the intrinsic parameters.

## Extrinsic Calibration

We use real road images for this experiment. The checkerboard is placed on
the ground and we already know $K$ from the intrinsic calibration.

Now, we solve for $R$ and $t$ using the planar homography formulation.

We want to know where the camera is relative to the road plane. We want to
convert image pixels here to real world coordinates (in meters).

Our camera model is $x \sim K [R \mid t] X_w$

And as the checkerboard is lying flat on the ground, $Z=0$ so every point
becomes

$$
X_w = \begin{bmatrix} X \\ Y \\ 1 \end{bmatrix}
$$

We have reduced 3D to a 2D plane.

Now, $x \sim K [r_1 \; r_2 \; t] \begin{bmatrix} X \\ Y \\ 1 \end{bmatrix}$ and $r_3$ is not needed because $Z=0$.

The homography matrix $H = K [r_1 \; r_2 \; t]$ so $x \sim H X_w$.

Here we know $X_w$ (the real-world checkerboard corners in meters) and $x$ (the
detected image corners in pixels). Using DLT, we will solve for $H$.

Now, $H = K [r_1 \; r_2 \; t]$ so $K^{-1} H = [r_1 \; r_2 \; t]$ but this is only true up to an
unknown scale factor $\lambda$. Let's call the unscaled matrix

$$
[\,\tilde{r}_1 \;\; \tilde{r}_2 \;\; \tilde{t}\,] = K^{-1} H
$$

Because the homography $H$ is defined only up to scale, the columns $\tilde{r}_1,\ \tilde{r}_2,\ \tilde{t}$ are also scaled. We need to recover the true $r_1, r_2, t$.

Since $r_1$ and $r_2$ are unit vectors (part of a rotation matrix), we can find $\lambda$ from the first column:

$$
\lambda = \frac{1}{\| \tilde{r}_1 \|}
$$

Then the true values are:

$$
r_1 = \lambda \,\tilde{r}_1, \qquad r_2 = \lambda \,\tilde{r}_2, \qquad t = \lambda \,\tilde{t}
$$

Now we have two orthonormal vectors. To get the third axis of the rotation
matrix, we compute $r_3 = r_1 \times r_2$ (cross product). Finally, the full rotation
matrix is $R = [r_1 \; r_2 \; r_3]$.

Because the checkerboard coordinates $X_w$ were given in meters, the translation
vector $t$ is also in meters. This is what gives us metric mapping from image
pixels to real-world coordinates.

We can verify that $H = K [r_1 \; r_2 \; t]$ holds. This $H$ is our final homography
matrix used for metric mapping.
