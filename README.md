Visual Relocalization
=====================

Monocular camera pose estimation on the COLMAP reconstruction model.

![](assets/demo-relocalization.gif)

## Clone the repository

**Recursively** clone the repository.

```
git clone --recursive https://github.com/CyberAgentAILab/visual-relocalization-colmap.git
```

## Requirements

* Docker
* Python >= Version 3.9
* Python pip

> [!NOTE]
> In the following instructions, we will launch rviz on a Docker container. Please note that it may not work if your host OS is incompatible with running rviz on a container.
> We confirmed that it works correctly when using Ubuntu 22.04 as the host OS.

## Download the dataset

Download the in-store visual localization dataset.

> [!NOTE]
> This project requires downloading a significantly large dataset.
> Please be mindful of network usage and ensure you have a stable and adequate internet connection before proceeding.

```
pip3 install huggingface_hub==0.24.5
python3 download_dataset.py
```

## Usage

Make sure that the `dataset` directory is placed under the repository root, then follow the instructions below.

Build the docker image.

```
./scripts/build_dockerfile.sh
```

Generate the PLY pointcloud file from the reconstruction model.

```
./scripts/make_ply.sh
```

Build the source files.

```
./scripts/build_source.sh
```

Run visual localization.

```
./scripts/run_localization.sh
```

Visualize the relocalization result. As RViz runs on a Docker container, rendering may be slower depending on the host OS or your computer specifications.

```
./scripts/launch_rviz.sh
```

## Dataset structure

Optionally, if you want to run on your own dataset, follow the directory structure below.

```
dataset/
  train/
    000000.png
    000001.png
    000002.png
    ...
  model/
    cameras.bin
    database.db
    images.bin
    points3D.bin
  test/
    000000.png
    000001.png
    000002.png
    ...
  intrinsic.xml
```

| Directory       | Contents                                     |
|-----------------|----------------------------------------------|
| `train`         | Images used to construct the COLMAP model.   |
| `model`         | The COLMAP model files.                      |
| `test`          | Test images to run relocalization.           |
| `intrinsic.xml` | Intrinsic camera parameters.                 |

### Camera intrinsic parameters

`intrinsic.xml` has the following structure.

```xml
<?xml version="1.0"?>
<opencv_storage>
<distortion type_id="opencv-matrix">
  <rows>5</rows>
  <cols>1</cols>
  <dt>d</dt>
  <data>
    -5.5531427264213562e-02 6.7047834396362305e-02
    -8.0864987103268504e-04 6.0979666886851192e-04
    -2.1628323942422867e-02</data></distortion>
<intrinsic type_id="opencv-matrix">
  <rows>3</rows>
  <cols>3</cols>
  <dt>d</dt>
  <data>
    6.4088488769531250e+02 0. 6.5048168945312500e+02 0.
    6.4027935791015625e+02 3.6642471313476562e+02 0. 0. 1.</data></intrinsic>
</opencv_storage>
```

| Tag        | Contents                                                            |
|------------|---------------------------------------------------------------------|
| distortion | Lens distortion parameters. Not used in the current implementation. |
| intrinsic  | Camera intrinsic parameters represented as a 3x3 matrix.            |

## LICENSE

Our code is distributed under the Apache 2 license.

### COLMAP

Our source code is making use of [colmap](https://github.com/colmap/colmap), which is distributed under the new BSD license.

```
Copyright (c) 2023, ETH Zurich and UNC Chapel Hill.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
      its contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
```
