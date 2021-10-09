# Control robot arm using eye-tracking
Enable moving robot arm using eye-tracking. (Master's thesis)

### Prerequisites
- Docker
- Get Tobii sdk
  - Download tobii sdk for C from https://www.tobiipro.com/product-listing/tobii-pro-sdk/
  - Extract the files from download to 'tobii_sdk' folder 

### Setup
Might have to change the id of the camera in tobii_driver launch

### Running

```
# Build image
bash docker/build.bash
# Run Image
bash docker/run_image.bash

```