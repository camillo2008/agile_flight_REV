## Dependencies

We assume the same dependencies of the `agileflight` environment are met (essentially, Pytorch and StableBaseline)

## How to run

- You should copy the directory "model" which contains the code for the feature extractor into the `ros` directory.

- You need to edit `flightmare/flightpy/configs/vision/config.yaml` to set the correct image settings to match the ones used in your model, i.e dimension, FoV, orientaiton.

- In `user_code.py` you have to set the parameters according to the ones used to train your model: the path to the model, whether it uses images or obstacle states, whether it uses depth or RGB images, and the number of frames to stack.