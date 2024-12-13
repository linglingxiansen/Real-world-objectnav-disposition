# Real-world-objectnav-disposition

**Author:** Lingfeng Zhang

## Installation

The code has been tested only with Python 3.8.

1. Installing Dependencies
- Install [habitat-sim](https://github.com/facebookresearch/habitat-sim)0.2.2 and [habitat-lab](https://github.com/facebookresearch/habitat-lab)0.2.2

2. Install [pytorch](https://pytorch.org) The code is tested on pytorch v1.13.0

3. Install [detectron2](https://github.com/facebookresearch/detectron2/) according to your system configuration.
  
4. Download the [segmentation model](https://drive.google.com/file/d/1U0dS44DIPZ22nTjw0RfO431zV-lMPcvv/view?usp=share_link) in RedNet/model path.

5. Download the [GPT2-large](https://huggingface.co/openai-community/gpt2-large) and change the path in Real-world-objectnav-disposition.py.

6. Clone the repository and install other requirements:

```
git clone https://github.com/linglingxiansen/Real-world-objectnav-disposition
cd Real-world-objectnav-disposition/
pip install -r requirements.txt
```


### For demo: 
For evaluating in the real-world environment:
```
python Real-world-objectnav-disposition.py --split val_mini --eval 1 --auto_gpu_config 0 -n 1
--num_eval_episodes 2  --use_gtsem 0
--num_local_steps 10 --print_images 1 --sem_pred_prob_thr 0.8
-efw 1280 -efh 720 -fw 320 -fh 180 --hfov 90 
(change the arguments according to your camera parameters)
```

## Acknowledgement
We would like to thank the following repos for their open-set work:
- This work is built upon the [L3MVN](https://github.com/haotian-liu/LLaVA](https://github.com/ybgdgh/L3MVN)) and [TriHelper](https://github.com/linglingxiansen/TriHelper).
