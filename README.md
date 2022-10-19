# GPMP2

This library is an implementation of GPMP2 (Gaussian Process Motion Planner 2) algorithm described in [Motion Planning as Probabilistic Inference using Gaussian Processes and Factor Graphs](http://www.cc.gatech.edu/~bboots3/files/GPMP2.pdf) (RSS 2016). The core library is developed in C++ language with optional Python and MATLAB toolboxes. GPMP2 was started at the Georgia Tech Robot Learning Lab, see [THANKS](THANKS.md) for contributors.


## Prerequisites

- CMake >= 3.0 (Ubuntu: `sudo apt-get install cmake`), compilation configuration tool.
- [Boost](http://www.boost.org/) >= 1.65 (Ubuntu: `sudo apt-get install libboost-all-dev`), portable C++ source libraries.
- [GTSAM](https://github.com/borglab/gtsam/tree/develop), a C++ library that implements smoothing and mapping (SAM) framework in robotics and vision. Here we use the factor graph implementations and inference/optimization tools provided by GTSAM.
- [Python 3.6+](https://www.python.org/) needed if installing python toolbox.
- Matlab 2019b+ for the Matlab toolbox.

## Installation (C++ only)

- Install GTSAM.
  ```bash
  git clone https://github.com/borglab/gtsam.git
  cd gtsam
  mkdir build && cd build
  cmake -DGTSAM_ALLOW_DEPRECATED_SINCE_V42:=OFF .. # disable deprecated functionality for compatibility
  make -j4 check # optional, run unit tests  
  sudo make install
  ```

- Setup paths.
  ```bash
  echo 'export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}' >> ~/.bashrc
  echo 'export LD_LIBRARY_PATH=/usr/local/share:${LD_LIBRARY_PATH}' >> ~/.bashrc
  source ~/.bashrc
  ```

- Install gpmp2.
  ```bash
  git clone https://github.com/borglab/gpmp2.git
  cd gpmp2 && mkdir build && cd build
  cmake ..
  make -j4 check  # optional, run unit tests
  sudo make install
  ```

## Python Package Installation

- [Optional] Setup virtual environment.
  ```bash
  conda create -n gpmp2 pip python=3.6
  conda activate gpmp2
  pip install cython numpy scipy matplotlib
  ```

- Install gpmp2.
  ```bash
  git clone https://github.com/borglab/gpmp2.git
  cd gpmp2 && mkdir build && cd build
  cmake -DGPMP2_BUILD_PYTHON_TOOLBOX:=ON ..
  make -j8 # build
  make python-install # install the python package
  ```

At this point, you should be able to start a Python interpreter and load `gpmp2` via `import gpmp2`.

## Matlab Toolbox Installation

We clone, build and install `gpmp2` as usual, making sure to set the `GPMP2_BUILD_MATLAB_TOOLBOX` cmake flag.

  ```bash
  git clone https://github.com/borglab/gpmp2.git
  cd gpmp2 && mkdir build && cd build
  cmake -DGPMP2_BUILD_MATLAB_TOOLBOX:=ON ..
  make -j8 # build
  sudo make install
  ```

Start matlab and load the toolboxes be entering the following commands window:
```
addpath('/usr/local/gtsam_toolbox')
addpath('/usr/local/gpmp2_toolbox')
```
You should now be able to run any of the scripts in the `matlab/gpmp2_examples` directory.

## Citing

If you use GPMP2 in an academic context, please cite following publications:

```
@inproceedings{Mukadam-IJRR-18,
  Author = {Mustafa Mukadam and Jing Dong and Xinyan Yan and Frank Dellaert and Byron Boots},
  Title = {Continuous-time {G}aussian Process Motion Planning via Probabilistic Inference},
  journal = {The International Journal of Robotics Research (IJRR)},
  volume = {37},
  number = {11},
  pages = {1319--1340},
  year = {2018}
}

@inproceedings{Dong-RSS-16,
  Author = {Jing Dong and Mustafa Mukadam and Frank Dellaert and Byron Boots},
  Title = {Motion Planning as Probabilistic Inference using {G}aussian Processes and Factor Graphs},
  booktitle = {Proceedings of Robotics: Science and Systems (RSS)},
  year = {2016}
}

@inproceedings{dong2018sparse,
  title={Sparse {G}aussian Processes on Matrix {L}ie Groups: A Unified Framework for Optimizing Continuous-Time Trajectories},
  author={Dong, Jing and Mukadam, Mustafa and Boots, Byron and Dellaert, Frank},
  booktitle={2018 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={6497--6504},
  year={2018},
  organization={IEEE}
}
```


## License

GPMP2 is released under the BSD license, reproduced in [LICENSE](LICENSE).
