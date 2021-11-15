# gym-firepower

### Installation
```
cd ~
git clone https://github.com/chhokrad/gym-firepower.git
cd gym-firepower
sudo pip install -e .
```
This will install all the associated dependencies except GAMS, please refer to https://www.gams.com/latest/docs/UG_MAIN.html#UG_INSTALL for installing GAMS framework followed by https://www.gams.com/latest/docs/API_PY_TUTORIAL.html for setting up the Python API.

### Usage

The environment can be instantaited as :

``` python
import gym
env = gym.envs.make("gym_firepower:firepower-v0", kwargs)
env.reset()
env.render()
```
