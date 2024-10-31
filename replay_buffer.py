from cs285.infrastructure.utils import *


class ReplayBuffer(object):

    def __init__(self, max_size=1000000):

        self.max_size = max_size

        # store each rollout
        self.paths = []

        # store (concatenated) component arrays from each rollout
        self.obs = None
        self.acs = None
        self.rews = None
        self.next_obs = None
        self.terminals = None

    def __len__(self):
        # Jing: Do not use "if self.obs: xxx" as how the hw is initially assigned because self.obs is a numpy array, directly evaluate bool value 
        # of a numpy array will raise ValueError: The truth value of an array with more than one element is ambiguous. Use a.any() or a.all().
        # Origianl array [x, x] in python can be evaluated to be bool. []-->False nonempty array-->True
        if self.obs is not None:  
            return self.obs.shape[0]
        else:
            return 0

    def add_rollouts(self, paths, concat_rew=True):

        # add new rollouts into our list of rollouts
        for path in paths:
            self.paths.append(path)

        # convert new rollouts into their component arrays, and append them onto
        # our arrays
        observations, actions, rewards, next_observations, terminals = (
            convert_listofrollouts(paths, concat_rew))

        if self.obs is None:
            self.obs = observations[-self.max_size:]
            self.acs = actions[-self.max_size:]
            self.rews = rewards[-self.max_size:]
            self.next_obs = next_observations[-self.max_size:]
            self.terminals = terminals[-self.max_size:]
        else:
            self.obs = np.concatenate([self.obs, observations])[-self.max_size:]
            self.acs = np.concatenate([self.acs, actions])[-self.max_size:]
            if concat_rew:
                self.rews = np.concatenate(
                    [self.rews, rewards]
                )[-self.max_size:]
            else:
                if isinstance(rewards, list):
                    self.rews += rewards
                else:
                    self.rews.append(rewards)
                self.rews = self.rews[-self.max_size:]
            self.next_obs = np.concatenate(
                [self.next_obs, next_observations]
            )[-self.max_size:]
            self.terminals = np.concatenate(
                [self.terminals, terminals]
            )[-self.max_size:]

