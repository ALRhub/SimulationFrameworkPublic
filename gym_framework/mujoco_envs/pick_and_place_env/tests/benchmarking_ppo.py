import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy as PpoMlpPolicy
from torch.utils.tensorboard import SummaryWriter

from gym_framework.mujoco_envs.pick_and_place_env.pick_and_place_env import PickAndPlaceMocapCtrl

EPISODE_LENGTH = 500
NSUBSTEPS = 12


def eval_model(env, model, deterministic=True, gamma=0.999):
    obs = env.reset()
    done = False
    rewards = []
    while not done:
        action, _ = model.predict(obs, deterministic=deterministic)
        obs, r, done, _ = env.step(action.reshape(-1, ))
        rewards.append(r)
        obs = env.reset() if done else obs
    return np.sum([gamma ** t * rew for t, rew in enumerate(rewards)])


if __name__ == '__main__':

    logger = SummaryWriter()

    env = PickAndPlaceMocapCtrl(render=0, max_steps=EPISODE_LENGTH, nsubsteps=NSUBSTEPS, random_env=False)
    policy_kwargs = dict(
        log_std_init=-1,
        # net_arch=[dict(vf=[128, 128], pi=[64, 64])],
        squash_output=True)

    model = PPO(policy=PpoMlpPolicy,
                env=env,
                # learning_rate=7e-4,
                verbose=0,
                n_steps=EPISODE_LENGTH * 4,
                policy_kwargs=policy_kwargs,
                gamma=0.999)

    Returns = []

    for i in range(1000):
        # Create the agent
        model.learn(total_timesteps=10000)
        return_det = eval_model(model=model, env=env, deterministic=True)
        Returns.append(return_det)
        logger.add_scalar(scalar_value=return_det, tag="PPO return", global_step=i)

        print("Iteration " + str(i), ". Return: ", return_det)

        # Save the last and the best model
        if return_det >= max(Returns):
            model.save("models/ppo_pickplace")
        model.save("models/ppo_pickplace_last")
