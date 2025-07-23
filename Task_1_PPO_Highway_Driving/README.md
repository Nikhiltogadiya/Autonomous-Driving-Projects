

# Task 1: Highway Driving with Proximal Policy Optimization (PPO)

This project implements a Deep Reinforcement Learning (DRL) agent to navigate a simulated highway environment. The agent's goal is to learn complex driving behaviors, such as lane-changing, acceleration, and deceleration, to drive safely and efficiently. The training is performed using a custom implementation of the **Proximal Policy Optimization (PPO)** algorithm.

### Project Goals

  * **Collision Avoidance**: The primary objective is for the agent to avoid crashing into other vehicles.
  * **Lane Discipline**: The agent should learn to stay in the rightmost lane whenever possible.
  * **Speed Regulation**: The agent must try to maintain a designated target speed.
  * **Hyperparameter Exploration**: Experiment with different PPO and network hyperparameters to understand their impact on training performance and agent behavior.

### Key Concepts

  * **Deep Reinforcement Learning (DRL)**: A machine learning paradigm where an agent learns to make decisions by performing actions in an environment to maximize a cumulative reward signal.
  * **Proximal Policy Optimization (PPO)**: A state-of-the-art DRL algorithm that optimizes the policy network while preventing excessively large updates, which ensures more stable and reliable training. PPO uses a clipped surrogate objective function to achieve this.
  * **Actor-Critic Architecture**: The implementation uses two neural networks:
      * An **Actor Network** that decides which action to take (the policy).
      * A **Critic Network** that estimates the value of the current state, helping to guide the actor's learning process.
  * **Generalized Advantage Estimation (GAE)**: A technique used to reduce the variance of policy gradient estimates, leading to more stable training. It is calculated using the TD-error, a discount factor ($\\gamma$), and a lambda ($\\lambda$) parameter.

### How It Works

The solution is contained within the `highway_ppo_task.ipynb` script, which defines and trains the PPO agent.

1.  **Environment**: The simulation is built using the `highway-env` package, a Gymnasium environment designed for autonomous driving research.

2.  **PPO Model**: A custom PPO agent was implemented from scratch.

      * **`PPOMemory`**: A buffer to store trajectories (states, actions, rewards, etc.) collected during each episode.
      * **`ActorNetwork` & `CriticNetwork`**: Two separate feed-forward neural networks built with PyTorch. The actor outputs a probability distribution over actions, while the critic outputs a single value estimating the state's quality.
      * **`PPOAgent`**: This class orchestrates the learning process. It computes advantages using GAE and then iteratively updates the actor and critic networks by minimizing their respective loss functions.

3.  **Training Loop**:

      * The agent interacts with the environment for a set number of steps (`max_ep_len`) to collect a batch of experiences.
      * After collecting data, the `agent.learn()` function is called.
      * Inside `learn()`, the agent performs multiple optimization steps (`train_pi_iters`) on the collected data to update the policy and value functions.
      * The model with the best score is saved periodically to the `highway_ppo` directory.

